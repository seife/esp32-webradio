/*
 * simple internet radio receiver
 * based on https://github.com/schreibfaul1/ESP32-audioI2S example ESP32_ES8388.ino
 *
 * (C) 2021-2022 Stefan Seyfried,  LICENSE: GPL version 3.0
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Key "features":
 * => no user interface yet, only rotary-encoder for volume
 * => control only via http interface
 *   curl 'http://webradio/control?play=st01.sslstream.dlf.de/dlf/01/mid/aac/stream.aac'
 *   => play http://st01.sslstream.dlf.de/dlf/01/mid/aac/stream.aac
 *   curl 'http://webradio/control?stop='
 *   => stop playback
 *   curl 'http://webradio/control?play='
 *   => resume last url
 *   curl 'http://webradio/control?vol=42'
 *   => set volume to 42
 *   curl 'http://webradio/control?vol=+5' or curl 'http://webradio/control?vol=-2'
 *   => increase / decrease volume
 *
 * Current config (url, volume, playstate) is saved to LITTLEFS 10 seconds after last change.
 * During start, config is loaded from LITTLEFS and playback resumes.
 * Littlefs tree:
 * -- /url => contains current URL
 *    /vol => contains current volume
 *    /playing => if present, radio was playing, if absent, radio was paused
 *
 * GPIO config is for the AI-Thinker ESP32 Audio Kit v2.2 with ES8388 audio chip
 * => https://github.com/Ai-Thinker-Open/ESP32-A1S-AudioKit
 *
 * TODO: add some User interface with a TFT, oled or a 16x2 LCD
 */

#define AAC_ENABLE_SBR 1
#include "Arduino.h"
#include "wifi_functions.h"

#include <WebServer.h>
#include <ElegantOTA.h>     // https://github.com/ayushsharma82/ElegantOTA, MIT License

#include "LITTLEFS.h"
#include "ES8388.h"         // https://github.com/maditnerd/es8388, GPLv3
#include "Audio.h"          // https://github.com/schreibfaul1/ESP32-audioI2S, GPLv3

#include <RotaryEncoder.h>  // https://github.com/mathertel/RotaryEncoder.git, BSD License


/* es8388 config */
// I2S GPIOs
#define I2S_SDOUT     26
#define I2S_BCLK      27
#define I2S_LRCK      25
#define I2S_MCLK       0    // shared with "BOOT" key on AudiKit v2.2, according to the schematic

// ES8388 I2C GPIOs
#define IIC_CLK       32
#define IIC_DATA      33

// Amplifier enable
#define GPIO_PA_EN    21
// Headphone detect
#define GPIO_HPD      39

/* Global variables */
int volume = 50; /* default if no config in flash */
unsigned long last_save = 0;
String A_station, A_streaminfo, A_streamtitle, A_bitrate, A_icyurl, A_lasthost, A_url;
bool playing = false;

enum { CONF_VOL = 1, CONF_URL = 2, CONF_PLAY = 4 };

RotaryEncoder *encoder = nullptr;

/* Global objects */
WebServer server(80);
ES8388 es;
Audio audio;

/* onboard  buttons
 *             BOOT
 *                1   2   3   4   5   6
 */
int keys[] = { 2, 36, 13, 19, 23, 18, 5, GPIO_HPD };
const char*keydesc[] = { "BOOT", "KEY1", "KEY2", "KEY3", "KEY4", "KEY5", "KEY6", "HPD" };
#if 0
/* SPI connection for future display code? */
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    5  // Chip select control pin
#define TFT_DC   22  // Data Command control pin
int keys[] = { 2, 36, 13, 19, GPIO_HPD };
const char*keydesc[] = { "BOOT", "KEY1", "KEY2", "KEY3", "HPD" };
#endif
#define NUMKEYS (sizeof(keys)/sizeof(int))

/* onboard LEDs */
#define LED_D4 22
#define LED_D5 19  /* conflicts with KEY_3 */

#define VOLCTRL ES8388::ES_MAIN

/* rotary encoder PINs, small JTAG pin header */
#define PIN_IN1 12
#define PIN_IN2 14

#define FORMAT_LITTLEFS_IF_FAILED true

/* maximum volume, 21 i2s + 96 es8288 steps */
#define MAX_VOL 117

/* web page helper function */
void add_header(String &s, String title)
{
    s += "<!DOCTYPE HTML><html><head>"
        "<title>" + title + "</title>"
        "<meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "</head>\n<body>"
        "<H1>" + title + "</H1>\n";
}

void handle_index()
{
    String index;
    add_header(index, "Webradio");
    index +=
        "<table>\n"
        "<tr><td>Currently playing</td><td>" + A_station + "</td></tr>\n"
        "<tr><td>Title</td><td>"+ A_streamtitle + "</td></tr>\n"
        "<tr><td>Stream URL</td><td>" + A_url + "</td></tr>\n"
        "<tr><td>found lasthost</td><td>" + A_lasthost + "</td></tr>\n"
        "<tr><td>found icy URL</td><td>" + A_icyurl +"</td></tr>\n"
        "</table>\n<p>\n"
        "<form action=\"/control\">"
        "Playback URL: "
        "<input type=\"text\" name=\"play\">"
        "<input type=\"submit\" value=\"Submit\"><p>\n"
        "<br><a href=\"/update\">Update software</a>\n"
        "</body>\n";
    server.send(200, "text/html", index);
}

/* just escape '"' in the json output. No idea if this is enough */
String json_replace(String &in)
{
    String out = "";
    for (int i = 0; i < in.length(); i++) {
        if (in[i] == '"')
            out += "\\\"";
        else
            out += in[i];
    }
    return out;
}

void handle_control()
{
    Serial.println("/control. Args:");
    for (int i = 0; i < server.args(); i++)
        Serial.printf("  %d: '%s'='%s'\r\n", i, server.argName(i).c_str(), server.arg(i).c_str());
    if (server.hasArg("play")) {
        String url = server.arg("play");
        if (url.length() > 0)
            A_url = url;
        change_station(A_url);
        last_save = millis();
    }
    if (server.hasArg("stop")) {
        playing = false;
        audio.stopSong();
        last_save = millis();
    }
    if (server.hasArg("vol")) {
        char sign = server.arg("vol")[0];
        int vol =server.arg("vol").toInt();
        if (sign == '-' || sign == '+')
            volume = set_volume(volume + vol);
        else
            volume = set_volume(vol);
    }
    /* TODO: use proper JSON liberary? */
    String index =
        "{\n"
        "  \"url\": \"" + json_replace(A_url) + "\",\n"
        "  \"station\": \"" + json_replace(A_station) + "\",\n"
        "  \"title\": \"" + json_replace(A_streamtitle) + "\",\n"
        "  \"playing\": " + String(playing) + ",\n"
        "  \"volume\": " + String(volume) + "\n"
        "}\n";
    server.send(200, "application/json", index);
}

/* encoder interrupt routine */
IRAM_ATTR void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

/*
 * set_volume(0...117)
 * 0...21: set hardware volume to 1, software i2s volume to 0...21
 * 22...117: set software i2s volume to max (21), hardware volume to vol-21
 */
int set_volume(int vol)
{
    if (vol < 0)
        vol = 0;
    if (vol > MAX_VOL)
        vol = MAX_VOL;
    if (vol < 22) {
        es.volume(VOLCTRL, 1);
        audio.setVolume(vol);
        Serial.printf("vol: %d ctrl: %d i2s: %d\r\n", vol, 1, vol);
    } else {
        es.volume(VOLCTRL, vol-21);
        audio.setVolume(21);
        Serial.printf("vol: %d ctrl: %d i2s: %d\r\n", vol, vol-21, 21);
    }
    last_save = millis();
    return vol;
}

/* LITTLEFS helper functions */
String read_file(File &file)
{
    String ret;
    while (file.available())
        ret += String((char)file.read());
    return ret;
}

int load_config(int &v, String &u, bool debug=false)
{
    int ret = 0;
    File cfg = LITTLEFS.open("/volume");
    if (cfg) {
        ret |= CONF_VOL;
        v = read_file(cfg).toInt();
        if (debug)
            Serial.printf("load_config: v is %d\r\n", v);
        cfg.close();
    }
    cfg = LITTLEFS.open("/url");
    if (cfg) {
        ret |= CONF_URL;
        u = read_file(cfg);
        if (debug)
            Serial.println("load_config: u is " + u);
        cfg.close();
    }
    if (LITTLEFS.exists("/playing"))
        ret |= CONF_PLAY;
    if (debug)
        Serial.println("load_config: ret = " +String(ret));
    return ret;
}

unsigned long save_config()
{
    int v, ret;
    String u;
    ret = load_config(v, u);
    if (!(ret & CONF_VOL) || v != volume) {
        Serial.println("Save volume...");
        File cfg = LITTLEFS.open("/volume", FILE_WRITE);
        cfg.print(volume);
        cfg.close();
    }
    if (!(ret & CONF_URL) || u != A_url) {
        Serial.println("Save URL...");
        File cfg = LITTLEFS.open("/url", FILE_WRITE);
        cfg.print(A_url);
        cfg.close();
    }
    if ((ret & CONF_PLAY) && !playing) {
        Serial.println("remove /playing");
        LITTLEFS.remove("/playing");
    }
    if (playing && !(ret & CONF_PLAY)) {
        Serial.println("create /playing");
        File cfg = LITTLEFS.open("/playing", FILE_WRITE);
        cfg.print("");
        cfg.close();
    }
    return millis();
}



void setup()
{
    Serial.begin(115200);
    Serial.println("\r\nReset");
    Serial.printf_P(PSTR("Free mem=%d\r\n"), ESP.getFreeHeap());
    psramInit();
    Serial.print("PSRAM: ");
    Serial.println(ESP.getPsramSize());
    bool littlefs_ok = LITTLEFS.begin(FORMAT_LITTLEFS_IF_FAILED);
    if (!littlefs_ok)
        Serial.println("LITTLEFS Mount Failed");
    else {
        Serial.print("LITTLEFS total: "); Serial.println(LITTLEFS.totalBytes());
        Serial.print("LITTLEFS used:  "); Serial.println(LITTLEFS.usedBytes());
        int ret = load_config(volume, A_url, true);
        playing = ret & CONF_PLAY;
    }

    encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);
    attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);

    start_WiFi();
    Serial.printf("Connect to ES8388 codec... ");
    while (not es.begin(IIC_DATA, IIC_CLK))
    {
        Serial.println("Failed!");
        delay(1000);
    }
    Serial.println("OK");

    es.volume(ES8388::ES_MAIN, 80);
    es.volume(ES8388::ES_OUT1, 80);
    es.volume(ES8388::ES_OUT2, 80);
    // dis- or enable amplifier
    pinMode(GPIO_PA_EN, OUTPUT);
    digitalWrite(GPIO_PA_EN, LOW); /* disable */

    audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_SDOUT);
    audio.i2s_mclk_pin_select(I2S_MCLK);

    volume = set_volume(volume);

    es.mute(ES8388::ES_OUT1, true);     /* loudspeaker */
    es.mute(ES8388::ES_OUT2, false);    /* headphone */
    es.mute(ES8388::ES_MAIN, false);
/*
    for (int i = 0; i < NUMKEYS; i++)
        pinMode(keys[i], INPUT_PULLUP);
*/
    server.on("/", handle_index);
    server.on("/index.html", handle_index);
    server.on("/control", handle_control);
    server.onNotFound([](){
        server.send(404, "text/plain", "The content you are looking for was not found.\n");
        Serial.println("404: " + server.uri());
    });

    ElegantOTA.begin(&server);
    server.begin();
}

void change_station(String url)
{
    A_station = A_streaminfo = A_streamtitle = A_bitrate = A_icyurl = A_lasthost = String();
    A_url = url;
    if (url.length() > 0) {
        playing = true;
        audio.connecttohost(A_url.c_str());
    } else {
        playing = false;
        audio.stopSong();
    }
}

void loop()
{
#if 0
    static bool key[NUMKEYS];
    static bool last[NUMKEYS];
    for (int i = 0; i < NUMKEYS; i++)
        key[i] = !digitalRead(keys[i]);

    if (key[1] && key[1] != last[1]) {
        Serial.println("WiFi.disconnect()!");
        /* for testing, sometimes seems to crash if audio is playing and wifi is disconnected */
        //audio.stopSong();
        WiFi.disconnect();
    }
    /* debug key presses... */
    int begin = true;
    for (int i = 0; i < NUMKEYS; i++) {
        if (key[i] != last[i]) {
            if (begin)
                Serial.print("key(s) changed:");
            Serial.printf(" %d[%d,%s]->%d", i, keys[i],keydesc[i], key[i]);
            begin = false;
        }
        last[i] = key[i];
    }
    if (!begin)
        Serial.println();
#endif
    audio.loop();

    /* if wifi is not connected initially, start playing after connect */
    if (playing && !audio.isRunning() && wifi_state == STATE_CONN) {
        if (A_url.length() > 0) {
            Serial.println("audio not running => connect!");
            change_station(A_url.c_str());
        }
    }
    /* volume control with the rotary encoder */
    int newPos = encoder->getPosition();
    if (newPos != 0) {
        encoder->setPosition(0);
        volume += newPos;
        volume = set_volume(volume);
    }
    server.handleClient();
    if (millis() - last_save > 10000)
        last_save = save_config();
}

/* functions for callbacks from audio decoder */
void audio_info(const char *info){
    Serial.print(">info        ====> "); Serial.println(info);
}
void audio_id3data(const char *info){  //id3 metadata
    Serial.print(">id3data     ");Serial.println(info);
}
void audio_eof_mp3(const char *info){  //end of file
    Serial.print(">eof_mp3     ");Serial.println(info);
}
void audio_showstation(const char *info){
    A_station = String(info);
    Serial.print(">station     ");Serial.println(info);
}
void audio_showstreaminfo(const char *info){
    A_streaminfo = String(info);
    Serial.print(">streaminfo  ");Serial.println(info);
}
void audio_showstreamtitle(const char *info){
    A_streamtitle = String(info);
    Serial.print(">streamtitle ");Serial.println(info);
}
void audio_bitrate(const char *info){
    A_bitrate = String(info);
    Serial.print(">bitrate     ");Serial.println(info);
}
void audio_commercial(const char *info){  //duration in sec
    Serial.print(">commercial  ");Serial.println(info);
}
void audio_icyurl(const char *info){  //homepage
    A_icyurl = String(info);
    Serial.print(">icyurl      ");Serial.println(info);
}
void audio_lasthost(const char *info){  //stream URL played
    A_lasthost = String(info);
    Serial.print(">lasthost    ");Serial.println(info);
}
void audio_eof_speech(const char *info){
    Serial.print(">eof_speech  ");Serial.println(info);
}
