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
 * => basic status display via a SPI connected SH1106 display
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
 * Current config (url, volume, playstate) is saved to LittleFS 10 seconds after last change.
 * During start, config is loaded from LittleFS and playback resumes.
 * Littlefs tree:
 * -- /url => contains current URL
 *    /vol => contains current volume
 *    /playing => if present, radio was playing, if absent, radio was paused
 *
 * GPIO config is for the AI-Thinker ESP32 Audio Kit v2.2 with ES8388 audio chip
 * => https://github.com/Ai-Thinker-Open/ESP32-A1S-AudioKit
 *
 * TODO: add some User interface to the display code.
 */

#define AAC_ENABLE_SBR 1
#include "Arduino.h"
#include "wifi_functions.h"

#include <WebServer.h>
#include <HTTPUpdateServer.h>

#include "LittleFS.h"
#include "ES8388.h"         // https://github.com/maditnerd/es8388, GPLv3
#include "Audio.h"          // https://github.com/schreibfaul1/ESP32-audioI2S, GPLv3

#include <RotaryEncoder.h>  // https://github.com/mathertel/RotaryEncoder.git, BSD License

#include "SH1106Spi.h"      // https://github.com/ThingPulse/esp8266-oled-ssd1306, MIT License
#include "OLEDDisplayUi.h"
#include "font.h"

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

/* SPI pins for SH1106 OLED display */
#define DISP_MOSI 23
#define DISP_SCLK 18
#define DISP_CS   -1  // Chip select control pin is hard wired to ground
#define DISP_DC    5  // Data Command control pin. 5 is VSPI_CS, but CS is unused.
#define DISP_RST  19  // original MISO, not used on OLED display

/* update rate for the OLED display */
#define DISPLAY_FPS 5

/* for easy handling of changed A_streaminfo... */
class String_plus : public String
{
 private:
     bool m_init;
 public:
     String_plus(String s = String()) : String(s), m_init { true } {};
     String_plus(const char* s) : String(s), m_init { true } {};
     bool changed(void) { bool _x = m_init; m_init = false; return _x; };
};

/* Global variables */
int volume = 50; /* default if no config in flash */
int enc_mode = (int)RotaryEncoder::LatchMode::FOUR0; /* 1,2 or3 */
int update_progress = 0;
unsigned long last_save = 0;
unsigned long last_volume = 0;
unsigned long last_reconnect = (unsigned long)-5000;
String A_streaminfo, A_bitrate, A_icyurl, A_lasthost, A_url;
String_plus A_streamtitle, A_station;
bool playing = false;
bool updating = false;

/* configuration */
int buf_sz = 32768;

enum { CONF_VOL = 1, CONF_URL = 2, CONF_PLAY = 4, CONF_ENC = 8 };

RotaryEncoder *encoder = nullptr;

/* Global objects */
WebServer server(80);
HTTPUpdateServer httpUpdater;
ES8388 es;
Audio audio;
SH1106Spi display(DISP_RST, DISP_DC, DISP_CS); /* CS is unused anyway */
OLEDDisplayUi ui(&display);

#if 0
/* onboard  buttons
 *             BOOT
 *                1   2   3   4   5   6
 */
int keys[] = { 2, 36, 13, 19, 23, 18, 5, GPIO_HPD };
const char*keydesc[] = { "BOOT", "KEY1", "KEY2", "KEY3", "KEY4", "KEY5", "KEY6", "HPD" };

/* with SPI display, most key GPIOs are used for SPI */
int keys[] = { 2, 36, 13, GPIO_HPD };
const char*keydesc[] = { "BOOT", "KEY1", "KEY2", "HPD" };
#define NUMKEYS (sizeof(keys)/sizeof(int))
#endif

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

uint32_t uptime_sec()
{
    return (esp_timer_get_time()/(int64_t)1000000);
}

/* web page helper function */
void add_header(String &s, String title)
{
    s += "<!DOCTYPE HTML><html lang=\"en\"><head>\n"
        "<meta charset=\"utf-8\">\n"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
        "<meta name=\"description\" content=\"my web radio\">\n"
        "<title>" + title + "</title>\n"
        "</head>\n<body>"
        "<H1>" + title + "</H1>\n";
}

String time_string(void)
{
    uint32_t now = uptime_sec();
    char timestr[10];
    String ret = "";
    if (now >= 24*60*60)
        ret += String(now / (24*60*60)) + "d ";
    now %= 24*60*60;
    snprintf(timestr, 10, "%02d:%02d:%02d", now / (60*60), (now % (60*60)) / 60, now % 60);
    ret += String(timestr);
    return ret;
}

void add_sysinfo(String &s)
{
    s += "<p>System information: "
        "Uptime: " + time_string() +
        ", Total PSRAM: " + String(ESP.getPsramSize() / 1024) +
        "kiB, Free PSRAM: " + String(ESP.getFreePsram() / 1024) +
        "kiB, Free heap: " + String(ESP.getFreeHeap() / 1024) +
        "kiB, build date: " + __DATE__ +", " + __TIME__ +
        "</p>\n";
}

void handle_index()
{
    String index;
    updating = false;
    add_header(index, "Webradio");
    index +=
        "<table>\n"
        "<tr><td>Currently playing</td><td>" + A_station + "</td></tr>\n"
        "<tr><td>Title</td><td>"+ A_streamtitle + "</td></tr>\n"
        "<tr><td>Stream URL</td><td>" + A_url + "</td></tr>\n"
        "<tr><td>found lasthost</td><td>" + A_lasthost + "</td></tr>\n"
        "<tr><td>found icy URL</td><td>" + A_icyurl +"</td></tr>\n"
        "</table>\n"
        "<p></p>\n"
        "<form action=\"/control\">"
        "Playback URL: "
        "<input name=\"play\" style=\"width: 50%\">"
        "<button type=\"submit\">Submit</button>\n"
        "<input type=\"hidden\" name=\"html\" value=\"true\">\n"
        "</form>\n"
        "<p><a href=\"/update\">Update software</a></p>\n";
    add_sysinfo(index);
    index +=
        "</body></html>\n";
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
    bool html = server.hasArg("html");
    if (server.hasArg("play")) {
        String url = server.arg("play");
        if (url.length() > 0)
            A_url = url;
        A_url.trim(); /* hack, a " " clears the URL */
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
    if (server.hasArg("enc_mode")) {
        int enc = server.arg("enc_mode").toInt();
        if (enc < (int)RotaryEncoder::LatchMode::FOUR3 || enc > (int)RotaryEncoder::LatchMode::TWO03)
            Serial.println("invalid encoder value: " + server.arg("enc_mode"));
        else {
            if (enc_mode != enc) {
                Serial.println("setting encoder type to " + server.arg("enc_mode"));
                RotaryEncoder *tmp = encoder;
                enc_mode = enc;
                encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, (RotaryEncoder::LatchMode)enc_mode);
                Serial.println("deleting original encoder");
                delete tmp;
                Serial.println("done");
            }
        }
    }
    /* TODO: use proper JSON liberary? */
    String index;
    if (html) {
        index =
        "<!DOCTYPE HTML><html lang=\"en\"><head>\n"
        "<meta charset=\"utf-8\">\n"
        "<meta http-equiv=\"refresh\" content=\"2; url=/\" />\n"
        "<title>Settings applied...</title>\n"
        "</head>\n<body>\n"
        "<p><a href=\"/\">back</a></p>\n"
        "</body>\n</html>\n";
    } else {
        index =
        "{\n"
        "  \"url\": \"" + json_replace(A_url) + "\",\n"
        "  \"station\": \"" + json_replace(A_station) + "\",\n"
        "  \"title\": \"" + json_replace(A_streamtitle) + "\",\n"
        "  \"playing\": " + String(playing) + ",\n"
        "  \"volume\": " + String(volume) + ",\n"
        "  \"enc_mode\": " + String(enc_mode) + ",\n"
        "  \"uptime\": " + String(uptime_sec()) + ",\n"
        "  \"heap_free\": " + String(ESP.getFreeHeap()) + ",\n"
        "  \"psram\": " + String(ESP.getPsramSize()) + ",\n"
        "  \"psram_free\": " + String(ESP.getFreePsram()) + "\n"
        "}\n";
    }
    server.send(200, html?"text/html":"application/json", index);
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
    last_volume = millis();
    return vol;
}

/* LittleFS helper functions */
String read_file(File &file)
{
    String ret;
    while (file.available())
        ret += String((char)file.read());
    return ret;
}

int load_config(int &v, int &e, String &u, bool debug=false)
{
    int ret = 0;
    File cfg = LittleFS.open("/volume");
    if (cfg) {
        ret |= CONF_VOL;
        v = read_file(cfg).toInt();
        if (debug)
            Serial.printf("load_config: v is %d\r\n", v);
        cfg.close();
    }
    cfg = LittleFS.open("/enc_mode");
    if (cfg) {
        ret |= CONF_ENC;
        e = read_file(cfg).toInt();
        if (debug)
            Serial.printf("load_config: e is %d\r\n", e);
        cfg.close();
    }
    cfg = LittleFS.open("/url");
    if (cfg) {
        ret |= CONF_URL;
        u = read_file(cfg);
        if (debug)
            Serial.println("load_config: u is " + u);
        cfg.close();
    }
    if (LittleFS.exists("/playing"))
        ret |= CONF_PLAY;
    if (debug)
        Serial.println("load_config: ret = " +String(ret));
    return ret;
}

unsigned long save_config()
{
    int v, e, ret;
    String u;
    ret = load_config(v, e, u);
    if (!(ret & CONF_VOL) || v != volume) {
        Serial.println("Save volume...");
        File cfg = LittleFS.open("/volume", FILE_WRITE);
        cfg.print(volume);
        cfg.close();
    }
    if (!(ret & CONF_ENC) || e != enc_mode) {
        Serial.println("Save enc_mode...");
        File cfg = LittleFS.open("/enc_mode", FILE_WRITE);
        cfg.print(enc_mode);
        cfg.close();
    }
    if (!(ret & CONF_URL) || u != A_url) {
        Serial.println("Save URL...");
        File cfg = LittleFS.open("/url", FILE_WRITE);
        cfg.print(A_url);
        cfg.close();
    }
    if ((ret & CONF_PLAY) && !playing) {
        Serial.println("remove /playing");
        LittleFS.remove("/playing");
    }
    if (playing && !(ret & CONF_PLAY)) {
        Serial.println("create /playing");
        File cfg = LittleFS.open("/playing", FILE_WRITE);
        cfg.print("");
        cfg.close();
    }
    return millis();
}

/* this gets called DISPLAY_FPS times per second */
void draw_display(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
    static uint16_t h_width = 0;
    static int offset = 0, off_line = 0;
    static int direction = -1;
    static int pause = 0, pause_line = 1;
    static int bufmax = buf_sz > 1024*100?1024*100:buf_sz;
    String status;
    display->setFont(DejaVu_Sans_12);
    if (A_streamtitle.changed()) {
        off_line = 0;
        pause_line = 1;
    }
    if (A_station.changed()) {
        h_width = display->getStringWidth(A_station);
        pause = 0;
        direction = -1;
        offset = 0;
    }
    /* draw station name, scrolling horizontally if it does not fit on the display */
    if (h_width > 128) {
        if (direction < 0 && h_width + offset <= 128 && pause <= 0) {
            pause = DISPLAY_FPS; /* wait for one second before reversing direction */
            direction = 1;
        }
        if (--pause <= 0) {
           offset += direction;
           pause = 0;
        }
        if (offset >= 0 && pause <= 0) {
            pause = DISPLAY_FPS;
            direction = -1;
        }
    } else {
        offset = 0;
        direction = -1;
    }
    display->drawString(x + offset, y, A_station);
    /* draw a small status line with codec, samplrate, bitrate, number of channels and
     * two bars: top is buffer fill, bottom bar is WiFi.rssi() */
    display->drawHorizontalLine(x, y + 16, 128);
    display->setFont(DejaVu_Sans_8);
    String sr = String(audio.getSampleRate() / 1000.0, 2);
    while (sr.endsWith("0")||sr.endsWith("."))
        sr.remove(sr.length()-1); /* remove trailing zeroes and dot */
    String br = String(audio.getBitRate(true)/1000);
    if (audio.isRunning())
        status = String(audio.getCodecname()) + "," +
            sr + "kHz," + br + "k," +
            String(audio.getChannels()) +"ch";
    else
        status = "not playing... ";
    int barstartx = display->getStringWidth(status) + 3;
    display->drawString(x, y+17, status);
    int barwidth;
    if (audio.isRunning()) {
        barwidth = (128-barstartx) * (audio.inBufferFilled() > bufmax?bufmax:audio.inBufferFilled()) / bufmax;
        display->fillRect(barstartx+x, y+18, barwidth, 3); /* buffer */
    }
    if (WiFi.RSSI() != 0)
        barwidth = (128-barstartx) * (100+(WiFi.RSSI() > -50?-50:WiFi.RSSI()))/ 50;
    else
        barwidth = 0;
    display->fillRect(barstartx+x, y+18+5, barwidth, 3); /* RSSI */
    display->drawHorizontalLine(x, y + 18 + 5 + 5, 128);
    /* draw stream title, scrolling vertically if it does not fit on the screen */
    display->setFont(DejaVu_Sans_11);
    while (isspace(A_streamtitle[off_line])) /* remove leading spaces. Should not be needed... */
        off_line++;
#if 0 /* unpatched esp8266-oled-ssd1306 */
    display->drawStringMaxWidth(x, y+28, 128, A_streamtitle);
#else /* patched github/seife/esp8266-oled-ssd1306 */
    int toomuch = display->drawStringMaxWidth(x, y+28, 128, A_streamtitle.substring(off_line));
    if (pause_line == 0) {
        if (toomuch != 0)
            off_line += toomuch;
        else
            off_line = 0;
    }
    pause_line++;
    pause_line %= (DISPLAY_FPS * 2); /* 2 seconds per line scroll */
#endif
    return;
}

#define VOL_H 12
void draw_volume(OLEDDisplay *display, OLEDDisplayUiState* state)
{
    if (!(updating && Update.isRunning()) && (millis() - last_volume > 5000))
        return; /* only drav volume bar for 5 seconds */
    int progress = volume * 100 / MAX_VOL;
    if (updating)
        progress = update_progress;
    /* clear background of volume bar */
    display->setColor(BLACK);
    display->fillRect(0, 64 - VOL_H - 1, 128, VOL_H + 1);
    /* draw volume_bar */
    display->setColor(WHITE);
    display->drawProgressBar(0, 64 - VOL_H - 1, 127, VOL_H, progress);
    /* draw caption inside volume bar */
    display->setColor(INVERSE);
    display->setFont(DejaVu_Sans_10);
    display->setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    display->drawString(64, 64 - VOL_H/2 - 1, String(updating ? "Update: ":"Volume ") +String(progress, DEC) + "%");
    /* reset settings */
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setColor(WHITE);
}

void draw_update_progress(size_t done, size_t total)
{
    static uint32_t size = ESP.getSketchSize();
    static int last_progress = -1;
    if (! updating) {
        updating = true;
        audio.stopSong();
        A_station = "OTA update...";
        A_streamtitle = "";
        Serial.println("\r\nUPDATE START");
    }
    /* due to httpupload's code, the size of the upload is not known :-(
     * "total" is just the maximum that fits into flash
     * so just use current sketch size as 100% for progress bar :-) */
    if (done < size)
        update_progress = done * 100 / size;
    else
        update_progress = 100;
    if (update_progress != last_progress) {
        Serial.printf("Progress: %u%% (%7d/%d)\r\n", update_progress, done, size);
        ui.update();
    }
}

FrameCallback frames[] = { draw_display };
int frameCount = 1;
OverlayCallback overlays[] = { draw_volume };
int overlaysCount = 1;

void setup()
{
    Serial.begin(115200);
    Serial.println("\r\nReset");
    Serial.printf_P(PSTR("Free mem=%d\r\n"), ESP.getFreeHeap());
    psramInit();
    Serial.print("PSRAM: ");
    Serial.println(ESP.getPsramSize());
    bool littlefs_ok = LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED);
    if (!littlefs_ok)
        Serial.println("LittleFS Mount Failed");
    else {
        Serial.print("LittleFS total: "); Serial.println(LittleFS.totalBytes());
        Serial.print("LittleFS used:  "); Serial.println(LittleFS.usedBytes());
        int ret = load_config(volume, enc_mode, A_url, true);
        playing = ret & CONF_PLAY;
    }

    SPI.begin(DISP_SCLK, DISP_MOSI, DISP_MOSI, DISP_CS); /* explicitly MISO=MOSI to free MISO pin for RESET */
    ui.setTargetFPS(DISPLAY_FPS);
    ui.setFrames(frames, frameCount);
    ui.setOverlays(overlays, overlaysCount);
    ui.disableAutoTransition();
    ui.disableAllIndicators();
    ui.init();
    display.setContrast(128);

    encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, (RotaryEncoder::LatchMode)enc_mode);
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
#if 0
    /* try to use PSRAM with buf_sz size */
    audio.setBufsize(-1, buf_sz);
#else
    /* do not try PSRAM */
    audio.setBufsize(buf_sz, 0);
#endif
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

    httpUpdater.setup(&server);
    server.begin();
    Update.onProgress(draw_update_progress);
}

void change_station(String url)
{
    A_station = A_streaminfo = A_streamtitle = A_bitrate = A_icyurl = A_lasthost = String();
    A_url = url;
    if (url.length() > 0) {
        playing = true;
        audio.connecttohost(A_url.c_str());
        last_reconnect = millis();
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
    if (!updating && playing && !audio.isRunning() && wifi_state == STATE_CONN) {
        if (A_url.length() > 0 && millis() - last_reconnect > 5000) {
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

    if (updating && Update.hasError()) {
        /* we should never be in the loop when Update.isRunning() ... so update failed. */
        updating = false;
        Update.clearError();
        A_streamtitle = "Update failed!";
        last_reconnect = millis(); /* show "Update failed" for 5 seconds... */
    }
    ui.update();
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
