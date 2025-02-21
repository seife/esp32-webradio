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

#if 0
#include "LittleFS.h"
#else
#include <Preferences.h>
#endif
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
int8_t balance = 0; /* -16 to +16 */
int enc_mode = (int)RotaryEncoder::LatchMode::FOUR0; /* 1,2 or3 */
int update_progress = 0;
unsigned long last_save = 0;
unsigned long last_volume = 0;
unsigned long last_reconnect = (unsigned long)-5000;
String A_streaminfo, A_bitrate, A_icyurl, A_lasthost, A_url;
String_plus A_streamtitle, A_station;
bool playing = false;
bool updating = false;
bool have_es8388 = true;
bool sleeping = false;
uint8_t brightness = 128;

/* encoder pushed? */
bool butt_down = false;
uint32_t butt_last = 0; /* last change */

/* configuration */
int buf_sz = 100*1024;

enum { CONF_VOL = 1, CONF_URL = 2, CONF_PLAY = 4, CONF_ENC = 8 };

RotaryEncoder *encoder = nullptr;

/* Global objects */
WebServer server(80);
HTTPUpdateServer httpUpdater;
ES8388 es;
Audio audio;
SH1106Spi display(DISP_RST, DISP_DC, DISP_CS); /* CS is unused anyway */
OLEDDisplayUi ui(&display);
Preferences pref;

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

/* ES_OUT1 Loudspeaker
 * ES_OUT2 Headphone
 * ES_MAIN ? => this is actually controlling DAC output attenuation, much like
 *              software volume control!
 * If ES_MAIN is used to control (and ES_OUT2 is left at high volume), then
 * low volume on ES_MAIN lead to significant backround noise
 */
#define VOLCTRL ES8388::ES_OUT2

/* rotary encoder PINs, small JTAG pin header */
#define PIN_IN1 12
#define PIN_IN2 14
#define PIN_BUT 13 /* push encoder, KEY2 on audiokit or JTAG header */

#define FORMAT_LITTLEFS_IF_FAILED true

/* maximum volume, audio.maxVolume i2s + 96 es8288 steps */
int MAX_VOL;

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
    snprintf(timestr, 10, "%02lu:%02lu:%02lu", now / (60*60), (now % (60*60)) / 60, now % 60);
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
        "<tr><td>found lasthost</td><td><a href=\"" + A_lasthost + "\">Link...</a></td></tr>\n"
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
        display.displayOn();
        sleeping = false;
        String url = server.arg("play");
        if (url.length() > 0)
            A_url = url;
        A_url.trim(); /* hack, a " " clears the URL */
        change_station(A_url);
        last_save = millis();
    }
    if (server.hasArg("stop")) {
        if (server.arg("stop").toInt()) {
            display.displayOff();
            sleeping = true;
        }
        playing = false;
        audio.stopSong();
        last_save = millis();
        A_station = "Stopped";
        A_streamtitle = "http://" + WiFi.localIP().toString();
    }
    if (server.hasArg("vol")) {
        char sign = server.arg("vol")[0];
        int vol =server.arg("vol").toInt();
        if (sign == '-' || sign == '+')
            volume = set_volume(volume + vol);
        else
            volume = set_volume(vol);
    }
    if (server.hasArg("bal")) {
        int bal =server.arg("bal").toInt();
        balance = set_balance(bal);
    }
    if (server.hasArg("brightness")) {
        brightness = server.arg("brightness").toInt();
        display.setBrightness(brightness);
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
        uint32_t b_fill = audio.inBufferFilled();
        uint32_t b_free = audio.inBufferFree();
        index =
        "{\n"
        "  \"url\": \"" + json_replace(A_url) + "\",\n"
        "  \"station\": \"" + json_replace(A_station) + "\",\n"
        "  \"title\": \"" + json_replace(A_streamtitle) + "\",\n"
        "  \"playing\": " + String(playing) + ",\n"
        "  \"bitrate\": " + String(audio.getBitRate(true)/1000) + ",\n"
        "  \"volume\": " + String(volume) + ",\n"
        "  \"volume_max\": " + String(MAX_VOL) + ",\n"
        "  \"balance\": " + String(balance) + ",\n"
        "  \"enc_mode\": " + String(enc_mode) + ",\n"
        "  \"brightness\": " + String(brightness) + ",\n"
        "  \"wifi_signal\": " + String(WiFi.RSSI()) + ",\n"
        "  \"wifi_bssid\": \"" + WiFi.BSSIDstr() + "\",\n"
        "  \"uptime\": " + String(uptime_sec()) + ",\n"
        "  \"heap_free\": " + String(ESP.getFreeHeap()) + ",\n"
        "  \"buffer_size\": " + String(b_fill + b_free) +",\n"
        "  \"buffer_free\": " + String(b_free) +",\n"
        "  \"buffer_perc\": " + String(b_fill * 100 /(b_fill + b_free)) +",\n"
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

IRAM_ATTR void button_push()
{
    butt_down = !digitalRead(PIN_BUT);
    butt_last = millis();
}

/* revert the 0-100 calculation in es.volume :-( */
void hw_volume(int vol)
{
    es.volume(VOLCTRL, vol * 100 / 0x21 + 1);
}

/*
 * set_volume(0...MAX_VOL)
 * 0...maxv: set hardware volume to 1, software i2s volume to vol
 * maxv+1...MAX_VOL: set software i2s to maxv, hardware volume to vol-maxv
 */
int set_volume(int vol)
{
    static uint8_t maxv = audio.maxVolume();
    if (vol < 0)
        vol = 0;
    if (vol > MAX_VOL)
        vol = MAX_VOL;
    if (vol <= maxv) {
        if (have_es8388)
            es.volume(VOLCTRL, 0); /* 0 == -30db, not muted */
        audio.setVolume(vol);
        Serial.printf("vol: %d ctrl: %d i2s: %d\r\n", vol, 0, vol);
    } else {
        if (have_es8388)
            hw_volume(vol - maxv);
            //es.volume(VOLCTRL, vol-maxv);
        audio.setVolume(maxv);
        Serial.printf("vol: %d ctrl: %d i2s: %d\r\n", vol, vol-maxv, maxv);
    }
    last_save = millis();
    last_volume = millis();
    return vol;
}

void hw_mute(bool mute)
{
    if (have_es8388) {
        es.mute(ES8388::ES_OUT1, true);     /* loudspeaker */
        es.mute(ES8388::ES_OUT2, mute);     /* headphone */
        es.mute(ES8388::ES_MAIN, mute);
    }
}

int set_balance(int bal)
{
    audio.setBalance(bal);
    return bal;
}

#if 0
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
#else
/* volume, enc_mode, url, playing */
void load_config()
{
    pref.begin("webradio", true);
    volume = pref.getInt("volume", volume);
    enc_mode = pref.getInt("enc_mode", enc_mode);
    A_url = pref.getString("url");
    playing = pref.getBool("playing", false);
    brightness = pref.getInt("brightness", brightness);
    pref.end();
}

unsigned long save_config()
{
    pref.begin("webradio", false);
    pref.putInt("volume", volume);
    pref.putInt("enc_mode", enc_mode);
    pref.putString("url", A_url);
    pref.putBool("playing", playing);
    pref.putInt("brightness", brightness);
    pref.end();
    return millis();
}
#endif

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
    sleeping = false;
    if (! updating) {
        hw_mute(true);
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
        last_progress = update_progress;
        Serial.printf("Progress: %u%% (%7d/%lu)\r\n", update_progress, done, size);
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
    Serial.printf_P(PSTR("Free mem=%lu\r\n"), ESP.getFreeHeap());
    psramInit();
    Serial.print("PSRAM: ");
    Serial.println(ESP.getPsramSize());
#if 0
    bool littlefs_ok = LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED);
    if (!littlefs_ok)
        Serial.println("LittleFS Mount Failed");
    else {
        Serial.print("LittleFS total: "); Serial.println(LittleFS.totalBytes());
        Serial.print("LittleFS used:  "); Serial.println(LittleFS.usedBytes());
        int ret = load_config(volume, enc_mode, A_url, true);
        playing = ret & CONF_PLAY;
    }
#else
    load_config();
#endif

    SPI.begin(DISP_SCLK, -1, DISP_MOSI, -1); /* unset MISO to free MISO pin for RESET */
    ui.setTargetFPS(DISPLAY_FPS);
    ui.setFrames(frames, frameCount);
    ui.setOverlays(overlays, overlaysCount);
    ui.disableAutoTransition();
    ui.disableAllIndicators();
    ui.init();
    display.setBrightness(brightness);

    encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, (RotaryEncoder::LatchMode)enc_mode);
    pinMode(PIN_BUT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_BUT), button_push, CHANGE);

    start_WiFi("esp-webradio");
    Serial.printf("Connect to ES8388 codec... ");
    if (not es.begin(IIC_DATA, IIC_CLK))
    {
        Serial.println("Failed!");
        have_es8388 = false;
        audio.setVolumeSteps(100); /* 100 steps for software control */
        MAX_VOL = audio.maxVolume();
    } else {
        Serial.println("OK");
        audio.setVolumeSteps(17); /* 17 steps for software control */
        MAX_VOL = audio.maxVolume() + 0x21; /* 0x21 = 33 real hw steps of es8388 */
    }
    Serial.print("MAX_VOL: "); Serial.println(MAX_VOL);

    if (have_es8388) {
        hw_mute(true);
        es.volume(ES8388::ES_MAIN, 100);
        es.volume(ES8388::ES_OUT1, 100);
        es.volume(ES8388::ES_OUT2, 100);
        // dis- or enable amplifier
        pinMode(GPIO_PA_EN, OUTPUT);
        digitalWrite(GPIO_PA_EN, LOW); /* disable */
    }

    if (have_es8388)
        audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_MCLK);
    else
        audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_SDOUT);
#if 1
    /* try to use PSRAM with buf_sz size */
    audio.setBufsize(-1, buf_sz + 4*4096);
#else
    /* do not try PSRAM */
    audio.setBufsize(buf_sz + 1600, 0);
#endif
    volume = set_volume(volume);

    hw_mute(false);
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
    butt_down = false; /* initialize to work around spurious "button down on start" problems */
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

static int _old_wifistate = -1;
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
    server.handleClient();
    if (sleeping) {
        delay(1000);
        return;
    }
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
    if (millis() - last_save > 10000)
        last_save = save_config();

    if (updating && Update.hasError()) {
        /* we should never be in the loop when Update.isRunning() ... so update failed. */
        updating = false;
        Update.clearError();
        A_streamtitle = "Update failed!";
        hw_mute(false);
        last_reconnect = millis(); /* show "Update failed" for 5 seconds... */
    }

    if (wifi_state == STATE_DISC && wifi_state != _old_wifistate) {
        A_station = "No WiFi connection";
        A_streamtitle = "Long press encoder for WPS";
    } else if (!playing && wifi_state == STATE_CONN && wifi_state != _old_wifistate) {
        A_station = "SSID: " + WiFi.SSID();
        A_streamtitle = "IP: " + WiFi.localIP().toString();
    }
    _old_wifistate = wifi_state;
    if (butt_down) {
        //Serial.print("button_down: "); Serial.print(butt_last); Serial.print(" "); Serial.print(digitalRead(PIN_BUT)); Serial.print(" ");Serial.println(millis());
        if (millis() - butt_last > 5000) {
            audio.stopSong();
            A_streamtitle = "Starting WPS";
            start_WPS();
            butt_down = false;
        }
    }
    ui.update();
}

/* functions for callbacks from audio decoder */

void audio_log(const char *tag, const char *info) {
    Serial.print(time_string());
    Serial.print(tag);
    Serial.println(info);
}

void audio_info(const char *info){
    audio_log(">info        ==> ", info);
}
void audio_id3data(const char *info){  //id3 metadata
    audio_log(">id3data     ", info);
}
void audio_eof_mp3(const char *info){  //end of file
    audio_log(">eof_mp3     ", info);
}
void audio_showstation(const char *info){
    A_station = String(info);
    audio_log(">station     ", info);
}
void audio_showstreaminfo(const char *info){
    A_streaminfo = String(info);
    audio_log(">streaminfo  ", info);
}
void audio_showstreamtitle(const char *info){
    A_streamtitle = String(info);
    // audio_log(">streamtitle ", info);
}
void audio_bitrate(const char *info){
    A_bitrate = String(info);
    audio_log(">bitrate     ", info);
}
#if 0
void audio_commercial(const char *info){  //duration in sec
    audio_log(">commercial  ", info);
}
#endif
void audio_icyurl(const char *info){  //homepage
    A_icyurl = String(info);
    audio_log(">icyurl      ", info);
}
void audio_lasthost(const char *info){  //stream URL played
    A_lasthost = String(info);
    audio_log(">lasthost    ", info);
}
void audio_eof_speech(const char *info){
    audio_log(">eof_speech  ", info);
}
