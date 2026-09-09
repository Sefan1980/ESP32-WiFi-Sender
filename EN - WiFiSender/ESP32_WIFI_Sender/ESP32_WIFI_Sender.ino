// ============================================================================
//  FIRST-TIME SETUP / INSTALLATION - please read this first!
// ============================================================================
//  1) Board settings in the Arduino IDE ("Tools" menu):
//     - Board: a matching ESP32 board (e.g. "ESP32 Dev Module")
//     - Partition Scheme:
//       "Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)"
//       This is IMPORTANT and NOT the default setting! This scheme
//       reserves space for TWO firmware copies (needed for OTA updates,
//       see below) AND a file area for LittleFS (where index.html,
//       settings.html and config.json live). With the default partition
//       scheme there's either no room for LittleFS, or OTA doesn't work
//       properly.
//
//  2) The VERY FIRST upload must be done via USB cable, not over OTA/WiFi!
//     Two reasons for that:
//     a) OTA (wireless updates) is a feature that only exists once THIS
//        sketch itself is running on the ESP32. A completely blank ESP32
//        doesn't know about OTA yet - the firmware has to get onto it
//        "the classic way" via cable once, before later updates can go
//        wirelessly.
//     b) The web pages (index.html, settings.html) are NOT uploaded when
//        you click "Upload" in the Arduino IDE - that only uploads this
//        .ino file. The HTML files live in the "data" subfolder and must
//        be transferred separately via the LittleFS upload plugin
//        (keyboard shortcut CTRL+SHIFT+P, see below) - and that plugin
//        only works over the USB cable connection, not over WiFi/OTA.
//
//     So the very first time around:
//       1. Connect the ESP32 via USB
//       2. Set the board + partition scheme as described above
//       3. Upload the sketch normally via USB (arrow icon/"Upload")
//       4. Right after that: press CTRL+SHIFT+P to transfer index.html,
//          settings.html (and optionally config.json) to the ESP32 via
//          LittleFS upload (also over USB)
//     Only AFTER this one-time USB setup can the firmware (the sketch
//     itself) be updated wirelessly via OTA. Changes to the HTML files in
//     the "data" folder still need to go through USB and the LittleFS
//     upload plugin.
// ============================================================================
//
// ============================================================================
//  DIYRobotLawnMower Headquarter 1.0 - 07.05.26
//  This program runs on an ESP32 and drives the "perimeter wire sender" of a
//  DIY robotic lawn mower. It sends a repeating electrical signal code through
//  a wire buried around the lawn; the mower's own sensor uses that signal to
//  know where the boundary of the lawn is and stay inside it.
//
//  On top of that, this sketch also:
//   - Runs a small web server so you can control everything from a browser
//   - Sends WhatsApp notifications (mower left/returned/timeout)
//   - Publishes sensor readings over MQTT (for home automation systems)
//   - Shows status information on a small OLED screen
//   - Supports "OTA" (Over-The-Air) firmware updates over WiFi
//
//  Beginner note: this file is one big ".ino" sketch. Arduino/ESP32 sketches
//  always need exactly two special functions:
//    - setup()  -> runs ONCE when the ESP32 boots, used for initialization
//    - loop()   -> runs OVER AND OVER AGAIN, forever, after setup() finishes
//  Every other function in this file (further below) is just a helper that
//  setup() or loop() (or the web server, or MQTT, ...) calls when needed.
// ============================================================================
//
// CTRL + SHIFT + P --> Upload LittleFS to Pico/ESP8266/ESP32 for DATA upload
//  --> Plugin: https://github.com/earlephilhower/arduino-littlefs-upload
//  (LittleFS is a tiny file system that lives inside the ESP32's flash memory.
//   The web pages (index.html, settings.html) and the saved settings
//   (config.json) are stored there - NOT in this .ino file.)
//
// Developer notes (kept from the original author):
// - MQTT works, but wasn't very stable at first (too many requests). The
//   client.flush() call around line ~1200 was disabled, which seems to help.
// - Solved: WhatsApp used to cause an HTTP 429 error ("Too Many Requests"),
//   and the log looked garbled ("...successfullyError sending the message").
//   That was simply a missing line break in a Serial.print() call, now fixed.
// - TODO: a freely adjustable sending frequency is not implemented yet.


//********************* Includes **********************************
// "#include" pulls in code from external libraries, so we don't have to write
// everything ourselves. Most of these need to be installed first through the
// Arduino/PlatformIO Library Manager.
#define CONFIG_LITTLEFS_FOR_IDF_3_2
#include <LittleFS.h>                  // Small file system inside the flash memory (stores web pages + settings)
#include <FS.h>                        // Generic file system support, needed by LittleFS
#include <ArduinoJson.h>               // Reads and writes JSON text (used for config.json) - library by Benoit
#include "soc/gpio_struct.h"           // Gives direct, low-level access to the GPIO pins so the interrupt (ISR) can run as fast as possible
#include "freertos/FreeRTOS.h"         // The ESP32's built-in mini "operating system" that can run several tasks at once
#include "freertos/task.h"             // Needed to create and manage background tasks (see whatsappTask() below)
#include "freertos/queue.h"            // Needed for the WhatsApp message queue (see "Code for WhatsApp" below)
#include <PubSubClient.h>              // MQTT client library - by Nick O'Leary
#include <WiFiManager.h>               // Handles WiFi setup through a captive portal, so you don't hardcode WiFi passwords - by tzapu
WiFiManager wm;                        // The one and only WiFiManager object we use throughout the sketch
#include <WiFiClient.h>                // Basic TCP/IP networking support
#include <HTTPClient.h>                // Lets the ESP32 make HTTP(S) requests (used to call the WhatsApp API)
#include <UrlEncode.h>                 // Makes text safe to put into a URL (spaces, special characters, ...) - by Masayuki
#include <INA226_WE.h>                 // Driver for the INA226 current/voltage sensor chip - by Wolfgang Ewald
#include <ArduinoOTA.h>                // Lets you upload new firmware over WiFi instead of a USB cable - by Juraj Andrassy

#include <U8x8lib.h>                   // Driver for small monochrome OLED displays - by Oliver Kraus
//********************* Display Settings **********************************
// Please UNCOMMENT exactly one of the constructor lines below - it depends on
// which exact OLED display chip you have (SSD1306, SH1106, ...).
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8x8setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
 //U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 	      
//U8X8_SSD1306_128X64_ALT0_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 	      // same as the NONAME variant, but may solve the "every 2nd line skipped" problem
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);           // <-- this is the display type actually in use
// End of constructor list


//********************* Defines **********************************
// "#define" creates a constant that gets replaced with its value everywhere
// in the code before compiling. It never changes while the program runs.
// Comment a line out (put "//" in front) if you don't need that feature -
// the surrounding "#ifdef ... #endif" blocks further down will then simply
// skip that piece of code entirely, which also makes the program smaller.
#define OTAUpdates 1                  // Enable OTA (wireless) firmware updates
#define MQTT 1                        // Enable sending sensor data to an MQTT broker
#define WhatsApp_messages 1           // Enable WhatsApp notifications (mower left/returned/timeout). Needs setup, see below:
                                      // 1) Create the Whatabot contact in your smartphone. The phone number is: +54 9 2364205798
                                      // 2) Send: "I allow whatabot to send me messages"
                                      // 3) Copy the phone number and the API key that Whatabot sent you.
                                      // Enter both on the settings page in the browser after uploading the sketch (/settings) - they get saved to config.json there.

// On the PCB there's a connector for a 2-color LED with common cathode(-).
// (Attention: the Matrix Mow800 board has a LED with common anode(+) instead!!!)
#define pinGreenLED 25                                  // Lights up when: station is ready / mower is charged / not charging

// Battery is charging if ChargeCurrent > LoadingThreshold
#define pinRedLED 26                                    // Lights up when: battery is actively charging

#define VER "DIYRobotLawnMower Headquarter 1.0 - 07.05.2026"   // Firmware version string, printed at startup and shown on screen


bool AUTO_START_SIGNAL = 1;           // If true: the perimeter wire signal starts/stops automatically based on the charging current (see loop())
//#define USE_BUTTON 0                // (not currently used) Use a physical button to start mowing or send the mower back to the station
#define SerialOutput 1              // If defined: print debugging text messages to the Serial Monitor (USB). Comment out to silence the log.
bool debug = false;                    // A second, separate debug switch, used mainly for WiFiManager-related messages
#define Screen 1                      // If defined: use the small OLED screen. Comment out if you don't have one connected.

#define WORKING_TIMEOUT_MINS 300      // Safety timeout (in minutes): if the mower is not back in its station after this long, the wire signal is switched off.
                                       // NOTE: this safety feature only works while AUTO_START_SIGNAL is OFF - if AUTO_START_SIGNAL is active, this setting is ignored!
#define PERI_CURRENT_MIN 200          // Minimum current (in milliamps) expected on the perimeter wire. Below this, we assume the wire is cut/broken.

// --- Pin assignments (which physical ESP32 pin does what) ---
#define I2C_SDA 21                    // I2C data pin, shared by both INA226 sensors and the OLED display
#define I2C_SCL 22                    // I2C clock pin, shared by both INA226 sensors and the OLED display
#define pinIN1 12                     // M1_IN1  ESP32 GPIO12       ( connect this pin to L298N-IN1)  - controls sender A's H-bridge
#define pinIN2 13                     // M1_IN2  ESP32 GPIO13       ( connect this pin to L298N-IN2)  - controls sender A's H-bridge
#define pinEnableA 23                 // ENA    ESP32 GPIO23         (connect this pin to L298N-ENA)  - turns sender A's H-bridge on/off
#define pinIN3 14                     // M1_IN3  ESP32 GPIO14       ( connect this pin to L298N-IN3)  - controls sender B's H-bridge
#define pinIN4 18                     // M1_IN4  ESP32 GPIO18       ( connect this pin to L298N-IN4)  - controls sender B's H-bridge
#define pinEnableB 19                 // ENB    ESP32 GPIO19        (connect this pin to L298N-ENA)   - turns sender B's H-bridge on/off
//#define pinDoorOpen 34              // Not in use (Magnetic switch)
//#define pinDoorClose 35             // Not in use (Magnetic switch)
//#define pinLDR 32                   // Not in use (Light Sensor)

// Beginner note: an "H-bridge" (here: an L298N driver module) is a small
// circuit that can push current through the perimeter wire in either
// direction (forward/backward) under the ESP32's control - that's exactly
// what's needed to create the alternating +1/-1 signal pattern.

WiFiClient espClient;                 // The underlying network connection object, shared by the MQTT client

//********************* WhatsApp Credentials **********************************
// Default values are intentionally empty. They're entered on the settings
// page in the browser and then saved to config.json (see processor(),
// saveSettingsToLittleFS() and the "/save" section further below).
// Important: these variables must be declared right HERE, near the very top -
// not further down under "Code for WhatsApp" - because processor() and
// saveSettingsToLittleFS() (both further up in the code) already use them.
// In C++/Arduino a variable must be declared BEFORE its first use, otherwise
// the compiler error "was not declared in this scope" happens.
String mobile_number = "";            // Your phone number (e.g. 4917012345678), only used if WhatsApp_messages is enabled
String api_key = "";                  // Your Whatabot API key, only used if WhatsApp_messages is enabled

//********************* MQTT Settings **********************************
// MQTT is a lightweight messaging protocol often used in home automation
// (e.g. with Home Assistant or Node-RED) to publish sensor readings.
// Important: these values are no longer fixed constants (const) - they're
// now regular variables (String/int), so they can be changed on the settings
// page in the browser and are permanently saved to config.json (see
// saveSettingsToLittleFS() and the "/save" section further below). The
// values here are just the starting values (used until your own settings
// have been saved).
#ifdef MQTT
String mqtt_server = "192.168.178.2";       // Address of your MQTT broker, e.g. "broker.hivemq.com" or a local IP address
int mqtt_port = 1883;                       // Standard (unencrypted) MQTT port
String clientID = "ESP32Sender";            // A unique name this device uses to identify itself to the broker
String MQTT_user = "mqtt";                  // Starting value (placeholder) - changed on the settings page and then saved to config.json
String MQTT_password = "mqtt";              // Starting value (placeholder) - changed on the settings page and then saved to config.json
String subTopic = "teensysender/input";              // MQTT "topic" (channel) this device listens to for incoming commands
String pubTopic = "teensysender/chargecurrent";      // Topic used to publish the charging current
String pubTopic2 = "teensysender/pericurrent";       // Topic used to publish the perimeter wire current
String pubTopic3 = "teensysender/chargevoltage";     // Topic used to publish the charging voltage
PubSubClient MQTTclient(espClient);         // The MQTT client object itself, built on top of the WiFi connection
long lastMsg = 0;                           // (currently unused leftover variable)
#endif

//********************* WiFi Settings **********************************
WiFiServer server(80);                       // A simple web server listening on port 80 (the standard HTTP port)
unsigned long lastWiFiCheckMillis = 0;       // Remembers when we last checked the WiFi connection (see keepWiFiAlive())
const unsigned long wifiCheckInterval = 15000; // How often to check the WiFi connection, in milliseconds (15 seconds)

//********************* INA226 Settings **********************************
// The INA226 is a small sensor chip that measures voltage and current very
// precisely. This project uses TWO of them: one to watch the perimeter wire,
// and one to watch the battery charging current.
INA226_WE InaPeri = INA226_WE(0x40);                    // 0x40 = the sensor's I2C address without the bridge soldered
float resistorPeri = 0.1;                               // Value (in Ohm) of the "shunt" resistor used for current measurement. For 10mOhm try 0.02, for 100mOhm use 0.1.
float rangePeri = 0.8;                                  // Expected measurement range in Amps. For a 10mOhm resistor try 8.0 or 4.0 - for 100mOhm use 0.8.

INA226_WE InaCharge = INA226_WE(0x44);                  // 0x44 = a different I2C address, set by soldering the "bridge" between pads A1 and VSS
float resistorCharge = 0.02;
float rangeCharge = 4.0;


//********************* Other **********************************
// Global variables: these values are shared across the whole program and can
// change while it's running (unlike the "#define" constants above).
bool WORKING_TIMEOUT = 0;               // Set to 1 once the safety timeout (WORKING_TIMEOUT_MINS) has triggered
bool mowerIsWorking = 0;                // Tracks whether the mower is currently believed to be out mowing (used for WhatsApp messages)
byte sigCodeInUse = 1;                  // Which signal code (0-4) is currently active. 1 is the original ardumower sigcode.
int sigDuration = 104;                  // How many microseconds each signal "bit" lasts (also commonly 50). Smaller = faster signal.
int8_t sigcode_norm[128];               // The signal code currently being transmitted by the ISR (copied here from sigcode0..sigcode4)
int sigcode_size;                       // How many entries of sigcode_norm[] are actually in use
hw_timer_t* timer = NULL;               // Handle for the ESP32's hardware timer that drives the signal output
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;  // A small lock used to protect the interrupt from being interrupted itself
volatile int stepA = 0;                 // Current position of sender A within the signal code (own counter, independent from B).
                                         // "volatile" tells the compiler this value can change at any time from inside an interrupt.
volatile int stepB = 0;                 // Current position of sender B within the signal code (own counter, independent from A)
boolean enableSenderA = false;          // Is sender A (wire loop A) currently transmitting? Starts OFF so the system boots into a safe state.
boolean enableSenderB = false;          // Is sender B (wire loop B) currently transmitting? Starts OFF so the system boots into a safe state.
int timeSeconds = 0;                    // Small seconds counter, used to build up whole minutes for the work timer
unsigned long nextTimeControl = 0;      // Timestamp (millis()) of the next "every 10 seconds" block in loop()
unsigned long nextTimeSec = 0;          // Timestamp (millis()) of the next "every 1 second" block in loop()
int workTimeMins = 0;                   // How many minutes the mower has been mowing since it last left the station
int workTimeChargeMins = 0;             // How many minutes the mower has been charging in the current charging session
int lastChargeMins = 0;                 // How many minutes the mower charged for during its last charging session
float PeriCurrent = 0.0;                // Current measured on the perimeter wire, in mA
float PeriBusVoltage = 0.0;             // Voltage measured at the perimeter wire, in V
float PeriShuntVoltage = 0.0;           // Tiny voltage drop across the perimeter measurement resistor, in mV
float ChargeCurrent = 0.0;              // Current measured on the charging contacts, in mA
float ChargeCurrentPrint = 0.0;         // Same as ChargeCurrent, but rounded down to 0 when below ChargeThreshold (nicer for display)
float ChargeBusVoltage = 0.0;           // Voltage measured at the charging contacts, in V
float ChargeShuntVoltage = 0.0;         // Tiny voltage drop across the charging measurement resistor, in mV
bool shouldSaveConfig = false;          // Flag set by WiFiManager when new WiFi settings need to be saved
bool wm_nonblocking = false;            // If true, WiFiManager's captive config portal would not block loop() (currently unused - portal always blocks)
//String AutoStartSignalPrint;                            // (unused leftover, kept for reference) for the web-part
//String linktext;                                        // (unused leftover, kept for reference) for the web-part
//String enableSenderAprint;                              // (unused leftover, kept for reference) for the web-part
//String enableSenderBprint;                              // (unused leftover, kept for reference) for the web-part


/*
  If the mower is in the station and fully charged, the current should be
  between PeriOnOffThreshold(3mA) and ChargeThreshold(10mA).

  If the perimeter starts up while the mower is in the station, set
  ChargeThreshold to 0. That way you can see the original ChargeCurrent
  value at http://Your-IP
  If the mower is outside, ChargeCurrent should be 0.
*/
float ChargeThreshold = 10.0;               // in mA. If the Chargecurrent is below this value, at  the Display shows "0mA" 
float PeriOnOffThreshold = 1.5;             // if ChargeCurrent is below this value, the perimeterloop starts working


//*********************  Sigcode list *********************************************
// Beginner note: these arrays are the actual "language" spoken over the
// perimeter wire. Each array is a sequence of +1 / -1 values; the ISR
// (onTimer(), further below) sends them out one after another, over and over,
// switching the wire's polarity for each value. The mower's onboard sensor
// recognizes the pattern and can tell WHICH area (0-4) it currently belongs
// to, which is how multi-zone gardens are possible. Must be a multiple of 2!
// More info: http://grauonline.de/alexwww/ardumower/filter/filter.html
// This is the "pseudonoise4_pw" signal (as used by the sender).

int8_t sigcode0[128] = { 1, -1 };
int sigcode0_size = 2;
int8_t sigcode1[128] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1 };
int sigcode1_size = 24;
int8_t sigcode2[128] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1 };
int sigcode2_size = 48;
int8_t sigcode3[128] = { 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1 };
int sigcode3_size = 128;
int8_t sigcode4[128] = { 1, 1, 1, -1, -1, -1 };
int sigcode4_size = 6;


//***** END OF CONFIGURATION *****
// Everything above this line is settings you might want to change.
// Everything below this line is the program logic itself.


//********************* Functions **********************************
// Beginner note: a "function" is a named, reusable block of code. Instead of
// writing the same instructions over and over, we write them once inside a
// function and then just "call" that function by name whenever we need it.

// Fills in the placeholders (like "%ChargeVoltage%") inside a web page's HTML
// text with the current, real sensor/status values, right before sending the
// page to the browser. This is how index.html and settings.html show live data.
// The "justSaved" parameter controls the %SAVE_NOTICE% placeholder (see
// below): if passed as "true", a short "Settings saved!" notice appears on
// the settings page. The parameter is optional (default: false), so existing
// calls to processor(html) without a second argument keep working.
String processor(String html, bool justSaved = false) {
    // 1. Simple numeric/text values: just find-and-replace the placeholder text
    html.replace("%ChargeVoltage%", String(ChargeBusVoltage, 2));
    html.replace("%ChargeCurrentPrint%", String(ChargeCurrentPrint, 1));
    html.replace("%ChargeCurrent%", String(ChargeCurrent, 1));
    html.replace("%PerimeterVoltage%", String(PeriBusVoltage, 2));
    html.replace("%PerimeterCurrent%", String(PeriCurrent, 2));
    html.replace("%PeriOnOffThreshold%", String(PeriOnOffThreshold));
    html.replace("%Uptime%", String(workTimeMins));
    html.replace("%SignalCode%", String(sigCodeInUse));
    html.replace("%Speed%", String(sigDuration));
    html.replace("%SigCode0%", sigcodeToString(sigcode0, sigcode0_size));
    html.replace("%SigCode1%", sigcodeToString(sigcode1, sigcode1_size));
    html.replace("%SigCode2%", sigcodeToString(sigcode2, sigcode2_size));
    html.replace("%SigCode3%", sigcodeToString(sigcode3, sigcode3_size));
    html.replace("%SigCode4%", sigcodeToString(sigcode4, sigcode4_size));

    // 2. Dynamic state (sender A) - decides the CSS class and ON/OFF text shown in the browser
    if (enableSenderA) { // Assuming this is your status check
        html.replace("%State_Class_A%", "label on");   // CSS class name - do NOT translate, must match td.on{...} in index.html!
        html.replace("%State_Text_A%", "on");
    } else {
        html.replace("%State_Class_A%", "label off");  // CSS class name - do NOT translate, must match td.off{...} in index.html!
        html.replace("%State_Text_A%", "off");
    }

    // 3. Dynamic state (sender B)
    if (enableSenderB) {
        html.replace("%State_Class_B%", "label on");
        html.replace("%State_Text_B%", "on");
    } else {
        html.replace("%State_Class_B%", "label off");
        html.replace("%State_Text_B%", "off");
    }

    // 4. Auto-Mode (based on the AUTO_START_SIGNAL variable)
    if (AUTO_START_SIGNAL) {
        html.replace("%State_Class_Auto%", "label on");
        html.replace("%State_Text_Auto%", "on");
    } else {
        html.replace("%State_Class_Auto%", "label off");
        html.replace("%State_Text_Auto%", "off");
    }

    // 5. MQTT settings block: only included if MQTT is enabled at all
    // (#ifdef MQTT). If MQTT is disabled, the placeholder is simply replaced
    // with an empty string - no MQTT section shows up on the page at all.
    #ifdef MQTT
      String mqttBlock = "";
      mqttBlock += "<h3 style='color:#1a3c34;margin-top:25px;'>MQTT Settings</h3>";
      mqttBlock += "<label><b>Broker address (IP or hostname):</b></label>";
      mqttBlock += "<input type='text' name='mqttServer' value='" + mqtt_server + "'><br />";
      mqttBlock += "<label><b>Port:</b></label>";
      mqttBlock += "<input type='text' name='mqttPort' value='" + String(mqtt_port) + "'><br />";
      mqttBlock += "<label><b>Client ID:</b></label>";
      mqttBlock += "<input type='text' name='mqttClientID' value='" + clientID + "'><br />";
      mqttBlock += "<label><b>Username:</b></label>";
      mqttBlock += "<input type='text' name='mqttUser' value='" + MQTT_user + "'><br />";
      mqttBlock += "<label><b>Password:</b></label>";
      mqttBlock += "<input type='password' name='mqttPassword' value='" + MQTT_password + "'><br />";
      mqttBlock += "<label><b>Subscribe topic (subTopic):</b></label>";
      mqttBlock += "<input type='text' name='mqttSubTopic' value='" + subTopic + "'><br />";
      mqttBlock += "<label><b>Publish topic - charge current:</b></label>";
      mqttBlock += "<input type='text' name='mqttPubTopic' value='" + pubTopic + "'><br />";
      mqttBlock += "<label><b>Publish topic - perimeter current:</b></label>";
      mqttBlock += "<input type='text' name='mqttPubTopic2' value='" + pubTopic2 + "'><br />";
      mqttBlock += "<label><b>Publish topic - charge voltage:</b></label>";
      mqttBlock += "<input type='text' name='mqttPubTopic3' value='" + pubTopic3 + "'><br />";
      html.replace("%MQTT_SETTINGS_BLOCK%", mqttBlock);
    #else
      html.replace("%MQTT_SETTINGS_BLOCK%", "");
    #endif

    // 6. WhatsApp settings block: only included if WhatsApp is enabled at all
    // (#ifdef WhatsApp_messages). Same principle as MQTT above.
    #ifdef WhatsApp_messages
      String waBlock = "";
      waBlock += "<h3 style='color:#1a3c34;margin-top:25px;'>WhatsApp Settings</h3>";
      waBlock += "<label><b>Phone number (e.g. 4917012345678):</b></label>";
      waBlock += "<input type='text' name='waPhone' value='" + mobile_number + "'><br />";
      waBlock += "<label><b>Whatabot API key:</b></label>";
      waBlock += "<input type='password' name='waApiKey' value='" + api_key + "'><br />";
      html.replace("%WHATSAPP_SETTINGS_BLOCK%", waBlock);
    #else
      html.replace("%WHATSAPP_SETTINGS_BLOCK%", "");
    #endif

    // 7. Save notice: only shown if we just arrived here via the redirect
    // from "/save" (see the justSaved parameter above). This gives the user
    // a clear, visible sign that their changes were applied - instead of the
    // page just reloading with no feedback at all.
    if (justSaved) {
      html.replace("%SAVE_NOTICE%", "<div style='background:#c8e6c9;color:#1a3c34;padding:10px;border-radius:6px;margin-bottom:15px;text-align:center;'>✅ Settings saved!</div>");
    } else {
      html.replace("%SAVE_NOTICE%", "");
    }

    return html;
}


// --- Helper functions for signal codes & URL parsing ---

// Turns a signal-code array (like sigcode0) back into a simple comma-separated
// text string (e.g. "1,-1,1,-1"), so it can be shown/edited on the settings page.
String sigcodeToString(int8_t* arr, int size) {
  String res = "";
  for (int i = 0; i < size; i++) {
    res += String(arr[i]);
    if (i < size - 1) res += ",";
  }
  return res;
}

// The opposite of sigcodeToString(): reads a comma-separated text string (as
// typed/edited by the user on the settings page, or loaded from config.json)
// and fills it into a signal-code array. Only accepts 1 and -1 as valid values,
// to protect the array from garbage input.
void parseSigcode(String input, int8_t* arr, int& size) {
  int tempSize = 0;
  int start = 0;
  // Note: URL-decoding (e.g. "%2C" -> ",") is no longer needed here - that's
  // now handled centrally by extractParam() (see urlDecode()) before the
  // value ever gets here. Values coming from config.json were never
  // URL-encoded in the first place.

  int idx = input.indexOf(',');

  while (idx != -1 && tempSize < 128) {
    int val = input.substring(start, idx).toInt();

    // Only accept exactly 1 or -1 - anything else is silently ignored
    if (val == 1 || val == -1) {
      arr[tempSize++] = (int8_t)val;
    }

    start = idx + 1;
    idx = input.indexOf(',', start);
  }

  // Check the very last value after the final comma (loop above stops one short)
  if (start < input.length() && tempSize < 128) {
    int val = input.substring(start).toInt();
    if (val == 1 || val == -1) {
      arr[tempSize++] = (int8_t)val;
    }
  }

  size = tempSize; // Report back how many valid 1/-1 values were actually found
}


// Reverses the URL-encoding that the browser automatically applies to form
// values when submitting a GET form. Browsers turn practically every
// "special" character into "%XX" (the two digits XX are the character code
// in hexadecimal) - a forward slash "/" becomes "%2F", a colon ":" becomes
// "%3A", a space becomes "+". Without this function, MQTT topics like
// "teensysender/input" would arrive as "teensysender%2Finput" and get saved
// exactly like that (wrong!).
String urlDecode(String input) {
  String decoded = "";
  char hexBuffer[3] = "00";   // Buffer for the 2 hex digits after a "%"
  unsigned int len = input.length();
  unsigned int i = 0;
  while (i < len) {
    char c = input.charAt(i);
    if (c == '+') {
      // A "+" in a query string stands for a space
      decoded += ' ';
      i++;
    } else if (c == '%' && i + 2 < len) {
      // "%XX" -> turn it into the one real character with code XX (hex)
      hexBuffer[0] = input.charAt(i + 1);
      hexBuffer[1] = input.charAt(i + 2);
      char realChar = (char) strtol(hexBuffer, NULL, 16);
      decoded += realChar;
      i += 3;
    } else {
      // A completely normal character, keep it unchanged
      decoded += c;
      i++;
    }
  }
  return decoded;
}


// Looks for "paramName=" inside a raw HTTP request/URL string and returns the
// text that comes right after it, up to the next "&" or space. This is how we
// read values submitted through the settings web form (e.g. .../save?peri=1.5).
// The returned text is automatically decoded with urlDecode(), so that e.g.
// "/" (sent as "%2F") arrives back as an actual forward slash.
String extractParam(String request, String paramName) {
  String searchStr = paramName + "=";
  int start = request.indexOf(searchStr);
  if (start == -1) return "";
  start += searchStr.length();
  int end = request.indexOf("&", start);
  if (end == -1) end = request.indexOf(" ", start);
  if (end == -1) return "";
  return urlDecode(request.substring(start, end));
}


// Saves the current settings (PeriOnOffThreshold, all 5 signal codes, and -
// if enabled - the MQTT and WhatsApp settings) as a JSON text file on the
// ESP32's built-in flash storage (LittleFS), so they survive a reboot/power
// loss. Call this manually whenever a setting changes.
void saveSettingsToLittleFS() {
  DynamicJsonDocument json(2048); // Larger buffer now, since MQTT and WhatsApp settings are also stored
  json["Peri"] = PeriOnOffThreshold;
  json["s0"] = sigcodeToString(sigcode0, sigcode0_size);
  json["s1"] = sigcodeToString(sigcode1, sigcode1_size);
  json["s2"] = sigcodeToString(sigcode2, sigcode2_size);
  json["s3"] = sigcodeToString(sigcode3, sigcode3_size);
  json["s4"] = sigcodeToString(sigcode4, sigcode4_size);

  // Only save MQTT settings if MQTT is actually compiled in - this way no
  // "dead" fields end up in the file when the feature isn't used.
  #ifdef MQTT
    json["mqttServer"] = mqtt_server;
    json["mqttPort"] = mqtt_port;
    json["mqttClientID"] = clientID;
    json["mqttUser"] = MQTT_user;
    json["mqttPassword"] = MQTT_password;
    json["mqttSubTopic"] = subTopic;
    json["mqttPubTopic"] = pubTopic;
    json["mqttPubTopic2"] = pubTopic2;
    json["mqttPubTopic3"] = pubTopic3;
  #endif

  // Only save WhatsApp settings if WhatsApp is actually compiled in
  #ifdef WhatsApp_messages
    json["waPhone"] = mobile_number;
    json["waApiKey"] = api_key;
  #endif

  File configFile = LittleFS.open("/config.json", "w");
  serializeJson(json, configFile);
  configFile.close();
  if(debug) Serial.println("Settings saved to flash memory.");
}


// Wipes the WiFi credentials AND the saved config.json, then reboots the
// ESP32 back into a completely fresh, "out of the box" state. Triggered from
// the settings web page ("Factory reset" button).
void factoryReset() {
  if(debug) Serial.println("FACTORY RESET! Deleting WiFi and config...");

  // 1. Erase the saved WiFi login (forces the WiFiManager setup portal next boot)
  wm.resetSettings();

  // 2. Delete only the settings file
  if (LittleFS.exists("/config.json")) {
    if (LittleFS.remove("/config.json")) {
      if(debug) Serial.println("config.json successfully deleted.");
    } else {
      if(debug) Serial.println("Error deleting config.json!");
    }
  } else {
    if(debug) Serial.println("No config.json found to delete.");
  }

  // 3. Optional: restore default values in RAM if needed (not currently used)
  // loadSettings(); // if you keep a "load defaults" function somewhere

  if(debug) Serial.println("Restarting...");
  delay(1000);
  ESP.restart();
}


//********************* Code for WhatsApp **********************************
// Phone number and API key are declared right at the top of the file (see
// "WhatsApp Credentials" further up) - that's also where the reason is explained.

// A "queue" is a thread-safe mailbox: sendWhatsappMessage() below drops a message into
// it and returns immediately, while a separate background task (whatsappTask(), started
// in setup()) picks messages up and actually sends them. This way the slow HTTPS request
// to the WhatsApp API never blocks the main loop() (web server, MQTT, timers, ...).
QueueHandle_t whatsappQueue = NULL;                     // Created in setup()
const long interval = 10000;                            // Minimum pause between two WhatsApp messages, to avoid HTTP 429 "Too Many Requests"

// Call this anywhere in the code to send a WhatsApp message. It only queues the message -
// the actual sending happens in whatsappTask().
void sendWhatsappMessage(String message){
  #ifdef WhatsApp_messages
    if (whatsappQueue != NULL) {
      // FreeRTOS queues can't store Arduino "String" objects directly, only plain
      // C-style character arrays, so we copy the message into a fixed-size buffer first.
      char msg_buffer[256];
      message.toCharArray(msg_buffer, sizeof(msg_buffer));
      if (xQueueSend(whatsappQueue, &msg_buffer, 0) != pdPASS) {
        // Queue was full (more than 5 messages already waiting) - message dropped.
        Serial.println("WhatsApp queue is full, message discarded.");
      }
    }
  #endif
}

// Background task: runs forever on core 1, completely separate from loop(). It sleeps
// (uses no CPU) until a message shows up in whatsappQueue, then sends it and waits
// "interval" milliseconds before handling the next one, so we don't hammer the API.
void whatsappTask(void *pvParameters) {
  char received_message_buffer[256];
  for (;;) {
    if (xQueueReceive(whatsappQueue, &received_message_buffer, portMAX_DELAY) == pdPASS) {
      String message = String(received_message_buffer);
      String API_URL = "https://api.whatabot.net/whatsapp/sendMessage?apikey=" + api_key + "&text=" + urlEncode(message) + "&phone=" + mobile_number;
      HTTPClient http;

      if (WiFi.status() == WL_CONNECTED) {
        http.begin(API_URL);
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");

        int http_response_code = http.GET();
        if (http_response_code == 200) {
          Serial.print("WhatsApp message sent successfully: ");
          Serial.println(message);
        } else {
          Serial.println("Error sending the message");
          Serial.print("HTTP response code: ");
          Serial.println(http_response_code);
        }

        http.end();
      } else {
        Serial.println("WhatsApp task: WiFi is not connected. Message was not sent.");
      }

      vTaskDelay(pdMS_TO_TICKS(interval));              // Cooldown before the next queued message is sent
    }
  }
}
// End of: Code for WhatsApp


//********************* SIGNAL MANAGEMENT **********************************
// This function is an Interrupt Service Routine (ISR): the hardware timer configured
// in setup() calls it automatically and very precisely every "sigDuration" microseconds
// (e.g. every 104us), no matter what else the main loop() is doing. That's what lets us
// output the perimeter-wire signal ("sigcode_norm" array) with exact timing.
// Because it's an ISR, it must be FAST and must not call things like Serial.print()
// or delay() - that's why it writes to the GPIO registers directly (GPIO.out_w1ts /
// GPIO.out_w1tc) instead of using the normal, slower digitalWrite().
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);                    // Briefly block other interrupts so this can't be disturbed halfway through
  if (enableSenderA) {
    if (sigcode_norm[stepA] == 1) {
      GPIO.out_w1tc = (1 << pinIN1);                    // Sets pinIN1 LOW  (w1tc = "write 1 to clear")
      GPIO.out_w1ts = (1 << pinIN2);                    // Sets pinIN2 HIGH (w1ts = "write 1 to set")
    } else if (sigcode_norm[stepA] == -1) {
      GPIO.out_w1ts = (1 << pinIN1);                    // Sets pinIN1 HIGH
      GPIO.out_w1tc = (1 << pinIN2);                    // Sets pinIN2 LOW
    }
    stepA++;                                            // Move sender A to the next bit of the signal code
    if (stepA == sigcode_size) {
      stepA = 0;                                        // Wrap back to the start once the whole code has been sent
    }
  }
  if (enableSenderB) {
    if (sigcode_norm[stepB] == 1) {
      GPIO.out_w1tc = (1 << pinIN3);                    // Sets pinIN3 LOW
      GPIO.out_w1ts = (1 << pinIN4);                    // Sets pinIN4 HIGH
    } else if (sigcode_norm[stepB] == -1) {
      GPIO.out_w1ts = (1 << pinIN3);                    // Sets pinIN3 HIGH
      GPIO.out_w1tc = (1 << pinIN4);                    // Sets pinIN4 LOW
    }
    stepB++;                                            // Move sender B to the next bit of the signal code (independent from stepA)
    if (stepB == sigcode_size) {
      stepB = 0;
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);                     // Allow other interrupts again
}
// End of signal management


//********************* CHANGE AREA **********************************
// Loads one of the sigcode0..sigcode4 arrays into sigcode_norm[], which is
// the array the onTimer() interrupt actually sends out bit by bit. This is
// how the mower is told to switch to mowing a different zone/area (0-4).
void changeArea(byte areaInMowing) {
  stepA = 0;                                            // Reset both senders back to the start of the (new) signal code
  stepB = 0;
  enableSenderA = false;                                // Briefly disable both senders while we swap the signal code, for safety
  enableSenderB = false;
  #ifdef SerialOutput
    Serial.print("Switch to area:");
    Serial.println(areaInMowing);
  #endif

  #ifdef Screen
    u8x8.clear();
    u8x8.setCursor(0,0);  
    u8x8.print("Area:");
    u8x8.println(areaInMowing);
  #endif
  for (int uu = 0; uu < 128; uu++) {  // Clear the whole buffer first (removes any leftovers from the previous code)
    sigcode_norm[uu] = 0;
  }
  sigcode_size = 0;

  // Copy the chosen signal code (and its length) into the "live" arrays used by the ISR
  switch (areaInMowing) {
    case 0:
      sigcode_size = sigcode0_size;
      for (int uu = 0; uu < sigcode_size; uu++) sigcode_norm[uu] = sigcode0[uu];
      break;
    case 1:
      sigcode_size = sigcode1_size;
      for (int uu = 0; uu < sigcode_size; uu++) sigcode_norm[uu] = sigcode1[uu];
      break;
    case 2:
      sigcode_size = sigcode2_size;
      for (int uu = 0; uu < sigcode_size; uu++) sigcode_norm[uu] = sigcode2[uu];
      break;
    case 3:
      sigcode_size = sigcode3_size;
      for (int uu = 0; uu < sigcode_size; uu++) sigcode_norm[uu] = sigcode3[uu];
      break;
    case 4:
      sigcode_size = sigcode4_size;
      for (int uu = 0; uu < sigcode_size; uu++) sigcode_norm[uu] = sigcode4[uu];
      break;
  }
  
    #ifdef SerialOutput
    Serial.print("New sigcode in use  : ");
    Serial.println(sigCodeInUse);

    for (int uu = 0; uu <= (sigcode_size - 1); uu++) {
      Serial.print(sigcode_norm[uu]);
      Serial.print(",");
    }
    
    Serial.println();
    Serial.print("New sigcode size  : ");
    Serial.println(sigcode_size);
  #endif

  #ifdef Screen
    u8x8.setCursor(0,2);
    u8x8.print("sigCode in use:");
    u8x8.print(sigCodeInUse);
    Serial.println();
    Serial.print("New sigcode size: ");
    Serial.println(sigcode_size);
    delay(2000);
  #endif
}
// END ChangeArea


//********************* STATICSCREENPARTS **********************************
// Draws the parts of the OLED screen layout that never change (labels like
// "Uptime:", "Peri mA:", ...). Called once every 10 seconds from loop() -
// the actual numeric values next to these labels are updated separately,
// more often, elsewhere in loop().
void StaticScreenParts() {
#ifdef Screen
  //line 0: Title
  u8x8.setCursor(0,0);
  u8x8.inverse();
  u8x8.print(" ESP32 Sender  ");
  u8x8.noInverse();

  //line 1: free
  u8x8.clearLine(1);

  //line 2: Sender ON/OFF
  u8x8.clearLine(2);

  //line 3: free
  u8x8.clearLine(3);

  //line 4: Uptime
  u8x8.setCursor(0, 4);
  u8x8.print("Uptime:");

  //line 5: Perimeter current
  u8x8.setCursor(0, 5);
  u8x8.print("Peri mA:");

  //line 6: Charge current
  u8x8.setCursor(0, 6);
  u8x8.print("Charge mA:");

//line 7: Area
  u8x8.setCursor(0, 7);
  u8x8.print("Area:");
#endif  
}
// END StaticScreenParts


//********************* SaveConfigCallback **********************************
// WiFiManager calls this function automatically (it's registered further
// down as a "callback") whenever the user has entered/changed WiFi settings
// through the captive setup portal. We just remember that with a flag here;
// the actual saving happens later in setup().
void saveConfigCallback () {  // callback notifying us of the need to save config
  if (debug) Serial.println("Should save config");
  shouldSaveConfig = true;
}
// END SaveConfigCallback


//********************* MQTTinput **********************************
// Called automatically by the MQTT library whenever a message arrives on a
// topic we're subscribed to. This lets you control the mower remotely from
// any MQTT-capable app or home automation system (Home Assistant, Node-RED, ...).
#ifdef MQTT
void MQTTinput(char* topic, byte* payload, unsigned int length) {

  Serial.print("Message arrived on topic: [");
  Serial.print(topic);
  Serial.print("] ");

  // Read the raw payload bytes and turn them into a normal text String
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();

  // command processing: compare the received text against known commands
  if (String(topic) == subTopic) {
    if (messageTemp == "AutoMode0") {
      AUTO_START_SIGNAL = 0;
    } else if (messageTemp == "AutoMode1") {
      AUTO_START_SIGNAL = 1;
    } else if (messageTemp == "A0") {
      enableSenderA = false;
      workTimeMins = 0;
      digitalWrite(pinEnableA, LOW);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
    } else if (messageTemp == "A1") {
      workTimeMins = 0;
      enableSenderA = true;
      digitalWrite(pinEnableA, HIGH);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);      
    } else if (messageTemp == "B0") {
      enableSenderA = false;
      workTimeMins = 0;
      digitalWrite(pinEnableB, LOW);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);
    } else if (messageTemp == "B1") {
      workTimeMins = 0;
      enableSenderB = true;
      digitalWrite(pinEnableB, HIGH);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);
    } else if (messageTemp == "sigDuration50") {
      sigDuration = 50;
      timerAlarm(timer, 50, true, 0);
    } else if (messageTemp == "sigDuration104") {
      sigDuration = 104;
      timerAlarm(timer, 104, true, 0);
    } else if (messageTemp == "sigCode0") {
      sigCodeInUse = 0;
      changeArea(sigCodeInUse);
    } else if (messageTemp == "sigCode1") {
      sigCodeInUse = 1;
      changeArea(sigCodeInUse);
    } else if (messageTemp == "sigCode2") {
      sigCodeInUse = 2;
      changeArea(sigCodeInUse);
    } else if (messageTemp == "sigCode3") {
      sigCodeInUse = 3;
      changeArea(sigCodeInUse);
    } else if (messageTemp == "sigCode4") {
      sigCodeInUse = 4;
      changeArea(sigCodeInUse);
    } else {
      Serial.println("An incorrect MQTT command was received.");
    }
  }

}   // END MQTTinput
#endif


//********************* Keep WiFi Alive **********************************
// Checks, every "wifiCheckInterval" milliseconds, whether the WiFi connection
// is still alive - and if not, tries to quietly reconnect in the background
// without blocking the rest of the program (loop() keeps running normally).
void keepWiFiAlive() {
  unsigned long currentMillis = millis();

  // Only actually check once the defined interval has passed
  if (currentMillis - lastWiFiCheckMillis >= wifiCheckInterval) {
    lastWiFiCheckMillis = currentMillis;

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Connection lost! Trying to reconnect in the background...");

      // Since WiFiManager already stored the WiFi credentials in the ESP32's
      // flash memory, a plain, non-blocking call to begin() is enough here:
      WiFi.begin(); 
    } else {
      Serial.print("[WiFi] Connected. IP: ");
      Serial.println(WiFi.localIP());
    }
  }
}




//********************* SETUP **********************************
// setup() runs exactly ONCE, right after the ESP32 powers on or resets.
// Its job is to prepare everything the rest of the program will need:
// starting the serial console, initializing sensors/display, starting the
// timer interrupt, connecting to WiFi, starting the web server, etc.
void setup() {
  Serial.begin(115200);                           // Start the USB serial connection (used for the debug log in your computer's Serial Monitor)
  Wire.begin(I2C_SDA, I2C_SCL);                   // Start the I2C bus (used to talk to both INA226 sensors and the OLED display)

  // *** FreeRTOS: create the WhatsApp queue and its background sending task ***
  // (see "Code for WhatsApp" further up for details)
  whatsappQueue = xQueueCreate(5, 256);           // Up to 5 pending messages, 256 bytes each
  xTaskCreatePinnedToCore(
    whatsappTask,                                 // Function that contains the task's code
    "WhatsappTask",                               // Task name (only used for debugging tools)
    10000,                                        // Stack size in bytes (needs to be large enough for HTTP/SSL)
    NULL,                                         // Parameters passed into the task function (none needed here)
    1,                                             // Task priority (a lower number means lower priority)
    NULL,                                         // Task handle (not needed here, so NULL)
    1                                              // Run on core 1 (core 0 is often busy with WiFi/Bluetooth)
  );

  InaPeri.init();                                 // Initialize the INA226 sensor used for perimeter wire measurement
  InaCharge.init();                               // Initialize the INA226 sensor used for charging measurement

  #ifdef Screen
  u8x8.begin();                                   // Start the OLED screen
  u8x8.setFont(u8x8_font_5x8_f);                  // Choose a small, readable font
  u8x8.clear();
  #endif
  timer = timerBegin(1000000);                    // Start a hardware timer that ticks 1,000,000 times per second (1 tick = 1 microsecond)
  timerAttachInterrupt(timer, &onTimer);           // Tell the timer to call onTimer() automatically on every alarm
  timerAlarm(timer, sigDuration, true, 0);         // Set the alarm to trigger every "sigDuration" microseconds, and repeat forever
  pinMode(pinIN1, OUTPUT);                        // Configure all the driver control pins as digital OUTPUTs
  pinMode(pinIN2, OUTPUT);
  pinMode(pinEnableA, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinEnableB, OUTPUT);
  pinMode(pinGreenLED, OUTPUT);                   // 2-color LED green
  pinMode(pinRedLED, OUTPUT);                     // 2-color LED red
  digitalWrite(pinGreenLED, LOW);                 // Make sure both LEDs start off
  digitalWrite(pinRedLED, LOW);
  
  #ifdef SerialOutput
    Serial.println("START");
    Serial.print("ESP32 Sender ");
    Serial.println(VER);
  #endif
  
  changeArea(sigCodeInUse);                       // Load the initially selected signal code into the ISR's working array
  if (enableSenderA) {
    digitalWrite(pinEnableA, HIGH);
  }
  if (enableSenderB) {
    digitalWrite(pinEnableB, HIGH);
  }

  //------------------------  Flash save parts  ----------------------------------------
  WiFi.mode(WIFI_STA);                             // "Station" mode: connect to an existing WiFi network (instead of creating our own)
  if (LittleFS.begin(true)) {   // Mount the flash file system (true = format it automatically if it's ever unreadable)
    if (LittleFS.exists("/config.json")) {  // If a saved settings file exists, load it
      File configFile = LittleFS.open("/config.json", "r");   // open the file for reading
      if (configFile) {
        size_t size = configFile.size();  // Find out how big the file is, so we can allocate a big enough buffer
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(2048); // Larger buffer now, since MQTT and WhatsApp settings are also stored
        auto deserializeError = deserializeJson(json, buf.get());
        serializeJson(json, Serial);
        
        if ( ! deserializeError ) {
          if (json["Peri"]) {
            PeriOnOffThreshold = json["Peri"];
          }
          if (json["s0"]) {
            parseSigcode(json["s0"].as<String>(), sigcode0, sigcode0_size);
          }
          if (json["s1"]) {
            parseSigcode(json["s1"].as<String>(), sigcode1, sigcode1_size);
          }
          if (json["s2"]) {
            parseSigcode(json["s2"].as<String>(), sigcode2, sigcode2_size);
          }
          if (json["s3"]) {
            parseSigcode(json["s3"].as<String>(), sigcode3, sigcode3_size);
          }
          if (json["s4"]) {
            parseSigcode(json["s4"].as<String>(), sigcode4, sigcode4_size);
          }
          // --- Load MQTT settings (only if MQTT is enabled at all) ---
          #ifdef MQTT
            if (json["mqttServer"]) mqtt_server = json["mqttServer"].as<String>();
            if (json["mqttPort"]) mqtt_port = json["mqttPort"];
            if (json["mqttClientID"]) clientID = json["mqttClientID"].as<String>();
            if (json["mqttUser"]) MQTT_user = json["mqttUser"].as<String>();
            if (json["mqttPassword"]) MQTT_password = json["mqttPassword"].as<String>();
            if (json["mqttSubTopic"]) subTopic = json["mqttSubTopic"].as<String>();
            if (json["mqttPubTopic"]) pubTopic = json["mqttPubTopic"].as<String>();
            if (json["mqttPubTopic2"]) pubTopic2 = json["mqttPubTopic2"].as<String>();
            if (json["mqttPubTopic3"]) pubTopic3 = json["mqttPubTopic3"].as<String>();
          #endif
          // --- Load WhatsApp settings (only if WhatsApp is enabled at all) ---
          #ifdef WhatsApp_messages
            if (json["waPhone"]) mobile_number = json["waPhone"].as<String>();
            if (json["waApiKey"]) api_key = json["waApiKey"].as<String>();
          #endif
        } else {
          if (debug) Serial.println("failed to load json config");
        }
      }
    }
  } else {
    if (debug) Serial.println("failed to mount FS");
  }   // end read

  //------------------------ MQTT ------------------------------------------------
  // Only call this HERE (after loading the settings above) with the actually
  // valid values - this way saved MQTT settings are actually used and not
  // overwritten by the starting values.
  #ifdef MQTT
  MQTTclient.setServer(mqtt_server.c_str(), mqtt_port);       // Set broker address and port
  MQTTclient.setCallback(MQTTinput);                          // MQTTinput() will be called automatically whenever data is received
  #endif

  wm.setSaveConfigCallback(saveConfigCallback);    // set config save notify callback

  // No static IP is set here on purpose - the router assigns the IP address (and
  // gateway/subnet) automatically via DHCP each time the ESP32 connects.

  // Limits how long the ESP32 tries to connect to a known WiFi network before giving up (in seconds)
  wm.setConnectTimeout(15); 

  // Limits how long the configuration WiFi (the "Access Point" you connect to for setup) stays active (in seconds).
  // If nobody connects to it in that time, it gives up and loop() starts anyway (offline mode).
  wm.setConfigPortalTimeout(60); 

  // Enable automatic background reconnect (a native ESP32 feature)
  WiFi.setAutoReconnect(true);

  if (!wm.autoConnect("MowerSender_AP","12345678")) { // The password should have at least 8 characters.
    Serial.println("WiFi connection failed or timed out. Starting in offline mode...");
  } else {
    if (debug) Serial.println("connected :)");
  }
  
  if (shouldSaveConfig) {   
    saveSettingsToLittleFS();
  }

  sendWhatsappMessage(String("ESP32 sender is now online. IP address: ") + WiFi.localIP().toString());          // Code for WhatsApp
  server.begin();                                  // Start the web server, so it's now ready to accept browser connections


  //------------------------  current sensor parts  ----------------------------------------
  #ifdef SerialOutput
    Serial.println("Measuring voltage and current using INA226 ...");
  #endif

  InaPeri.setAverage(INA226_AVERAGE_4);                          // Average 4 samples per reading, to smooth out noise
  InaPeri.setResistorRange(resistorPeri, rangePeri);
//  InaPeri.waitUntilConversionCompleted();
  InaCharge.setAverage(INA226_AVERAGE_4);
  InaCharge.setResistorRange(resistorCharge, rangeCharge);
//  InaCharge.waitUntilConversionCompleted();



  //------------------------  ArduinoOTA  ----------------------------------------
  // OTA = "Over-The-Air" updates: lets you upload new firmware over WiFi
  // instead of needing a USB cable plugged in.
 #ifdef OTAUpdates
 ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else
        type = "filesystem";

      // NOTE: if updating LittleFS this would be the place to unmount LittleFS using LittleFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  #endif
}
// END SETUP


//********************* LOOP **********************************
// loop() runs over and over again, forever, as fast as the ESP32 can manage,
// right after setup() finishes. Important: keep it lightweight/fast wherever
// possible - all truly time-critical work (the perimeter signal output)
// happens separately in the onTimer() interrupt, NOT here. loop() handles
// everything else: checking WiFi, OTA updates, sensor readings, timers,
// MQTT, and serving the web pages.
void loop() {

  keepWiFiAlive(); // Checks the WiFi connection at set intervals and tries to restore it if needed.

  #ifdef MQTT
    // IMPORTANT: MQTTclient.loop() must be called regularly so the library
    // keeps the connection alive AND processes incoming messages (calls
    // MQTTinput() when needed). This was completely missing before - without
    // this call, MQTT commands never arrived, no matter what was subscribed.
    MQTTclient.loop();
  #endif

  // Check for wireless software updates
  #ifdef OTAUpdates
    ArduinoOTA.handle();
  #endif

  
  // --- TIMER BLOCK: Executes every 10 seconds ---
  if (millis() >= nextTimeControl) {
    // Set the time for the next execution (current time + 10 seconds)
    nextTimeControl = millis() + 10000;

    // Update the static text labels on the OLED display
    #ifdef Screen
      StaticScreenParts();
    #endif


    #ifdef MQTT
      // Only try ONCE per 10-second cycle instead of looping forever - if the broker
      // is unreachable, an endless while() loop here would freeze the whole ESP32
      // (web server, OTA, WiFi...) and could trigger a watchdog reset. If this attempt
      // fails, we simply try again the next time this block runs.
      if (!MQTTclient.connected()) {
        if (MQTTclient.connect(clientID.c_str(), MQTT_user.c_str(), MQTT_password.c_str())) {
          // After a new connection, we need to (re-)subscribe to the desired
          // topic with the broker, otherwise commands never arrive via
          // MQTTinput() - this was missing in the original version!
          MQTTclient.subscribe(subTopic.c_str());
        }
      }
      // Convert the charge current to a String 
      String MQTTmsgString = String(ChargeCurrent, 2); // float to String with 2 decimals
      // Publish the charging current to the MQTT broker
      MQTTclient.publish(pubTopic.c_str(), MQTTmsgString.c_str());
      
      // Convert the perimeter loop current to a String
      String MQTTmsgString2 = String(PeriCurrent, 2); // float to String with 2 decimals
      // Publish the perimeter current to the MQTT broker
      MQTTclient.publish(pubTopic2.c_str(), MQTTmsgString2.c_str());
 
      // Convert the ChargeBusVoltage (float) to a String for MQTT
      String MQTTmsgString3 = String(ChargeBusVoltage, 2); // float to String with 2 decimals
      // Publish the charging current to the MQTT broker
      MQTTclient.publish(pubTopic3.c_str(), MQTTmsgString3.c_str());
    #endif
    // END MQTT

    // Read the voltage from the INA226 sensor (Perimeter)
    PeriBusVoltage = InaPeri.getBusVoltage_V();
    // Read the tiny voltage drop across the measurement resistor
    PeriShuntVoltage = InaPeri.getShuntVoltage_mV();
    // Read the actual current flowing through the perimeter wire
    PeriCurrent = InaPeri.getCurrent_mA();

    // Subtract the base consumption of the electronics (ESP32/Drivers)
    PeriCurrent = PeriCurrent - 80.0;                        //the DC/DC, ESP32, LN298N drain between 80 and 100 mA when nothing is ON and a wifi access point is found (To confirm ????)

    // If the current is too low, just set it to 0 to avoid "ghost" readings
    if (PeriCurrent <= PERI_CURRENT_MIN) PeriCurrent = 0;

    // If a sender is active but no current flows, the wire is likely broken
    if ((enableSenderA) && (PeriCurrent < PERI_CURRENT_MIN)) {
      workTimeMins = 0; // Reset work timer
      #ifdef Screen
        u8x8.setCursor(0, 5);
        u8x8.inverse();   // Highlight text
        u8x8.print("  Wire is Cut!  ");
        u8x8.noInverse();
      #endif
      #ifdef SerialOutput
        Serial.println("WIRE IS CUT!!!");
      #endif
        
    } else {
      // If everything is fine, show the current on the display
      #ifdef Screen
        u8x8.setCursor(8, 5);
        u8x8.print("        ");  // Clear old value
        u8x8.setCursor(10, 5);
        u8x8.print(PeriCurrent);
      #endif
      #ifdef SerialOutput
        Serial.print("Pericurr ");
        Serial.println(PeriCurrent);
        Serial.print("PeriVoltage ");
        Serial.println(PeriBusVoltage);
      #endif
    }

    // Safety switch-off: If the mower has been working for too long (e.g. 5 hours)
    if (workTimeMins >= WORKING_TIMEOUT_MINS && AUTO_START_SIGNAL == 1) {
      enableSenderA = false;  // Turn off Loop A
      enableSenderB = false;  // Turn off Loop B
       workTimeMins = 0;      // Reset timer
      WORKING_TIMEOUT = 1;    // Set timeout flag
      digitalWrite(pinEnableA, LOW);
      
      // Physically pull pins LOW to stop the L298N driver
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      digitalWrite(pinEnableB, LOW);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);

      // Alert the user via WhatsApp
      sendWhatsappMessage("TIMEOUT! Mower didn't come back home! Perimeterwire switched off!");          // Code for WhatsApp
      Serial.println("********************************   Timeout, so stop Sender  **********************************");
    }
  }

  // --- SECOND BLOCK: Executes every 1 second ---
  if (millis() >= nextTimeSec) {          // Do it every second
    nextTimeSec = millis() + 1000;

    // Update work time and current signal code on the display
    #ifdef Screen
      u8x8.setCursor(9, 4);
      u8x8.print("       ");
      u8x8.setCursor(10, 4);
      u8x8.print(workTimeMins);
    
      //line 7: Area
      u8x8.setCursor(10, 7);
      u8x8.print("      ");
      u8x8.setCursor(10, 7);
      u8x8.print(sigCodeInUse);
    #endif
    #ifdef SerialOutput
      Serial.print("Area:");
      Serial.println(sigCodeInUse);
    #endif

    // Read current and voltage from the charging sensor
    ChargeBusVoltage = InaCharge.getBusVoltage_V();
    ChargeShuntVoltage = InaCharge.getShuntVoltage_mV();
    ChargeCurrent = InaCharge.getCurrent_mA();

    // Control the Status LED: Red if charging, Green if standby/ready
    if (ChargeCurrent > ChargeThreshold) {  //Just for charge LED
      digitalWrite(pinGreenLED, LOW);
      digitalWrite(pinRedLED, HIGH);  // Charging
    } else  {
      digitalWrite(pinRedLED, LOW);
      digitalWrite(pinGreenLED, HIGH);  // Ready
    }

    // Prepare the current value for display (ignore noise below threshold)
    ChargeCurrentPrint = ChargeCurrent;
    if (ChargeCurrent < ChargeThreshold) ChargeCurrentPrint = 0;  // shows 0 when the mower is not charging

    #ifdef Screen
      u8x8.setCursor(10, 6);
      u8x8.print("      ");
      u8x8.setCursor(10, 6);
      u8x8.print(ChargeCurrentPrint);
    #endif
    #ifdef SerialOutput
      Serial.print("Charcurr: ");
      Serial.println(ChargeCurrentPrint);
    Serial.print("ChargeVolt: ");
    Serial.println(ChargeBusVoltage);
    #endif

    // If a higher current is detected, the mower is in the station
    if (ChargeCurrent > PeriOnOffThreshold && AUTO_START_SIGNAL == 1) {   // mower is into the station, in my test 410 ma are drained so possible to stop sender - When ist fully loaded the current is about 4mA.
                                                // So keep keep the value small to avoid an activation from the perimeterwire before the mower starts.
      enableSenderA = false;  // Stop the wire signal (mower is home)
      enableSenderB = false;

      // If the mower was previously working, it has now returned
      if(mowerIsWorking == 1)  {
        mowerIsWorking = 0;
        WORKING_TIMEOUT = 0;
        sendWhatsappMessage("Mower is back at Home!");          // Code for WhatsApp
      }

      workTimeMins = 0; // Reset the work timer
      // Turn off all driver outputs
      digitalWrite(pinEnableA, LOW);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      digitalWrite(pinEnableB, LOW);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);
      //delay(200);
    } else {

      if (AUTO_START_SIGNAL == 1 && !WORKING_TIMEOUT ) {
         // If the mower just left, mark it as working
         if(mowerIsWorking == 0)  {
          mowerIsWorking = 1;
          sendWhatsappMessage("Mower is going to work!");          // Send a WhatsApp message
        }
        // Enable the wire signal so the mower can see the boundary
        if (!enableSenderB) {
          enableSenderA = true;
          digitalWrite(pinEnableA, HIGH);
          digitalWrite(pinIN1, LOW);
          digitalWrite(pinIN2, LOW);
        } else {
          enableSenderB = true;
          digitalWrite(pinEnableB, HIGH);
          digitalWrite(pinIN3, LOW);
          digitalWrite(pinIN4, LOW);
        }
      }
    }
  
    // Increment the seconds counter
    timeSeconds++;
    if (((enableSenderA) || (enableSenderB)) && (timeSeconds >= 60)) {                    // If Sender is ON & 60 seconds are left
      if (workTimeMins < 1440)  {                                                         // avoid overflow
        workTimeMins++;                                                                   // count up a minute
        timeSeconds = 0;                                                                  // set seconds back to 0
        if (workTimeChargeMins > 0) {                                                     // If workTimeCharge > 0 (last state was CHARGING)
          lastChargeMins = workTimeChargeMins;                                            // save this Time in lastChargeMins
          workTimeChargeMins = 0;                                                         // and set it back to 0
        }
      }
    } else  {
      if ((workTimeChargeMins < 1440) && (ChargeCurrent > ChargeThreshold) && (timeSeconds >= 60)) {   // If Chargecurrent bigger than ChargeThreshold (Mower is in Station and Charge)
      workTimeMins = 0;                                                                   // WorktimeMins reset
        workTimeChargeMins++;                                                             // count up the workTimeChargeMins
        timeSeconds = 0;                                                                  // seconds reset
      }
    }
    
    // Display the status of the senders (which loop is active)
    if ((enableSenderA) || (enableSenderB)) {

      #ifdef Screen
        u8x8.setCursor(0, 2);
        u8x8.print("Sender ON :     ");
      #endif
      #ifdef SerialOutput
        Serial.print("Sender ON : ");
      #endif

      if (enableSenderA && !enableSenderB) {
        #ifdef Screen
          u8x8.setCursor(10, 2);
          u8x8.print("A");
        #endif
        #ifdef SerialOutput
          Serial.print("A");
        #endif
      }
      if (enableSenderB && !enableSenderA) {
        #ifdef Screen
          u8x8.setCursor(10, 2);
          u8x8.print("B");
        #endif
        #ifdef SerialOutput
          Serial.print("B");
        #endif
      }
      if (enableSenderA && enableSenderB) {
        #ifdef Screen
          u8x8.setCursor(10, 2);
          u8x8.print("AB");
        #endif
        #ifdef SerialOutput
          Serial.print("AB");
        #endif
      }
    } else {
      workTimeMins = 0;
      #ifdef Screen
        u8x8.setCursor(0, 2);
        u8x8.print("Sender OFF      ");
      #endif
      #ifdef SerialOutput
        Serial.print("Sender OFF");
      #endif

    }
    #ifdef SerialOutput
      Serial.println("");
    #endif
  }

  // --- WEB SERVER BLOCK: Handles browser requests ---
  WiFiClient client = server.available(); // Check if someone opened the IP in a browser
  if (client) {
    unsigned long currentTime = millis();
    unsigned long previousTime = currentTime;
    unsigned long timeoutTime = 500;  // Max time to wait for data
    String req; // Stores the request string
    String currentLine;

    // While the client is connected and hasn't timed out
    while (client.connected() && currentTime - previousTime <= timeoutTime) {
      currentTime = millis();
      if (client.available()) {             // if there are bytes to read from the client,
        char c = client.read();             // read a byte, then
        //Serial.write(c);                  // print it out the serial monitor
        req += c;                           // and add it to our request string
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html; charset=utf-8"); // Important!
            client.println("Connection: close");
            client.println(); // Blank line marks the end of the header

            // Keeps track of whether one of the following cases (e.g. /save or
            // /download-settings) already sent its own, complete response.
            // Without this flag, index.html/settings.html would ALWAYS get
            // appended at the very end as well - which, e.g. after saving,
            // resulted in two complete HTML pages in one response, which the
            // browser can't render cleanly (blank/white page)!
            bool responseAlreadySent = false;

            // Evaluate which page/action was requested
            if (req.indexOf("GET /toggleA") != -1) {
              // Logic to toggle sender A on/off
              if (enableSenderA){
                enableSenderA = false;
                workTimeMins = 0;
                digitalWrite(pinEnableA, LOW);
                digitalWrite(pinIN1, LOW);
                digitalWrite(pinIN2, LOW); 
              }
              else if(!enableSenderA){
                enableSenderA = true;
                workTimeMins = 0;
                digitalWrite(pinEnableA, HIGH);
                digitalWrite(pinIN1, LOW);
                digitalWrite(pinIN2, LOW);
              }

            }

            else if (req.indexOf("GET /toggleB") != -1) {
              // Logic to toggle sender B on/off
              if (enableSenderB){
                enableSenderB = false;
                workTimeMins = 0;
                digitalWrite(pinEnableB, LOW);
                digitalWrite(pinIN1, LOW);
                digitalWrite(pinIN2, LOW); 
              }
              else if(!enableSenderB){
                enableSenderB = true;
                workTimeMins = 0;
                digitalWrite(pinEnableB, HIGH);
                digitalWrite(pinIN1, LOW);
                digitalWrite(pinIN2, LOW);
              }
            }

            else if (req.indexOf("GET /toggleauto") != -1) {
              if(AUTO_START_SIGNAL == 1){
                AUTO_START_SIGNAL = 0;
              } else {
                AUTO_START_SIGNAL = 1;
              }
            }

            else if (req.indexOf("GET /changeSig") != -1) {
              sigCodeInUse++;
              if (sigCodeInUse > 4) sigCodeInUse = 0;
              saveSettingsToLittleFS(); // Save the settings!
              changeArea(sigCodeInUse);
            }

            else if (req.indexOf("GET /reset ") != -1) {
              factoryReset();
            }
            
            else if (req.indexOf("GET /save?") != -1) {
              // Read the values submitted through the settings form
              String pVal = extractParam(req, "peri");
              if (pVal != "") PeriOnOffThreshold = pVal.toFloat();
              
              String s0 = extractParam(req, "s0"); if (s0 != "") parseSigcode(s0, sigcode0, sigcode0_size);
              String s1 = extractParam(req, "s1"); if (s1 != "") parseSigcode(s1, sigcode1, sigcode1_size);
              String s2 = extractParam(req, "s2"); if (s2 != "") parseSigcode(s2, sigcode2, sigcode2_size);
              String s3 = extractParam(req, "s3"); if (s3 != "") parseSigcode(s3, sigcode3, sigcode3_size);
              String s4 = extractParam(req, "s4"); if (s4 != "") parseSigcode(s4, sigcode4, sigcode4_size);

              // --- Apply MQTT settings (only if MQTT is enabled) ---
              #ifdef MQTT
                bool mqttSettingsChanged = false;
                String newMqttServer = extractParam(req, "mqttServer");
                if (newMqttServer != "" && newMqttServer != mqtt_server) { mqtt_server = newMqttServer; mqttSettingsChanged = true; }

                String newMqttPortStr = extractParam(req, "mqttPort");
                if (newMqttPortStr != "") {
                  int newMqttPort = newMqttPortStr.toInt();
                  if (newMqttPort > 0 && newMqttPort != mqtt_port) { mqtt_port = newMqttPort; mqttSettingsChanged = true; }
                }

                String newClientID = extractParam(req, "mqttClientID");
                if (newClientID != "") clientID = newClientID;

                String newMqttUser = extractParam(req, "mqttUser");
                if (newMqttUser != "" && newMqttUser != MQTT_user) { MQTT_user = newMqttUser; mqttSettingsChanged = true; }

                String newMqttPassword = extractParam(req, "mqttPassword");
                if (newMqttPassword != "" && newMqttPassword != MQTT_password) { MQTT_password = newMqttPassword; mqttSettingsChanged = true; }

                String newSubTopic = extractParam(req, "mqttSubTopic");
                if (newSubTopic != "" && newSubTopic != subTopic) {
                  subTopic = newSubTopic;
                  // If the subscribe topic changed, it needs to be
                  // re-subscribed on the next connection - that happens
                  // automatically in loop() as soon as MQTTclient.connected()
                  // returns false the next time.
                  mqttSettingsChanged = true;
                }

                String newPubTopic = extractParam(req, "mqttPubTopic");
                if (newPubTopic != "") pubTopic = newPubTopic;

                String newPubTopic2 = extractParam(req, "mqttPubTopic2");
                if (newPubTopic2 != "") pubTopic2 = newPubTopic2;

                String newPubTopic3 = extractParam(req, "mqttPubTopic3");
                if (newPubTopic3 != "") pubTopic3 = newPubTopic3;

                // If server/port/credentials/topic changed, disconnect the
                // existing connection - loop() will then automatically
                // reconnect with the new values and re-subscribe to the
                // (possibly new) subTopic.
                if (mqttSettingsChanged) {
                  MQTTclient.disconnect();
                  MQTTclient.setServer(mqtt_server.c_str(), mqtt_port);
                }
              #endif

              // --- Apply WhatsApp settings (only if WhatsApp is enabled) ---
              #ifdef WhatsApp_messages
                String newWaPhone = extractParam(req, "waPhone");
                if (newWaPhone != "") mobile_number = newWaPhone;

                String newWaApiKey = extractParam(req, "waApiKey");
                if (newWaApiKey != "") api_key = newWaApiKey;
              #endif

              saveSettingsToLittleFS();
              changeArea(sigCodeInUse); // Immediately apply the current code with the new values
              
              // Redirect the browser back to the settings page - with
              // "?saved=1" in the URL, so the settings page knows it should
              // show the "Settings saved!" confirmation notice.
              client.println("<html><head><meta http-equiv='refresh' content='0; url=/settings?saved=1'></head><body>Saving...</body></html>");
              responseAlreadySent = true;   // Prevents index.html from also being appended at the end
            }
            
            else if (req.indexOf("GET /changeSpeed") != -1) {
              if (sigDuration == 104) {
                sigDuration = 50;
                timerAlarm(timer, sigDuration, true, 0);
              } else {
                sigDuration = 104;
                timerAlarm(timer, sigDuration, true, 0);
              }
            }

            else if (req.indexOf("GET /download-settings") != -1) {
              if (LittleFS.exists("/config.json")) {
                File file = LittleFS.open("/config.json", "r");
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: application/octet-stream"); // Forces a file download instead of showing it
                client.println("Content-Disposition: attachment; filename=\"config.json\"");
                client.println("Connection: close");
                client.println();
        
                while (file.available()) {
                  client.write(file.read());
                }
                file.close();
              } else {
                client.println("HTTP/1.1 404 Not Found\n\nFile not found.");
              }
              responseAlreadySent = true;   // Prevents index.html from also being appended at the end
            }

            // READ AND SEND THE REQUESTED HTML FILE
            // Only run this if no response was already sent above (e.g. by
            // /save or /download-settings) - see responseAlreadySent further up.
            if (!responseAlreadySent) {
            String filename = "/index.html";
            if (req.indexOf("GET /settings") != -1) filename = "/settings.html";

            // Was this the redirect after saving (URL contains "saved=1"),
            // the confirmation notice is shown on the settings page.
            bool justSaved = (req.indexOf("saved=1") != -1);

            if (LittleFS.exists(filename)) {
              File file = LittleFS.open(filename, "r");
              String html = file.readString();
              file.close();
              client.print(processor(html, justSaved)); // This is where the %placeholders% get replaced with real values!
            } else {
              client.println("File not found!");
            }
            }   // END if (!responseAlreadySent)
            break;   // This request is fully handled - the connection is about to close (regardless of which case above applied)
          } else {
            // The line was NOT blank - so it's a normal header line
            // (e.g. "Host: ..." or "User-Agent: ..."). Reset for the next
            // line, so that currentLine.length()==0 really only matches the
            // actual blank line (end of all headers).
            currentLine = "";
          }
        } else if (c != '\r') {
          // A normal character (not a newline, not a carriage return) -
          // append it to the line currently being read.
          currentLine += c;
        }
      }
      // Clear the buffer and close connection
      //client.flush();
    }
  }  
}
//END LOOP
