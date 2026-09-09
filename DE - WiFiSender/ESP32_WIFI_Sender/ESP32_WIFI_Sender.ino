// ============================================================================
//  ERSTEINRICHTUNG / INSTALLATION - bitte unbedingt zuerst lesen!
// ============================================================================
//  1) Board-Einstellungen in der Arduino-IDE (Menü "Werkzeuge"/"Tools"):
//     - Board: ein passendes ESP32-Board (z.B. "ESP32 Dev Module")
//     - Partition Scheme / Partitionsschema:
//       "Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)"
//       Das ist WICHTIG und nicht die Standardeinstellung! Dieses Schema
//       reserviert Platz für ZWEI Firmware-Kopien (nötig für OTA-Updates,
//       siehe unten) UND einen Datei-Bereich für LittleFS (in dem
//       index.html, settings.html und config.json liegen). Mit dem
//       Standard-Partitionsschema ist entweder kein Platz für LittleFS
//       oder OTA funktioniert nicht richtig.
//
//  2) Der ALLERERSTE Upload muss per USB-Kabel erfolgen, nicht über OTA/WLAN!
//     Das hat zwei Gründe:
//     a) OTA (drahtlose Updates) ist eine Funktion, die erst durch DIESEN
//        Sketch selbst auf dem ESP32 läuft. Ein komplett jungfräulicher
//        ESP32 kennt OTA noch gar nicht - die Firmware muss also einmal
//        "klassisch" per Kabel drauf, bevor spätere Updates drahtlos gehen.
//     b) Die Webseiten (index.html, settings.html) werden NICHT mit
//        hochgeladen, wenn man in der Arduino-IDE auf "Hochladen" klickt -
//        das lädt nur diese .ino-Datei. Die HTML-Dateien liegen im
//        Unterordner "data" und müssen separat über das LittleFS-Upload-
//        Plugin übertragen werden (Tastenkombination STRG+SHIFT+P, siehe
//        unten) - und dieses Plugin funktioniert nur über die USB-
//        Kabelverbindung, nicht über WLAN/OTA.
//
//     Ablauf beim ersten Mal also:
//       1. ESP32 per USB anschließen
//       2. Board + Partitionsschema wie oben einstellen
//       3. Sketch ganz normal per USB hochladen (Pfeil-Symbol/"Hochladen")
//       4. Direkt danach: STRG+SHIFT+P drücken, um index.html, settings.html
//          (und optional config.json) per LittleFS-Upload auf den ESP32
//          zu übertragen (ebenfalls per USB)
//     Erst NACH diesem einmaligen USB-Setup kann die Firmware (der Sketch
//     selbst) über OTA drahtlos aktualisiert werden. Änderungen an den
//     HTML-Dateien im "data"-Ordner müssen weiterhin per USB und dem
//     LittleFS-Upload-Plugin übertragen werden.
// ============================================================================
//
// ============================================================================
//  DIYRobotLawnMower Headquarter 1.0 - 07.05.26
//  Dieses Programm läuft auf einem ESP32 und steuert den "Perimeterdraht-
//  Sender" eines selbstgebauten Rasenmäher-Roboters. Es sendet einen sich
//  wiederholenden elektrischen Signalcode durch einen im Rasen vergrabenen
//  Draht; der eigene Sensor des Mähers erkennt dieses Signal und weiß so,
//  wo die Grenze des Rasens verläuft, um innerhalb zu bleiben.
//
//  Zusätzlich macht dieser Sketch noch:
//   - Er startet einen kleinen Webserver, damit du alles über den Browser
//     steuern kannst
//   - Er sendet WhatsApp-Benachrichtigungen (Mäher gestartet/zurück/Timeout)
//   - Er veröffentlicht Sensorwerte über MQTT (für Hausautomatisierungssysteme)
//   - Er zeigt Statusinformationen auf einem kleinen OLED-Bildschirm an
//   - Er unterstützt "OTA"-Firmware-Updates (Over-The-Air, also drahtlos)
//
//  Hinweis für Anfänger: Diese Datei ist ein einzelner großer ".ino"-Sketch.
//  Arduino/ESP32-Sketches brauchen immer genau zwei besondere Funktionen:
//    - setup()  -> läuft EINMAL, wenn der ESP32 startet, für Initialisierungen
//    - loop()   -> läuft danach IMMER WIEDER, endlos, nachdem setup() fertig ist
//  Alle anderen Funktionen in dieser Datei (weiter unten) sind nur Hilfs-
//  funktionen, die setup() oder loop() (oder der Webserver, oder MQTT, ...)
//  bei Bedarf aufrufen.
// ============================================================================
//
// STRG + SHIFT + P --> Upload LittleFS to Pico/ESP8266/ESP32 for DATA upload
//  --> Plugin: https://github.com/earlephilhower/arduino-littlefs-upload
//  (LittleFS ist ein winziges Dateisystem, das im Flash-Speicher des ESP32
//   liegt. Die Webseiten (index.html, settings.html) und die gespeicherten
//   Einstellungen (config.json) liegen dort - NICHT in dieser .ino-Datei.)
//
// Entwicklerhinweise (aus der Originalversion übernommen):
// - MQTT funktioniert zwar, lief anfangs aber nicht gut (zu viele Anfragen).
//   Der Aufruf client.flush() um Zeile ~1200 wurde deaktiviert, das scheint
//   zu helfen.
// - Gelöst: WhatsApp verursachte früher einen HTTP-429-Fehler ("Too Many
//   Requests"), und im Log sah es verstümmelt aus ("...successfullyError
//   sending the message"). Das lag einfach an einem fehlenden Zeilenumbruch
//   in einem Serial.print()-Aufruf, ist jetzt behoben.
// - TODO: eine frei einstellbare Sendefrequenz ist noch nicht umgesetzt.


//********************* Includes **********************************
// "#include" bindet Code aus externen Bibliotheken ein, damit wir nicht
// alles selbst schreiben müssen. Die meisten davon müssen vorher über den
// Arduino/PlatformIO-Bibliotheksverwalter installiert werden.
#define CONFIG_LITTLEFS_FOR_IDF_3_2
#include <LittleFS.h>                  // Kleines Dateisystem im Flash-Speicher (speichert Webseiten + Einstellungen)
#include <FS.h>                        // Allgemeine Dateisystem-Unterstützung, wird von LittleFS benötigt
#include <ArduinoJson.h>               // Liest und schreibt JSON-Text (wird für config.json genutzt) - Bibliothek von Benoit
#include "soc/gpio_struct.h"           // Ermöglicht direkten, schnellen Zugriff auf die GPIO-Pins, damit der Interrupt (ISR) so schnell wie möglich läuft
#include "freertos/FreeRTOS.h"         // Das im ESP32 eingebaute Mini-Betriebssystem, das mehrere Aufgaben gleichzeitig ausführen kann
#include "freertos/task.h"             // Wird gebraucht, um Hintergrund-Tasks zu erstellen und zu verwalten (siehe whatsappTask() weiter unten)
#include "freertos/queue.h"            // Wird für die WhatsApp-Nachrichten-Warteschlange gebraucht (siehe "Code for Whatsapp" weiter unten)
#include <PubSubClient.h>              // MQTT-Client-Bibliothek - von Nick O'Leary
#include <WiFiManager.h>               // Übernimmt die WLAN-Einrichtung über ein Konfigurationsportal, damit man WLAN-Passwörter nicht fest in den Code schreiben muss - von tzapu
WiFiManager wm;                        // Das einzige WiFiManager-Objekt, das im ganzen Sketch verwendet wird
#include <WiFiClient.h>                // Grundlegende TCP/IP-Netzwerkunterstützung
#include <HTTPClient.h>                // Ermöglicht dem ESP32, HTTP(S)-Anfragen zu senden (wird für den Aufruf der WhatsApp-API genutzt)
#include <UrlEncode.h>                 // Macht Text sicher für die Verwendung in einer URL (Leerzeichen, Sonderzeichen, ...) - von Masayuki
#include <INA226_WE.h>                 // Treiber für den INA226-Strom-/Spannungssensor-Chip - von Wolfgang Ewald
#include <ArduinoOTA.h>                // Ermöglicht das Hochladen neuer Firmware über WLAN statt per USB-Kabel - von Juraj Andrassy

#include <U8x8lib.h>                   // Treiber für kleine monochrome OLED-Displays - von Oliver Kraus
//********************* Display-Einstellungen **********************************
// Bitte genau EINE der Konstruktorzeilen unten aktivieren (Kommentarzeichen
// entfernen) - hängt davon ab, welchen genauen OLED-Display-Chip du hast
// (SSD1306, SH1106, ...).
// Die komplette Liste gibt es hier: https://github.com/olikraus/u8g2/wiki/u8x8setupcpp
// Bitte die Pin-Nummern an deinen Aufbau anpassen. U8X8_PIN_NONE verwenden, wenn der Reset-Pin nicht angeschlossen ist
 //U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 	      
//U8X8_SSD1306_128X64_ALT0_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 	      // wie die NONAME-Variante, kann aber das "jede 2. Zeile fehlt"-Problem lösen
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);           // <-- dieser Displaytyp wird tatsächlich verwendet
// Ende der Konstruktorliste


//********************* Definitionen (defines) **********************************
// "#define" erstellt eine Konstante, die vor dem Kompilieren überall im Code
// durch ihren Wert ersetzt wird. Sie ändert sich nie während das Programm läuft.
// Eine Zeile auskommentieren (mit "//" davor), wenn diese Funktion nicht
// gebraucht wird - die umliegenden "#ifdef ... #endif"-Blöcke weiter unten
// überspringen dann den betreffenden Codeabschnitt komplett, was das
// Programm außerdem kleiner macht.
#define OTAUpdates 1                  // OTA-Updates (drahtlose Firmware-Updates) aktivieren
#define MQTT 1                        // Senden von Sensordaten an einen MQTT-Broker aktivieren
#define WhatsApp_messages 1           // WhatsApp-Benachrichtigungen aktivieren (Mäher gestartet/zurück/Timeout). Braucht Einrichtung, siehe unten:
                                      // 1) Erstelle den Whatabot-Kontakt auf deinem Smartphone. Die Telefonnummer ist: +54 9 2364205798
                                      // 2) Sende: "I allow whatabot to send me messages"
                                      // 3) Kopiere die Telefonnummer und den API-Key, die dir Whatabot geschickt hat.
                                      // Trage beides nach dem Hochladen des Sketches auf der Einstellungsseite im Browser (/settings) ein - dort werden sie in config.json gespeichert.

// Auf der Platine ist ein Anschluss für eine 2-farbige LED mit gemeinsamer
// Kathode(-). (Achtung: Das Matrix Mow800-Board hat stattdessen eine LED mit
// gemeinsamer Anode(+)!!!)
#define pinGreenLED 25                                  // Leuchtet, wenn: Station bereit / Mäher geladen / lädt gerade nicht

// Der Akku lädt, wenn ChargeCurrent > LoadingThreshold
#define pinRedLED 26                                    // Leuchtet, wenn: Akku lädt gerade aktiv

#define VER "DIYRobotLawnMower Headquarter 1.0 - 07.05.2026"   // Firmware-Versionsangabe, wird beim Start ausgegeben und am Display gezeigt


bool AUTO_START_SIGNAL = 1;           // Wenn true: das Perimeterdraht-Signal startet/stoppt automatisch basierend auf dem Ladestrom (siehe loop())
//#define USE_BUTTON 0                // (aktuell nicht genutzt) Physischer Taster, um Mähen zu starten oder den Mäher zur Station zu schicken
#define SerialOutput 1              // Wenn definiert: Debug-Textmeldungen am Serial Monitor (USB) ausgeben. Auskommentieren, um das Log stummzuschalten.
bool debug = false;                    // Ein zweiter, separater Debug-Schalter, hauptsächlich für WiFiManager-bezogene Meldungen
#define Screen 1                      // Wenn definiert: das kleine OLED-Display verwenden. Auskommentieren, falls keins angeschlossen ist.

#define WORKING_TIMEOUT_MINS 300      // Sicherheits-Timeout (in Minuten): Ist der Mäher nach dieser Zeit nicht zurück in der Station, wird das Drahtsignal abgeschaltet.
                                       // HINWEIS: diese Sicherheitsfunktion wirkt nur, solange AUTO_START_SIGNAL AUS ist - ist AUTO_START_SIGNAL aktiv, wird dieser Wert ignoriert!
#define PERI_CURRENT_MIN 200          // Minimal erwarteter Strom (in Milliampere) auf dem Perimeterdraht. Darunter wird angenommen, der Draht ist durchtrennt/defekt.

// --- Pin-Belegung (welcher physische ESP32-Pin macht was) ---
#define I2C_SDA 21                    // I2C-Datenleitung, wird von beiden INA226-Sensoren und dem OLED-Display gemeinsam genutzt
#define I2C_SCL 22                    // I2C-Taktleitung, wird von beiden INA226-Sensoren und dem OLED-Display gemeinsam genutzt
#define pinIN1 12                     // M1_IN1  ESP32 GPIO12       ( diesen Pin mit L298N-IN1 verbinden)  - steuert die H-Brücke von Sender A
#define pinIN2 13                     // M1_IN2  ESP32 GPIO13       ( diesen Pin mit L298N-IN2 verbinden)  - steuert die H-Brücke von Sender A
#define pinEnableA 23                 // ENA    ESP32 GPIO23         (diesen Pin mit L298N-ENA verbinden)  - schaltet die H-Brücke von Sender A ein/aus
#define pinIN3 14                     // M1_IN3  ESP32 GPIO14       ( diesen Pin mit L298N-IN3 verbinden)  - steuert die H-Brücke von Sender B
#define pinIN4 18                     // M1_IN4  ESP32 GPIO18       ( diesen Pin mit L298N-IN4 verbinden)  - steuert die H-Brücke von Sender B
#define pinEnableB 19                 // ENB    ESP32 GPIO19        (diesen Pin mit L298N-ENA verbinden)   - schaltet die H-Brücke von Sender B ein/aus
//#define pinDoorOpen 34              // Nicht in Verwendung (Magnetschalter)
//#define pinDoorClose 35             // Nicht in Verwendung (Magnetschalter)
//#define pinLDR 32                   // Nicht in Verwendung (Lichtsensor)

// Hinweis für Anfänger: Eine "H-Brücke" (hier: ein L298N-Treibermodul) ist
// eine kleine Schaltung, die den Strom im Perimeterdraht unter Kontrolle des
// ESP32 in beide Richtungen (vorwärts/rückwärts) fließen lassen kann - genau
// das wird gebraucht, um das abwechselnde +1/-1-Signalmuster zu erzeugen.

WiFiClient espClient;                 // Das zugrundeliegende Netzwerkverbindungsobjekt, wird vom MQTT-Client mitbenutzt

//********************* WhatsApp Zugangsdaten **********************************
// Startwerte sind bewusst leer. Sie werden über die Einstellungsseite im
// Browser eingetragen und dann in config.json gespeichert (siehe processor(),
// saveSettingsToLittleFS() und den "/save"-Bereich weiter unten).
// Wichtig: Diese Variablen müssen schon HIER, ganz am Anfang, deklariert sein -
// nicht erst weiter unten bei "Code for Whatsapp" - weil processor() und
// saveSettingsToLittleFS() (die beide weiter oben im Code stehen) sie bereits
// verwenden. In C++/Arduino muss eine Variable VOR ihrer ersten Verwendung
// deklariert sein, sonst gibt es beim Kompilieren den Fehler
// "was not declared in this scope".
String mobile_number = "";            // Deine Telefonnummer (z.B. 4917012345678), wird nur bei aktiviertem WhatsApp_messages benutzt
String api_key = "";                  // Dein Whatabot-API-Key, wird nur bei aktiviertem WhatsApp_messages benutzt

//********************* MQTT-Einstellungen **********************************
// MQTT ist ein leichtgewichtiges Nachrichtenprotokoll, das oft in der
// Hausautomatisierung (z.B. mit Home Assistant oder Node-RED) verwendet
// wird, um Sensorwerte zu veröffentlichen.
// Wichtig: Diese Werte sind jetzt KEINE festen Konstanten (const) mehr,
// sondern normale Variablen (String/int) - dadurch können sie über die
// Einstellungsseite im Browser geändert und dauerhaft in config.json
// gespeichert werden (siehe saveSettingsToLittleFS() und den "/save"-Bereich
// weiter unten). Die Werte hier sind nur die Startwerte (falls noch keine
// eigenen Einstellungen gespeichert wurden).
#ifdef MQTT
String mqtt_server = "192.168.178.2";       // Adresse deines MQTT-Brokers, z.B. "broker.hivemq.com" oder eine lokale IP-Adresse
int mqtt_port = 1883;                       // Standard-Port für unverschlüsseltes MQTT
String clientID = "ESP32Sender";            // Ein eindeutiger Name, mit dem sich dieses Gerät beim Broker identifiziert
String MQTT_user = "mqtt";                  // Startwert (Platzhalter) - wird über die Einstellungsseite geändert und dann in config.json gespeichert
String MQTT_password = "mqtt";              // Startwert (Platzhalter) - wird über die Einstellungsseite geändert und dann in config.json gespeichert
String subTopic = "teensysender/input";              // MQTT-"Topic" (Kanal), auf dem dieses Gerät auf eingehende Befehle lauscht
String pubTopic = "teensysender/chargecurrent";      // Topic, über das der Ladestrom veröffentlicht wird
String pubTopic2 = "teensysender/pericurrent";       // Topic, über das der Perimeterdraht-Strom veröffentlicht wird
String pubTopic3 = "teensysender/chargevoltage";     // Topic, über das die Ladespannung veröffentlicht wird
PubSubClient MQTTclient(espClient);         // Das eigentliche MQTT-Client-Objekt, aufbauend auf der WLAN-Verbindung
long lastMsg = 0;                           // (aktuell ungenutzte Restvariable)
#endif

//********************* WLAN-Einstellungen **********************************
WiFiServer server(80);                       // Ein einfacher Webserver, der auf Port 80 lauscht (der Standard-HTTP-Port)
unsigned long lastWiFiCheckMillis = 0;       // Merkt sich, wann die WLAN-Verbindung zuletzt geprüft wurde (siehe keepWiFiAlive())
const unsigned long wifiCheckInterval = 15000; // Wie oft die WLAN-Verbindung geprüft wird, in Millisekunden (15 Sekunden)

//********************* INA226-Einstellungen **********************************
// Der INA226 ist ein kleiner Sensor-Chip, der Spannung und Strom sehr genau
// misst. Dieses Projekt verwendet ZWEI davon: einen zur Überwachung des
// Perimeterdrahts, einen zur Überwachung des Ladestroms der Batterie.
INA226_WE InaPeri = INA226_WE(0x40);                    // 0x40 = die I2C-Adresse des Sensors ohne gelötete Brücke
float resistorPeri = 0.1;                               // Wert (in Ohm) des "Shunt"-Widerstands für die Strommessung. Bei 10mOhm z.B. 0.02, bei 100mOhm 0.1 verwenden.
float rangePeri = 0.8;                                  // Erwarteter Messbereich in Ampere. Bei einem 10mOhm-Widerstand z.B. 8.0 oder 4.0 - bei 100mOhm 0.8 verwenden.

INA226_WE InaCharge = INA226_WE(0x44);                  // 0x44 = eine andere I2C-Adresse, eingestellt durch Löten der "Brücke" zwischen den Pads A1 und VSS
float resistorCharge = 0.02;
float rangeCharge = 4.0;


//********************* Sonstiges **********************************
// Globale Variablen: Diese Werte gelten für das ganze Programm und können
// sich während des Betriebs ändern (im Gegensatz zu den "#define"-Konstanten oben).
bool WORKING_TIMEOUT = 0;               // Wird auf 1 gesetzt, sobald der Sicherheits-Timeout (WORKING_TIMEOUT_MINS) ausgelöst wurde
bool mowerIsWorking = 0;                // Merkt sich, ob der Mäher gerade vermutlich beim Mähen ist (wird für WhatsApp-Nachrichten genutzt)
byte sigCodeInUse = 1;                  // Welcher Signalcode (0-4) gerade aktiv ist. 1 ist der ursprüngliche Ardumower-Sigcode.
int sigDuration = 104;                  // Wie viele Mikrosekunden jedes Signal-"Bit" dauert (üblich sind auch 50). Kleiner = schnelleres Signal.
int8_t sigcode_norm[128];               // Der Signalcode, der gerade von der ISR gesendet wird (wird aus sigcode0..sigcode4 hierher kopiert)
int sigcode_size;                       // Wie viele Einträge von sigcode_norm[] tatsächlich verwendet werden
hw_timer_t* timer = NULL;               // Handle für den Hardware-Timer des ESP32, der die Signalausgabe steuert
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;  // Eine kleine Sperre, die den Interrupt davor schützt, selbst unterbrochen zu werden
volatile int stepA = 0;                 // Aktuelle Position von Sender A innerhalb des Signalcodes (eigener Zähler, unabhängig von B).
                                         // "volatile" sagt dem Compiler, dass sich dieser Wert jederzeit aus einem Interrupt heraus ändern kann.
volatile int stepB = 0;                 // Aktuelle Position von Sender B innerhalb des Signalcodes (eigener Zähler, unabhängig von A)
boolean enableSenderA = false;          // Sendet Sender A (Drahtschleife A) gerade? Startet AUS, damit das System sicher hochfährt.
boolean enableSenderB = false;          // Sendet Sender B (Drahtschleife B) gerade? Startet AUS, damit das System sicher hochfährt.
int timeSeconds = 0;                    // Kleiner Sekundenzähler, um daraus ganze Minuten für die Arbeitszeit-Zählung zu bilden
unsigned long nextTimeControl = 0;      // Zeitstempel (millis()) für den nächsten "alle 10 Sekunden"-Block in loop()
unsigned long nextTimeSec = 0;          // Zeitstempel (millis()) für den nächsten "jede Sekunde"-Block in loop()
int workTimeMins = 0;                   // Wie viele Minuten der Mäher am Mähen ist, seit er zuletzt die Station verlassen hat
int workTimeChargeMins = 0;             // Wie viele Minuten der Mäher in der aktuellen Ladesitzung schon lädt
int lastChargeMins = 0;                 // Wie viele Minuten der Mäher bei der letzten Ladesitzung geladen hat
float PeriCurrent = 0.0;                // Am Perimeterdraht gemessener Strom, in mA
float PeriBusVoltage = 0.0;             // Am Perimeterdraht gemessene Spannung, in V
float PeriShuntVoltage = 0.0;           // Winziger Spannungsabfall über dem Perimeter-Messwiderstand, in mV
float ChargeCurrent = 0.0;              // An den Ladekontakten gemessener Strom, in mA
float ChargeCurrentPrint = 0.0;         // Wie ChargeCurrent, aber auf 0 abgerundet, wenn unter ChargeThreshold (schöner für die Anzeige)
float ChargeBusVoltage = 0.0;           // An den Ladekontakten gemessene Spannung, in V
float ChargeShuntVoltage = 0.0;         // Winziger Spannungsabfall über dem Lade-Messwiderstand, in mV
bool shouldSaveConfig = false;          // Flag, das von WiFiManager gesetzt wird, wenn neue WLAN-Einstellungen gespeichert werden müssen
bool wm_nonblocking = false;            // Wenn true, würde das Konfigurationsportal von WiFiManager loop() nicht blockieren (aktuell ungenutzt - das Portal blockiert immer)
//String AutoStartSignalPrint;                            // (ungenutzte Restvariable, zur Referenz belassen) für den Web-Teil
//String linktext;                                        // (ungenutzte Restvariable, zur Referenz belassen) für den Web-Teil
//String enableSenderAprint;                              // (ungenutzte Restvariable, zur Referenz belassen) für den Web-Teil
//String enableSenderBprint;                              // (ungenutzte Restvariable, zur Referenz belassen) für den Web-Teil


/*
  Ist der Mäher in der Station und voll geladen, sollte der Strom zwischen
  PeriOnOffThreshold(3mA) und ChargeThreshold(10mA) liegen.

  Startet der Perimeter, während der Mäher in der Station ist, ChargeThreshold
  auf 0 setzen. So kann man den ursprünglichen ChargeCurrent-Wert unter
  http://Deine-IP sehen.
  Ist der Mäher draußen, sollte ChargeCurrent 0 sein.
*/
float ChargeThreshold = 10.0;               // in mA. Ist der ChargeCurrent unter diesem Wert, zeigt das Display "0mA" 
float PeriOnOffThreshold = 1.5;             // Ist ChargeCurrent unter diesem Wert, beginnt die Perimeterschleife zu arbeiten


//*********************  Sigcode-Liste *********************************************
// Hinweis für Anfänger: Diese Arrays sind die eigentliche "Sprache", die über
// den Perimeterdraht gesprochen wird. Jedes Array ist eine Folge von +1/-1-
// Werten; die ISR (onTimer(), weiter unten) sendet sie immer wieder nachein-
// ander aus und schaltet dabei bei jedem Wert die Polung des Drahtes um. Der
// Sensor am Mäher erkennt das Muster und kann so feststellen, zu WELCHEM
// Bereich (0-4) er gerade gehört - so sind Mehrzonen-Gärten möglich. Muss
// ein Vielfaches von 2 sein! Mehr Infos:
// http://grauonline.de/alexwww/ardumower/filter/filter.html
// Dies ist das "pseudonoise4_pw"-Signal (wie beim Sender verwendet).

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
// Alles oberhalb dieser Zeile sind Einstellungen, die man ggf. ändern möchte.
// Alles unterhalb dieser Zeile ist die eigentliche Programmlogik.


//********************* Funktionen **********************************
// Hinweis für Anfänger: Eine "Funktion" ist ein benannter, wiederverwendbarer
// Codeblock. Statt dieselben Anweisungen immer wieder zu schreiben, schreiben
// wir sie einmal in einer Funktion und "rufen" diese Funktion dann einfach
// bei Bedarf über ihren Namen auf.

// Ersetzt die Platzhalter (wie "%ChargeVoltage%") im HTML-Text einer Webseite
// durch die aktuellen, echten Sensor-/Statuswerte, kurz bevor die Seite an
// den Browser gesendet wird. So zeigen index.html und settings.html Live-Daten.
// Der Parameter "justSaved" steuert den %SAVE_NOTICE%-Platzhalter (siehe unten):
// wird er "true" übergeben, erscheint auf der Einstellungsseite ein kurzer
// Hinweis "Einstellungen gespeichert!". Der Parameter ist optional (Standard:
// false), damit bestehende Aufrufe von processor(html) ohne zweites Argument
// weiter funktionieren.
String processor(String html, bool justSaved = false) {
    // 1. Einfache Zahlen-/Textwerte: Platzhaltertext einfach suchen und ersetzen
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

    // 2. Dynamischer Zustand (Sender A) - bestimmt CSS-Klasse und AN/AUS-Text im Browser
    if (enableSenderA) { // Annahme: das ist die Statusprüfung
        html.replace("%State_Class_A%", "label on");   // CSS-Klassenname - NICHT übersetzen, passt zu td.on{...} in index.html!
        html.replace("%State_Text_A%", "an");
    } else {
        html.replace("%State_Class_A%", "label off");  // CSS-Klassenname - NICHT übersetzen, passt zu td.off{...} in index.html!
        html.replace("%State_Text_A%", "aus");
    }

    // 3. Dynamischer Zustand (Sender B)
    if (enableSenderB) {
        html.replace("%State_Class_B%", "label on");
        html.replace("%State_Text_B%", "an");
    } else {
        html.replace("%State_Class_B%", "label off");
        html.replace("%State_Text_B%", "aus");
    }

    // 4. Auto-Mode (basierend auf der Variable AUTO_START_SIGNAL)
    if (AUTO_START_SIGNAL) {
        html.replace("%State_Class_Auto%", "label on");
        html.replace("%State_Text_Auto%", "an");
    } else {
        html.replace("%State_Class_Auto%", "label off");
        html.replace("%State_Text_Auto%", "aus");
    }

    // 5. MQTT-Einstellungsblock: Nur einbauen, wenn MQTT überhaupt aktiviert ist
    // (#ifdef MQTT). Ist MQTT nicht aktiviert, wird der Platzhalter einfach durch
    // einen leeren Text ersetzt - auf der Seite erscheint dann gar kein MQTT-Bereich.
    #ifdef MQTT
      String mqttBlock = "";
      mqttBlock += "<h3 style='color:#1a3c34;margin-top:25px;'>MQTT-Einstellungen</h3>";
      mqttBlock += "<label><b>Broker-Adresse (IP oder Hostname):</b></label>";
      mqttBlock += "<input type='text' name='mqttServer' value='" + mqtt_server + "'><br />";
      mqttBlock += "<label><b>Port:</b></label>";
      mqttBlock += "<input type='text' name='mqttPort' value='" + String(mqtt_port) + "'><br />";
      mqttBlock += "<label><b>Client-ID:</b></label>";
      mqttBlock += "<input type='text' name='mqttClientID' value='" + clientID + "'><br />";
      mqttBlock += "<label><b>Benutzername:</b></label>";
      mqttBlock += "<input type='text' name='mqttUser' value='" + MQTT_user + "'><br />";
      mqttBlock += "<label><b>Passwort:</b></label>";
      mqttBlock += "<input type='password' name='mqttPassword' value='" + MQTT_password + "'><br />";
      mqttBlock += "<label><b>Empfangs-Topic (subTopic):</b></label>";
      mqttBlock += "<input type='text' name='mqttSubTopic' value='" + subTopic + "'><br />";
      mqttBlock += "<label><b>Sende-Topic Ladestrom:</b></label>";
      mqttBlock += "<input type='text' name='mqttPubTopic' value='" + pubTopic + "'><br />";
      mqttBlock += "<label><b>Sende-Topic Perimeterstrom:</b></label>";
      mqttBlock += "<input type='text' name='mqttPubTopic2' value='" + pubTopic2 + "'><br />";
      mqttBlock += "<label><b>Sende-Topic Ladespannung:</b></label>";
      mqttBlock += "<input type='text' name='mqttPubTopic3' value='" + pubTopic3 + "'><br />";
      html.replace("%MQTT_SETTINGS_BLOCK%", mqttBlock);
    #else
      html.replace("%MQTT_SETTINGS_BLOCK%", "");
    #endif

    // 6. WhatsApp-Einstellungsblock: Nur einbauen, wenn WhatsApp überhaupt
    // aktiviert ist (#ifdef WhatsApp_messages). Gleiches Prinzip wie oben bei MQTT.
    #ifdef WhatsApp_messages
      String waBlock = "";
      waBlock += "<h3 style='color:#1a3c34;margin-top:25px;'>WhatsApp-Einstellungen</h3>";
      waBlock += "<label><b>Telefonnummer (z.B. 4917012345678):</b></label>";
      waBlock += "<input type='text' name='waPhone' value='" + mobile_number + "'><br />";
      waBlock += "<label><b>Whatabot API-Key:</b></label>";
      waBlock += "<input type='password' name='waApiKey' value='" + api_key + "'><br />";
      html.replace("%WHATSAPP_SETTINGS_BLOCK%", waBlock);
    #else
      html.replace("%WHATSAPP_SETTINGS_BLOCK%", "");
    #endif

    // 7. Speicher-Hinweis: Nur anzeigen, wenn wir gerade per Weiterleitung von
    // "/save" hierher gekommen sind (siehe justSaved-Parameter oben). So sieht
    // der Nutzer klar und deutlich, dass seine Änderungen übernommen wurden -
    // statt der Seite einfach nur ohne Rückmeldung neu zu laden.
    if (justSaved) {
      html.replace("%SAVE_NOTICE%", "<div style='background:#c8e6c9;color:#1a3c34;padding:10px;border-radius:6px;margin-bottom:15px;text-align:center;'>✅ Einstellungen gespeichert!</div>");
    } else {
      html.replace("%SAVE_NOTICE%", "");
    }

    return html;
}


// --- Hilfsfunktionen für Signalcodes & URL-Parsing ---

// Wandelt ein Signalcode-Array (wie sigcode0) zurück in eine einfache,
// kommagetrennte Textzeichenkette (z.B. "1,-1,1,-1"), damit sie auf der
// Einstellungsseite angezeigt/bearbeitet werden kann.
String sigcodeToString(int8_t* arr, int size) {
  String res = "";
  for (int i = 0; i < size; i++) {
    res += String(arr[i]);
    if (i < size - 1) res += ",";
  }
  return res;
}

// Das Gegenteil von sigcodeToString(): liest eine kommagetrennte Zeichenkette
// (wie vom Nutzer auf der Einstellungsseite getippt/bearbeitet, oder aus
// config.json geladen) ein und füllt sie in ein Signalcode-Array. Akzeptiert
// nur 1 und -1 als gültige Werte, um das Array vor unsinnigen Eingaben zu schützen.
void parseSigcode(String input, int8_t* arr, int& size) {
  int tempSize = 0;
  int start = 0;
  // Hinweis: Eine URL-Dekodierung (z.B. "%2C" -> ",") ist hier nicht mehr nötig -
  // das übernimmt jetzt zentral extractParam() (siehe urlDecode()), bevor der
  // Wert überhaupt hier ankommt. Werte aus config.json waren nie URL-kodiert.

  int idx = input.indexOf(',');

  while (idx != -1 && tempSize < 128) {
    int val = input.substring(start, idx).toInt();

    // Nur genau 1 oder -1 akzeptieren - alles andere wird stillschweigend ignoriert
    if (val == 1 || val == -1) {
      arr[tempSize++] = (int8_t)val;
    }

    start = idx + 1;
    idx = input.indexOf(',', start);
  }

  // Den letzten Wert nach dem letzten Komma prüfen (die Schleife oben endet einen zu früh)
  if (start < input.length() && tempSize < 128) {
    int val = input.substring(start).toInt();
    if (val == 1 || val == -1) {
      arr[tempSize++] = (int8_t)val;
    }
  }

  size = tempSize; // Zurückmelden, wie viele gültige 1/-1-Werte tatsächlich gefunden wurden
}


// Kehrt die URL-Kodierung um, die der Browser beim Absenden eines
// GET-Formulars automatisch auf die Eingabewerte anwendet. Browser wandeln
// dabei praktisch jedes "besondere" Zeichen in "%XX" um (die zwei Stellen XX
// sind der Zeichencode in Hexadezimal) - ein Schrägstrich "/" wird z.B. zu
// "%2F", ein Doppelpunkt ":" zu "%3A", ein Leerzeichen zu "+". Ohne diese
// Funktion würden z.B. MQTT-Topics wie "teensysender/input" als
// "teensysender%2Finput" ankommen und genau so (falsch!) gespeichert werden.
String urlDecode(String input) {
  String decoded = "";
  char hexBuffer[3] = "00";   // Puffer für die 2 Hex-Ziffern nach einem "%"
  unsigned int len = input.length();
  unsigned int i = 0;
  while (i < len) {
    char c = input.charAt(i);
    if (c == '+') {
      // Ein "+" im Query-String steht für ein Leerzeichen
      decoded += ' ';
      i++;
    } else if (c == '%' && i + 2 < len) {
      // "%XX" -> das eine echte Zeichen mit dem Code XX (hexadezimal) umwandeln
      hexBuffer[0] = input.charAt(i + 1);
      hexBuffer[1] = input.charAt(i + 2);
      char realChar = (char) strtol(hexBuffer, NULL, 16);
      decoded += realChar;
      i += 3;
    } else {
      // Ganz normales Zeichen, unverändert übernehmen
      decoded += c;
      i++;
    }
  }
  return decoded;
}


// Sucht nach "paramName=" innerhalb einer rohen HTTP-Anfrage-/URL-Zeichenkette
// und gibt den Text zurück, der direkt danach kommt, bis zum nächsten "&" oder
// Leerzeichen. So werden über das Einstellungs-Webformular übermittelte Werte
// gelesen (z.B. .../save?peri=1.5). Der zurückgegebene Text wird automatisch
// mit urlDecode() dekodiert, damit z.B. "/" (als "%2F" gesendet) wieder als
// echter Schrägstrich ankommt.
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


// Speichert die aktuellen Einstellungen (PeriOnOffThreshold, alle 5 Signal-
// codes, sowie - falls aktiviert - die MQTT- und WhatsApp-Einstellungen) als
// JSON-Textdatei im eingebauten Flash-Speicher des ESP32 (LittleFS), damit
// sie einen Neustart/Stromausfall überstehen. Diese Funktion manuell
// aufrufen, wann immer sich eine Einstellung ändert.
void saveSettingsToLittleFS() {
  DynamicJsonDocument json(2048); // Größerer Puffer, da jetzt auch MQTT- und WhatsApp-Einstellungen mit gespeichert werden
  json["Peri"] = PeriOnOffThreshold;
  json["s0"] = sigcodeToString(sigcode0, sigcode0_size);
  json["s1"] = sigcodeToString(sigcode1, sigcode1_size);
  json["s2"] = sigcodeToString(sigcode2, sigcode2_size);
  json["s3"] = sigcodeToString(sigcode3, sigcode3_size);
  json["s4"] = sigcodeToString(sigcode4, sigcode4_size);

  // MQTT-Einstellungen nur speichern, wenn MQTT überhaupt einkompiliert ist -
  // so landen keine "toten" Felder in der Datei, wenn die Funktion nicht genutzt wird.
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

  // WhatsApp-Einstellungen nur speichern, wenn WhatsApp überhaupt einkompiliert ist
  #ifdef WhatsApp_messages
    json["waPhone"] = mobile_number;
    json["waApiKey"] = api_key;
  #endif

  File configFile = LittleFS.open("/config.json", "w");
  serializeJson(json, configFile);
  configFile.close();
  if(debug) Serial.println("Einstellungen im Flash gespeichert.");
}


// Löscht die WLAN-Zugangsdaten UND die gespeicherte config.json und startet
// den ESP32 danach neu, komplett in einem frischen, werksneuen Zustand.
// Wird über die Einstellungs-Webseite ausgelöst (Button "Werksreset").
void factoryReset() {
  if(debug) Serial.println("WERKSRESET! Lösche WLAN und Config...");

  // 1. Gespeicherte WLAN-Zugangsdaten löschen (erzwingt beim nächsten Start das WiFiManager-Einrichtungsportal)
  wm.resetSettings();

  // 2. Nur die Konfigurationsdatei löschen
  if (LittleFS.exists("/config.json")) {
    if (LittleFS.remove("/config.json")) {
      if(debug) Serial.println("config.json erfolgreich gelöscht.");
    } else {
      if(debug) Serial.println("Fehler beim Löschen der config.json!");
    }
  } else {
    if(debug) Serial.println("Keine config.json zum Löschen gefunden.");
  }

  // 3. Optional: Standardwerte im RAM wiederherstellen, falls nötig (aktuell nicht verwendet)
  // loadSettings(); // falls es irgendwo eine "Standardwerte laden"-Funktion gibt

  if(debug) Serial.println("Neustart...");
  delay(1000);
  ESP.restart();
}


//********************* Code für WhatsApp **********************************
// Telefonnummer und API-Key sind ganz am Anfang der Datei deklariert (siehe
// "WhatsApp Zugangsdaten" weiter oben) - dort steht auch, warum.

// Eine "Warteschlange" (Queue) ist ein thread-sicherer Briefkasten: sendWhatsappMessage()
// legt unten eine Nachricht hinein und kehrt sofort zurück, während ein separater
// Hintergrund-Task (whatsappTask(), gestartet in setup()) die Nachrichten abholt und
// tatsächlich sendet. So blockiert die langsame HTTPS-Anfrage an die WhatsApp-API nie
// die Haupt-loop() (Webserver, MQTT, Timer, ...).
QueueHandle_t whatsappQueue = NULL;                     // Wird in setup() erstellt
const long interval = 10000;                            // Mindestpause zwischen zwei WhatsApp-Nachrichten, um HTTP 429 "Too Many Requests" zu vermeiden

// Diese Funktion kann von überall im Code aufgerufen werden, um eine WhatsApp-Nachricht
// zu senden. Sie legt die Nachricht nur in die Warteschlange - das tatsächliche Senden
// passiert in whatsappTask().
void sendWhatsappMessage(String message){
  #ifdef WhatsApp_messages
    if (whatsappQueue != NULL) {
      // FreeRTOS-Warteschlangen können keine Arduino-"String"-Objekte direkt speichern,
      // nur einfache C-Zeichenketten-Arrays, daher kopieren wir die Nachricht zuerst in
      // einen Puffer fester Größe.
      char msg_buffer[256];
      message.toCharArray(msg_buffer, sizeof(msg_buffer));
      if (xQueueSend(whatsappQueue, &msg_buffer, 0) != pdPASS) {
        // Warteschlange war voll (mehr als 5 wartende Nachrichten) - Nachricht verworfen.
        Serial.println("WhatsApp-Warteschlange ist voll, Nachricht wird verworfen.");
      }
    }
  #endif
}

// Hintergrund-Task: läuft für immer auf Core 1, komplett getrennt von loop(). Er schläft
// (verbraucht keine CPU-Zeit), bis eine Nachricht in whatsappQueue erscheint, sendet sie
// dann und wartet "interval" Millisekunden, bevor er die nächste bearbeitet, damit die
// API nicht überlastet wird.
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
          Serial.print("WhatsApp-Nachricht erfolgreich gesendet: ");
          Serial.println(message);
        } else {
          Serial.println("Fehler beim Senden der Nachricht");
          Serial.print("HTTP-Antwortcode: ");
          Serial.println(http_response_code);
        }

        http.end();
      } else {
        Serial.println("WhatsApp-Task: WLAN ist nicht verbunden. Nachricht wurde nicht gesendet.");
      }

      vTaskDelay(pdMS_TO_TICKS(interval));              // Abkühlpause, bevor die nächste wartende Nachricht gesendet wird
    }
  }
}
// Ende: Code für WhatsApp


//********************* SIGNALVERWALTUNG **********************************
// Diese Funktion ist eine Interrupt Service Routine (ISR): der in setup()
// eingerichtete Hardware-Timer ruft sie automatisch und sehr präzise alle
// "sigDuration" Mikrosekunden auf (z.B. alle 104us), egal was die Haupt-loop()
// gerade macht. Das ermöglicht die zeitgenaue Ausgabe des Perimeterdraht-
// Signals ("sigcode_norm"-Array).
// Da es eine ISR ist, muss sie SCHNELL sein und darf keine Dinge wie
// Serial.print() oder delay() aufrufen - deshalb schreibt sie direkt in die
// GPIO-Register (GPIO.out_w1ts / GPIO.out_w1tc) statt das normale, langsamere
// digitalWrite() zu verwenden.
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);                    // Andere Interrupts kurz blockieren, damit dies nicht mittendrin gestört werden kann
  if (enableSenderA) {
    if (sigcode_norm[stepA] == 1) {
      GPIO.out_w1tc = (1 << pinIN1);                    // Setzt pinIN1 auf LOW  (w1tc = "write 1 to clear")
      GPIO.out_w1ts = (1 << pinIN2);                    // Setzt pinIN2 auf HIGH (w1ts = "write 1 to set")
    } else if (sigcode_norm[stepA] == -1) {
      GPIO.out_w1ts = (1 << pinIN1);                    // Setzt pinIN1 auf HIGH
      GPIO.out_w1tc = (1 << pinIN2);                    // Setzt pinIN2 auf LOW
    }
    stepA++;                                            // Sender A auf das nächste Bit des Signalcodes weiterschalten
    if (stepA == sigcode_size) {
      stepA = 0;                                        // Nach dem letzten Bit wieder zum Anfang springen
    }
  }
  if (enableSenderB) {
    if (sigcode_norm[stepB] == 1) {
      GPIO.out_w1tc = (1 << pinIN3);                    // Setzt pinIN3 auf LOW
      GPIO.out_w1ts = (1 << pinIN4);                    // Setzt pinIN4 auf HIGH
    } else if (sigcode_norm[stepB] == -1) {
      GPIO.out_w1ts = (1 << pinIN3);                    // Setzt pinIN3 auf HIGH
      GPIO.out_w1tc = (1 << pinIN4);                    // Setzt pinIN4 auf LOW
    }
    stepB++;                                            // Sender B auf das nächste Bit des Signalcodes weiterschalten (unabhängig von stepA)
    if (stepB == sigcode_size) {
      stepB = 0;
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);                     // Andere Interrupts wieder zulassen
}
// Ende Signalmanagement


//********************* BEREICH WECHSELN **********************************
// Lädt eines der Arrays sigcode0..sigcode4 in sigcode_norm[], das Array, das
// der onTimer()-Interrupt tatsächlich Bit für Bit aussendet. So wird dem
// Mäher mitgeteilt, dass er in eine andere Zone/einen anderen Bereich (0-4)
// wechseln soll.
void changeArea(byte areaInMowing) {
  stepA = 0;                                            // Beide Sender wieder an den Anfang des (neuen) Signalcodes setzen
  stepB = 0;
  enableSenderA = false;                                // Beide Sender kurz deaktivieren, während der Signalcode getauscht wird, zur Sicherheit
  enableSenderB = false;
  #ifdef SerialOutput
    Serial.print("Wechsel zu Bereich:");
    Serial.println(areaInMowing);
  #endif

  #ifdef Screen
    u8x8.clear();
    u8x8.setCursor(0,0);  
    u8x8.print("Bereich:");
    u8x8.println(areaInMowing);
  #endif
  for (int uu = 0; uu < 128; uu++) {  // Zuerst den kompletten Puffer leeren (entfernt Reste vom vorherigen Code)
    sigcode_norm[uu] = 0;
  }
  sigcode_size = 0;

  // Den gewählten Signalcode (und seine Länge) in die "live" von der ISR verwendeten Arrays kopieren
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
    Serial.print("Neuer verwendeter Sigcode  : ");
    Serial.println(sigCodeInUse);

    for (int uu = 0; uu <= (sigcode_size - 1); uu++) {
      Serial.print(sigcode_norm[uu]);
      Serial.print(",");
    }
    
    Serial.println();
    Serial.print("Neue Sigcode-Größe  : ");
    Serial.println(sigcode_size);
  #endif

  #ifdef Screen
    u8x8.setCursor(0,2);
    u8x8.print("Sigcode aktiv:");
    u8x8.print(sigCodeInUse);
    Serial.println();
    Serial.print("Neue Sigcode-Größe: ");
    Serial.println(sigcode_size);
    delay(2000);
  #endif
}
// ENDE ChangeArea


//********************* STATICSCREENPARTS **********************************
// Zeichnet die Teile des OLED-Bildschirm-Layouts, die sich nie ändern
// (Beschriftungen wie "Laufzeit:", "Peri mA:", ...). Wird alle 10 Sekunden
// aus loop() aufgerufen - die eigentlichen Zahlenwerte neben diesen
// Beschriftungen werden separat, häufiger, an anderer Stelle in loop() aktualisiert.
void StaticScreenParts() {
#ifdef Screen
  //Zeile 0: Titel
  u8x8.setCursor(0,0);
  u8x8.inverse();
  u8x8.print(" ESP32 Sender  ");
  u8x8.noInverse();

  //Zeile 1: frei
  u8x8.clearLine(1);

  //Zeile 2: Sender EIN/AUS
  u8x8.clearLine(2);

  //Zeile 3: frei
  u8x8.clearLine(3);

  //Zeile 4: Arbeitszeit
  u8x8.setCursor(0, 4);
  u8x8.print("Laufzeit:");

  //Zeile 5: Perimeterstrom
  u8x8.setCursor(0, 5);
  u8x8.print("Peri mA:");

  //Zeile 6: Ladestrom
  u8x8.setCursor(0, 6);
  u8x8.print("Laden mA:");

//Zeile 7: Bereich
  u8x8.setCursor(0, 7);
  u8x8.print("Bereich:");
#endif  
}
// ENDE StaticScreenParts


//********************* SaveConfigCallback **********************************
// WiFiManager ruft diese Funktion automatisch auf (sie wird weiter unten
// als "Callback" registriert), immer wenn der Nutzer WLAN-Einstellungen
// über das Konfigurationsportal eingegeben/geändert hat. Wir merken uns
// das hier nur mit einem Flag; das tatsächliche Speichern passiert später in setup().
void saveConfigCallback () {  // Callback, der uns benachrichtigt, dass die Konfiguration gespeichert werden muss
  if (debug) Serial.println("Config sollte gespeichert werden");
  shouldSaveConfig = true;
}
// ENDE SaveConfigCallback


//********************* MQTTinput **********************************
// Wird automatisch von der MQTT-Bibliothek aufgerufen, sobald eine Nachricht
// auf einem abonnierten Topic eintrifft. So kann der Mäher aus jeder
// MQTT-fähigen App oder jedem Hausautomatisierungssystem (Home Assistant,
// Node-RED, ...) ferngesteuert werden.
#ifdef MQTT
void MQTTinput(char* topic, byte* payload, unsigned int length) {

  Serial.print("Nachricht eingetroffen auf Topic: [");
  Serial.print(topic);
  Serial.print("] ");

  // Die rohen Payload-Bytes lesen und in einen normalen Text-String umwandeln
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();

  // Befehlsverarbeitung: den empfangenen Text mit bekannten Befehlen vergleichen
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
      Serial.println("Ein ungültiger MQTT-Befehl wurde empfangen.");
    }
  }

}   // ENDE MQTTinput
#endif


//********************* WLAN am Leben erhalten **********************************
// Prüft alle "wifiCheckInterval" Millisekunden, ob die WLAN-Verbindung noch
// steht - und versucht bei Bedarf, sich still im Hintergrund neu zu
// verbinden, ohne den Rest des Programms zu blockieren (loop() läuft normal weiter).
void keepWiFiAlive() {
  unsigned long currentMillis = millis();

  // Nur tatsächlich prüfen, wenn das definierte Intervall abgelaufen ist
  if (currentMillis - lastWiFiCheckMillis >= wifiCheckInterval) {
    lastWiFiCheckMillis = currentMillis;

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Verbindung verloren! Versuche im Hintergrund neu zu verbinden...");

      // Da WiFiManager die Zugangsdaten bereits im Flash-Speicher des ESP32
      // gespeichert hat, reicht hier ein einfacher, nicht-blockierender
      // Aufruf von begin():
      WiFi.begin(); 
    } else {
      Serial.print("[WiFi] Verbunden. IP: ");
      Serial.println(WiFi.localIP());
    }
  }
}




//********************* SETUP **********************************
// setup() läuft genau EINMAL, direkt nachdem der ESP32 eingeschaltet wurde
// oder ein Reset gemacht hat. Ihre Aufgabe ist es, alles vorzubereiten, was
// der Rest des Programms braucht: die serielle Konsole starten, Sensoren/
// Display initialisieren, den Timer-Interrupt starten, sich mit dem WLAN
// verbinden, den Webserver starten usw.
void setup() {
  Serial.begin(115200);                           // Die USB-Serial-Verbindung starten (wird für das Debug-Log im Serial Monitor am Computer verwendet)
  Wire.begin(I2C_SDA, I2C_SCL);                   // Den I2C-Bus starten (wird verwendet, um mit beiden INA226-Sensoren und dem OLED-Display zu sprechen)

  // *** FreeRTOS: die WhatsApp-Warteschlange und ihren Hintergrund-Sende-Task erstellen ***
  // (Details siehe "Code for Whatsapp" weiter oben)
  whatsappQueue = xQueueCreate(5, 256);           // Bis zu 5 wartende Nachrichten, je 256 Bytes
  xTaskCreatePinnedToCore(
    whatsappTask,                                 // Funktion, die den Code des Tasks enthält
    "WhatsappTask",                               // Name des Tasks (nur für Debugging-Werkzeuge verwendet)
    10000,                                        // Stack-Größe in Bytes (muss groß genug für HTTP/SSL sein)
    NULL,                                         // Parameter, die an die Task-Funktion übergeben werden (hier keine nötig)
    1,                                             // Priorität des Tasks (eine kleinere Zahl bedeutet niedrigere Priorität)
    NULL,                                         // Task-Handle (hier nicht benötigt, daher NULL)
    1                                              // Auf Core 1 ausführen (Core 0 ist oft mit WLAN/Bluetooth beschäftigt)
  );

  InaPeri.init();                                 // Den INA226-Sensor für die Perimeterdraht-Messung initialisieren
  InaCharge.init();                               // Den INA226-Sensor für die Lademessung initialisieren

  #ifdef Screen
  u8x8.begin();                                   // Das OLED-Display starten
  u8x8.setFont(u8x8_font_5x8_f);                  // Eine kleine, gut lesbare Schriftart wählen
  u8x8.clear();
  #endif
  timer = timerBegin(1000000);                    // Einen Hardware-Timer starten, der 1.000.000 Mal pro Sekunde tickt (1 Tick = 1 Mikrosekunde)
  timerAttachInterrupt(timer, &onTimer);           // Dem Timer sagen, dass er bei jedem Alarm automatisch onTimer() aufrufen soll
  timerAlarm(timer, sigDuration, true, 0);         // Den Alarm so einstellen, dass er alle "sigDuration" Mikrosekunden auslöst, und zwar endlos wiederholt
  pinMode(pinIN1, OUTPUT);                        // Alle Treiber-Steuerpins als digitale OUTPUTs konfigurieren
  pinMode(pinIN2, OUTPUT);
  pinMode(pinEnableA, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinEnableB, OUTPUT);
  pinMode(pinGreenLED, OUTPUT);                   // 2-farbige LED grün
  pinMode(pinRedLED, OUTPUT);                     // 2-farbige LED rot
  digitalWrite(pinGreenLED, LOW);                 // Sicherstellen, dass beide LEDs zu Beginn aus sind
  digitalWrite(pinRedLED, LOW);
  
  #ifdef SerialOutput
    Serial.println("START");
    Serial.print("ESP32 Sender ");
    Serial.println(VER);
  #endif
  
  changeArea(sigCodeInUse);                       // Den anfangs gewählten Signalcode in das von der ISR verwendete Arbeits-Array laden
  if (enableSenderA) {
    digitalWrite(pinEnableA, HIGH);
  }
  if (enableSenderB) {
    digitalWrite(pinEnableB, HIGH);
  }

  //------------------------  Flash save parts  ----------------------------------------
  WiFi.mode(WIFI_STA);                             // "Station"-Modus: sich mit einem bestehenden WLAN-Netzwerk verbinden (statt selbst eins aufzuspannen)
  if (LittleFS.begin(true)) {   // Das Flash-Dateisystem einhängen (true = automatisch formatieren, falls es einmal unlesbar sein sollte)
    if (LittleFS.exists("/config.json")) {  // Wenn eine gespeicherte Einstellungsdatei existiert, diese laden
      File configFile = LittleFS.open("/config.json", "r");   // Datei zum Lesen öffnen
      if (configFile) {
        size_t size = configFile.size();  // Herausfinden, wie groß die Datei ist, um einen ausreichend großen Puffer zu reservieren
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(2048); // Größerer Puffer, da jetzt auch MQTT- und WhatsApp-Einstellungen mit gespeichert werden
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
          // --- MQTT-Einstellungen laden (nur, wenn MQTT überhaupt aktiviert ist) ---
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
          // --- WhatsApp-Einstellungen laden (nur, wenn WhatsApp überhaupt aktiviert ist) ---
          #ifdef WhatsApp_messages
            if (json["waPhone"]) mobile_number = json["waPhone"].as<String>();
            if (json["waApiKey"]) api_key = json["waApiKey"].as<String>();
          #endif
        } else {
          if (debug) Serial.println("JSON-Konfiguration konnte nicht geladen werden");
        }
      }
    }
  } else {
    if (debug) Serial.println("Dateisystem (LittleFS) konnte nicht eingehängt werden");
  }   // Ende Lesen

  //------------------------ MQTT ------------------------------------------------
  // Erst HIER (nach dem Laden der Einstellungen oben) mit den tatsächlich
  // gültigen Werten aufrufen - so werden gespeicherte MQTT-Einstellungen auch
  // wirklich verwendet und nicht durch die Startwerte überschrieben.
  #ifdef MQTT
  MQTTclient.setServer(mqtt_server.c_str(), mqtt_port);       // Broker-Adresse und Port setzen
  MQTTclient.setCallback(MQTTinput);                          // Die Funktion (MQTTinput) wird aufgerufen, wenn Daten empfangen werden.
  #endif

  wm.setSaveConfigCallback(saveConfigCallback);    // saveConfigCallback() registrieren, damit sie aufgerufen wird, wenn WiFiManager Einstellungen speichern muss

  // Hier wird bewusst KEINE statische IP gesetzt - der Router vergibt die
  // IP-Adresse (und Gateway/Subnetz) automatisch per DHCP bei jeder Verbindung des ESP32.

  // Begrenzt, wie lange der ESP32 versucht, sich mit einem bekannten WLAN zu verbinden, bevor er aufgibt (in Sekunden)
  wm.setConnectTimeout(15); 

  // Begrenzt, wie lange das Konfigurations-WLAN (der "Access Point", mit dem man sich zur Einrichtung verbindet) aktiv bleibt (in Sekunden).
  // Verbindet sich in dieser Zeit niemand damit, gibt es auf und loop() startet trotzdem (Offline-Modus).
  wm.setConfigPortalTimeout(60); 

  // Automatisches Wiederverbinden im Hintergrund aktivieren (eine native ESP32-Funktion)
  WiFi.setAutoReconnect(true);

  if (!wm.autoConnect("MowerSender_AP","12345678")) { // Das Passwort sollte mindestens 8 Zeichen lang sein.
    Serial.println("WLAN-Verbindung fehlgeschlagen oder Timeout abgelaufen. Starte im Offline-Modus...");
  } else {
    if (debug) Serial.println("connected :)");
  }
  
  if (shouldSaveConfig) {   
    saveSettingsToLittleFS();
  }

  sendWhatsappMessage(String("ESP32 sender is now online. IP-Adress: ") + WiFi.localIP().toString());          // Code for Whatsapp
  server.begin();                                  // Den Webserver starten, damit er jetzt bereit ist, Browserverbindungen anzunehmen


  //------------------------  current sensor parts  ----------------------------------------
  #ifdef SerialOutput
    Serial.println("Spannung und Strom werden mit dem INA226 gemessen ...");
  #endif

  InaPeri.setAverage(INA226_AVERAGE_4);                          // 4 Messwerte pro Ablesung mitteln, um Rauschen zu glätten
  InaPeri.setResistorRange(resistorPeri, rangePeri);
//  InaPeri.waitUntilConversionCompleted();
  InaCharge.setAverage(INA226_AVERAGE_4);
  InaCharge.setResistorRange(resistorCharge, rangeCharge);
//  InaCharge.waitUntilConversionCompleted();



  //------------------------  ArduinoOTA  ----------------------------------------
  // OTA = "Over-The-Air"-Updates: erlaubt das Hochladen neuer Firmware über
  // WLAN, statt ein USB-Kabel anschließen zu müssen.
 #ifdef OTAUpdates
 ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "Sketch";
      else
        type = "Dateisystem";

      // HINWEIS: Wenn LittleFS aktualisiert wird, wäre hier die Stelle, LittleFS mit LittleFS.end() auszuhängen
      Serial.println("Update wird gestartet: " + type);
    })
    .onEnd([]() {
      Serial.println("\nFertig");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Fortschritt: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Fehler[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Authentifizierung fehlgeschlagen");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Start fehlgeschlagen");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Verbindung fehlgeschlagen");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Empfang fehlgeschlagen");
      else if (error == OTA_END_ERROR) Serial.println("Abschluss fehlgeschlagen");
    });

  ArduinoOTA.begin();
  #endif
}
// ENDE SETUP


//********************* LOOP **********************************
// loop() läuft immer wieder, endlos, so schnell der ESP32 es schafft, direkt
// nachdem setup() fertig ist. Wichtig: sie sollte so schlank/schnell wie
// möglich bleiben - alle wirklich zeitkritische Arbeit (die Ausgabe des
// Perimetersignals) passiert getrennt im onTimer()-Interrupt, NICHT hier.
// loop() kümmert sich um alles andere: WLAN prüfen, OTA-Updates, Sensor-
// werte lesen, Timer, MQTT und das Ausliefern der Webseiten.
void loop() {

  keepWiFiAlive(); // Prüft die WLAN-Verbindung in festgelegten Abständen und versucht, sie bei Bedarf wiederherzustellen.

  #ifdef MQTT
    // WICHTIG: MQTTclient.loop() muss regelmäßig aufgerufen werden, damit die
    // Bibliothek die Verbindung am Leben hält UND eingehende Nachrichten
    // verarbeitet (ruft bei Bedarf MQTTinput() auf). Das fehlte bisher komplett -
    // ohne diesen Aufruf kamen nie MQTT-Befehle an, egal was abonniert war.
    MQTTclient.loop();
  #endif

  // Auf drahtlose Software-Updates prüfen
  #ifdef OTAUpdates
    ArduinoOTA.handle();
  #endif

  
  // --- TIMER-BLOCK: läuft alle 10 Sekunden ---
  if (millis() >= nextTimeControl) {
    // Den Zeitpunkt für die nächste Ausführung festlegen (aktuelle Zeit + 10 Sekunden)
    nextTimeControl = millis() + 10000;

    // Die statischen Textbeschriftungen auf dem OLED-Display aktualisieren
    #ifdef Screen
      StaticScreenParts();
    #endif


    #ifdef MQTT
      // Nur EINMAL pro 10-Sekunden-Zyklus versuchen statt endlos in einer Schleife -
      // ist der Broker nicht erreichbar, würde eine Endlos-while()-Schleife hier den
      // ganzen ESP32 einfrieren (Webserver, OTA, WLAN...) und könnte einen Watchdog-
      // Reset auslösen. Schlägt dieser Versuch fehl, wird es einfach beim nächsten
      // Durchlauf dieses Blocks erneut versucht.
      if (!MQTTclient.connected()) {
        if (MQTTclient.connect(clientID.c_str(), MQTT_user.c_str(), MQTT_password.c_str())) {
          // Nach einer neuen Verbindung muss man sich beim Broker erneut für das
          // gewünschte Topic anmelden ("abonnieren"), sonst kommen nie Befehle über
          // MQTTinput() an - das hat in der ursprünglichen Version gefehlt!
          MQTTclient.subscribe(subTopic.c_str());
        }
      }
      // Den Ladestrom in einen String umwandeln 
      String MQTTmsgString = String(ChargeCurrent, 2); // float in String mit 2 Nachkommastellen
      // Den Ladestrom an den MQTT-Broker veröffentlichen
      MQTTclient.publish(pubTopic.c_str(), MQTTmsgString.c_str());
      
      // Den Perimeterschleifenstrom in einen String umwandeln
      String MQTTmsgString2 = String(PeriCurrent, 2); // float in String mit 2 Nachkommastellen
      // Den Perimeterstrom an den MQTT-Broker veröffentlichen
      MQTTclient.publish(pubTopic2.c_str(), MQTTmsgString2.c_str());
 
      // Die ChargeBusVoltage (float) für MQTT in einen String umwandeln
      String MQTTmsgString3 = String(ChargeBusVoltage, 2); // float in String mit 2 Nachkommastellen
      // Die Ladespannung an den MQTT-Broker veröffentlichen
      MQTTclient.publish(pubTopic3.c_str(), MQTTmsgString3.c_str());
    #endif
    // ENDE MQTT

    // Die Spannung vom INA226-Sensor auslesen (Perimeter)
    PeriBusVoltage = InaPeri.getBusVoltage_V();
    // Den winzigen Spannungsabfall über dem Messwiderstand auslesen
    PeriShuntVoltage = InaPeri.getShuntVoltage_mV();
    // Den tatsächlich durch den Perimeterdraht fließenden Strom auslesen
    PeriCurrent = InaPeri.getCurrent_mA();

    // Den Grundverbrauch der Elektronik (ESP32/Treiber) abziehen
    PeriCurrent = PeriCurrent - 80.0;                        //DC/DC-Wandler, ESP32, L298N ziehen zwischen 80 und 100 mA, wenn nichts AN ist und ein WLAN-Access-Point gefunden wurde (noch zu bestätigen ????)

    // Ist der Strom zu niedrig, einfach auf 0 setzen, um "Geister"-Messwerte zu vermeiden
    if (PeriCurrent <= PERI_CURRENT_MIN) PeriCurrent = 0;

    // Ist ein Sender aktiv, aber es fließt kein Strom, ist der Draht wahrscheinlich durchtrennt
    if ((enableSenderA) && (PeriCurrent < PERI_CURRENT_MIN)) {
      workTimeMins = 0; // Arbeitszeit-Zähler zurücksetzen
      #ifdef Screen
        u8x8.setCursor(0, 5);
        u8x8.inverse();   // Text hervorheben
        u8x8.print(" Draht getrennt!");
        u8x8.noInverse();
      #endif
      #ifdef SerialOutput
        Serial.println("DRAHT IST DURCHTRENNT!!!");
      #endif
        
    } else {
      // Ist alles in Ordnung, den Strom auf dem Display anzeigen
      #ifdef Screen
        u8x8.setCursor(8, 5);
        u8x8.print("        ");  // Alten Wert löschen
        u8x8.setCursor(10, 5);
        u8x8.print(PeriCurrent);
      #endif
      #ifdef SerialOutput
        Serial.print("Peristrom ");
        Serial.println(PeriCurrent);
        Serial.print("PeriSpannung ");
        Serial.println(PeriBusVoltage);
      #endif
    }

    // Sicherheitsabschaltung: Wenn der Mäher zu lange gearbeitet hat (z.B. 5 Stunden)
    if (workTimeMins >= WORKING_TIMEOUT_MINS && AUTO_START_SIGNAL == 1) {
      enableSenderA = false;  // Schleife A abschalten
      enableSenderB = false;  // Schleife B abschalten
       workTimeMins = 0;      // Zähler zurücksetzen
      WORKING_TIMEOUT = 1;    // Timeout-Flag setzen
      digitalWrite(pinEnableA, LOW);
      
      // Pins physisch auf LOW ziehen, um den L298N-Treiber zu stoppen
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      digitalWrite(pinEnableB, LOW);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);

      // Den Nutzer per WhatsApp alarmieren
      sendWhatsappMessage("TIMEOUT! Mower didn't come back home! Perimeterwire switched off!");          // Code for Whatsapp
      Serial.println("********************************   Timeout, Sender wird gestoppt  **********************************");
    }
  }

  // --- SEKUNDEN-BLOCK: läuft jede Sekunde ---
  if (millis() >= nextTimeSec) {          // Jede Sekunde ausführen
    nextTimeSec = millis() + 1000;

    // Arbeitszeit und aktuellen Signalcode auf dem Display aktualisieren
    #ifdef Screen
      u8x8.setCursor(9, 4);
      u8x8.print("       ");
      u8x8.setCursor(10, 4);
      u8x8.print(workTimeMins);
    
      //Zeile 7: Bereich
      u8x8.setCursor(10, 7);
      u8x8.print("      ");
      u8x8.setCursor(10, 7);
      u8x8.print(sigCodeInUse);
    #endif
    #ifdef SerialOutput
      Serial.print("Bereich:");
      Serial.println(sigCodeInUse);
    #endif

    // Strom und Spannung vom Lade-Sensor auslesen
    ChargeBusVoltage = InaCharge.getBusVoltage_V();
    ChargeShuntVoltage = InaCharge.getShuntVoltage_mV();
    ChargeCurrent = InaCharge.getCurrent_mA();

    // Status-LED steuern: Rot beim Laden, Grün im Standby/bereit
    if (ChargeCurrent > ChargeThreshold) {  //Nur für die Lade-LED
      digitalWrite(pinGreenLED, LOW);
      digitalWrite(pinRedLED, HIGH);  // Lädt
    } else  {
      digitalWrite(pinRedLED, LOW);
      digitalWrite(pinGreenLED, HIGH);  // Bereit
    }

    // Den Stromwert für die Anzeige vorbereiten (Rauschen unter dem Schwellwert ignorieren)
    ChargeCurrentPrint = ChargeCurrent;
    if (ChargeCurrent < ChargeThreshold) ChargeCurrentPrint = 0;  // zeigt 0, wenn der Mäher nicht lädt

    #ifdef Screen
      u8x8.setCursor(10, 6);
      u8x8.print("      ");
      u8x8.setCursor(10, 6);
      u8x8.print(ChargeCurrentPrint);
    #endif
    #ifdef SerialOutput
      Serial.print("Ladestrom: ");
      Serial.println(ChargeCurrentPrint);
    Serial.print("Ladespannung: ");
    Serial.println(ChargeBusVoltage);
    #endif

    // Wird ein höherer Strom erkannt, ist der Mäher in der Station
    if (ChargeCurrent > PeriOnOffThreshold && AUTO_START_SIGNAL == 1) {   // Mäher ist in der Station, im Test wurden 410 mA gezogen, daher kann der Sender gestoppt werden - Bei Vollladung liegt der Strom bei ca. 4mA.
                                                // Den Wert also klein halten, um eine Aktivierung durch den Perimeterdraht zu vermeiden, bevor der Mäher startet.
      enableSenderA = false;  // Das Drahtsignal stoppen (Mäher ist zuhause)
      enableSenderB = false;

      // War der Mäher vorher am Arbeiten, ist er jetzt zurückgekehrt
      if(mowerIsWorking == 1)  {
        mowerIsWorking = 0;
        WORKING_TIMEOUT = 0;
        sendWhatsappMessage("Mower is back at Home!");          // Code for Whatsapp
      }

      workTimeMins = 0; // Arbeitszeit-Zähler zurücksetzen
      // Alle Treiber-Ausgänge abschalten
      digitalWrite(pinEnableA, LOW);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      digitalWrite(pinEnableB, LOW);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);
      //delay(200);
    } else {

      if (AUTO_START_SIGNAL == 1 && !WORKING_TIMEOUT ) {
         // Hat der Mäher gerade erst die Station verlassen, als "arbeitend" markieren
         if(mowerIsWorking == 0)  {
          mowerIsWorking = 1;
          sendWhatsappMessage("Mower is going to work!");          // Eine WhatsApp-Nachricht senden
        }
        // Das Drahtsignal aktivieren, damit der Mäher die Grenze erkennen kann
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
  
    // Den Sekundenzähler erhöhen
    timeSeconds++;
    if (((enableSenderA) || (enableSenderB)) && (timeSeconds >= 60)) {                    // Ist der Sender AN & 60 Sekunden vorbei
      if (workTimeMins < 1440)  {                                                         // Überlauf vermeiden
        workTimeMins++;                                                                   // eine Minute hochzählen
        timeSeconds = 0;                                                                  // Sekunden auf 0 zurücksetzen
        if (workTimeChargeMins > 0) {                                                     // War workTimeCharge > 0 (letzter Zustand war LADEN)
          lastChargeMins = workTimeChargeMins;                                            // diese Zeit in lastChargeMins speichern
          workTimeChargeMins = 0;                                                         // und wieder auf 0 setzen
        }
      }
    } else  {
      if ((workTimeChargeMins < 1440) && (ChargeCurrent > ChargeThreshold) && (timeSeconds >= 60)) {   // Ist ChargeCurrent größer als ChargeThreshold (Mäher ist in Station und lädt)
      workTimeMins = 0;                                                                   // WorktimeMins zurücksetzen
        workTimeChargeMins++;                                                             // workTimeChargeMins hochzählen
        timeSeconds = 0;                                                                  // Sekunden zurücksetzen
      }
    }
    
    // Den Status der Sender anzeigen (welche Schleife aktiv ist)
    if ((enableSenderA) || (enableSenderB)) {

      #ifdef Screen
        u8x8.setCursor(0, 2);
        u8x8.print("Sender AN :     ");
      #endif
      #ifdef SerialOutput
        Serial.print("Sender AN : ");
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
        u8x8.print("Sender AUS      ");
      #endif
      #ifdef SerialOutput
        Serial.print("Sender AUS");
      #endif

    }
    #ifdef SerialOutput
      Serial.println("");
    #endif
  }

  // --- WEBSERVER-BLOCK: Verarbeitet Browseranfragen ---
  WiFiClient client = server.available(); // Prüfen, ob jemand die IP im Browser geöffnet hat
  if (client) {
    unsigned long currentTime = millis();
    unsigned long previousTime = currentTime;
    unsigned long timeoutTime = 500;  // Maximale Wartezeit auf Daten
    String req; // Speichert die Anfrage als Zeichenkette
    String currentLine;

    // Solange der Client verbunden ist und nicht in Timeout gelaufen ist
    while (client.connected() && currentTime - previousTime <= timeoutTime) {
      currentTime = millis();
      if (client.available()) {             // Sind Bytes vom Client zu lesen,
        char c = client.read();             // ein Byte lesen, dann
        //Serial.write(c);                  // es am Serial Monitor ausgeben
        req += c;                           // und es zur Anfrage-Zeichenkette hinzufügen
        if (c == '\n') {                    // Ist das Byte ein Zeilenumbruch
          // Ist die aktuelle Zeile leer, gab es zwei Zeilenumbrüche hintereinander.
          // Das ist das Ende der HTTP-Anfrage des Clients, also eine Antwort senden:
          if (currentLine.length() == 0) {
            // HTTP-Header beginnen immer mit einem Antwortcode (z.B. HTTP/1.1 200 OK)
            // und einem Content-Type, damit der Client weiß, was kommt, dann eine Leerzeile:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html; charset=utf-8"); // Wichtig!
            client.println("Connection: close");
            client.println(); // Leerzeile markiert das Ende des Headers

            // Merkt sich, ob einer der folgenden Fälle (z.B. /save oder
            // /download-settings) bereits eine eigene, vollständige Antwort
            // gesendet hat. Ohne dieses Flag würde ganz am Ende IMMER zusätzlich
            // noch index.html/settings.html angehängt werden - das ergab z.B.
            // nach dem Speichern zwei komplette HTML-Seiten in einer Antwort,
            // was der Browser nicht sauber anzeigen kann (weiße/leere Seite)!
            bool responseAlreadySent = false;

            // Auswerten, welche Seite/Aktion angefragt wurde
            if (req.indexOf("GET /toggleA") != -1) {
              // Logik zum Umschalten von Sender A
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
              // Logik zum Umschalten von Sender B
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
              saveSettingsToLittleFS(); // Einstellungen speichern!
              changeArea(sigCodeInUse);
            }

            else if (req.indexOf("GET /reset ") != -1) {
              factoryReset();
            }
            
            else if (req.indexOf("GET /save?") != -1) {
              // Die über das Einstellungsformular übermittelten Werte auslesen
              String pVal = extractParam(req, "peri");
              if (pVal != "") PeriOnOffThreshold = pVal.toFloat();
              
              String s0 = extractParam(req, "s0"); if (s0 != "") parseSigcode(s0, sigcode0, sigcode0_size);
              String s1 = extractParam(req, "s1"); if (s1 != "") parseSigcode(s1, sigcode1, sigcode1_size);
              String s2 = extractParam(req, "s2"); if (s2 != "") parseSigcode(s2, sigcode2, sigcode2_size);
              String s3 = extractParam(req, "s3"); if (s3 != "") parseSigcode(s3, sigcode3, sigcode3_size);
              String s4 = extractParam(req, "s4"); if (s4 != "") parseSigcode(s4, sigcode4, sigcode4_size);

              // --- MQTT-Einstellungen übernehmen (nur, wenn MQTT aktiviert ist) ---
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
                  // Wurde das Empfangs-Topic geändert, muss beim nächsten Verbindungsaufbau
                  // neu abonniert werden - das passiert automatisch in loop(), sobald
                  // MQTTclient.connected() das nächste Mal false zurückgibt.
                  mqttSettingsChanged = true;
                }

                String newPubTopic = extractParam(req, "mqttPubTopic");
                if (newPubTopic != "") pubTopic = newPubTopic;

                String newPubTopic2 = extractParam(req, "mqttPubTopic2");
                if (newPubTopic2 != "") pubTopic2 = newPubTopic2;

                String newPubTopic3 = extractParam(req, "mqttPubTopic3");
                if (newPubTopic3 != "") pubTopic3 = newPubTopic3;

                // Haben sich Server/Port/Zugangsdaten/Topic geändert, die bestehende
                // Verbindung trennen - loop() verbindet sich dann automatisch mit den
                // neuen Werten und abonniert das (ggf. neue) subTopic wieder neu.
                if (mqttSettingsChanged) {
                  MQTTclient.disconnect();
                  MQTTclient.setServer(mqtt_server.c_str(), mqtt_port);
                }
              #endif

              // --- WhatsApp-Einstellungen übernehmen (nur, wenn WhatsApp aktiviert ist) ---
              #ifdef WhatsApp_messages
                String newWaPhone = extractParam(req, "waPhone");
                if (newWaPhone != "") mobile_number = newWaPhone;

                String newWaApiKey = extractParam(req, "waApiKey");
                if (newWaApiKey != "") api_key = newWaApiKey;
              #endif

              saveSettingsToLittleFS();
              changeArea(sigCodeInUse); // Aktuellen Code sofort mit den neuen Werten aktualisieren
              
              // Den Browser zurück zur Einstellungsseite umleiten - mit "?saved=1"
              // in der URL, damit die Einstellungsseite weiß, dass sie den
              // Bestätigungshinweis "Einstellungen gespeichert!" anzeigen soll.
              client.println("<html><head><meta http-equiv='refresh' content='0; url=/settings?saved=1'></head><body>Speichere...</body></html>");
              responseAlreadySent = true;   // Verhindert, dass am Ende zusätzlich noch index.html angehängt wird
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
                client.println("Content-Type: application/octet-stream"); // Erzwingt einen Download statt Anzeige
                client.println("Content-Disposition: attachment; filename=\"config.json\"");
                client.println("Connection: close");
                client.println();
        
                while (file.available()) {
                  client.write(file.read());
                }
                file.close();
              } else {
                client.println("HTTP/1.1 404 Not Found\n\nDatei nicht gefunden.");
              }
              responseAlreadySent = true;   // Verhindert, dass am Ende zusätzlich noch index.html angehängt wird
            }

            // ANGEFRAGTE HTML-DATEI LESEN UND SENDEN
            // Nur ausführen, wenn oben noch KEINE eigene Antwort gesendet wurde
            // (z.B. bei /save oder /download-settings) - siehe responseAlreadySent weiter oben.
            if (!responseAlreadySent) {
            String filename = "/index.html";
            if (req.indexOf("GET /settings") != -1) filename = "/settings.html";

            // War das die Weiterleitung nach dem Speichern (URL enthält "saved=1"),
            // wird der Bestätigungshinweis auf der Einstellungsseite angezeigt.
            bool justSaved = (req.indexOf("saved=1") != -1);

            if (LittleFS.exists(filename)) {
              File file = LittleFS.open(filename, "r");
              String html = file.readString();
              file.close();
              client.print(processor(html, justSaved)); // Hier werden die %Platzhalter% durch echte Werte ersetzt!
            } else {
              client.println("Datei nicht gefunden!");
            }
            }   // ENDE if (!responseAlreadySent)
            break;   // Diese Anfrage ist fertig bearbeitet - Verbindung wird gleich geschlossen (egal welcher Fall oben zutraf)
          } else {
            // Die Zeile war NICHT leer - also eine normale Header-Zeile
            // (z.B. "Host: ..." oder "User-Agent: ..."). Für die nächste
            // Zeile zurücksetzen, damit currentLine.length()==0 wirklich nur
            // bei der echten Leerzeile (Ende aller Header) zutrifft.
            currentLine = "";
          }
        } else if (c != '\r') {
          // Normales Zeichen (kein Zeilenumbruch, kein Wagenrücklauf) -
          // an die aktuell gelesene Zeile anhängen.
          currentLine += c;
        }
      }
      // Puffer leeren und Verbindung schließen
      //client.flush();
    }
  }  
}
//ENDE LOOP
