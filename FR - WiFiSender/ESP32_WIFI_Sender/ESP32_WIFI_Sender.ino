// ============================================================================
//  PREMIERE CONFIGURATION / INSTALLATION - a lire absolument en premier !
// ============================================================================
//  1) Reglages de la carte dans l'IDE Arduino (menu "Outils"/"Tools") :
//     - Board : une carte ESP32 adaptee (ex. "ESP32 Dev Module")
//     - Partition Scheme :
//       "Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)"
//       C'est IMPORTANT et ce n'est PAS le reglage par defaut ! Ce schema
//       reserve de la place pour DEUX copies du firmware (necessaire pour
//       les mises a jour OTA, voir plus bas) ET une zone de fichiers pour
//       LittleFS (ou se trouvent index.html, settings.html et config.json).
//       Avec le schema de partition par defaut, soit il n'y a pas de place
//       pour LittleFS, soit l'OTA ne fonctionne pas correctement.
//
//  2) Le TOUT PREMIER televersement doit se faire par cable USB, pas par OTA/WiFi !
//     Deux raisons a cela :
//     a) L'OTA (mises a jour sans fil) est une fonctionnalite qui n'existe
//        que grace a CE sketch lui-meme, une fois qu'il tourne sur l'ESP32.
//        Un ESP32 totalement vierge ne connait pas encore l'OTA - le
//        firmware doit donc d'abord y arriver "a l'ancienne" par cable,
//        avant que les mises a jour suivantes puissent se faire sans fil.
//     b) Les pages web (index.html, settings.html) ne sont PAS televersees
//        quand on clique sur "Televerser" dans l'IDE Arduino - cela ne
//        televerse que ce fichier .ino. Les fichiers HTML se trouvent dans
//        le sous-dossier "data" et doivent etre transferes separement via
//        le plugin d'upload LittleFS (raccourci clavier CTRL+MAJ+P, voir
//        plus bas) - et ce plugin ne fonctionne que par cable USB, pas par
//        WiFi/OTA.
//
//     Deroulement la toute premiere fois :
//       1. Brancher l'ESP32 en USB
//       2. Regler la carte + le schema de partition comme indique ci-dessus
//       3. Televerser le sketch normalement par USB (icone fleche/"Televerser")
//       4. Juste apres : appuyer sur CTRL+MAJ+P pour transferer index.html,
//          settings.html (et eventuellement config.json) sur l'ESP32 via
//          l'upload LittleFS (egalement par USB)
//     Ce n'est QU'APRES cette configuration USB unique que le firmware (le
//     sketch lui-meme) peut etre mis a jour sans fil via OTA. Les
//     modifications des fichiers HTML dans le dossier "data" doivent
//     toujours passer par USB et le plugin d'upload LittleFS.
// ============================================================================
//
// ============================================================================
//  DIYRobotLawnMower Headquarter 1.0 - 07.05.26
//  Ce programme s'execute sur un ESP32 et pilote l'"emetteur de fil
//  perimetrique" d'une tondeuse-robot faite maison. Il envoie un code de
//  signal repetitif dans un fil enterre autour de la pelouse ; le capteur
//  de la tondeuse utilise ce signal pour connaitre la limite de la pelouse
//  et rester a l'interieur.
//
//  En plus de cela, ce sketch fait aussi :
//   - Il lance un petit serveur web pour tout controler depuis un navigateur
//   - Il envoie des notifications WhatsApp (tondeuse partie/revenue/timeout)
//   - Il publie les valeurs des capteurs via MQTT (pour la domotique)
//   - Il affiche les informations d'etat sur un petit ecran OLED
//   - Il prend en charge les mises a jour de firmware "OTA" (sans fil)
//
//  Remarque pour debutants : ce fichier est un seul gros sketch ".ino". Les
//  sketches Arduino/ESP32 ont toujours besoin exactement de deux fonctions
//  speciales :
//    - setup()  -> s'execute UNE SEULE FOIS au demarrage de l'ESP32, pour l'initialisation
//    - loop()   -> s'execute ensuite EN BOUCLE, indefiniment, une fois setup() termine
//  Toutes les autres fonctions de ce fichier (plus bas) sont juste des
//  fonctions utilitaires que setup() ou loop() (ou le serveur web, ou MQTT, ...)
//  appellent en cas de besoin.
// ============================================================================
//
// CTRL + MAJ + P --> Upload LittleFS to Pico/ESP8266/ESP32 for DATA upload
//  --> Plugin : https://github.com/earlephilhower/arduino-littlefs-upload
//  (LittleFS est un tout petit systeme de fichiers qui vit dans la memoire
//   flash de l'ESP32. Les pages web (index.html, settings.html) et les
//   reglages enregistres (config.json) y sont stockes - PAS dans ce fichier .ino.)
//
// Notes du developpeur (conservees depuis la version originale) :
// - MQTT fonctionne, mais n'etait pas tres stable au debut (trop de requetes).
//   L'appel client.flush() vers la ligne ~1200 a ete desactive, ce qui semble aider.
// - Resolu : WhatsApp provoquait auparavant une erreur HTTP 429 ("Too Many
//   Requests"), et le journal semblait corrompu ("...successfullyError
//   sending the message"). C'etait simplement un retour a la ligne manquant
//   dans un appel Serial.print(), maintenant corrige.
// - A FAIRE : une frequence d'envoi librement reglable n'est pas encore implementee.


//********************* Includes **********************************
// "#include" importe du code depuis des bibliotheques externes, pour ne pas
// avoir a tout reecrire soi-meme. La plupart doivent d'abord etre installees
// via le gestionnaire de bibliotheques Arduino/PlatformIO.
#define CONFIG_LITTLEFS_FOR_IDF_3_2
#include <LittleFS.h>                  // Petit systeme de fichiers dans la memoire flash (stocke les pages web + reglages)
#include <FS.h>                        // Support generique de systeme de fichiers, necessaire pour LittleFS
#include <ArduinoJson.h>               // Lit et ecrit du texte JSON (utilise pour config.json) - bibliotheque de Benoit
#include "soc/gpio_struct.h"           // Donne un acces direct et bas niveau aux broches GPIO, pour que l'interruption (ISR) soit la plus rapide possible
#include "freertos/FreeRTOS.h"         // Le mini "systeme d'exploitation" integre a l'ESP32, capable d'executer plusieurs taches en meme temps
#include "freertos/task.h"             // Necessaire pour creer et gerer des taches en arriere-plan (voir whatsappTask() plus bas)
#include "freertos/queue.h"            // Necessaire pour la file d'attente des messages WhatsApp (voir "Code for WhatsApp" plus bas)
#include <PubSubClient.h>              // Bibliotheque client MQTT - de Nick O'Leary
#include <WiFiManager.h>               // Gere la configuration WiFi via un portail captif, pour eviter d'ecrire les mots de passe WiFi en dur dans le code - de tzapu
WiFiManager wm;                        // L'unique objet WiFiManager utilise dans tout le sketch
#include <WiFiClient.h>                // Support reseau TCP/IP de base
#include <HTTPClient.h>                // Permet a l'ESP32 d'envoyer des requetes HTTP(S) (utilise pour appeler l'API WhatsApp)
#include <UrlEncode.h>                 // Rend un texte sur pour une URL (espaces, caracteres speciaux, ...) - de Masayuki
#include <INA226_WE.h>                 // Pilote pour la puce capteur de courant/tension INA226 - de Wolfgang Ewald
#include <ArduinoOTA.h>                // Permet de televerser un nouveau firmware par WiFi au lieu d'un cable USB - de Juraj Andrassy

#include <U8x8lib.h>                   // Pilote pour petits ecrans OLED monochromes - de Oliver Kraus
//********************* Display Settings **********************************
// Merci de DECOMMENTER exactement une des lignes de constructeur ci-dessous -
// cela depend du modele exact de puce d'ecran OLED que vous avez (SSD1306, SH1106, ...).
// La liste complete est disponible ici : https://github.com/olikraus/u8g2/wiki/u8x8setupcpp
// Merci d'adapter les numeros de broches a votre montage. Utilisez U8X8_PIN_NONE si la broche reset n'est pas connectee
 //U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 	      
//U8X8_SSD1306_128X64_ALT0_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 	      // identique a la variante NONAME, mais peut resoudre le probleme "une ligne sur deux manquante"
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);           // <-- c'est ce type d'ecran qui est reellement utilise
// Fin de la liste des constructeurs


//********************* Defines **********************************
// "#define" cree une constante qui est remplacee par sa valeur partout dans
// le code avant la compilation. Elle ne change jamais pendant que le
// programme s'execute. Commentez une ligne (ajoutez "//" devant) si vous
// n'avez pas besoin de cette fonctionnalite - les blocs "#ifdef ... #endif"
// environnants plus bas sauteront alors completement ce morceau de code, ce
// qui rend aussi le programme plus petit.
#define OTAUpdates 1                  // Active les mises a jour OTA (sans fil) du firmware
#define MQTT 1                        // Active l'envoi des donnees des capteurs vers un broker MQTT
#define WhatsApp_messages 1           // Active les notifications WhatsApp (tondeuse partie/revenue/timeout). Necessite une configuration, voir ci-dessous :
                                      // 1) Creez le contact Whatabot sur votre smartphone. Le numero de telephone est : +54 9 2364205798
                                      // 2) Envoyez : "I allow whatabot to send me messages"
                                      // 3) Copiez le numero de telephone et la cle API que Whatabot vous a envoyes.
                                      // Saisissez les deux sur la page de reglages dans le navigateur apres avoir televerse le sketch (/settings) - ils y sont enregistres dans config.json.

// Sur le circuit imprime se trouve un connecteur pour une LED bicolore a
// cathode commune(-). (Attention : la carte Matrix Mow800 a une LED a anode
// commune(+) a la place !!!)
#define pinGreenLED 25                                  // S'allume quand : la station est prete / la tondeuse est chargee / pas en charge

// La batterie charge si ChargeCurrent > LoadingThreshold
#define pinRedLED 26                                    // S'allume quand : la batterie est en train de charger

#define VER "DIYRobotLawnMower Headquarter 1.0 - 07.05.2026"   // Chaine de version du firmware, affichee au demarrage et sur l'ecran


bool AUTO_START_SIGNAL = 1;           // Si vrai : le signal du fil perimetrique demarre/s'arrete automatiquement selon le courant de charge (voir loop())
//#define USE_BUTTON 0                // (non utilise actuellement) Utiliser un bouton physique pour demarrer la tonte ou renvoyer la tondeuse a la station
#define SerialOutput 1              // Si defini : affiche des messages de debogage sur le moniteur serie (USB). Commentez pour couper le journal.
bool debug = false;                    // Un second interrupteur de debogage separe, utilise surtout pour les messages lies a WiFiManager
#define Screen 1                      // Si defini : utilise le petit ecran OLED. Commentez si vous n'en avez pas connecte un.

#define WORKING_TIMEOUT_MINS 300      // Securite (en minutes) : si la tondeuse n'est pas revenue a sa station apres ce delai, le signal du fil est coupe.
                                       // REMARQUE : cette securite ne fonctionne que si AUTO_START_SIGNAL est DESACTIVE - si AUTO_START_SIGNAL est actif, ce reglage est ignore !
#define PERI_CURRENT_MIN 200          // Courant minimum attendu (en milliamperes) sur le fil perimetrique. En dessous, on suppose que le fil est coupe/defectueux.

// --- Affectation des broches (quelle broche physique de l'ESP32 fait quoi) ---
#define I2C_SDA 21                    // Broche de donnees I2C, partagee par les deux capteurs INA226 et l'ecran OLED
#define I2C_SCL 22                    // Broche d'horloge I2C, partagee par les deux capteurs INA226 et l'ecran OLED
#define pinIN1 12                     // M1_IN1  ESP32 GPIO12       ( relier cette broche a L298N-IN1)  - controle le pont en H de l'emetteur A
#define pinIN2 13                     // M1_IN2  ESP32 GPIO13       ( relier cette broche a L298N-IN2)  - controle le pont en H de l'emetteur A
#define pinEnableA 23                 // ENA    ESP32 GPIO23         (relier cette broche a L298N-ENA)  - active/desactive le pont en H de l'emetteur A
#define pinIN3 14                     // M1_IN3  ESP32 GPIO14       ( relier cette broche a L298N-IN3)  - controle le pont en H de l'emetteur B
#define pinIN4 18                     // M1_IN4  ESP32 GPIO18       ( relier cette broche a L298N-IN4)  - controle le pont en H de l'emetteur B
#define pinEnableB 19                 // ENB    ESP32 GPIO19        (relier cette broche a L298N-ENA)   - active/desactive le pont en H de l'emetteur B
//#define pinDoorOpen 34              // Non utilise (interrupteur magnetique)
//#define pinDoorClose 35             // Non utilise (interrupteur magnetique)
//#define pinLDR 32                   // Non utilise (capteur de lumiere)

// Remarque pour debutants : un "pont en H" (ici : un module driver L298N)
// est un petit circuit capable de faire circuler le courant dans le fil
// perimetrique dans les deux sens (avant/arriere) sous le controle de
// l'ESP32 - c'est exactement ce qu'il faut pour creer le motif de signal
// alterne +1/-1.

WiFiClient espClient;                 // L'objet de connexion reseau sous-jacent, partage avec le client MQTT

//********************* Identifiants WhatsApp **********************************
// Les valeurs de depart sont volontairement vides. Elles sont saisies sur la
// page de reglages dans le navigateur puis enregistrees dans config.json
// (voir processor(), saveSettingsToLittleFS() et la section "/save" plus bas).
// Important : ces variables doivent etre declarees ICI, tout en haut - pas
// plus bas sous "Code for WhatsApp" - car processor() et
// saveSettingsToLittleFS() (toutes deux plus haut dans le code) les utilisent
// deja. En C++/Arduino, une variable doit etre declaree AVANT sa premiere
// utilisation, sinon l'erreur de compilation "was not declared in this
// scope" apparait.
String mobile_number = "";            // Votre numero de telephone (ex. 4917012345678), utilise seulement si WhatsApp_messages est active
String api_key = "";                  // Votre cle API Whatabot, utilisee seulement si WhatsApp_messages est active

//********************* MQTT Settings **********************************
// MQTT est un protocole de messagerie leger souvent utilise en domotique
// (par ex. avec Home Assistant ou Node-RED) pour publier les valeurs des capteurs.
// Important : ces valeurs ne sont plus des constantes fixes (const) - ce
// sont maintenant des variables normales (String/int), afin de pouvoir etre
// modifiees sur la page de reglages dans le navigateur et enregistrees de
// facon permanente dans config.json (voir saveSettingsToLittleFS() et la
// section "/save" plus bas). Les valeurs ici ne sont que les valeurs de
// depart (utilisees jusqu'a ce que vos propres reglages aient ete enregistres).
#ifdef MQTT
String mqtt_server = "192.168.178.2";       // Adresse de votre broker MQTT, ex. "broker.hivemq.com" ou une IP locale
int mqtt_port = 1883;                       // Port MQTT standard (non chiffre)
String clientID = "ESP32Sender";            // Un nom unique utilise par cet appareil pour s'identifier aupres du broker
String MQTT_user = "mqtt";                  // Valeur de depart (placeholder) - modifiee sur la page de reglages puis enregistree dans config.json
String MQTT_password = "mqtt";              // Valeur de depart (placeholder) - modifiee sur la page de reglages puis enregistree dans config.json
String subTopic = "teensysender/input";              // "Topic" (canal) MQTT sur lequel cet appareil ecoute les commandes entrantes
String pubTopic = "teensysender/chargecurrent";      // Topic utilise pour publier le courant de charge
String pubTopic2 = "teensysender/pericurrent";       // Topic utilise pour publier le courant du fil perimetrique
String pubTopic3 = "teensysender/chargevoltage";     // Topic utilise pour publier la tension de charge
PubSubClient MQTTclient(espClient);         // L'objet client MQTT lui-meme, construit sur la connexion WiFi
long lastMsg = 0;                           // (variable residuelle actuellement inutilisee)
#endif

//********************* WiFi Settings **********************************
WiFiServer server(80);                       // Un serveur web simple qui ecoute sur le port 80 (le port HTTP standard)
unsigned long lastWiFiCheckMillis = 0;       // Memorise le dernier moment ou la connexion WiFi a ete verifiee (voir keepWiFiAlive())
const unsigned long wifiCheckInterval = 15000; // Frequence de verification de la connexion WiFi, en millisecondes (15 secondes)

//********************* INA226 Settings **********************************
// Le INA226 est une petite puce capteur qui mesure la tension et le courant
// avec une grande precision. Ce projet en utilise DEUX : un pour surveiller
// le fil perimetrique, et un pour surveiller le courant de charge de la batterie.
INA226_WE InaPeri = INA226_WE(0x40);                    // 0x40 = l'adresse I2C du capteur sans le pont soude
float resistorPeri = 0.1;                               // Valeur (en Ohm) de la resistance de "shunt" utilisee pour la mesure du courant. Pour 10mOhm essayez 0.02, pour 100mOhm utilisez 0.1.
float rangePeri = 0.8;                                  // Plage de mesure attendue en Amperes. Pour une resistance de 10mOhm essayez 8.0 ou 4.0 - pour 100mOhm utilisez 0.8.

INA226_WE InaCharge = INA226_WE(0x44);                  // 0x44 = une adresse I2C differente, obtenue en soudant le "pont" entre les pastilles A1 et VSS
float resistorCharge = 0.02;
float rangeCharge = 4.0;


//********************* Other **********************************
// Variables globales : ces valeurs sont partagees dans tout le programme et
// peuvent changer pendant l'execution (contrairement aux constantes "#define" ci-dessus).
bool WORKING_TIMEOUT = 0;               // Passe a 1 une fois que la securite (WORKING_TIMEOUT_MINS) s'est declenchee
bool mowerIsWorking = 0;                // Indique si la tondeuse est actuellement supposee etre en train de tondre (utilise pour les messages WhatsApp)
byte sigCodeInUse = 1;                  // Quel code de signal (0-4) est actuellement actif. 1 est le sigcode ardumower d'origine.
int sigDuration = 104;                  // Nombre de microsecondes pendant lesquelles chaque "bit" du signal dure (50 est aussi courant). Plus petit = signal plus rapide.
int8_t sigcode_norm[128];               // Le code de signal actuellement transmis par l'ISR (copie ici depuis sigcode0..sigcode4)
int sigcode_size;                       // Combien d'elements de sigcode_norm[] sont reellement utilises
hw_timer_t* timer = NULL;               // Handle vers le timer materiel de l'ESP32 qui pilote la sortie du signal
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;  // Un petit verrou qui protege l'interruption contre le fait d'etre elle-meme interrompue
volatile int stepA = 0;                 // Position actuelle de l'emetteur A dans le code de signal (compteur propre, independant de B).
                                         // "volatile" indique au compilateur que cette valeur peut changer a tout moment depuis une interruption.
volatile int stepB = 0;                 // Position actuelle de l'emetteur B dans le code de signal (compteur propre, independant de A)
boolean enableSenderA = false;          // L'emetteur A (boucle de fil A) transmet-il actuellement ? Demarre sur OFF pour un demarrage sur du systeme.
boolean enableSenderB = false;          // L'emetteur B (boucle de fil B) transmet-il actuellement ? Demarre sur OFF pour un demarrage sur du systeme.
int timeSeconds = 0;                    // Petit compteur de secondes, utilise pour construire des minutes entieres pour le minuteur de travail
unsigned long nextTimeControl = 0;      // Horodatage (millis()) du prochain bloc "toutes les 10 secondes" dans loop()
unsigned long nextTimeSec = 0;          // Horodatage (millis()) du prochain bloc "chaque seconde" dans loop()
int workTimeMins = 0;                   // Combien de minutes la tondeuse tond depuis qu'elle a quitte la station
int workTimeChargeMins = 0;             // Combien de minutes la tondeuse charge dans la session de charge actuelle
int lastChargeMins = 0;                 // Combien de minutes la tondeuse a charge lors de sa derniere session de charge
float PeriCurrent = 0.0;                // Courant mesure sur le fil perimetrique, en mA
float PeriBusVoltage = 0.0;             // Tension mesuree sur le fil perimetrique, en V
float PeriShuntVoltage = 0.0;           // Minuscule chute de tension aux bornes de la resistance de mesure du perimetre, en mV
float ChargeCurrent = 0.0;              // Courant mesure sur les contacts de charge, en mA
float ChargeCurrentPrint = 0.0;         // Identique a ChargeCurrent, mais ramene a 0 en dessous de ChargeThreshold (plus lisible a l'affichage)
float ChargeBusVoltage = 0.0;           // Tension mesuree sur les contacts de charge, en V
float ChargeShuntVoltage = 0.0;         // Minuscule chute de tension aux bornes de la resistance de mesure de charge, en mV
bool shouldSaveConfig = false;          // Indicateur positionne par WiFiManager quand de nouveaux reglages WiFi doivent etre enregistres
bool wm_nonblocking = false;            // Si vrai, le portail de configuration de WiFiManager ne bloquerait pas loop() (actuellement inutilise - le portail bloque toujours)
//String AutoStartSignalPrint;                            // (reliquat inutilise, conserve pour reference) pour la partie web
//String linktext;                                        // (reliquat inutilise, conserve pour reference) pour la partie web
//String enableSenderAprint;                              // (reliquat inutilise, conserve pour reference) pour la partie web
//String enableSenderBprint;                              // (reliquat inutilise, conserve pour reference) pour la partie web


/*
  Si la tondeuse est a la station et completement chargee, le courant devrait
  etre compris entre PeriOnOffThreshold(3mA) et ChargeThreshold(10mA).

  Si le perimetre demarre alors que la tondeuse est a la station, mettez
  ChargeThreshold a 0. Vous pourrez ainsi voir la valeur d'origine de
  ChargeCurrent sur http://Votre-IP
  Si la tondeuse est dehors, ChargeCurrent devrait etre a 0.
*/
float ChargeThreshold = 10.0;               // en mA. Si ChargeCurrent est en dessous de cette valeur, l'ecran affiche "0mA" 
float PeriOnOffThreshold = 1.5;             // Si ChargeCurrent est en dessous de cette valeur, la boucle perimetrique se met a fonctionner


//*********************  Sigcode list *********************************************
// Remarque pour debutants : ces tableaux sont le "langage" reellement
// transmis sur le fil perimetrique. Chaque tableau est une suite de valeurs
// +1/-1 ; l'ISR (onTimer(), plus bas) les envoie les unes apres les autres,
// encore et encore, en inversant la polarite du fil a chaque valeur. Le
// capteur embarque de la tondeuse reconnait le motif et peut determiner A
// QUELLE zone (0-4) elle appartient actuellement - c'est ce qui rend
// possibles les jardins multi-zones. Doit etre un multiple de 2 !
// Plus d'infos : http://grauonline.de/alexwww/ardumower/filter/filter.html
// C'est le signal "pseudonoise4_pw" (tel qu'utilise par l'emetteur).

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
// Tout ce qui est au-dessus de cette ligne concerne des reglages que vous pourriez vouloir modifier.
// Tout ce qui est en dessous de cette ligne est la logique du programme elle-meme.


//********************* Functions **********************************
// Remarque pour debutants : une "fonction" est un bloc de code nomme et
// reutilisable. Plutot que de reecrire sans cesse les memes instructions,
// on les ecrit une fois dans une fonction, puis on "appelle" cette fonction
// par son nom chaque fois qu'on en a besoin.

// Remplace les espaces reserves (comme "%ChargeVoltage%") dans le texte HTML
// d'une page web par les valeurs reelles et actuelles des capteurs/de l'etat,
// juste avant d'envoyer la page au navigateur. C'est ainsi qu'index.html et
// settings.html affichent des donnees en direct.
// Le parametre "justSaved" controle l'espace reserve %SAVE_NOTICE% (voir plus
// bas) : s'il vaut "true", un court message "Reglages enregistres !"
// apparait sur la page de reglages. Le parametre est optionnel (par defaut :
// false), pour que les appels existants a processor(html) sans second
// argument continuent de fonctionner.
String processor(String html, bool justSaved = false) {
    // 1. Valeurs numeriques/texte simples : on cherche et remplace le texte de l'espace reserve
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

    // 2. Etat dynamique (emetteur A) - determine la classe CSS et le texte ON/OFF affiches dans le navigateur
    if (enableSenderA) { // On suppose que c'est le controle d'etat
        html.replace("%State_Class_A%", "label on");   // Nom de classe CSS - NE PAS traduire, doit correspondre a td.on{...} dans index.html !
        html.replace("%State_Text_A%", "on");
    } else {
        html.replace("%State_Class_A%", "label off");  // Nom de classe CSS - NE PAS traduire, doit correspondre a td.off{...} dans index.html !
        html.replace("%State_Text_A%", "off");
    }

    // 3. Etat dynamique (emetteur B)
    if (enableSenderB) {
        html.replace("%State_Class_B%", "label on");
        html.replace("%State_Text_B%", "on");
    } else {
        html.replace("%State_Class_B%", "label off");
        html.replace("%State_Text_B%", "off");
    }

    // 4. Mode auto (base sur la variable AUTO_START_SIGNAL)
    if (AUTO_START_SIGNAL) {
        html.replace("%State_Class_Auto%", "label on");
        html.replace("%State_Text_Auto%", "on");
    } else {
        html.replace("%State_Class_Auto%", "label off");
        html.replace("%State_Text_Auto%", "off");
    }

    // 5. Bloc reglages MQTT : inclus seulement si MQTT est active du tout
    // (#ifdef MQTT). Si MQTT est desactive, l'espace reserve est simplement
    // remplace par une chaine vide - aucune section MQTT n'apparait sur la page.
    #ifdef MQTT
      String mqttBlock = "";
      mqttBlock += "<h3 style='color:#1a3c34;margin-top:25px;'>Reglages MQTT</h3>";
      mqttBlock += "<label><b>Adresse du broker (IP ou nom d'hote) :</b></label>";
      mqttBlock += "<input type='text' name='mqttServer' value='" + mqtt_server + "'><br />";
      mqttBlock += "<label><b>Port :</b></label>";
      mqttBlock += "<input type='text' name='mqttPort' value='" + String(mqtt_port) + "'><br />";
      mqttBlock += "<label><b>ID client :</b></label>";
      mqttBlock += "<input type='text' name='mqttClientID' value='" + clientID + "'><br />";
      mqttBlock += "<label><b>Nom d'utilisateur :</b></label>";
      mqttBlock += "<input type='text' name='mqttUser' value='" + MQTT_user + "'><br />";
      mqttBlock += "<label><b>Mot de passe :</b></label>";
      mqttBlock += "<input type='password' name='mqttPassword' value='" + MQTT_password + "'><br />";
      mqttBlock += "<label><b>Topic d'abonnement (subTopic) :</b></label>";
      mqttBlock += "<input type='text' name='mqttSubTopic' value='" + subTopic + "'><br />";
      mqttBlock += "<label><b>Topic de publication - courant de charge :</b></label>";
      mqttBlock += "<input type='text' name='mqttPubTopic' value='" + pubTopic + "'><br />";
      mqttBlock += "<label><b>Topic de publication - courant perimetrique :</b></label>";
      mqttBlock += "<input type='text' name='mqttPubTopic2' value='" + pubTopic2 + "'><br />";
      mqttBlock += "<label><b>Topic de publication - tension de charge :</b></label>";
      mqttBlock += "<input type='text' name='mqttPubTopic3' value='" + pubTopic3 + "'><br />";
      html.replace("%MQTT_SETTINGS_BLOCK%", mqttBlock);
    #else
      html.replace("%MQTT_SETTINGS_BLOCK%", "");
    #endif

    // 6. Bloc reglages WhatsApp : inclus seulement si WhatsApp est active du
    // tout (#ifdef WhatsApp_messages). Meme principe que MQTT ci-dessus.
    #ifdef WhatsApp_messages
      String waBlock = "";
      waBlock += "<h3 style='color:#1a3c34;margin-top:25px;'>Reglages WhatsApp</h3>";
      waBlock += "<label><b>Numero de telephone (ex. 4917012345678) :</b></label>";
      waBlock += "<input type='text' name='waPhone' value='" + mobile_number + "'><br />";
      waBlock += "<label><b>Cle API Whatabot :</b></label>";
      waBlock += "<input type='password' name='waApiKey' value='" + api_key + "'><br />";
      html.replace("%WHATSAPP_SETTINGS_BLOCK%", waBlock);
    #else
      html.replace("%WHATSAPP_SETTINGS_BLOCK%", "");
    #endif

    // 7. Message de confirmation : affiche seulement si on vient d'arriver
    // ici via la redirection depuis "/save" (voir le parametre justSaved
    // ci-dessus). Cela donne a l'utilisateur un signal clair et visible que
    // ses modifications ont ete appliquees - plutot que la page se recharge
    // simplement sans aucun retour.
    if (justSaved) {
      html.replace("%SAVE_NOTICE%", "<div style='background:#c8e6c9;color:#1a3c34;padding:10px;border-radius:6px;margin-bottom:15px;text-align:center;'>✅ Reglages enregistres !</div>");
    } else {
      html.replace("%SAVE_NOTICE%", "");
    }

    return html;
}


// --- Fonctions utilitaires pour les codes de signal et l'analyse d'URL ---

// Convertit un tableau de code de signal (comme sigcode0) en une simple
// chaine de texte separee par des virgules (ex. "1,-1,1,-1"), pour pouvoir
// l'afficher/la modifier sur la page de reglages.
String sigcodeToString(int8_t* arr, int size) {
  String res = "";
  for (int i = 0; i < size; i++) {
    res += String(arr[i]);
    if (i < size - 1) res += ",";
  }
  return res;
}

// L'inverse de sigcodeToString() : lit une chaine de texte separee par des
// virgules (telle que saisie/modifiee par l'utilisateur sur la page de
// reglages, ou chargee depuis config.json) et la place dans un tableau de
// code de signal. N'accepte que 1 et -1 comme valeurs valides, pour proteger
// le tableau contre des donnees invalides.
void parseSigcode(String input, int8_t* arr, int& size) {
  int tempSize = 0;
  int start = 0;
  // Remarque : un decodage d'URL (ex. "%2C" -> ",") n'est plus necessaire
  // ici - c'est maintenant gere de facon centrale par extractParam() (voir
  // urlDecode()) avant que la valeur n'arrive ici. Les valeurs venant de
  // config.json n'ont jamais ete encodees en URL de toute facon.

  int idx = input.indexOf(',');

  while (idx != -1 && tempSize < 128) {
    int val = input.substring(start, idx).toInt();

    // N'accepte que 1 ou -1 exactement - tout le reste est silencieusement ignore
    if (val == 1 || val == -1) {
      arr[tempSize++] = (int8_t)val;
    }

    start = idx + 1;
    idx = input.indexOf(',', start);
  }

  // Verifie la toute derniere valeur apres la derniere virgule (la boucle ci-dessus s'arrete un cran trop tot)
  if (start < input.length() && tempSize < 128) {
    int val = input.substring(start).toInt();
    if (val == 1 || val == -1) {
      arr[tempSize++] = (int8_t)val;
    }
  }

  size = tempSize; // Indique combien de valeurs 1/-1 valides ont reellement ete trouvees
}


// Annule l'encodage URL que le navigateur applique automatiquement aux
// valeurs d'un formulaire lors de l'envoi en GET. Les navigateurs
// transforment pratiquement tout caractere "special" en "%XX" (les deux
// chiffres XX sont le code du caractere en hexadecimal) - une barre oblique
// "/" devient "%2F", un deux-points ":" devient "%3A", un espace devient
// "+". Sans cette fonction, des topics MQTT comme "teensysender/input"
// arriveraient sous la forme "teensysender%2Finput" et seraient enregistres
// tels quels (faux !).
String urlDecode(String input) {
  String decoded = "";
  char hexBuffer[3] = "00";   // Tampon pour les 2 chiffres hexadecimaux apres un "%"
  unsigned int len = input.length();
  unsigned int i = 0;
  while (i < len) {
    char c = input.charAt(i);
    if (c == '+') {
      // Un "+" dans une chaine de requete represente un espace
      decoded += ' ';
      i++;
    } else if (c == '%' && i + 2 < len) {
      // "%XX" -> le transformer en l'unique vrai caractere de code XX (hexadecimal)
      hexBuffer[0] = input.charAt(i + 1);
      hexBuffer[1] = input.charAt(i + 2);
      char realChar = (char) strtol(hexBuffer, NULL, 16);
      decoded += realChar;
      i += 3;
    } else {
      // Un caractere tout a fait normal, on le garde inchange
      decoded += c;
      i++;
    }
  }
  return decoded;
}


// Cherche "nomParam=" dans une chaine brute de requete HTTP/URL et renvoie
// le texte qui suit immediatement, jusqu'au prochain "&" ou espace. C'est
// ainsi qu'on lit les valeurs envoyees via le formulaire web de reglages
// (ex. .../save?peri=1.5). Le texte renvoye est automatiquement decode avec
// urlDecode(), pour que par ex. "/" (envoye comme "%2F") revienne sous forme
// d'une veritable barre oblique.
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


// Enregistre les reglages actuels (PeriOnOffThreshold, les 5 codes de
// signal, et - si actives - les reglages MQTT et WhatsApp) sous forme de
// fichier texte JSON dans le stockage flash integre de l'ESP32 (LittleFS),
// afin qu'ils survivent a un redemarrage/une coupure de courant. Appelez
// cette fonction manuellement chaque fois qu'un reglage change.
void saveSettingsToLittleFS() {
  DynamicJsonDocument json(2048); // Tampon plus grand maintenant, car les reglages MQTT et WhatsApp sont aussi stockes
  json["Peri"] = PeriOnOffThreshold;
  json["s0"] = sigcodeToString(sigcode0, sigcode0_size);
  json["s1"] = sigcodeToString(sigcode1, sigcode1_size);
  json["s2"] = sigcodeToString(sigcode2, sigcode2_size);
  json["s3"] = sigcodeToString(sigcode3, sigcode3_size);
  json["s4"] = sigcodeToString(sigcode4, sigcode4_size);

  // N'enregistre les reglages MQTT que si MQTT est reellement compile - ainsi
  // aucun champ "mort" ne se retrouve dans le fichier quand la fonctionnalite n'est pas utilisee.
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

  // N'enregistre les reglages WhatsApp que si WhatsApp est reellement compile
  #ifdef WhatsApp_messages
    json["waPhone"] = mobile_number;
    json["waApiKey"] = api_key;
  #endif

  File configFile = LittleFS.open("/config.json", "w");
  serializeJson(json, configFile);
  configFile.close();
  if(debug) Serial.println("Reglages enregistres dans la memoire flash.");
}


// Efface les identifiants WiFi ET le fichier config.json enregistre, puis
// redemarre l'ESP32 dans un etat completement neuf, "sorti d'usine".
// Declenche depuis la page web de reglages (bouton "Reinitialisation d'usine").
void factoryReset() {
  if(debug) Serial.println("REINITIALISATION D'USINE ! Suppression du WiFi et de la config...");

  // 1. Effacer les identifiants WiFi enregistres (force le portail de configuration WiFiManager au prochain demarrage)
  wm.resetSettings();

  // 2. Supprimer uniquement le fichier de configuration
  if (LittleFS.exists("/config.json")) {
    if (LittleFS.remove("/config.json")) {
      if(debug) Serial.println("config.json supprime avec succes.");
    } else {
      if(debug) Serial.println("Erreur lors de la suppression de config.json !");
    }
  } else {
    if(debug) Serial.println("Aucun config.json trouve a supprimer.");
  }

  // 3. Optionnel : restaurer les valeurs par defaut en RAM si necessaire (non utilise actuellement)
  // loadSettings(); // s'il existe quelque part une fonction "charger les valeurs par defaut"

  if(debug) Serial.println("Redemarrage...");
  delay(1000);
  ESP.restart();
}


//********************* Code for WhatsApp **********************************
// Le numero de telephone et la cle API sont declares tout en haut du fichier
// (voir "Identifiants WhatsApp" plus haut) - c'est aussi la que la raison est expliquee.

// Une "file d'attente" (queue) est une boite aux lettres thread-safe : sendWhatsappMessage()
// ci-dessous y depose un message et retourne immediatement, tandis qu'une tache separee en
// arriere-plan (whatsappTask(), demarree dans setup()) recupere les messages et les envoie
// reellement. Ainsi, la lente requete HTTPS vers l'API WhatsApp ne bloque jamais la boucle
// principale loop() (serveur web, MQTT, minuteurs, ...).
QueueHandle_t whatsappQueue = NULL;                     // Creee dans setup()
const long interval = 10000;                            // Pause minimale entre deux messages WhatsApp, pour eviter l'erreur HTTP 429 "Too Many Requests"

// Appelez cette fonction n'importe ou dans le code pour envoyer un message WhatsApp. Elle ne
// fait que mettre le message en file d'attente - l'envoi reel se produit dans whatsappTask().
void sendWhatsappMessage(String message){
  #ifdef WhatsApp_messages
    if (whatsappQueue != NULL) {
      // Les files d'attente FreeRTOS ne peuvent pas stocker directement des objets "String"
      // Arduino, seulement de simples tableaux de caracteres de style C ; on copie donc
      // d'abord le message dans un tampon de taille fixe.
      char msg_buffer[256];
      message.toCharArray(msg_buffer, sizeof(msg_buffer));
      if (xQueueSend(whatsappQueue, &msg_buffer, 0) != pdPASS) {
        // La file d'attente etait pleine (plus de 5 messages deja en attente) - message abandonne.
        Serial.println("La file d'attente WhatsApp est pleine, message abandonne.");
      }
    }
  #endif
}

// Tache en arriere-plan : s'execute indefiniment sur le coeur 1, totalement separee de loop().
// Elle dort (n'utilise aucun CPU) jusqu'a ce qu'un message apparaisse dans whatsappQueue,
// l'envoie, puis attend "interval" millisecondes avant de traiter le suivant, pour ne pas
// surcharger l'API.
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
          Serial.print("Message WhatsApp envoye avec succes : ");
          Serial.println(message);
        } else {
          Serial.println("Erreur lors de l'envoi du message");
          Serial.print("Code de reponse HTTP : ");
          Serial.println(http_response_code);
        }

        http.end();
      } else {
        Serial.println("Tache WhatsApp : le WiFi n'est pas connecte. Message non envoye.");
      }

      vTaskDelay(pdMS_TO_TICKS(interval));              // Pause avant de traiter le prochain message en attente
    }
  }
}
// Fin de : Code for WhatsApp


//********************* SIGNAL MANAGEMENT **********************************
// Cette fonction est une routine de service d'interruption (ISR) : le timer
// materiel configure dans setup() l'appelle automatiquement et tres
// precisement toutes les "sigDuration" microsecondes (ex. toutes les 104us),
// quoi que fasse la boucle principale loop() a ce moment-la. C'est ce qui
// permet de generer le signal du fil perimetrique (tableau "sigcode_norm")
// avec une synchronisation exacte.
// Etant une ISR, elle doit etre RAPIDE et ne doit pas appeler des choses
// comme Serial.print() ou delay() - c'est pourquoi elle ecrit directement
// dans les registres GPIO (GPIO.out_w1ts / GPIO.out_w1tc) au lieu d'utiliser
// le digitalWrite() normal, plus lent.
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);                    // Bloque brievement les autres interruptions pour que celle-ci ne soit pas perturbee en cours de route
  if (enableSenderA) {
    if (sigcode_norm[stepA] == 1) {
      GPIO.out_w1tc = (1 << pinIN1);                    // Met pinIN1 a LOW  (w1tc = "write 1 to clear")
      GPIO.out_w1ts = (1 << pinIN2);                    // Met pinIN2 a HIGH (w1ts = "write 1 to set")
    } else if (sigcode_norm[stepA] == -1) {
      GPIO.out_w1ts = (1 << pinIN1);                    // Met pinIN1 a HIGH
      GPIO.out_w1tc = (1 << pinIN2);                    // Met pinIN2 a LOW
    }
    stepA++;                                            // Fait avancer l'emetteur A au bit suivant du code de signal
    if (stepA == sigcode_size) {
      stepA = 0;                                        // Revient au debut une fois tout le code envoye
    }
  }
  if (enableSenderB) {
    if (sigcode_norm[stepB] == 1) {
      GPIO.out_w1tc = (1 << pinIN3);                    // Met pinIN3 a LOW
      GPIO.out_w1ts = (1 << pinIN4);                    // Met pinIN4 a HIGH
    } else if (sigcode_norm[stepB] == -1) {
      GPIO.out_w1ts = (1 << pinIN3);                    // Met pinIN3 a HIGH
      GPIO.out_w1tc = (1 << pinIN4);                    // Met pinIN4 a LOW
    }
    stepB++;                                            // Fait avancer l'emetteur B au bit suivant du code de signal (independant de stepA)
    if (stepB == sigcode_size) {
      stepB = 0;
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);                     // Reautorise les autres interruptions
}
// Fin de la gestion du signal


//********************* CHANGE AREA **********************************
// Charge un des tableaux sigcode0..sigcode4 dans sigcode_norm[], le tableau
// que l'interruption onTimer() envoie reellement bit par bit. C'est ainsi
// qu'on indique a la tondeuse de passer a une autre zone/un autre secteur (0-4).
void changeArea(byte areaInMowing) {
  stepA = 0;                                            // Remet les deux emetteurs au debut du (nouveau) code de signal
  stepB = 0;
  enableSenderA = false;                                // Desactive brievement les deux emetteurs pendant le changement de code de signal, par securite
  enableSenderB = false;
  #ifdef SerialOutput
    Serial.print("Changement vers la zone :");
    Serial.println(areaInMowing);
  #endif

  #ifdef Screen
    u8x8.clear();
    u8x8.setCursor(0,0);  
    u8x8.print("Zone:");
    u8x8.println(areaInMowing);
  #endif
  for (int uu = 0; uu < 128; uu++) {  // Vide d'abord tout le tampon (supprime les restes du code precedent)
    sigcode_norm[uu] = 0;
  }
  sigcode_size = 0;

  // Copie le code de signal choisi (et sa longueur) dans les tableaux "en direct" utilises par l'ISR
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
    Serial.print("Nouveau sigcode utilise  : ");
    Serial.println(sigCodeInUse);

    for (int uu = 0; uu <= (sigcode_size - 1); uu++) {
      Serial.print(sigcode_norm[uu]);
      Serial.print(",");
    }
    
    Serial.println();
    Serial.print("Nouvelle taille de sigcode  : ");
    Serial.println(sigcode_size);
  #endif

  #ifdef Screen
    u8x8.setCursor(0,2);
    u8x8.print("Sigcode actif:");
    u8x8.print(sigCodeInUse);
    Serial.println();
    Serial.print("Nouvelle taille de sigcode: ");
    Serial.println(sigcode_size);
    delay(2000);
  #endif
}
// FIN ChangeArea


//********************* STATICSCREENPARTS **********************************
// Dessine les parties de la mise en page de l'ecran OLED qui ne changent
// jamais (etiquettes comme "Temps de fonctionnement:", "Peri mA:", ...).
// Appelee toutes les 10 secondes depuis loop() - les valeurs numeriques
// reelles a cote de ces etiquettes sont mises a jour separement, plus
// souvent, ailleurs dans loop().
void StaticScreenParts() {
#ifdef Screen
  //ligne 0 : titre
  u8x8.setCursor(0,0);
  u8x8.inverse();
  u8x8.print(" ESP32 Sender  ");
  u8x8.noInverse();

  //ligne 1 : libre
  u8x8.clearLine(1);

  //ligne 2 : emetteur ON/OFF
  u8x8.clearLine(2);

  //ligne 3 : libre
  u8x8.clearLine(3);

  //ligne 4 : temps de fonctionnement
  u8x8.setCursor(0, 4);
  u8x8.print("Duree:");

  //ligne 5 : courant perimetrique
  u8x8.setCursor(0, 5);
  u8x8.print("Peri mA:");

  //ligne 6 : courant de charge
  u8x8.setCursor(0, 6);
  u8x8.print("Charge mA:");

//ligne 7 : zone
  u8x8.setCursor(0, 7);
  u8x8.print("Zone:");
#endif  
}
// FIN StaticScreenParts


//********************* SaveConfigCallback **********************************
// WiFiManager appelle automatiquement cette fonction (elle est enregistree
// plus bas comme "callback") chaque fois que l'utilisateur a saisi/modifie
// des reglages WiFi via le portail de configuration captif. On memorise
// simplement cela ici avec un indicateur ; l'enregistrement reel se produit plus tard dans setup().
void saveConfigCallback () {  // callback nous informant qu'il faut enregistrer la config
  if (debug) Serial.println("La config devrait etre enregistree");
  shouldSaveConfig = true;
}
// FIN SaveConfigCallback


//********************* MQTTinput **********************************
// Appelee automatiquement par la bibliotheque MQTT chaque fois qu'un message
// arrive sur un topic auquel on est abonne. Cela permet de controler la
// tondeuse a distance depuis n'importe quelle application compatible MQTT ou
// systeme domotique (Home Assistant, Node-RED, ...).
#ifdef MQTT
void MQTTinput(char* topic, byte* payload, unsigned int length) {

  Serial.print("Message recu sur le topic : [");
  Serial.print(topic);
  Serial.print("] ");

  // Lit les octets bruts du payload et les transforme en un String de texte normal
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();

  // traitement des commandes : compare le texte recu aux commandes connues
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
      Serial.println("Une commande MQTT incorrecte a ete recue.");
    }
  }

}   // FIN MQTTinput
#endif


//********************* Garder le WiFi actif **********************************
// Verifie, toutes les "wifiCheckInterval" millisecondes, si la connexion
// WiFi est toujours active - et sinon, tente de se reconnecter discretement
// en arriere-plan sans bloquer le reste du programme (loop() continue de
// s'executer normalement).
void keepWiFiAlive() {
  unsigned long currentMillis = millis();

  // Ne verifie reellement qu'une fois l'intervalle defini ecoule
  if (currentMillis - lastWiFiCheckMillis >= wifiCheckInterval) {
    lastWiFiCheckMillis = currentMillis;

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Connexion perdue ! Tentative de reconnexion en arriere-plan...");

      // Comme WiFiManager a deja enregistre les identifiants WiFi dans la
      // memoire flash de l'ESP32, un simple appel non bloquant a begin()
      // suffit ici :
      WiFi.begin(); 
    } else {
      Serial.print("[WiFi] Connecte. IP : ");
      Serial.println(WiFi.localIP());
    }
  }
}




//********************* SETUP **********************************
// setup() s'execute exactement UNE FOIS, juste apres la mise sous tension ou
// la reinitialisation de l'ESP32. Son role est de preparer tout ce dont le
// reste du programme aura besoin : demarrer la console serie, initialiser
// les capteurs/l'ecran, demarrer l'interruption du timer, se connecter au
// WiFi, demarrer le serveur web, etc.
void setup() {
  Serial.begin(115200);                           // Demarre la connexion serie USB (utilisee pour le journal de debogage dans le moniteur serie de votre ordinateur)
  Wire.begin(I2C_SDA, I2C_SCL);                   // Demarre le bus I2C (utilise pour communiquer avec les deux capteurs INA226 et l'ecran OLED)

  // *** FreeRTOS : creer la file d'attente WhatsApp et sa tache d'envoi en arriere-plan ***
  // (voir "Code for WhatsApp" plus haut pour les details)
  whatsappQueue = xQueueCreate(5, 256);           // Jusqu'a 5 messages en attente, 256 octets chacun
  xTaskCreatePinnedToCore(
    whatsappTask,                                 // Fonction contenant le code de la tache
    "WhatsappTask",                               // Nom de la tache (utilise uniquement par les outils de debogage)
    10000,                                        // Taille de la pile en octets (doit etre suffisamment grande pour HTTP/SSL)
    NULL,                                         // Parametres passes a la fonction de la tache (aucun necessaire ici)
    1,                                             // Priorite de la tache (un nombre plus petit signifie une priorite plus basse)
    NULL,                                         // Handle de la tache (non necessaire ici, donc NULL)
    1                                              // S'execute sur le coeur 1 (le coeur 0 est souvent occupe par le WiFi/Bluetooth)
  );

  InaPeri.init();                                 // Initialise le capteur INA226 utilise pour la mesure du fil perimetrique
  InaCharge.init();                               // Initialise le capteur INA226 utilise pour la mesure de charge

  #ifdef Screen
  u8x8.begin();                                   // Demarre l'ecran OLED
  u8x8.setFont(u8x8_font_5x8_f);                  // Choisit une police petite et lisible
  u8x8.clear();
  #endif
  timer = timerBegin(1000000);                    // Demarre un timer materiel qui tique 1 000 000 de fois par seconde (1 tick = 1 microseconde)
  timerAttachInterrupt(timer, &onTimer);           // Indique au timer d'appeler automatiquement onTimer() a chaque alarme
  timerAlarm(timer, sigDuration, true, 0);         // Regle l'alarme pour qu'elle se declenche toutes les "sigDuration" microsecondes, en boucle infinie
  pinMode(pinIN1, OUTPUT);                        // Configure toutes les broches de controle du driver en sorties numeriques (OUTPUT)
  pinMode(pinIN2, OUTPUT);
  pinMode(pinEnableA, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinEnableB, OUTPUT);
  pinMode(pinGreenLED, OUTPUT);                   // LED bicolore verte
  pinMode(pinRedLED, OUTPUT);                     // LED bicolore rouge
  digitalWrite(pinGreenLED, LOW);                 // S'assure que les deux LED sont eteintes au depart
  digitalWrite(pinRedLED, LOW);
  
  #ifdef SerialOutput
    Serial.println("START");
    Serial.print("ESP32 Sender ");
    Serial.println(VER);
  #endif
  
  changeArea(sigCodeInUse);                       // Charge le code de signal initialement selectionne dans le tableau de travail utilise par l'ISR
  if (enableSenderA) {
    digitalWrite(pinEnableA, HIGH);
  }
  if (enableSenderB) {
    digitalWrite(pinEnableB, HIGH);
  }

  //------------------------  Flash save parts  ----------------------------------------
  WiFi.mode(WIFI_STA);                             // Mode "Station" : se connecter a un reseau WiFi existant (plutot que d'en creer un soi-meme)
  if (LittleFS.begin(true)) {   // Monte le systeme de fichiers flash (true = le formater automatiquement s'il devient illisible)
    if (LittleFS.exists("/config.json")) {  // Si un fichier de reglages enregistre existe, le charger
      File configFile = LittleFS.open("/config.json", "r");   // ouvre le fichier en lecture
      if (configFile) {
        size_t size = configFile.size();  // Determine la taille du fichier, pour allouer un tampon suffisamment grand
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(2048); // Tampon plus grand maintenant, car les reglages MQTT et WhatsApp sont aussi stockes
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
          // --- Charge les reglages MQTT (seulement si MQTT est active du tout) ---
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
          // --- Charge les reglages WhatsApp (seulement si WhatsApp est active du tout) ---
          #ifdef WhatsApp_messages
            if (json["waPhone"]) mobile_number = json["waPhone"].as<String>();
            if (json["waApiKey"]) api_key = json["waApiKey"].as<String>();
          #endif
        } else {
          if (debug) Serial.println("echec du chargement de la config JSON");
        }
      }
    }
  } else {
    if (debug) Serial.println("echec du montage du systeme de fichiers");
  }   // fin de la lecture

  //------------------------ MQTT ------------------------------------------------
  // A appeler seulement ICI (apres avoir charge les reglages ci-dessus) avec
  // les valeurs reellement valides - ainsi les reglages MQTT enregistres sont
  // vraiment utilises et non ecrases par les valeurs de depart.
  #ifdef MQTT
  MQTTclient.setServer(mqtt_server.c_str(), mqtt_port);       // Definit l'adresse du broker et le port
  MQTTclient.setCallback(MQTTinput);                          // La fonction (MQTTinput) sera appelee automatiquement a chaque reception de donnees
  #endif

  wm.setSaveConfigCallback(saveConfigCallback);    // Enregistre saveConfigCallback() pour qu'elle soit appelee quand WiFiManager doit enregistrer des reglages

  // Aucune IP statique n'est definie ici, volontairement - le routeur
  // attribue l'adresse IP (et la passerelle/le sous-reseau) automatiquement
  // via DHCP a chaque connexion de l'ESP32.

  // Limite la duree pendant laquelle l'ESP32 essaie de se connecter a un WiFi connu avant d'abandonner (en secondes)
  wm.setConnectTimeout(15); 

  // Limite la duree pendant laquelle le WiFi de configuration (le "point d'acces" auquel on se connecte pour la configuration) reste actif (en secondes).
  // Si personne ne s'y connecte dans ce delai, il abandonne et loop() demarre quand meme (mode hors ligne).
  wm.setConfigPortalTimeout(60); 

  // Active la reconnexion automatique en arriere-plan (une fonctionnalite native de l'ESP32)
  WiFi.setAutoReconnect(true);

  if (!wm.autoConnect("MowerSender_AP","12345678")) { // Le mot de passe doit comporter au moins 8 caracteres.
    Serial.println("Connexion WiFi echouee ou delai expire. Demarrage en mode hors ligne...");
  } else {
    if (debug) Serial.println("connected :)");
  }
  
  if (shouldSaveConfig) {   
    saveSettingsToLittleFS();
  }

  sendWhatsappMessage(String("Le sender ESP32 est maintenant en ligne. Adresse IP : ") + WiFi.localIP().toString());          // Code for WhatsApp
  server.begin();                                  // Demarre le serveur web, maintenant pret a accepter les connexions du navigateur


  //------------------------  current sensor parts  ----------------------------------------
  #ifdef SerialOutput
    Serial.println("Mesure de la tension et du courant avec l'INA226 ...");
  #endif

  InaPeri.setAverage(INA226_AVERAGE_4);                          // Moyenne 4 echantillons par mesure, pour lisser le bruit
  InaPeri.setResistorRange(resistorPeri, rangePeri);
//  InaPeri.waitUntilConversionCompleted();
  InaCharge.setAverage(INA226_AVERAGE_4);
  InaCharge.setResistorRange(resistorCharge, rangeCharge);
//  InaCharge.waitUntilConversionCompleted();



  //------------------------  ArduinoOTA  ----------------------------------------
  // OTA = mises a jour "Over-The-Air" (sans fil) : permet de televerser un
  // nouveau firmware par WiFi, sans avoir besoin d'un cable USB branche.
 #ifdef OTAUpdates
 ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else
        type = "systeme de fichiers";

      // REMARQUE : en cas de mise a jour de LittleFS, c'est ici qu'il faudrait le demonter avec LittleFS.end()
      Serial.println("Debut de la mise a jour : " + type);
    })
    .onEnd([]() {
      Serial.println("\nTermine");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progression : %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Erreur[%u] : ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Echec de l'authentification");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Echec du demarrage");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Echec de la connexion");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Echec de la reception");
      else if (error == OTA_END_ERROR) Serial.println("Echec de la finalisation");
    });

  ArduinoOTA.begin();
  #endif
}
// FIN SETUP


//********************* LOOP **********************************
// loop() s'execute encore et encore, indefiniment, aussi vite que l'ESP32 en
// est capable, juste apres que setup() se termine. Important : la garder
// aussi legere/rapide que possible - tout le travail reellement critique en
// temps (la sortie du signal perimetrique) se deroule separement dans
// l'interruption onTimer(), PAS ici. loop() s'occupe de tout le reste :
// verification du WiFi, mises a jour OTA, lecture des capteurs, minuteurs,
// MQTT et distribution des pages web.
void loop() {

  keepWiFiAlive(); // Verifie la connexion WiFi a intervalles definis et tente de la restaurer si besoin.

  #ifdef MQTT
    // IMPORTANT : MQTTclient.loop() doit etre appelee regulierement pour que
    // la bibliotheque maintienne la connexion active ET traite les messages
    // entrants (appelle MQTTinput() si besoin). Cela manquait completement
    // avant - sans cet appel, les commandes MQTT n'arrivaient jamais, quel
    // que soit le topic abonne.
    MQTTclient.loop();
  #endif

  // Verifie les mises a jour logicielles sans fil
  #ifdef OTAUpdates
    ArduinoOTA.handle();
  #endif

  
  // --- BLOC MINUTEUR : s'execute toutes les 10 secondes ---
  if (millis() >= nextTimeControl) {
    // Definit l'heure de la prochaine execution (heure actuelle + 10 secondes)
    nextTimeControl = millis() + 10000;

    // Met a jour les etiquettes de texte statiques sur l'ecran OLED
    #ifdef Screen
      StaticScreenParts();
    #endif


    #ifdef MQTT
      // N'essaie qu'UNE SEULE FOIS par cycle de 10 secondes plutot que de boucler
      // indefiniment - si le broker est injoignable, une boucle while() sans fin ici
      // figerait tout l'ESP32 (serveur web, OTA, WiFi...) et pourrait declencher une
      // reinitialisation par le watchdog. Si cette tentative echoue, on reessaiera
      // simplement au prochain passage de ce bloc.
      if (!MQTTclient.connected()) {
        if (MQTTclient.connect(clientID.c_str(), MQTT_user.c_str(), MQTT_password.c_str())) {
          // Apres une nouvelle connexion, il faut se (re)abonner au topic
          // souhaite aupres du broker, sinon les commandes n'arrivent jamais
          // via MQTTinput() - cela manquait dans la version originale !
          MQTTclient.subscribe(subTopic.c_str());
        }
      }
      // Convertit le courant de charge en String 
      String MQTTmsgString = String(ChargeCurrent, 2); // float en String avec 2 decimales
      // Publie le courant de charge sur le broker MQTT
      MQTTclient.publish(pubTopic.c_str(), MQTTmsgString.c_str());
      
      // Convertit le courant de la boucle perimetrique en String
      String MQTTmsgString2 = String(PeriCurrent, 2); // float en String avec 2 decimales
      // Publie le courant perimetrique sur le broker MQTT
      MQTTclient.publish(pubTopic2.c_str(), MQTTmsgString2.c_str());
 
      // Convertit la tension de charge (float) en String pour MQTT
      String MQTTmsgString3 = String(ChargeBusVoltage, 2); // float en String avec 2 decimales
      // Publie le courant de charge sur le broker MQTT
      MQTTclient.publish(pubTopic3.c_str(), MQTTmsgString3.c_str());
    #endif
    // FIN MQTT

    // Lit la tension du capteur INA226 (Perimetre)
    PeriBusVoltage = InaPeri.getBusVoltage_V();
    // Lit la minuscule chute de tension aux bornes de la resistance de mesure
    PeriShuntVoltage = InaPeri.getShuntVoltage_mV();
    // Lit le courant reel circulant dans le fil perimetrique
    PeriCurrent = InaPeri.getCurrent_mA();

    // Soustrait la consommation de base de l'electronique (ESP32/drivers)
    PeriCurrent = PeriCurrent - 80.0;                        //le convertisseur DC/DC, l'ESP32, le L298N consomment entre 80 et 100 mA quand rien n'est active et qu'un point d'acces WiFi est trouve (a confirmer ????)

    // Si le courant est trop faible, le mettre simplement a 0 pour eviter les mesures "fantomes"
    if (PeriCurrent <= PERI_CURRENT_MIN) PeriCurrent = 0;

    // Si un emetteur est actif mais qu'aucun courant ne circule, le fil est probablement coupe
    if ((enableSenderA) && (PeriCurrent < PERI_CURRENT_MIN)) {
      workTimeMins = 0; // Reinitialise le minuteur de travail
      #ifdef Screen
        u8x8.setCursor(0, 5);
        u8x8.inverse();   // Met le texte en surbrillance
        u8x8.print(" Fil coupe !    ");
        u8x8.noInverse();
      #endif
      #ifdef SerialOutput
        Serial.println("FIL COUPE !!!");
      #endif
        
    } else {
      // Si tout va bien, affiche le courant sur l'ecran
      #ifdef Screen
        u8x8.setCursor(8, 5);
        u8x8.print("        ");  // Efface l'ancienne valeur
        u8x8.setCursor(10, 5);
        u8x8.print(PeriCurrent);
      #endif
      #ifdef SerialOutput
        Serial.print("Courant peri ");
        Serial.println(PeriCurrent);
        Serial.print("Tension peri ");
        Serial.println(PeriBusVoltage);
      #endif
    }

    // Coupure de securite : si la tondeuse travaille depuis trop longtemps (ex. 5 heures)
    if (workTimeMins >= WORKING_TIMEOUT_MINS && AUTO_START_SIGNAL == 1) {
      enableSenderA = false;  // Coupe la boucle A
      enableSenderB = false;  // Coupe la boucle B
       workTimeMins = 0;      // Reinitialise le minuteur
      WORKING_TIMEOUT = 1;    // Positionne l'indicateur de timeout
      digitalWrite(pinEnableA, LOW);
      
      // Force physiquement les broches a LOW pour arreter le driver L298N
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      digitalWrite(pinEnableB, LOW);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);

      // Alerte l'utilisateur via WhatsApp
      sendWhatsappMessage("TIMEOUT! Mower didn't come back home! Perimeterwire switched off!");          // Code for WhatsApp
      Serial.println("********************************   Timeout, la tondeuse n'est pas rentree, arret de l'emetteur  **********************************");
    }
  }

  // --- BLOC SECONDE : s'execute chaque seconde ---
  if (millis() >= nextTimeSec) {          // Le faire chaque seconde
    nextTimeSec = millis() + 1000;

    // Met a jour le temps de travail et le code de signal actuel sur l'ecran
    #ifdef Screen
      u8x8.setCursor(9, 4);
      u8x8.print("       ");
      u8x8.setCursor(10, 4);
      u8x8.print(workTimeMins);
    
      //ligne 7 : zone
      u8x8.setCursor(10, 7);
      u8x8.print("      ");
      u8x8.setCursor(10, 7);
      u8x8.print(sigCodeInUse);
    #endif
    #ifdef SerialOutput
      Serial.print("Zone:");
      Serial.println(sigCodeInUse);
    #endif

    // Lit le courant et la tension du capteur de charge
    ChargeBusVoltage = InaCharge.getBusVoltage_V();
    ChargeShuntVoltage = InaCharge.getShuntVoltage_mV();
    ChargeCurrent = InaCharge.getCurrent_mA();

    // Controle la LED d'etat : rouge si en charge, verte si en attente/prete
    if (ChargeCurrent > ChargeThreshold) {  //Juste pour la LED de charge
      digitalWrite(pinGreenLED, LOW);
      digitalWrite(pinRedLED, HIGH);  // En charge
    } else  {
      digitalWrite(pinRedLED, LOW);
      digitalWrite(pinGreenLED, HIGH);  // Prete
    }

    // Prepare la valeur du courant pour l'affichage (ignore le bruit en dessous du seuil)
    ChargeCurrentPrint = ChargeCurrent;
    if (ChargeCurrent < ChargeThreshold) ChargeCurrentPrint = 0;  // affiche 0 quand la tondeuse ne charge pas

    #ifdef Screen
      u8x8.setCursor(10, 6);
      u8x8.print("      ");
      u8x8.setCursor(10, 6);
      u8x8.print(ChargeCurrentPrint);
    #endif
    #ifdef SerialOutput
      Serial.print("Courant charge: ");
      Serial.println(ChargeCurrentPrint);
    Serial.print("Tension charge: ");
    Serial.println(ChargeBusVoltage);
    #endif

    // Si un courant plus eleve est detecte, la tondeuse est a la station
    if (ChargeCurrent > PeriOnOffThreshold && AUTO_START_SIGNAL == 1) {   // la tondeuse est a la station, dans mon test 410 mA sont consommes donc il est possible d'arreter l'emetteur - Une fois pleinement chargee, le courant est d'environ 4mA.
                                                // Garder cette valeur petite pour eviter une activation par le fil perimetrique avant que la tondeuse ne demarre.
      enableSenderA = false;  // Arrete le signal du fil (la tondeuse est a la maison)
      enableSenderB = false;

      // Si la tondeuse travaillait auparavant, elle vient de revenir
      if(mowerIsWorking == 1)  {
        mowerIsWorking = 0;
        WORKING_TIMEOUT = 0;
        sendWhatsappMessage("Mower is back at Home!");          // Code for WhatsApp
      }

      workTimeMins = 0; // Reinitialise le minuteur de travail
      // Coupe toutes les sorties du driver
      digitalWrite(pinEnableA, LOW);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      digitalWrite(pinEnableB, LOW);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);
      //delay(200);
    } else {

      if (AUTO_START_SIGNAL == 1 && !WORKING_TIMEOUT ) {
         // Si la tondeuse vient de quitter la station, la marquer comme en train de travailler
         if(mowerIsWorking == 0)  {
          mowerIsWorking = 1;
          sendWhatsappMessage("Mower is going to work!");          // Envoie un message WhatsApp
        }
        // Active le signal du fil pour que la tondeuse puisse voir la limite
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
  
    // Incremente le compteur de secondes
    timeSeconds++;
    if (((enableSenderA) || (enableSenderB)) && (timeSeconds >= 60)) {                    // Si l'emetteur est ON & 60 secondes se sont ecoulees
      if (workTimeMins < 1440)  {                                                         // evite le debordement
        workTimeMins++;                                                                   // compte une minute de plus
        timeSeconds = 0;                                                                  // remet les secondes a 0
        if (workTimeChargeMins > 0) {                                                     // Si workTimeCharge > 0 (le dernier etat etait CHARGE)
          lastChargeMins = workTimeChargeMins;                                            // sauvegarde ce temps dans lastChargeMins
          workTimeChargeMins = 0;                                                         // et le remet a 0
        }
      }
    } else  {
      if ((workTimeChargeMins < 1440) && (ChargeCurrent > ChargeThreshold) && (timeSeconds >= 60)) {   // Si ChargeCurrent est superieur a ChargeThreshold (la tondeuse est a la station et charge)
      workTimeMins = 0;                                                                   // Reinitialise WorktimeMins
        workTimeChargeMins++;                                                             // compte workTimeChargeMins en plus
        timeSeconds = 0;                                                                  // remet les secondes a 0
      }
    }
    
    // Affiche l'etat des emetteurs (quelle boucle est active)
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

  // --- BLOC SERVEUR WEB : traite les requetes du navigateur ---
  WiFiClient client = server.available(); // Verifie si quelqu'un a ouvert l'IP dans un navigateur
  if (client) {
    unsigned long currentTime = millis();
    unsigned long previousTime = currentTime;
    unsigned long timeoutTime = 500;  // Delai maximal d'attente des donnees
    String req; // Stocke la chaine de requete
    String currentLine;

    // Tant que le client est connecte et n'a pas depasse le delai
    while (client.connected() && currentTime - previousTime <= timeoutTime) {
      currentTime = millis();
      if (client.available()) {             // s'il y a des octets a lire depuis le client,
        char c = client.read();             // lit un octet, puis
        //Serial.write(c);                  // l'affiche sur le moniteur serie
        req += c;                           // et l'ajoute a notre chaine de requete
        if (c == '\n') {                    // si l'octet est un retour a la ligne
          // si la ligne actuelle est vide, on a recu deux retours a la ligne d'affilee.
          // c'est la fin de la requete HTTP du client, donc on envoie une reponse :
          if (currentLine.length() == 0) {
            // Les en-tetes HTTP commencent toujours par un code de reponse (ex. HTTP/1.1 200 OK)
            // et un type de contenu pour que le client sache ce qui arrive, puis une ligne vide :
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html; charset=utf-8"); // Important !
            client.println("Connection: close");
            client.println(); // La ligne vide marque la fin de l'en-tete

            // Memorise si l'un des cas suivants (ex. /save ou
            // /download-settings) a deja envoye sa propre reponse complete.
            // Sans cet indicateur, index.html/settings.html serait TOUJOURS
            // ajoute aussi a la toute fin - ce qui, par ex. apres
            // l'enregistrement, donnait deux pages HTML completes dans une
            // seule reponse, que le navigateur ne peut pas afficher
            // correctement (page blanche) !
            bool responseAlreadySent = false;

            // Determine quelle page/action a ete demandee
            if (req.indexOf("GET /toggleA") != -1) {
              // Logique pour basculer l'emetteur A
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
              // Logique pour basculer l'emetteur B
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
              saveSettingsToLittleFS(); // Enregistre les reglages !
              changeArea(sigCodeInUse);
            }

            else if (req.indexOf("GET /reset ") != -1) {
              factoryReset();
            }
            
            else if (req.indexOf("GET /save?") != -1) {
              // Lit les valeurs envoyees via le formulaire de reglages
              String pVal = extractParam(req, "peri");
              if (pVal != "") PeriOnOffThreshold = pVal.toFloat();
              
              String s0 = extractParam(req, "s0"); if (s0 != "") parseSigcode(s0, sigcode0, sigcode0_size);
              String s1 = extractParam(req, "s1"); if (s1 != "") parseSigcode(s1, sigcode1, sigcode1_size);
              String s2 = extractParam(req, "s2"); if (s2 != "") parseSigcode(s2, sigcode2, sigcode2_size);
              String s3 = extractParam(req, "s3"); if (s3 != "") parseSigcode(s3, sigcode3, sigcode3_size);
              String s4 = extractParam(req, "s4"); if (s4 != "") parseSigcode(s4, sigcode4, sigcode4_size);

              // --- Applique les reglages MQTT (seulement si MQTT est active) ---
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
                  // Si le topic d'abonnement a change, il doit etre
                  // reabonne a la prochaine connexion - cela se produit
                  // automatiquement dans loop() des que MQTTclient.connected()
                  // renvoie false la prochaine fois.
                  mqttSettingsChanged = true;
                }

                String newPubTopic = extractParam(req, "mqttPubTopic");
                if (newPubTopic != "") pubTopic = newPubTopic;

                String newPubTopic2 = extractParam(req, "mqttPubTopic2");
                if (newPubTopic2 != "") pubTopic2 = newPubTopic2;

                String newPubTopic3 = extractParam(req, "mqttPubTopic3");
                if (newPubTopic3 != "") pubTopic3 = newPubTopic3;

                // Si le serveur/port/identifiants/topic ont change, on
                // deconnecte la connexion existante - loop() se reconnectera
                // alors automatiquement avec les nouvelles valeurs et se
                // reabonnera au subTopic (eventuellement nouveau).
                if (mqttSettingsChanged) {
                  MQTTclient.disconnect();
                  MQTTclient.setServer(mqtt_server.c_str(), mqtt_port);
                }
              #endif

              // --- Applique les reglages WhatsApp (seulement si WhatsApp est active) ---
              #ifdef WhatsApp_messages
                String newWaPhone = extractParam(req, "waPhone");
                if (newWaPhone != "") mobile_number = newWaPhone;

                String newWaApiKey = extractParam(req, "waApiKey");
                if (newWaApiKey != "") api_key = newWaApiKey;
              #endif

              saveSettingsToLittleFS();
              changeArea(sigCodeInUse); // Applique immediatement le code actuel avec les nouvelles valeurs
              
              // Redirige le navigateur vers la page de reglages - avec
              // "?saved=1" dans l'URL, pour que la page de reglages sache
              // qu'elle doit afficher le message de confirmation
              // "Reglages enregistres !".
              client.println("<html><head><meta http-equiv='refresh' content='0; url=/settings?saved=1'></head><body>Enregistrement...</body></html>");
              responseAlreadySent = true;   // Empeche index.html d'etre egalement ajoute a la fin
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
                client.println("Content-Type: application/octet-stream"); // Force un telechargement au lieu d'un affichage
                client.println("Content-Disposition: attachment; filename=\"config.json\"");
                client.println("Connection: close");
                client.println();
        
                while (file.available()) {
                  client.write(file.read());
                }
                file.close();
              } else {
                client.println("HTTP/1.1 404 Not Found\n\nFichier introuvable.");
              }
              responseAlreadySent = true;   // Empeche index.html d'etre egalement ajoute a la fin
            }

            // LIT ET ENVOIE LE FICHIER HTML DEMANDE
            // N'execute ceci que si aucune reponse n'a deja ete envoyee
            // ci-dessus (ex. par /save ou /download-settings) - voir
            // responseAlreadySent plus haut.
            if (!responseAlreadySent) {
            String filename = "/index.html";
            if (req.indexOf("GET /settings") != -1) filename = "/settings.html";

            // Si c'etait la redirection apres l'enregistrement (l'URL
            // contient "saved=1"), le message de confirmation est affiche
            // sur la page de reglages.
            bool justSaved = (req.indexOf("saved=1") != -1);

            if (LittleFS.exists(filename)) {
              File file = LittleFS.open(filename, "r");
              String html = file.readString();
              file.close();
              client.print(processor(html, justSaved)); // C'est ici que les %espaces reserves% sont remplaces par de vraies valeurs !
            } else {
              client.println("Fichier introuvable !");
            }
            }   // FIN if (!responseAlreadySent)
            break;   // Cette requete est entierement traitee - la connexion va se fermer (quel que soit le cas ci-dessus qui s'est applique)
          } else {
            // La ligne n'etait PAS vide - c'est donc une ligne d'en-tete
            // normale (ex. "Host: ..." ou "User-Agent: ..."). On la remet a
            // zero pour la ligne suivante, pour que currentLine.length()==0
            // ne corresponde vraiment qu'a la vraie ligne vide (fin de tous les en-tetes).
            currentLine = "";
          }
        } else if (c != '\r') {
          // Un caractere normal (ni retour a la ligne, ni retour chariot) -
          // on l'ajoute a la ligne en cours de lecture.
          currentLine += c;
        }
      }
      // Vide le tampon et ferme la connexion
      //client.flush();
    }
  }  
}
//FIN LOOP
