/*
 * AOX4000 Oxygen Sensor - ESP32 WROOM Integration
 * 
 * Funcționalități:
 * - Citire concentrație oxigen de la senzor AOX4000 prin UART
 * - Buzzer: bip scurt la >23%, bip lung la >25%
 * - Buton pentru oprire buzzer
 * - Releu pentru ventilator activat la >25%
 * - Conexiune MQTT pentru transmisie date
 * - Calibrare senzor cu offset și factor de corecție
 * - Salvare calibrare în EEPROM (persistentă)
 * - Configurare WiFi prin aplicație mobilă (API REST)
 * - Access Point pentru configurare inițială
 * - Salvare credențiale WiFi în EEPROM
 * 
 * PINI CONEXIUNI:
 * ===============
 * SENZOR AOX4000:
 *   - TX (senzor) -> GPIO 16 (RX2 ESP32)
 *   - RX (senzor) -> GPIO 17 (TX2 ESP32)
 *   - VCC -> 5V sau 3.3V (verificați specificațiile)
 *   - GND -> GND
 * 
 * BUZZER:
 *   - Pin pozitiv -> GPIO 25
 *   - Pin negativ -> GND
 *   (Folosiți rezistență 220Ω dacă buzzerul nu are rezistență internă)
 * 
 * BUTON OPRIRE BUZZER:
 *   - Un capăt -> GPIO 26
 *   - Celălalt capăt -> GND
 *   (ESP32 are pull-up intern activat)
 * 
 * RELEU VENTILATOR:
 *   - IN (control) -> GPIO 27
 *   - VCC -> 5V (sau 3.3V dacă releul e 3.3V)
 *   - GND -> GND
 *   - NO (Normally Open) -> Ventilator (+)
 *   - COM -> Alimentare ventilator (+)
 *   - Ventilator (-) -> GND
 * 
 * CONFIGURARE:
 * - WiFi se configurează prin aplicația mobilă sau Access Point
 * - La prima pornire, ESP32 va crea Access Point "O2-Sentinel-Config"
 * - Conectează-te la AP și accesează http://192.168.4.1 pentru configurare
 * - Sau folosește aplicația mobilă pentru configurare automată
 * - Modificați adresa serverului MQTT în cod dacă e necesar
 * - Ajustați baud rate-ul senzorului dacă e necesar (default: 9600)
 * 
 * API REST PENTRU APLICAȚIA MOBILĂ:
 * ==================================
 * Toate endpoint-urile returnează JSON și suportă CORS:
 * 
 * GET  /api/status           - Status sistem (WiFi, MQTT, oxigen)
 * GET  /api/wifi/scan         - Scanează rețele WiFi disponibile
 * GET  /api/wifi/current      - Obține configurare WiFi curentă
 * POST /api/wifi/config       - Configurează WiFi nou (JSON: {"ssid":"...", "password":"..."})
 * POST /api/wifi/reset        - Resetează configurare WiFi
 * GET  /api/sensor/data       - Obține date senzor (oxigen, buzzer, relay)
 * 
 * Exemplu POST /api/wifi/config:
 * {
 *   "ssid": "NumeleRețelei",
 *   "password": "ParolaRețelei"
 * }
 * 
 * CALIBRARE SENZOR:
 * =================
 * Calibrarea se poate face prin comenzi MQTT sau prin Serial Monitor:
 * 
 * 1. Calibrare automată (recomandat):
 *    - Trimite "calibrate" sau "calibrate:20.9" la topic-ul commands
 *    - Senzorul va colecta 10 eșantioane și va calcula offset-ul automat
 *    - Asigură-te că senzorul este în aer normal (20.9% O2) când calibrezi
 * 
 * 2. Setare manuală offset:
 *    - Trimite "set_offset:7.4" (exemplu: pentru a corecta 12.6% -> 20.0%)
 *    - Offset = valoare_dorită - valoare_citită
 * 
 * 3. Setare manuală factor:
 *    - Trimite "set_factor:1.05" (exemplu: multiplică valoarea cu 1.05)
 * 
 * 4. Resetare calibrare:
 *    - Trimite "reset_calibration" pentru a reveni la valori default
 * 
 * 5. Status calibrare:
 *    - Trimite "calibration_status" pentru a vedea setările curente
 * 
 * Calibrarea este salvată automat în EEPROM și se încarcă la pornire.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include "config.h"

// ==================== CONFIGURARE WIFI ====================
// Credențiale WiFi (se încarcă din Preferences sau EEPROM pentru compatibilitate)
String wifiSSID = "";
String wifiPassword = "";

// Preferences pentru salvare permanentă WiFi (mai sigur decât EEPROM)
Preferences prefs;

// Access Point pentru configurare
const char* apSSID = "O2-Sentinel-Config";
const char* apPassword = "useru123";  // Parolă minimă 8 caractere (ESP32 cerință)

// Web Server pentru configurare
WebServer server(80);

// Flag pentru modul configurare
bool configMode = false;

// ==================== CONFIGURARE MQTT ====================
// Datele de conectare la serverul Adafruit
#define MQTT_SERVER      "io.adafruit.com"
#define MQTT_PORT        1883
#define MQTT_USER        "mrclaudiu"
#define MQTT_KEY         "8d5e30a51abe485b8f0af6fdf6032c19"

// Topic-urile (trebuie să conțină username-ul tău)
// Exemplu: "mrclaudiu/feeds/oxigen"
#define MQTT_TOPIC_DATA     "mrclaudiu/feeds/oxigen" 
#define MQTT_TOPIC_STATUS   "mrclaudiu/feeds/status"
#define MQTT_TOPIC_COMMANDS "mrclaudiu/feeds/comands"  // ATENȚIE: "comands" cu un singur "m" (conform Adafruit)
#define MQTT_TOPIC_CONFIG   "mrclaudiu/feeds/config"   // Topic pentru configurare WiFi prin MQTT
#define MQTT_TOPIC_ESP_CODE "mrclaudiu/feeds/esp-code" // Topic pentru OTA update (URL către .bin)
#define MQTT_CLIENT_ID      "ESP32_AOX4000"
// ==================== VARIABILE GLOBALE ====================
WiFiClient espClient;
PubSubClient client(espClient);

float oxygenLevel = 0.0;
bool buzzerEnabled = true;
bool lastButtonState = HIGH;
bool relayState = false;
unsigned long lastBuzzerTime = 0;
unsigned long lastMQTTPublish = 0;
unsigned long lastSensorRead = 0;
unsigned long lastSensorDataReceived = 0;  // Timestamp ultimei date primite
unsigned long lastConfigCheck = 0;  // Timestamp ultimei verificări configurare WiFi
String lastProcessedConfig = "";  // Hash/ID al ultimei configurații procesate
String lastProcessedCommandId = "";  // ID-ul ultimului mesaj procesat din feed-ul "comands"
unsigned long lastOTACheck = 0;  // Timestamp ultimei verificări OTA update
String lastProcessedOTAUrl = "";  // URL-ul ultimului update procesat (pentru a evita loop-uri)

// Flag-uri pentru control manual (prin MQTT sau API)
bool manualBuzzer = false;  // Dacă e true, buzzerul este controlat manual și nu răspunde la senzor
bool manualFan = false;     // Dacă e true, ventilatorul este controlat manual și nu răspunde la senzor
bool buzzerMuted = false;   // Flag pentru mute activat prin buton extern

// Buffer pentru datele de la senzor
String sensorBuffer = "";

// ==================== DECLARAȚII FORWARD ====================
void loadCalibration();
void publishMQTTData();
void publishMQTTStatus(String status);
void collectCalibrationSample(float value);
void finishCalibration();
void processWiFiConfig(String configJson, String messageId);
void checkOTAUpdate();
bool performOTAUpdate(String firmwareUrl);

// ==================== CALIBRARE SENZOR ====================
// Offset și factor de corecție pentru calibrare
float calibrationOffset = 0.0;      // Offset în % (ex: +7.4 pentru a corecta 12.6 -> 20.0)
float calibrationFactor = 1.0;       // Factor de multiplicare (default: 1.0 = fără corecție)
float referenceValue = 20.9;         // Valoare de referință pentru calibrare (aer normal = 20.9%)
bool calibrationActive = false;       // Flag pentru modul calibrare
unsigned long calibrationStartTime = 0;
const int CALIBRATION_SAMPLES = 10;   // Număr de eșantioane pentru calibrare
float calibrationSamples[CALIBRATION_SAMPLES];
int calibrationSampleCount = 0;

// Adrese EEPROM pentru salvare calibrare și WiFi
#define EEPROM_SIZE 256  // Mărit pentru WiFi credentials
#define EEPROM_OFFSET_ADDR 0
#define EEPROM_FACTOR_ADDR 4
#define EEPROM_MAGIC_ADDR 8  // Magic number pentru verificare validitate
#define EEPROM_MAGIC_VALUE 0xABCD

// Adrese EEPROM pentru WiFi
#define EEPROM_WIFI_MAGIC_ADDR 12
#define EEPROM_WIFI_MAGIC_VALUE 0x1234
#define EEPROM_WIFI_SSID_ADDR 16
#define EEPROM_WIFI_PASSWORD_ADDR 80  // SSID max 64 bytes, password la 80
#define MAX_SSID_LENGTH 64
#define MAX_PASSWORD_LENGTH 64

// ==================== SETUP ====================
void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);  // Serial pentru debug
  Serial2.begin(SENSOR_BAUD, SERIAL_CONFIG, SENSOR_RX, SENSOR_TX);  // Serial2 pentru senzor
  
  // Configurare pinuri
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTON_PIN, INPUT_PULLUP);  // Pull-up intern activat
  pinMode(RELAY_PIN, OUTPUT);
  
  // Inițializare stări
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  
  Serial.println("\n=== AOX4000 ESP32 System ===");
  Serial.println("Inițializare...");
  Serial.print("Pin buton (");
  Serial.print(BUTON_PIN);
  Serial.print(") configurat ca INPUT_PULLUP - Stare inițială: ");
  Serial.println(digitalRead(BUTON_PIN) ? "HIGH (neapăsat)" : "LOW (apăsat - VERIFICĂ CONEXIUNEA!)");
  Serial.println("Butonul trebuie legat: un fir la Pin 27, altul la GND");
  
  // Inițializare Preferences pentru WiFi (metodă nouă, mai sigură)
  prefs.begin("o2-sentinel", false);
  
  Serial.println("\n\n========================================");
  Serial.println("🚀 O2 SENTINEL - PORNIRE SISTEM");
  Serial.println("========================================");
  
  // Inițializare EEPROM și încărcare calibrare
  EEPROM.begin(EEPROM_SIZE);
  loadCalibration();
  
  // Încarcă credențiale WiFi din Preferences (prioritar) sau EEPROM (compatibilitate)
  loadWiFiCredentials();
  
  // Conectare WiFi sau pornire Access Point
  if (wifiSSID.length() > 0) {
  connectWiFi();
    // Dacă nu se conectează în 10 secunde, pornește AP
    if (WiFi.status() != WL_CONNECTED) {
      delay(10000);
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi nu s-a conectat, pornire Access Point pentru configurare...");
        startConfigMode();
      }
    }
  } else {
    Serial.println("Nu există credențiale WiFi salvate, pornire Access Point...");
    startConfigMode();
  }
  
  // Setup server web pentru configurare (după WiFi/AP)
  setupWebServer();
  
  // Conectare MQTT (doar dacă WiFi este conectat și nu suntem în modul configurare)
  if (WiFi.status() == WL_CONNECTED && !configMode) {
    Serial.println("[MQTT] Inițializare MQTT...");
    client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(mqttCallback);
    client.setBufferSize(512);  // Mărește buffer-ul pentru mesaje
  connectMQTT();
    
    if (client.connected()) {
      Serial.println("[MQTT] ✓ MQTT inițializat și conectat!");
    } else {
      Serial.println("[MQTT] ⚠️ MQTT nu s-a conectat în setup, se va reîncerca în loop");
    }
  } else {
    Serial.print("[MQTT] ⚠️ MQTT nu se inițializează - WiFi: ");
    Serial.print(WiFi.status() == WL_CONNECTED ? "CONECTAT" : "DECONECTAT");
    Serial.print(", ConfigMode: ");
    Serial.println(configMode ? "DA" : "NU");
  }
  
  Serial.println("Sistem gata!");
  Serial.println("Aștept date de la senzor AOX4000...");
}

// ==================== LOOP PRINCIPAL ====================
void loop() {
  // Gestionează server web (pentru configurare) - doar ocazional
  static unsigned long lastWebServerCheck = 0;
  if (millis() - lastWebServerCheck > 50) {  // Verifică la fiecare 50ms
    server.handleClient();
    lastWebServerCheck = millis();
  }
  
  // Verifică și menține conexiunea WiFi (doar dacă nu suntem în modul configurare)
  static unsigned long lastWiFiCheck = 0;
  static bool wasConnected = false; // Verifică dacă WiFi-ul era conectat anterior
  
  if (!configMode && (millis() - lastWiFiCheck > 5000)) {  // Verifică la fiecare 5 secunde
    bool isConnected = (WiFi.status() == WL_CONNECTED);
    
    // Reconectează DOAR dacă s-a deconectat (nu dacă era deja conectat)
    if (!isConnected && wasConnected) {
      Serial.println("[WiFi] ⚠️ WiFi deconectat, încercare reconectare...");
      connectWiFi();
      // Dacă nu se conectează după 10 secunde, pornește AP
      if (WiFi.status() != WL_CONNECTED) {
        delay(2000);
        if (WiFi.status() != WL_CONNECTED) {
          Serial.println("[WiFi] ⚠️ Nu s-a putut reconecta, pornire Access Point...");
          startConfigMode();
        }
      }
    } else if (!isConnected && !wasConnected) {
      // Prima dată când nu este conectat (la pornire sau după restart)
      // Nu face nimic aici - connectWiFi() este apelat în setup()
    }
    // Dacă este conectat, nu face nimic (nu scanează, nu reconectează)
    
    wasConnected = isConnected; // Actualizează starea
    lastWiFiCheck = millis();
  }
  
  // Verifică conexiunea MQTT (doar dacă WiFi este conectat)
  static unsigned long lastMQTTReconnect = 0;
  if (!configMode && WiFi.status() == WL_CONNECTED) {
  if (!client.connected()) {
      if (millis() - lastMQTTReconnect > 5000) {  // Reîncearcă la fiecare 5 secunde
        Serial.println("[MQTT] ⚠️ MQTT deconectat, reîncercare conectare...");
    connectMQTT();
        lastMQTTReconnect = millis();
      }
    } else {
      client.loop();  // Procesează mesaje MQTT doar dacă este conectat
    }
  }
  
  // Verifică periodic feed-ul "comands" prin HTTP API pentru configurare WiFi
  // (Adafruit IO nu trimite automat mesaje MQTT pentru datele adăugate prin API REST)
  // IMPORTANT: Verifică și când WiFi-ul nu este conectat (în modul AP) pentru a primi configurarea nouă
  if (millis() - lastConfigCheck > 10000) {  // La fiecare 10 secunde
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("[LOOP] Verificare configurare HTTP (WiFi conectat, ultima verificare acum ");
      Serial.print((millis() - lastConfigCheck) / 1000);
      Serial.println(" secunde)");
      checkConfigViaHTTP();
    } else if (configMode) {
      // În modul AP, verifică configurarea dacă dispozitivul conectat la AP are internet
      Serial.print("[LOOP] Verificare configurare HTTP (modul AP, ultima verificare acum ");
      Serial.print((millis() - lastConfigCheck) / 1000);
      Serial.println(" secunde)");
      Serial.println("[LOOP] ⚠️ În modul AP - verificare configurare (necesită internet pe dispozitivul conectat)");
      checkConfigViaHTTP();  // Încearcă să verifice chiar și în modul AP
    } else {
      static unsigned long lastWiFiNotConnectedMsg = 0;
      if (millis() - lastWiFiNotConnectedMsg > 30000) {  // La fiecare 30 secunde
        Serial.println("[LOOP] ⚠️ WiFi nu este conectat - nu se poate verifica configurarea prin HTTP");
        Serial.println("[LOOP] ⚠️ Conectează-te la WiFi pentru a primi configurarea nouă!");
        lastWiFiNotConnectedMsg = millis();
      }
    }
    lastConfigCheck = millis();
  }
  
  // Verifică periodic feed-ul "esp-code" pentru update-uri OTA
  if (millis() - lastOTACheck > 30000) {  // La fiecare 30 secunde
    if (WiFi.status() == WL_CONNECTED && !configMode) {
      checkOTAUpdate();
    }
    lastOTACheck = millis();
  }
  
  // Citește date de la senzor
  readSensorData();
  
  // Verifică butonul fizic (PIN 27) - SINCRONIZARE cu interfața web
  // Dacă butonul este apăsat și buzzerul este activ, oprește buzzerul
  // și trimite "buzzer_off" la MQTT pentru sincronizare cu aplicația
  checkButton();
  
  // Gestionează buzzerul (va respecta buzzerMuted setat de checkButton)
  handleBuzzer();
  
  // Gestionează releul
  handleRelay();
  
  // Publică date MQTT periodic (doar dacă MQTT este conectat)
  static unsigned long lastMQTTDebug = 0;
  if (!configMode && WiFi.status() == WL_CONNECTED) {
    if (millis() - lastMQTTDebug > 10000) {  // Debug la fiecare 10 secunde
      Serial.print("[MQTT] Status: ");
      Serial.print(client.connected() ? "CONECTAT" : "DECONECTAT");
      Serial.print(", WiFi: ");
      Serial.print(WiFi.status() == WL_CONNECTED ? "CONECTAT" : "DECONECTAT");
      Serial.print(", ConfigMode: ");
      Serial.println(configMode ? "DA" : "NU");
      lastMQTTDebug = millis();
    }
    
    if (client.connected() && (millis() - lastMQTTPublish > MQTT_PUBLISH_INTERVAL)) {
    publishMQTTData();
    lastMQTTPublish = millis();
    } else if (!client.connected() && (millis() - lastMQTTPublish > MQTT_PUBLISH_INTERVAL)) {
      Serial.println("[MQTT] ⚠️ MQTT nu este conectat - nu se pot publica date");
      lastMQTTPublish = millis();  // Resetează timer-ul pentru a evita spam-ul
    }
  }
  
  delay(10);  // Mic delay pentru stabilitate
}

// ==================== CITIRE DATE SENZOR ====================
void readSensorData() {
  // Citește continuu date disponibile de la senzor (fără limitare de timp)
  bool dataReceived = false;
  
  while (Serial2.available()) {
    char c = Serial2.read();
    dataReceived = true;
    
    // Procesează caracterele primite
    if (c == '\n' || c == '\r') {
      if (sensorBuffer.length() > 0) {
        // Afișează datele raw primite de la senzor
        Serial.print("[SENZOR RAW] ");
        Serial.println(sensorBuffer);
        
        parseSensorData(sensorBuffer);
        lastSensorDataReceived = millis();  // Actualizează timestamp-ul
        sensorBuffer = "";
      }
    } else if (c >= 32 && c <= 126) {  // Doar caractere printable ASCII
      sensorBuffer += c;
    }
  }
  
  // Dacă buffer-ul are date dar nu s-a primit newline, procesează după un timp
  static unsigned long bufferStartTime = 0;
  if (sensorBuffer.length() > 0) {
    if (bufferStartTime == 0) {
      bufferStartTime = millis();
    }
    // Dacă au trecut mai mult de 500ms și buffer-ul are cel puțin 3 caractere, procesează
    if (millis() - bufferStartTime > 500 && sensorBuffer.length() >= 3) {
      Serial.print("[SENZOR RAW] ");
      Serial.println(sensorBuffer);
      parseSensorData(sensorBuffer);
      lastSensorDataReceived = millis();
      sensorBuffer = "";
      bufferStartTime = 0;
    }
  } else {
    bufferStartTime = 0;
  }
  
  // Dacă buffer-ul devine prea mare, îl resetează
  if (sensorBuffer.length() > 100) {
    Serial.println("[SENZOR] Buffer prea mare, resetare...");
    sensorBuffer = "";
    bufferStartTime = 0;
  }
  
  // Afișează periodic valoarea curentă și statusul
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {  // La fiecare 2 secunde
    Serial.print("[OXIGEN] Concentrație curentă: ");
    Serial.print(oxygenLevel);
    Serial.print("%");
    
    // Verifică dacă sunt date noi de la senzor
    if (lastSensorDataReceived == 0 || (millis() - lastSensorDataReceived > 10000)) {
      Serial.print(" ⚠️ Fără date noi de la senzor!");
      if (lastSensorDataReceived > 0) {
        Serial.print(" (ultima dată: ");
        Serial.print((millis() - lastSensorDataReceived) / 1000);
        Serial.print(" secunde în urmă)");
      } else {
        Serial.print(" (verifică conexiunile: TX senzor -> GPIO16, RX senzor -> GPIO17)");
      }
    } else {
      Serial.print(" ✓ Date active");
    }
    Serial.print(" | Bytes în buffer: ");
    Serial.print(sensorBuffer.length());
    Serial.print(" | Serial2 available: ");
    Serial.println(Serial2.available());
    lastPrint = millis();
  }
}

// ==================== PARSARE DATE SENZOR ====================
void parseSensorData(String data) {
  // Format AOX4000: "O 0012.7 T +22.6 P 1010 % 012.61 e 1441"
  // Senzorul este calibrat din fabrică și ar trebui să afișeze ~20-21% în aer normal
  // Testăm ambele valori pentru a găsi cea corectă
  
  float valueFromO = 0.0;   // Valoarea de după "O" (ex: 0012.7)
  float valueFromPercent = 0.0;  // Valoarea de după "%" (ex: 012.61)
  bool foundO = false;
  bool foundPercent = false;
  
  // EXTRAGERE 1: Valoarea de după "O " (presiune parțială sau concentrație?)
  int oIndex = data.indexOf(" O ");
  if (oIndex == -1) {
    if (data.startsWith("O ")) {
      oIndex = 0;
    } else {
      oIndex = data.indexOf("O ");
    }
  }
  
  if (oIndex != -1) {
    int startIdx = oIndex + (data[oIndex] == ' ' ? 2 : 1);
    int endIdx = data.indexOf(" ", startIdx);
    if (endIdx == -1) endIdx = data.length();
    
    String oValueStr = data.substring(startIdx, endIdx);
    oValueStr.trim();
    valueFromO = oValueStr.toFloat();
    foundO = true;
    Serial.print("[PARSARE] Valoare după 'O': ");
    Serial.print(valueFromO);
    Serial.println("%");
  }
  
  // EXTRAGERE 2: Valoarea de după "% " (concentrație?)
  int percentIndex = data.indexOf("%");
  if (percentIndex != -1) {
    int startIndex = percentIndex + 2;  // Salt peste "% "
    int endIndex = data.indexOf(" ", startIndex);
    if (endIndex == -1) {
      endIndex = data.length();
    }
    
    String percentValueStr = data.substring(startIndex, endIndex);
    percentValueStr.trim();
    valueFromPercent = percentValueStr.toFloat();
    foundPercent = true;
    Serial.print("[PARSARE] Valoare după '%': ");
    Serial.print(valueFromPercent);
    Serial.println("%");
  }
  
  // DECIZIE: Care valoare este cea corectă?
  // În aer normal, concentrația de oxigen este ~20.9%
  // Alegem valoarea cea mai apropiată de 20-21%
  float newValue = 0.0;
  bool valueFound = false;
  
  if (foundO && foundPercent) {
    float targetValue = 20.9;  // Concentrația normală de oxigen în aer
    float diffO = abs(valueFromO - targetValue);
    float diffPercent = abs(valueFromPercent - targetValue);
    
    Serial.print("[PARSARE] Diferență față de 20.9%: O=");
    Serial.print(diffO);
    Serial.print(", %=");
    Serial.println(diffPercent);
    
    // Alegem valoarea cea mai apropiată de 20.9%
    if (diffO < diffPercent && valueFromO >= 15 && valueFromO <= 25) {
      newValue = valueFromO;
      valueFound = true;
      Serial.print("[PARSARE] ✓ Folosind valoarea de după 'O': ");
      Serial.print(newValue);
      Serial.println("%");
    } else if (valueFromPercent >= 15 && valueFromPercent <= 25) {
      newValue = valueFromPercent;
      valueFound = true;
      Serial.print("[PARSARE] ✓ Folosind valoarea de după '%': ");
      Serial.print(newValue);
      Serial.println("%");
    } else {
      // Dacă niciuna nu este în intervalul normal, folosim valoarea de după "%"
      // (conform documentației, aceasta este concentrația)
      newValue = valueFromPercent;
      valueFound = true;
      Serial.print("[PARSARE] ⚠️ Ambele valori sunt în afara intervalului normal, folosim '%': ");
      Serial.print(newValue);
      Serial.println("%");
    }
  } else if (foundPercent) {
    newValue = valueFromPercent;
    valueFound = true;
  } else if (foundO) {
    newValue = valueFromO;
    valueFound = true;
  }
  
  // PRIORITATE 2: Format "O2: XX.X%" sau "O2:XX.X%"
  if (!valueFound) {
    int o2Index = data.indexOf("O2");
    int pctIndex = data.indexOf("%");
    
    if (o2Index != -1 && pctIndex != -1 && pctIndex > o2Index) {
      int separatorIndex = data.indexOf(":", o2Index);
      if (separatorIndex == -1) {
        separatorIndex = data.indexOf("=", o2Index);
      }
      if (separatorIndex != -1 && separatorIndex < pctIndex) {
        String valueStr = data.substring(separatorIndex + 1, pctIndex);
        valueStr.trim();
        newValue = valueStr.toFloat();
        if (newValue > 0) {
          valueFound = true;
        }
      }
    }
  }
  
  // PRIORITATE 3: Format "O XX.X" (valoarea de după "O " în format AOX4000)
  if (!valueFound) {
    int oIndex = data.indexOf(" O ");
    if (oIndex == -1) oIndex = data.indexOf("O ");
    if (oIndex == -1 && data.startsWith("O ")) oIndex = 0;
    
    if (oIndex != -1) {
      int startIdx = oIndex + (data[oIndex] == ' ' ? 2 : 1);
      int endIdx = data.indexOf(" ", startIdx);
      if (endIdx == -1) endIdx = data.length();
      
      String valueStr = data.substring(startIdx, endIdx);
      valueStr.trim();
      newValue = valueStr.toFloat();
      
      // În format AOX4000, valoarea după "O" este în format 0012.6 (cu zerouri în față)
      // Trebuie convertită corect
      if (newValue > 0 && newValue < 100) {
        // Poate fi deja corectă sau poate necesita ajustare
        valueFound = true;
        Serial.print("[PARSARE] Valoare extrasă după 'O': ");
        Serial.print(newValue);
        Serial.println("%");
      }
    }
  }
  
  // Validare și aplicare valoare
  if (valueFound && newValue >= MIN_OXYGEN_LEVEL && newValue <= MAX_OXYGEN_LEVEL) {
    // Aplică calibrarea (offset și factor)
    oxygenLevel = (newValue * calibrationFactor) + calibrationOffset;
    
    Serial.print("[PARSARE] ✓ Valoare validă: ");
    Serial.print(newValue);
    Serial.print("% (raw) -> ");
  Serial.print(oxygenLevel);
    Serial.print("% (calibrat, offset=");
    Serial.print(calibrationOffset);
    Serial.print(", factor=");
    Serial.print(calibrationFactor);
    Serial.println(")");
    
    // Dacă suntem în modul calibrare, colectăm eșantioane
    if (calibrationActive) {
      collectCalibrationSample(oxygenLevel);
    }
  } else {
    Serial.print("[PARSARE] ✗ Nu s-a putut extrage valoare validă din: \"");
    Serial.print(data);
    Serial.println("\"");
    if (!valueFound) {
      Serial.println("[PARSARE] Verifică formatul datelor - așteptat: 'O XX.X T ... P ... % XX.XX ...'");
    } else {
      Serial.print("[PARSARE] Valoarea ");
      Serial.print(newValue);
      Serial.print("% este în afara intervalului valid (");
      Serial.print(MIN_OXYGEN_LEVEL);
      Serial.print("-");
      Serial.print(MAX_OXYGEN_LEVEL);
      Serial.println("%)");
    }
    // Nu resetează la 0 dacă avem deja o valoare validă
    if (oxygenLevel == 0.0) {
      Serial.println("[PARSARE] Valoarea rămâne 0.00%");
    }
  }
}


// ==================== VERIFICARE BUTON ====================
void checkButton() {
  static unsigned long lastButtonPress = 0;
  bool currentButtonState = digitalRead(BUTON_PIN);
  
  // Butonul e LOW când e apăsat (pull-up) - buton fără reținere (momentary)
  // Verifică dacă butonul este apăsat ȘI dacă a trecut suficient timp de la ultima apăsare (debounce)
  if (currentButtonState == LOW && (millis() - lastButtonPress > 500)) {
    lastButtonPress = millis();
    
    // Verifică dacă buzzerul este activ (manual din aplicație SAU alertă automată)
    if (manualBuzzer || oxygenLevel > 23.0) {
      // Buton apăsat - oprește buzzerul imediat
      manualBuzzer = false;  // Dezactivează modul manual
      buzzerMuted = true;    // Activează mute pentru a preveni reactivarea
      digitalWrite(BUZZER_PIN, LOW);  // Oprește fizic buzzerul imediat
      
      Serial.println("[BUTON] ✓✓✓ BUTON FIZIC APĂSAT - Buzzer oprit ✓✓✓");
      
      // IMPORTANT: Trimite "buzzer_off" la topic-ul de comenzi pentru sincronizare cu aplicația
      // Aplicația citește acest topic și actualizează UI-ul automat
      if (client.connected()) {
        bool published = client.publish(MQTT_TOPIC_COMMANDS, "buzzer_off");
        if (published) {
          Serial.println("[BUTON] ✓ Mesaj 'buzzer_off' trimis la Adafruit IO pentru sincronizare");
          Serial.print("[BUTON] Topic: ");
          Serial.println(MQTT_TOPIC_COMMANDS);
        } else {
          Serial.println("[BUTON] ✗ Eroare la trimitere mesaj MQTT");
        }
      } else {
        Serial.println("[BUTON] ⚠️ MQTT nu este conectat - nu se poate trimite mesaj");
      }
    } else {
      Serial.println("[BUTON] Buton apăsat dar buzzerul nu este activ");
    }
  }
  
  // Dacă oxigenul revine la normal (< 21%), resetează mute
  if (oxygenLevel < 21.0 && buzzerMuted) {
    buzzerMuted = false;
    Serial.println("[BUTON] ✓ Mute resetat - oxigen normal");
  }
}

// ==================== GESTIONARE BUZZER ====================
void handleBuzzer() {
  // PRIORITATE 1: Dacă buzzerul este mutat (prin buton fizic), oprește-l IMEDIAT
  if (buzzerMuted) {
    digitalWrite(BUZZER_PIN, LOW);
    return;  // Ieșire imediată - nu verifică alte condiții
  }
  
  // PRIORITATE 2: Dacă buzzerul este dezactivat global, oprește-l
  if (!buzzerEnabled) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }
  
  // PRIORITATE 3: Dacă buzzerul este controlat manual (prin MQTT sau API)
  if (manualBuzzer) {
    digitalWrite(BUZZER_PIN, HIGH);  // Sunet continuu în mod manual
    return;
  }
  
  // PRIORITATE 4: Control automat bazat pe nivelul de oxigen
  // Sunet intermitent la >23%
  if (oxygenLevel > OXYGEN_THRESHOLD_23) {
    // Sunet intermitent: ON/OFF (500ms ON, 500ms OFF)
    digitalWrite(BUZZER_PIN, (millis() % 1000 < 500) ? HIGH : LOW);
  }
  // Nivel normal - oprește buzzerul
  else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ==================== GESTIONARE RELEU ====================
void handleRelay() {
  // Control releu: manual SAU automat bazat pe oxigen (>25%)
  if (manualFan || oxygenLevel > OXYGEN_THRESHOLD_25) {
      digitalWrite(RELAY_PIN, HIGH);
    if (!relayState) {
      relayState = true;
      #if ENABLE_DEBUG
      Serial.print("Ventilator PORNIT (");
      Serial.print(manualFan ? "MANUAL" : "O2 > 25%");
      Serial.println(")");
      #endif
      publishMQTTStatus(manualFan ? "fan_on_manual" : "fan_on");
    }
  }
  // Dezactivează releul când nivelul scade sub prag (histerezis) și nu e manual
  else {
      digitalWrite(RELAY_PIN, LOW);
    if (relayState) {
      relayState = false;
      #if ENABLE_DEBUG
      Serial.println("Ventilator OPrit");
      #endif
      publishMQTTStatus("fan_off");
    }
  }
}

// ==================== GESTIUNE WIFI CREDENTIALS ====================

// Încarcă credențiale WiFi din Preferences (prioritar) sau EEPROM (compatibilitate)
void loadWiFiCredentials() {
  Serial.println("[WiFi] ========== ÎNCĂRCARE CREDENȚIALE WIFI ==========");
  
  // Încearcă să încarce din Preferences (metodă nouă, mai sigură)
  String storedSSID = prefs.getString("ssid", "");
  String storedPass = prefs.getString("pass", "");
  
  Serial.print("[WiFi] Preferences - SSID: '");
  Serial.print(storedSSID);
  Serial.print("', Pass: '");
  Serial.print(storedPass.length() > 0 ? "***" : "(gol)");
  Serial.println("'");
  
  if (storedSSID.length() > 0) {
    wifiSSID = storedSSID;
    wifiPassword = storedPass;
    Serial.println("[WiFi] ✓✓✓ Credențiale încărcate din Preferences ✓✓✓");
    Serial.print("[WiFi] SSID: ");
    Serial.println(wifiSSID);
  Serial.print("[WiFi] Parolă: ");
  Serial.print(wifiPassword.length() > 0 ? "***" : "(gol)");
  Serial.print(" (lungime: ");
  Serial.print(wifiPassword.length());
  Serial.println(" caractere)");
  
  // Debug: afișează primele și ultimele caractere pentru verificare (fără a expune parola completă)
  if (wifiPassword.length() > 0) {
    Serial.print("[WiFi] Debug parolă - primul caracter: '");
    Serial.print(wifiPassword[0]);
    Serial.print("', ultimul caracter: '");
    Serial.print(wifiPassword[wifiPassword.length() - 1]);
    Serial.print("', lungime totală: ");
    Serial.println(wifiPassword.length());
  }
  
  Serial.println("[WiFi] ================================================");
  return;
  }
  
  Serial.println("[WiFi] ⚠️ Nu există credențiale în Preferences, verifică EEPROM...");
  
  // Dacă nu există în Preferences, încearcă EEPROM (compatibilitate cu codul vechi)
  uint8_t magicHigh = EEPROM.read(EEPROM_WIFI_MAGIC_ADDR);
  uint8_t magicLow = EEPROM.read(EEPROM_WIFI_MAGIC_ADDR + 1);
  uint16_t magic = (magicHigh << 8) | magicLow;
  
  if (magic == EEPROM_WIFI_MAGIC_VALUE) {
    // Citește SSID
    wifiSSID = "";
    for (int i = 0; i < MAX_SSID_LENGTH; i++) {
      char c = EEPROM.read(EEPROM_WIFI_SSID_ADDR + i);
      if (c == 0) break;
      wifiSSID += c;
    }
    
    // Citește Password
    wifiPassword = "";
    for (int i = 0; i < MAX_PASSWORD_LENGTH; i++) {
      char c = EEPROM.read(EEPROM_WIFI_PASSWORD_ADDR + i);
      if (c == 0) break;
      wifiPassword += c;
    }
    
    Serial.println("[WiFi] ✓ Credențiale încărcate din EEPROM (compatibilitate)");
    Serial.print("[WiFi] SSID: ");
    Serial.println(wifiSSID);
    
    // Migrează din EEPROM în Preferences pentru viitor
    saveWiFiCredentials(wifiSSID, wifiPassword);
  } else {
    Serial.println("[WiFi] ⚠️ Nu există credențiale WiFi salvate");
    wifiSSID = "";
    wifiPassword = "";
  }
}

// Salvează credențiale WiFi în Preferences (metodă nouă) și EEPROM (compatibilitate)
void saveWiFiCredentials(String ssid, String pass) {
  // Salvează în Preferences (metodă nouă, mai sigură)
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  
  // Salvează și în EEPROM pentru compatibilitate cu codul vechi
  EEPROM.write(EEPROM_WIFI_MAGIC_ADDR, (EEPROM_WIFI_MAGIC_VALUE >> 8) & 0xFF);
  EEPROM.write(EEPROM_WIFI_MAGIC_ADDR + 1, EEPROM_WIFI_MAGIC_VALUE & 0xFF);
  
  // Salvează SSID
  for (int i = 0; i < MAX_SSID_LENGTH; i++) {
    if (i < ssid.length()) {
      EEPROM.write(EEPROM_WIFI_SSID_ADDR + i, ssid[i]);
    } else {
      EEPROM.write(EEPROM_WIFI_SSID_ADDR + i, 0);
    }
  }
  
  // Salvează Password
  for (int i = 0; i < MAX_PASSWORD_LENGTH; i++) {
    if (i < pass.length()) {
      EEPROM.write(EEPROM_WIFI_PASSWORD_ADDR + i, pass[i]);
    } else {
      EEPROM.write(EEPROM_WIFI_PASSWORD_ADDR + i, 0);
    }
  }
  
  EEPROM.commit();
  
  wifiSSID = ssid;
  wifiPassword = pass;
  
  Serial.println("[WiFi] ========== SALVARE CREDENȚIALE WIFI ==========");
  Serial.print("[WiFi] SSID: ");
  Serial.println(ssid);
  Serial.print("[WiFi] Parolă: ");
  Serial.println(pass.length() > 0 ? "***" : "(gol)");
  
  // Verificare imediată după salvare
  String verifySSID = prefs.getString("ssid", "");
  Serial.print("[WiFi] Verificare - SSID citit: '");
  Serial.print(verifySSID);
  Serial.print("' == '");
  Serial.print(ssid);
  Serial.print("' ? ");
  Serial.println(verifySSID == ssid ? "✓ DA" : "✗ NU");
  
  Serial.println("[WiFi] ✓✓✓ Credențiale WiFi salvate în Preferences și EEPROM ✓✓✓");
  Serial.println("[WiFi] ================================================");
}

// ==================== CONEXIUNE WIFI ====================
void connectWiFi() {
  // Verifică dacă deja este conectat
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi deja conectat!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    return;
  }
  
  if (wifiSSID.length() == 0) {
    Serial.println("[WiFi] ⚠️ Nu există SSID configurat!");
    return;
  }
  
  Serial.print("Conectare la WiFi: ");
  Serial.println(wifiSSID);
  Serial.print("[WiFi] SSID lungime: ");
  Serial.print(wifiSSID.length());
  Serial.print(" caractere, Parolă lungime: ");
  Serial.print(wifiPassword.length());
  Serial.println(" caractere");
  
  // Debug: verifică dacă parola conține caractere valide
  if (wifiPassword.length() == 0) {
    Serial.println("[WiFi] ⚠️ ATENȚIE: Parola este GOLĂ!");
    Serial.println("[WiFi] ⚠️ Nu se poate conecta fără parolă!");
    return;
  } else {
    Serial.print("[WiFi] Parolă validă (lungime: ");
    Serial.print(wifiPassword.length());
    Serial.println(" caractere)");
    
    // Debug: afișează primele și ultimele caractere pentru verificare
    Serial.print("[WiFi] Debug parolă - primul: '");
    Serial.print(wifiPassword[0]);
    Serial.print("', ultimul: '");
    Serial.print(wifiPassword[wifiPassword.length() - 1]);
    Serial.println("'");
  }
  
  // Verifică din nou dacă s-a conectat între timp (după delay-uri)
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] WiFi deja conectat după verificare!");
    Serial.print("[WiFi] IP address: ");
    Serial.println(WiFi.localIP());
    return;
  }
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  // SCANARE WIFI pentru a verifica dacă rețeaua există (doar dacă nu suntem conectați)
  bool networkFound = false;
  uint8_t* targetBSSID = nullptr;
  int targetChannel = 0;
  int targetRSSI = 0;
  String exactSSID = ""; // SSID-ul EXACT din scanare (cu spații dacă există)
  
  if (WiFi.status() != WL_CONNECTED) {
  // Verifică din nou dacă s-a conectat între timp (după disconnect)
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] ✓ WiFi conectat după disconnect!");
    Serial.print("[WiFi] IP address: ");
    Serial.println(WiFi.localIP());
    return;
  }
  
  Serial.println("[WiFi] Scanare rețele WiFi disponibile...");
  int n = WiFi.scanNetworks();
  Serial.print("[WiFi] Găsite ");
  Serial.print(n);
  Serial.println(" rețele WiFi");
  
  for (int i = 0; i < n; i++) {
    String foundSSID = WiFi.SSID(i); // SSID-ul EXACT din scanare
    String foundSSIDTrimmed = foundSSID;
    foundSSIDTrimmed.trim(); // Pentru comparare
    
    Serial.print("[WiFi] ");
    Serial.print(i + 1);
    Serial.print(". SSID: '");
    Serial.print(foundSSID);
    Serial.print("' (lungime: ");
    Serial.print(foundSSID.length());
    Serial.print(", RSSI: ");
    Serial.print(WiFi.RSSI(i));
    Serial.print(" dBm, Canal: ");
    Serial.print(WiFi.channel(i));
    Serial.println(")");
    
    // Compară SSID-ul găsit (fără spații) cu cel configurat (fără spații)
    String configSSID = wifiSSID;
    configSSID.trim();
    
    if (foundSSIDTrimmed == configSSID) {
      networkFound = true;
      exactSSID = foundSSID; // Folosim SSID-ul EXACT din scanare (cu spații dacă există)
      targetBSSID = (uint8_t*)WiFi.BSSID(i);
      targetChannel = WiFi.channel(i);
      targetRSSI = WiFi.RSSI(i);
      
      Serial.print("[WiFi] ✓✓✓ REȚEAUĂ GĂSITĂ! ✓✓✓ SSID EXACT: '");
      Serial.print(exactSSID);
      Serial.print("' (lungime: ");
      Serial.print(exactSSID.length());
      Serial.print("), RSSI: ");
      Serial.print(targetRSSI);
      Serial.print(" dBm, Canal: ");
      Serial.print(targetChannel);
      Serial.print(", BSSID: ");
      for (int j = 0; j < 6; j++) {
        if (j > 0) Serial.print(":");
        if (targetBSSID[j] < 0x10) Serial.print("0");
        Serial.print(targetBSSID[j], HEX);
      }
      Serial.println();
    }
  }
  } // Închide if (WiFi.status() != WL_CONNECTED) pentru scanare
  
  // Verifică din nou dacă s-a conectat între timp (după scanare)
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] ✓ WiFi conectat după scanare!");
    Serial.print("[WiFi] IP address: ");
    Serial.println(WiFi.localIP());
    return;
  }
  
  if (!networkFound) {
    Serial.println("[WiFi] ⚠️ ATENȚIE: Rețeaua WiFi cu SSID-ul '(-_-)' NU a fost găsită!");
    Serial.println("[WiFi] ⚠️ Verifică dacă:");
    Serial.println("[WiFi]   1. Rețeaua WiFi este pornită");
    Serial.println("[WiFi]   2. SSID-ul este exact '(-_-)' (cu paranteze și cratime)");
    Serial.println("[WiFi]   3. ESP32 este în raza de acoperire");
    Serial.println("[WiFi] ⚠️ Continuăm cu încercarea de conectare...");
  }
  
  Serial.println("[WiFi] Apel WiFi.begin()...");
  
  // Folosim SSID-ul EXACT din scanare dacă rețeaua a fost găsită
  String ssidToUse = "";
  if (networkFound && exactSSID.length() > 0) {
    ssidToUse = exactSSID; // SSID-ul EXACT din scanare (cu spații dacă există)
    Serial.print("[WiFi] ✓ Folosim SSID EXACT din scanare: '");
    Serial.print(ssidToUse);
    Serial.print("' (lungime: ");
    Serial.print(ssidToUse.length());
    Serial.println(")");
  } else {
    // Dacă nu a fost găsită, folosim SSID-ul din Preferences (fără spații)
    ssidToUse = wifiSSID;
    ssidToUse.trim();
    Serial.print("[WiFi] ⚠️ Folosim SSID din Preferences (fără scanare): '");
    Serial.print(ssidToUse);
    Serial.print("' (lungime: ");
    Serial.print(ssidToUse.length());
    Serial.println(")");
  }
  
  Serial.print("[WiFi] SSID trimis la WiFi.begin(): '");
  Serial.print(ssidToUse);
  Serial.print("' (lungime: ");
  Serial.print(ssidToUse.length());
  Serial.print("), Parolă trimisă: '");
  Serial.print(wifiPassword.length() > 0 ? "***" : "(gol)");
  Serial.print("' (lungime: ");
  Serial.print(wifiPassword.length());
  Serial.println(")");
  
  // Încearcă conectare cu BSSID și canal dacă rețeaua a fost găsită
  if (networkFound && targetBSSID != nullptr && targetChannel > 0) {
    Serial.println("[WiFi] Încercare conectare cu BSSID și canal specificat...");
    WiFi.begin(ssidToUse.c_str(), wifiPassword.c_str(), targetChannel, targetBSSID, true);
  } else {
    Serial.println("[WiFi] Încercare conectare standard (fără BSSID/canal)...");
    WiFi.begin(ssidToUse.c_str(), wifiPassword.c_str());
  }
  
  int attempts = 0;
  int maxAttempts = 40; // Mărim la 40 încercări (20 secunde)
  
  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    delay(500);
    Serial.print(".");
    attempts++;
    
    // Debug la fiecare 5 secunde
    if (attempts % 10 == 0) {
      int status = WiFi.status();
      Serial.print("\n[WiFi] Status: ");
      Serial.print(status);
      Serial.print(" (");
      switch(status) {
        case WL_IDLE_STATUS: Serial.print("IDLE"); break;
        case WL_NO_SSID_AVAIL: Serial.print("NO_SSID_AVAIL"); break;
        case WL_SCAN_COMPLETED: Serial.print("SCAN_COMPLETED"); break;
        case WL_CONNECTED: Serial.print("CONNECTED"); break;
        case WL_CONNECT_FAILED: Serial.print("CONNECT_FAILED"); break;
        case WL_CONNECTION_LOST: Serial.print("CONNECTION_LOST"); break;
        case WL_DISCONNECTED: Serial.print("DISCONNECTED"); break;
        default: Serial.print("UNKNOWN"); break;
      }
      Serial.print("), Attempts: ");
      Serial.print(attempts);
      Serial.print("/");
      Serial.println(maxAttempts);
      
      // Dacă după 20 de încercări tot nu se conectează, încearcă din nou cu scanare
      if (attempts == 20 && status == WL_NO_SSID_AVAIL) {
        Serial.println("[WiFi] ⚠️ Reîncercare cu scanare nouă...");
        WiFi.disconnect();
        delay(1000);
        
        // Re-scanare rapidă
        int n2 = WiFi.scanNetworks();
        String exactSSID2 = "";
        uint8_t* targetBSSID2 = nullptr;
        int targetChannel2 = 0;
        
        for (int i = 0; i < n2; i++) {
          String foundSSID2 = WiFi.SSID(i); // SSID EXACT
          String foundSSID2Trimmed = foundSSID2;
          foundSSID2Trimmed.trim();
          
          String configSSID2 = wifiSSID;
          configSSID2.trim();
          
          if (foundSSID2Trimmed == configSSID2) {
            exactSSID2 = foundSSID2; // SSID EXACT din scanare
            targetBSSID2 = (uint8_t*)WiFi.BSSID(i);
            targetChannel2 = WiFi.channel(i);
            
            Serial.print("[WiFi] Rețeaua încă disponibilă, reîncercare cu SSID EXACT: '");
            Serial.print(exactSSID2);
            Serial.print("' (lungime: ");
            Serial.print(exactSSID2.length());
            Serial.println(")");
            
            if (targetBSSID2 != nullptr && targetChannel2 > 0) {
              WiFi.begin(exactSSID2.c_str(), wifiPassword.c_str(), targetChannel2, targetBSSID2, true);
            } else {
              WiFi.begin(exactSSID2.c_str(), wifiPassword.c_str());
            }
            break;
          }
        }
      }
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectat cu succes!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("\nEroare conectare WiFi!");
    Serial.print("Status WiFi: ");
    int status = WiFi.status();
    Serial.print(status);
    Serial.print(" (");
    switch(status) {
      case WL_IDLE_STATUS: Serial.print("IDLE"); break;
      case WL_NO_SSID_AVAIL: Serial.print("NO_SSID_AVAIL"); break;
      case WL_SCAN_COMPLETED: Serial.print("SCAN_COMPLETED"); break;
      case WL_CONNECTED: Serial.print("CONNECTED"); break;
      case WL_CONNECT_FAILED: Serial.print("CONNECT_FAILED"); break;
      case WL_CONNECTION_LOST: Serial.print("CONNECTION_LOST"); break;
      case WL_DISCONNECTED: Serial.print("DISCONNECTED"); break;
      default: Serial.print("UNKNOWN"); break;
    }
    Serial.println(")");
    Serial.println("[WiFi] Verifică SSID și parolă!");
  }
}

// Pornește Access Point pentru configurare
void startConfigMode() {
  configMode = true;
  Serial.println("\n========================================");
  Serial.println("=== MODUL CONFIGURARE WIFI ===");
  Serial.println("========================================");
  Serial.print("Access Point: ");
  Serial.println(apSSID);
  Serial.print("Parolă AP: ");
  Serial.println(apPassword);
  
  WiFi.mode(WIFI_AP_STA);
  
  // Configurează AP cu sau fără parolă
  if (strlen(apPassword) > 0) {
    WiFi.softAP(apSSID, apPassword);
    Serial.print("Parolă AP: ");
    Serial.println(apPassword);
  } else {
    WiFi.softAP(apSSID);  // Fără parolă
    Serial.println("AP fără parolă (open network)");
  }
  
  // Așteaptă puțin pentru inițializare
  delay(500);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.println("\n📱 INSTRUCȚIUNI:");
  Serial.print("1. Conectează-te la rețeaua WiFi: ");
  Serial.println(apSSID);
  if (strlen(apPassword) > 0) {
    Serial.print("2. Parola: ");
    Serial.println(apPassword);
  } else {
    Serial.println("2. Fără parolă (rețea deschisă)");
  }
  Serial.println("3. Deschide browser și accesează: http://192.168.4.1");
  Serial.println("4. Sau folosește aplicația mobilă pentru configurare");
  Serial.println("5. Configurează SSID și parola hotspot-ului telefonului");
  Serial.println("========================================\n");
}

// ==================== WEB SERVER ====================

// Configurează server web cu endpoint-uri API
void setupWebServer() {
  // Pagină principală HTML pentru configurare
  server.on("/", HTTP_GET, handleRoot);
  
  // Endpoint pentru status
  server.on("/api/status", HTTP_GET, handleStatus);
  
  // Endpoint pentru scanare rețele WiFi
  server.on("/api/wifi/scan", HTTP_GET, handleWiFiScan);
  
  // Endpoint pentru configurare WiFi (POST)
  server.on("/api/wifi/config", HTTP_POST, handleWiFiConfig);
  
  // Endpoint pentru obținere configurare WiFi curentă
  server.on("/api/wifi/current", HTTP_GET, handleWiFiCurrent);
  
  // Endpoint pentru resetare configurare
  server.on("/api/wifi/reset", HTTP_POST, handleWiFiReset);
  
  // Endpoint pentru date senzor (JSON)
  server.on("/api/sensor/data", HTTP_GET, handleSensorData);
  
  // Handler pentru răspunsuri CORS (pentru aplicația mobilă)
  server.onNotFound(handleNotFound);
  
  server.begin();
  delay(100);  // Mic delay pentru inițializare
  Serial.println("[WebServer] ✓ Server web pornit pe port 80");
  Serial.println("[WebServer] Endpoint-uri disponibile:");
  Serial.println("  GET  / (pagina principală)");
  Serial.println("  GET  /api/status");
  Serial.println("  GET  /api/wifi/scan");
  Serial.println("  GET  /api/wifi/current");
  Serial.println("  POST /api/wifi/config");
  Serial.println("  POST /api/wifi/reset");
  Serial.println("  GET  /api/sensor/data");
}

// Handler pentru pagina principală HTML
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>O2-Sentinel Configurare WiFi</title>";
  html += "<style>";
  html += "body { font-family: Arial; background: #1a1a1a; color: #fff; padding: 20px; }";
  html += ".container { max-width: 500px; margin: 0 auto; background: #2a2a2a; padding: 20px; border-radius: 10px; }";
  html += "h1 { color: #4CAF50; }";
  html += "input, button { width: 100%; padding: 12px; margin: 10px 0; border: none; border-radius: 5px; font-size: 16px; }";
  html += "input { background: #3a3a3a; color: #fff; }";
  html += "button { background: #4CAF50; color: white; cursor: pointer; font-weight: bold; }";
  html += "button:hover { background: #45a049; }";
  html += ".status { padding: 10px; margin: 10px 0; border-radius: 5px; }";
  html += ".success { background: #4CAF50; }";
  html += ".error { background: #f44336; }";
  html += "</style></head><body>";
  html += "<div class='container'>";
  html += "<h1>🔧 O2-Sentinel Configurare WiFi</h1>";
  html += "<div id='status'></div>";
  html += "<h3>Rețele WiFi disponibile:</h3>";
  html += "<button onclick='scanWiFi()'>🔍 Scanează rețele</button>";
  html += "<div id='networks'></div>";
  html += "<h3>Configurează WiFi:</h3>";
  html += "<input type='text' id='ssid' placeholder='Numele rețelei WiFi (SSID)' required>";
  html += "<input type='password' id='password' placeholder='Parola WiFi'>";
  html += "<button onclick='saveWiFi()'>💾 Salvează și conectează</button>";
  html += "<hr>";
  html += "<h3>Status:</h3>";
  html += "<button onclick='checkStatus()'>🔄 Verifică status</button>";
  html += "<div id='statusInfo'></div>";
  html += "</div>";
  html += "<script>";
  html += "function scanWiFi() {";
  html += "  fetch('/api/wifi/scan').then(r => r.json()).then(data => {";
  html += "    let html = '<ul style=\"list-style:none;padding:0;\">';";
  html += "    data.forEach(net => {";
  html += "      html += '<li style=\"padding:5px;cursor:pointer;background:#3a3a3a;margin:5px 0;border-radius:5px;\" onclick=\"selectNetwork(\\'' + net.ssid + '\\')\">' + net.ssid + ' (' + net.rssi + ' dBm)</li>';";
  html += "    });";
  html += "    html += '</ul>';";
  html += "    document.getElementById('networks').innerHTML = html;";
  html += "  });";
  html += "}";
  html += "function selectNetwork(ssid) { document.getElementById('ssid').value = ssid; }";
  html += "function saveWiFi() {";
  html += "  const ssid = document.getElementById('ssid').value;";
  html += "  const password = document.getElementById('password').value;";
  html += "  if (!ssid) { showStatus('⚠️ Introdu SSID-ul!', 'error'); return; }";
  html += "  showStatus('⏳ Se salvează...', 'success');";
  html += "  fetch('/api/wifi/config', {";
  html += "    method: 'POST',";
  html += "    headers: { 'Content-Type': 'application/json' },";
  html += "    body: JSON.stringify({ ssid: ssid, password: password })";
  html += "  }).then(r => r.json()).then(data => {";
  html += "    if (data.status === 'success') {";
  html += "      showStatus('✅ WiFi configurat! Reconectare...', 'success');";
  html += "      setTimeout(() => location.reload(), 3000);";
  html += "    } else {";
  html += "      showStatus('❌ Eroare: ' + (data.error || 'Necunoscut'), 'error');";
  html += "    }";
  html += "  }).catch(e => showStatus('❌ Eroare: ' + e, 'error'));";
  html += "}";
  html += "function checkStatus() {";
  html += "  fetch('/api/status').then(r => r.json()).then(data => {";
  html += "    let info = 'WiFi: ' + (data.wifi_connected ? '✅ Conectat' : '❌ Deconectat') + '<br>';";
  html += "    if (data.wifi_connected) info += 'IP: ' + data.ip_address + '<br>';";
  html += "    info += 'MQTT: ' + (data.mqtt_connected ? '✅ Conectat' : '❌ Deconectat') + '<br>';";
  html += "    info += 'Oxigen: ' + data.oxygen_level + '%';";
  html += "    document.getElementById('statusInfo').innerHTML = info;";
  html += "  });";
  html += "}";
  html += "function showStatus(msg, type) {";
  html += "  const div = document.getElementById('status');";
  html += "  div.className = 'status ' + type;";
  html += "  div.innerHTML = msg;";
  html += "}";
  html += "scanWiFi(); checkStatus();";
  html += "</script></body></html>";
  
  server.send(200, "text/html", html);
}

// Funcție helper pentru adăugare header-e CORS
void addCORSHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// Handler pentru status sistem
void handleStatus() {
  addCORSHeaders();
  
  StaticJsonDocument<300> doc;
  doc["status"] = "online";
  doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
  doc["wifi_ssid"] = wifiSSID;
  doc["config_mode"] = configMode;
  if (WiFi.status() == WL_CONNECTED) {
    doc["ip_address"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
  }
  doc["oxygen_level"] = oxygenLevel;
  doc["mqtt_connected"] = client.connected();
  
  String response;
  serializeJson(doc, response);
  
  server.send(200, "application/json", response);
}

// Handler pentru scanare rețele WiFi
void handleWiFiScan() {
  addCORSHeaders();
  
  Serial.println("[WebServer] Scanare rețele WiFi...");
  
  int n = WiFi.scanNetworks();
  
  StaticJsonDocument<2048> doc;
  JsonArray networks = doc.to<JsonArray>();
  
  for (int i = 0; i < n; i++) {
    JsonObject network = networks.createNestedObject();
    network["ssid"] = WiFi.SSID(i);
    network["rssi"] = WiFi.RSSI(i);
    network["encryption"] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "none" : "encrypted";
  }
  
  String response;
  serializeJson(doc, response);
  
  server.send(200, "application/json", response);
  Serial.print("[WebServer] ✓ Găsite ");
  Serial.print(n);
  Serial.println(" rețele WiFi");
}

// Handler pentru configurare WiFi
void handleWiFiConfig() {
  addCORSHeaders();
  
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }
    
    String newSSID = doc["ssid"] | "";
    String newPassword = doc["password"] | "";
    
    if (newSSID.length() == 0) {
      server.send(400, "application/json", "{\"error\":\"SSID required\"}");
      return;
    }
    
    Serial.println("[WebServer] Configurare WiFi nouă:");
    Serial.print("  SSID: ");
    Serial.println(newSSID);
    
    // Salvează credențiale
    saveWiFiCredentials(newSSID, newPassword);
    
    // Răspunde cu succes
    StaticJsonDocument<200> responseDoc;
    responseDoc["status"] = "success";
    responseDoc["message"] = "WiFi credentials saved";
    responseDoc["ssid"] = newSSID;
    
    String response;
    serializeJson(responseDoc, response);
    server.send(200, "application/json", response);
    
    // Reconectează la noul WiFi
    Serial.println("[WebServer] Reconectare la noul WiFi...");
    delay(1000);
    configMode = false;
    WiFi.mode(WIFI_STA);
    connectWiFi();
    
    // Dacă se conectează, reconectează și MQTT
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("[WebServer] ✓ WiFi conectat! Inițializare MQTT...");
      configMode = false;  // Ieșire din modul configurare
      client.setServer(MQTT_SERVER, MQTT_PORT);
      client.setCallback(mqttCallback);
      client.setBufferSize(512);
      connectMQTT();
      
      if (client.connected()) {
        Serial.println("[WebServer] ✓ MQTT conectat! Sistemul este gata.");
      }
    } else {
      Serial.println("[WebServer] ⚠️ WiFi nu s-a conectat, rămâne în modul configurare");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"No data provided\"}");
  }
}

// Handler pentru configurare WiFi curentă
void handleWiFiCurrent() {
  addCORSHeaders();
  
  StaticJsonDocument<200> doc;
  doc["ssid"] = wifiSSID;
  doc["connected"] = (WiFi.status() == WL_CONNECTED);
  if (WiFi.status() == WL_CONNECTED) {
    doc["ip_address"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
  }
  
  String response;
  serializeJson(doc, response);
  
  server.send(200, "application/json", response);
}

// Handler pentru resetare configurare WiFi
void handleWiFiReset() {
  addCORSHeaders();
  
  // Șterge credențiale din EEPROM
  EEPROM.write(EEPROM_WIFI_MAGIC_ADDR, 0);
  EEPROM.write(EEPROM_WIFI_MAGIC_ADDR + 1, 0);
  EEPROM.commit();
  
  wifiSSID = "";
  wifiPassword = "";
  
  StaticJsonDocument<200> doc;
  doc["status"] = "success";
  doc["message"] = "WiFi credentials reset";
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
  
  Serial.println("[WebServer] ✓ Credențiale WiFi resetate");
  delay(1000);
  ESP.restart();  // Repornește pentru a porni AP
}

// Handler pentru date senzor
void handleSensorData() {
  addCORSHeaders();
  
  StaticJsonDocument<300> doc;
  doc["oxygen_level"] = oxygenLevel;
  doc["timestamp"] = millis();
  doc["buzzer_enabled"] = buzzerEnabled;
  doc["relay_state"] = relayState;
  
  String response;
  serializeJson(doc, response);
  
  server.send(200, "application/json", response);
}

// Handler pentru răspunsuri CORS și 404
void handleNotFound() {
  // Adaugă header-e CORS pentru aplicația mobilă
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  
  if (server.method() == HTTP_OPTIONS) {
    server.send(200);
    return;
  }
  
  server.send(404, "application/json", "{\"error\":\"Not found\"}");
}

// ==================== CONEXIUNE MQTT ====================
void connectMQTT() {
  // Verifică mai întâi dacă WiFi-ul este conectat
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi nu este conectat! Reconectare WiFi...");
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Nu se poate conecta la MQTT - WiFi indisponibil!");
      return;
    }
  }
  
  // Verifică dacă deja este conectat
  if (client.connected()) {
    return;
  }
  
  Serial.print("Conectare la server MQTT Adafruit IO (");
  Serial.print(MQTT_SERVER);
  Serial.print(":");
  Serial.print(MQTT_PORT);
  Serial.println(")...");
  
  // Generează un client ID unic pentru fiecare sesiune
  String clientId = String(MQTT_CLIENT_ID) + "_" + String(random(0xffff), HEX);
  
  // Conectare la Adafruit IO cu username și key
  if (client.connect(clientId.c_str(), MQTT_USER, MQTT_KEY)) {
    Serial.println("✓ Conectat la Adafruit IO!");
    Serial.print("Client ID: ");
    Serial.println(clientId);
      
      // Subscrie la topic-uri de comandă
    Serial.println("[MQTT] Subscriere la topic-uri...");
    if (client.subscribe(MQTT_TOPIC_COMMANDS)) {
      Serial.print("[MQTT] ✓ Subscris la COMANZI: ");
      Serial.println(MQTT_TOPIC_COMMANDS);
    } else {
      Serial.print("[MQTT] ✗ EROARE la subscriere la COMANZI: ");
      Serial.println(MQTT_TOPIC_COMMANDS);
    }
    
    // Subscrie la topic-ul de configurare WiFi
    Serial.print("[MQTT] Încercare subscriere la CONFIG: ");
    Serial.println(MQTT_TOPIC_CONFIG);
    if (client.subscribe(MQTT_TOPIC_CONFIG)) {
      Serial.print("[MQTT] ✓✓✓ Subscris cu SUCCES la CONFIG: ");
      Serial.println(MQTT_TOPIC_CONFIG);
      Serial.println("[MQTT] Gata să primească configurare WiFi prin MQTT!");
    } else {
      Serial.print("[MQTT] ✗✗✗ EROARE la subscriere la CONFIG: ");
      Serial.println(MQTT_TOPIC_CONFIG);
      Serial.println("[MQTT] ⚠️ Verifică dacă topic-ul 'config' există în Adafruit IO!");
      Serial.println("[MQTT] ⚠️ Creează feed-ul 'config' în Adafruit IO dacă nu există!");
    }
    
    // Subscrie la topic-ul pentru OTA update
    Serial.print("[MQTT] Încercare subscriere la ESP-CODE (OTA): ");
    Serial.println(MQTT_TOPIC_ESP_CODE);
    if (client.subscribe(MQTT_TOPIC_ESP_CODE)) {
      Serial.print("[MQTT] ✓✓✓ Subscris cu SUCCES la ESP-CODE: ");
      Serial.println(MQTT_TOPIC_ESP_CODE);
      Serial.println("[MQTT] Gata să primească update-uri OTA!");
    } else {
      Serial.print("[MQTT] ✗✗✗ EROARE la subscriere la ESP-CODE: ");
      Serial.println(MQTT_TOPIC_ESP_CODE);
      Serial.println("[MQTT] ⚠️ Verifică dacă feed-ul 'esp-code' există în Adafruit IO!");
      Serial.println("[MQTT] ⚠️ Creează feed-ul 'esp-code' în Adafruit IO pentru OTA updates!");
    }
      
      publishMQTTStatus("online");
    } else {
    Serial.print("✗ Eșec conectare MQTT, rc=");
    int state = client.state();
    Serial.print(state);
    Serial.print(" (");
    switch(state) {
      case -4: Serial.print("MQTT_CONNECTION_TIMEOUT"); break;
      case -3: Serial.print("MQTT_CONNECTION_LOST"); break;
      case -2: Serial.print("MQTT_CONNECT_FAILED - Verifică WiFi și credențiale"); break;
      case -1: Serial.print("MQTT_DISCONNECTED"); break;
      case 1: Serial.print("MQTT_CONNECT_BAD_PROTOCOL"); break;
      case 2: Serial.print("MQTT_CONNECT_BAD_CLIENT_ID"); break;
      case 3: Serial.print("MQTT_CONNECT_UNAVAILABLE"); break;
      case 4: Serial.print("MQTT_CONNECT_BAD_CREDENTIALS"); break;
      case 5: Serial.print("MQTT_CONNECT_UNAUTHORIZED"); break;
      default: Serial.print("UNKNOWN"); break;
    }
    Serial.println(")");
    Serial.println("Reîncercare în 5 secunde...");
      delay(5000);
    }
}

// ==================== VERIFICARE CONFIGURARE PRIN HTTP ====================
// Verifică periodic feed-urile "comands" și "config" prin HTTP API pentru mesaje de configurare WiFi
// (Adafruit IO nu trimite automat mesaje MQTT pentru datele adăugate prin API REST)
void checkConfigViaHTTP() {
  Serial.print("[HTTP] checkConfigViaHTTP() apelată - WiFi status: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "CONECTAT" : "DECONECTAT");
  Serial.print("[HTTP] ConfigMode: ");
  Serial.println(configMode ? "DA (AP)" : "NU (STA)");
  
  // Permite verificarea și în modul AP (dacă dispozitivul conectat la AP are internet)
  // Dar verifică dacă există o conexiune la internet (chiar și prin AP)
  if (WiFi.status() != WL_CONNECTED && !configMode) {
    Serial.println("[HTTP] ⚠️ WiFi nu este conectat și nu suntem în modul AP - nu se poate verifica configurarea prin HTTP");
    return;
  }
  
  if (configMode) {
    Serial.println("[HTTP] ⚠️ În modul AP - verificare configurare (necesită internet pe dispozitivul conectat la AP)");
  } else {
    Serial.println("[HTTP] ✓ WiFi conectat - verificare configurare prin HTTP...");
  }
  
  HTTPClient http;
  
  // Verifică feed-ul "comands" pentru mesaje wifi_config:
  String urlComands = "https://io.adafruit.com/api/v2/" + String(MQTT_USER) + "/feeds/comands/data/last";
  http.begin(urlComands);
  http.addHeader("X-AIO-Key", MQTT_KEY);
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.GET();
  
  Serial.print("[HTTP] Feed 'comands' - HTTP Code: ");
  Serial.println(httpCode);
  
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    http.end();
    
    Serial.print("[HTTP] Payload primit de la 'comands': ");
    Serial.println(payload);
    
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (!error) {
      String messageId = doc["id"] | "";
      String messageValue = doc["value"] | "";
      
      Serial.print("[HTTP] Message ID: '");
      Serial.print(messageId);
      Serial.print("', Last processed ID: '");
      Serial.print(lastProcessedCommandId);
      Serial.println("'");
      Serial.print("[HTTP] Message Value: '");
      Serial.print(messageValue);
      Serial.println("'");
      
      // Verifică dacă este un mesaj nou de configurare WiFi din "comands"
      if (messageValue.startsWith("wifi_config:") && messageId != lastProcessedCommandId) {
        Serial.println("\n[HTTP] ╔════════════════════════════════════════╗");
        Serial.println("[HTTP] ║ 📝 CONFIGURARE WIFI GĂSITĂ PRIN HTTP! ║");
        Serial.println("[HTTP] ╚════════════════════════════════════════╝\n");
        Serial.print("[HTTP] Feed: comands, ID mesaj: ");
        Serial.println(messageId);
        Serial.print("[HTTP] Valoare: ");
        Serial.println(messageValue);
        
        // Procesează mesajul
        String configJson = messageValue.substring(12);
        processWiFiConfig(configJson, messageId);
        return;  // Ieșire - am procesat configurarea
      }
    }
  } else {
    http.end();
  }
  
  // Verifică și feed-ul "config" direct (format JSON simplu)
  String urlConfig = "https://io.adafruit.com/api/v2/" + String(MQTT_USER) + "/feeds/config/data/last";
  http.begin(urlConfig);
  http.addHeader("X-AIO-Key", MQTT_KEY);
  http.addHeader("Content-Type", "application/json");
  
  httpCode = http.GET();
  
  Serial.print("[HTTP] Feed 'config' - HTTP Code: ");
  Serial.println(httpCode);
  
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    http.end();
    
    Serial.print("[HTTP] Payload primit de la 'config': ");
    Serial.println(payload);
    
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (!error) {
      String messageId = doc["id"] | "";
      String messageValue = doc["value"] | "";
      
      Serial.print("[HTTP] Message ID: '");
      Serial.print(messageId);
      Serial.print("', Last processed ID: '");
      Serial.print(lastProcessedCommandId);
      Serial.println("'");
      Serial.print("[HTTP] Message Value: '");
      Serial.print(messageValue);
      Serial.println("'");
      
      // Verifică dacă este un mesaj nou de configurare WiFi din "config"
      // Format: {"ssid":"...","pass":"...","threshold":25.0} sau {"value":"{...}"}
      if (messageId != lastProcessedCommandId) {
        String configJson = "";
        
        // Verifică dacă este format cu "value"
        StaticJsonDocument<256> outerDoc;
        DeserializationError outerError = deserializeJson(outerDoc, messageValue);
        if (!outerError && outerDoc.containsKey("value")) {
          configJson = outerDoc["value"] | "";
        } else {
          // Format direct JSON
          configJson = messageValue;
        }
        
        // Verifică dacă conține "ssid" și "pass"
        if (configJson.indexOf("ssid") >= 0 && configJson.indexOf("pass") >= 0) {
          Serial.println("\n[HTTP] ╔════════════════════════════════════════╗");
          Serial.println("[HTTP] ║ 📝 CONFIGURARE WIFI GĂSITĂ PRIN HTTP! ║");
          Serial.println("[HTTP] ╚════════════════════════════════════════╝\n");
          Serial.print("[HTTP] Feed: config, ID mesaj: ");
          Serial.println(messageId);
          Serial.print("[HTTP] Valoare: ");
          Serial.println(messageValue);
          
          processWiFiConfig(configJson, messageId);
        }
      }
    }
  } else {
    http.end();
  }
}

// Funcție helper pentru procesarea configurației WiFi
void processWiFiConfig(String configJson, String messageId) {
  Serial.print("[HTTP] JSON de procesat: '");
  Serial.print(configJson);
  Serial.println("'");
  
  StaticJsonDocument<256> configDoc;
  DeserializationError configError = deserializeJson(configDoc, configJson);
  
  if (!configError) {
    String newSSID = configDoc["ssid"] | "";
    String newPass = configDoc["pass"] | "";
    float newThreshold = configDoc["threshold"] | OXYGEN_THRESHOLD_25;
    
    Serial.print("[HTTP] SSID extras: '");
    Serial.print(newSSID);
    Serial.print("' (lungime: ");
    Serial.print(newSSID.length());
    Serial.print("), Pass: '");
    Serial.print(newPass.length() > 0 ? "***" : "(gol)");
    Serial.print("' (lungime: ");
    Serial.print(newPass.length());
    Serial.print("), Threshold: ");
    Serial.println(newThreshold);
    
    // Debug: verifică dacă parola este goală
    if (newPass.length() == 0) {
      Serial.println("[HTTP] ⚠️ ATENȚIE: Parola primită este GOLĂ!");
      Serial.println("[HTTP] Verifică dacă aplicația trimite corect câmpul 'pass' în JSON");
    } else {
      Serial.print("[HTTP] Parolă validă primită (lungime: ");
      Serial.print(newPass.length());
      Serial.println(" caractere)");
    }
    
    if (newSSID.length() > 0) {
      // ========== LOGICA ANTI-LOOP: Compară setările actuale cu cele primite ==========
      String oldSSID = prefs.getString("ssid", "");
      String oldPass = prefs.getString("pass", "");
      float oldThreshold = prefs.getFloat("threshold", OXYGEN_THRESHOLD_25);
      
      Serial.println("[HTTP] ========== VERIFICARE ANTI-LOOP ==========");
      Serial.print("[HTTP] SSID actual: '");
      Serial.print(oldSSID);
      Serial.print("' vs SSID nou: '");
      Serial.print(newSSID);
      Serial.println("'");
      Serial.print("[HTTP] Pass actual: '");
      Serial.print(oldPass.length() > 0 ? "***" : "(gol)");
      Serial.print("' (lungime: ");
      Serial.print(oldPass.length());
      Serial.print(") vs Pass nou: '");
      Serial.print(newPass.length() > 0 ? "***" : "(gol)");
      Serial.print("' (lungime: ");
      Serial.print(newPass.length());
      Serial.println(")");
      Serial.print("[HTTP] Threshold actual: ");
      Serial.print(oldThreshold);
      Serial.print(" vs Threshold nou: ");
      Serial.println(newThreshold);
      
      // Compară datele: dacă sunt identice, NU face restart
      bool ssidChanged = (newSSID != oldSSID);
      bool passChanged = (newPass != oldPass);
      bool thresholdChanged = (abs(newThreshold - oldThreshold) > 0.01); // Toleranță pentru float
      
      if (!ssidChanged && !passChanged && !thresholdChanged) {
        Serial.println("[HTTP] ╔═══════════════════════════════════════════════════════╗");
        Serial.println("[HTTP] ║ ⚠️ DATE IDENTICE DETECTATE - ANTI-LOOP ACTIVAT! ║");
        Serial.println("[HTTP] ╚═══════════════════════════════════════════════════════╝");
        Serial.println("[HTTP] >>> Datele primite sunt identice cu cele salvate. <<<");
        Serial.println("[HTTP] >>> Nu este nevoie de restart - evităm loop infinit! <<<");
        Serial.println("[HTTP] >>> Mesajul va fi ignorat pentru a preveni restartul. <<<");
        
        // Marchează mesajul ca procesat pentru a nu-l mai procesa
        lastProcessedCommandId = messageId;
        return; // Ieșire - nu face restart
      }
      
      // Dacă datele sunt diferite, continuă cu salvare și restart
      Serial.println("[HTTP] ╔═══════════════════════════════════════════════════════╗");
      Serial.println("[HTTP] ║ ✓✓✓ DATE NOI DETECTATE - SALVARE ȘI RESTART ✓✓✓ ║");
      Serial.println("[HTTP] ╚═══════════════════════════════════════════════════════╝");
      if (ssidChanged) Serial.println("[HTTP] → SSID s-a schimbat!");
      if (passChanged) Serial.println("[HTTP] → Parola s-a schimbat!");
      if (thresholdChanged) Serial.println("[HTTP] → Threshold s-a schimbat!");
      
      // Salvează configurarea
      bool ssidSaved = prefs.putString("ssid", newSSID);
      bool passSaved = prefs.putString("pass", newPass);
      bool thresholdSaved = prefs.putFloat("threshold", newThreshold);
      
      Serial.print("[HTTP] Salvare - SSID: ");
      Serial.print(ssidSaved ? "✓" : "✗");
      Serial.print(", Pass: ");
      Serial.print(passSaved ? "✓" : "✗");
      Serial.print(" (lungime: ");
      Serial.print(newPass.length());
      Serial.print("), Threshold: ");
      Serial.println(thresholdSaved ? "✓" : "✗");
      
      saveWiFiCredentials(newSSID, newPass);
      
      // Marchează mesajul ca procesat
      lastProcessedCommandId = messageId;
      
      Serial.println("[HTTP] ✓✓✓ Configurare nouă salvată! ✓✓✓");
      Serial.print("[HTTP] SSID: '");
      Serial.print(newSSID);
      Serial.println("'");
      
      // Verificare
      String verifySSID = prefs.getString("ssid", "");
      Serial.print("[HTTP] ✓ Verificare: SSID salvat: '");
      Serial.print(verifySSID);
      Serial.print("' == '");
      Serial.print(newSSID);
      Serial.print("' ? ");
      Serial.println(verifySSID == newSSID ? "✓ DA" : "✗ NU");
      
      if (verifySSID == newSSID) {
        Serial.println("[HTTP] ✓✓✓ CONFIGURARE SALVATĂ CU SUCCES! ✓✓✓");
      } else {
        Serial.println("[HTTP] ⚠️ ATENȚIE: Configurarea nu pare să fie salvată corect!");
      }
      
      // Restart
      Serial.println("[HTTP] ⏳ Restart în 3 secunde pentru a aplica configurarea...");
      Serial.print("[HTTP] După restart, placa ar trebui să se conecteze la: '");
      Serial.print(newSSID);
      Serial.println("'");
      delay(3000);
      ESP.restart();
    } else {
      Serial.println("[HTTP] ✗ SSID invalid sau gol în configurare");
    }
  } else {
    Serial.print("[HTTP] ✗ Eroare la parsare JSON: ");
    Serial.println(configError.c_str());
    Serial.print("[HTTP] JSON primit: '");
    Serial.print(configJson);
    Serial.println("'");
  }
}

// ==================== CALLBACK MQTT ====================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  String topicStr = String(topic);
  
  Serial.println("\n\n╔════════════════════════════════════════╗");
  Serial.print("║ [MQTT] 📨 MESAJ PRIMIT!              ║\n");
  Serial.print("║ Topic: '");
  Serial.print(topicStr);
  int spaces = 30 - topicStr.length();
  for (int i = 0; i < spaces; i++) Serial.print(" ");
  Serial.println("║");
  Serial.print("║ Lungime: ");
  Serial.print(length);
  Serial.print(" bytes");
  spaces = 30 - (String(length).length() + 6);
  for (int i = 0; i < spaces; i++) Serial.print(" ");
  Serial.println("║");
  Serial.print("║ Conținut: ");
  String shortMsg = message;
  if (shortMsg.length() > 20) {
    shortMsg = message.substring(0, 20) + "...";
  }
  Serial.print(shortMsg);
  spaces = 30 - (shortMsg.length() + 11);
  for (int i = 0; i < spaces; i++) Serial.print(" ");
  Serial.println("║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  // Procesare configurare WiFi prin MQTT (topic config)
  // Verifică dacă topic-ul se potrivește (case-sensitive)
  Serial.print("[MQTT DEBUG] Comparare topic: '");
  Serial.print(topicStr);
  Serial.print("' == '");
  Serial.print(MQTT_TOPIC_CONFIG);
  Serial.print("' ? ");
  bool topicMatches = (topicStr == MQTT_TOPIC_CONFIG);
  Serial.println(topicMatches ? "DA ✓" : "NU ✗");
  
  // Verifică dacă topic-ul conține "config" (chiar dacă nu se potrivește exact)
  if (!topicMatches && topicStr.indexOf("config") >= 0) {
    Serial.println("[MQTT] ⚠️ ATENȚIE: Topic-ul conține 'config' dar nu se potrivește exact!");
    Serial.print("[MQTT] Topic primit: '");
    Serial.print(topicStr);
    Serial.print("' (");
    Serial.print(topicStr.length());
    Serial.println(" caractere)");
    Serial.print("[MQTT] Topic așteptat: '");
    Serial.print(MQTT_TOPIC_CONFIG);
    Serial.print("' (");
    Serial.print(String(MQTT_TOPIC_CONFIG).length());
    Serial.println(" caractere)");
  }
  
  // Procesează configurarea WiFi dacă topic-ul se potrivește SAU dacă mesajul conține "ssid"
  bool shouldProcess = (topicStr == MQTT_TOPIC_CONFIG) || 
                       (topicStr.indexOf("config") >= 0 && message.indexOf("ssid") >= 0);
  
  if (shouldProcess) {
    Serial.println("[MQTT] 📝✓✓✓ Configurare WiFi detectată! ✓✓✓");
    Serial.print("[MQTT] Topic: '");
    Serial.print(topicStr);
    Serial.println("'");
    Serial.print("[MQTT] Mesaj primit (raw): ");
  Serial.println(message);
    
    // Aplicația poate trimite JSON-ul în două formate:
    // 1. Format cu "value": {"value":"{\"ssid\":\"...\",\"pass\":\"...\",\"threshold\":25.0}"}
    // 2. Format direct JSON: {"ssid":"(-_-)","pass":"...","threshold":25}
    
    String newSSID = "";
    String newPass = "";
    float newThreshold = OXYGEN_THRESHOLD_25;
    bool configParsed = false;
    
    StaticJsonDocument<512> outerDoc;
    DeserializationError error = deserializeJson(outerDoc, message);
    
    if (!error) {
      // Verifică dacă există câmpul "value" (format 1)
      if (outerDoc.containsKey("value")) {
        // Format 1: {"value":"{...}"}
        String configJson = outerDoc["value"] | "";
        if (configJson.length() > 0) {
          Serial.print("[MQTT] Format cu 'value' detectat, JSON: ");
          Serial.println(configJson);
          
          StaticJsonDocument<256> configDoc;
          DeserializationError configError = deserializeJson(configDoc, configJson);
          if (!configError) {
            newSSID = configDoc["ssid"] | "";
            newPass = configDoc["pass"] | "";
            newThreshold = configDoc["threshold"] | OXYGEN_THRESHOLD_25;
            configParsed = true;
          } else {
            Serial.print("[MQTT] ✗ Eroare la parsare JSON din 'value': ");
            Serial.println(configError.c_str());
            Serial.print("[MQTT] JSON din 'value': ");
            Serial.println(configJson);
          }
        } else {
          Serial.println("[MQTT] ✗ Câmpul 'value' este gol sau lipsă");
        }
      } else if (outerDoc.containsKey("ssid")) {
        // Format 2: JSON direct {"ssid":"...","pass":"...","threshold":25}
        Serial.println("[MQTT] Format direct JSON detectat (fără 'value')");
        newSSID = outerDoc["ssid"] | "";
        newPass = outerDoc["pass"] | "";
        newThreshold = outerDoc["threshold"] | OXYGEN_THRESHOLD_25;
        configParsed = true;
      }
    } else {
      Serial.print("[MQTT] ✗ Eroare la parsare JSON exterior: ");
      Serial.println(error.c_str());
      Serial.print("[MQTT] Mesaj primit (raw): ");
      Serial.println(message);
    }
    
    if (configParsed) {
      Serial.print("[MQTT] SSID extras: '");
      Serial.print(newSSID);
      Serial.print("', Pass: '");
      Serial.print(newPass);
      Serial.print("', Threshold: ");
      Serial.println(newThreshold);
      
      if (newSSID.length() > 0) {
        // Salvează configurarea în Preferences
        prefs.putString("ssid", newSSID);
        prefs.putString("pass", newPass);
        prefs.putFloat("threshold", newThreshold);
        
        // ========== LOGICA ANTI-LOOP: Compară setările actuale cu cele primite ==========
        String oldSSID = prefs.getString("ssid", "");
        String oldPass = prefs.getString("pass", "");
        float oldThreshold = prefs.getFloat("threshold", OXYGEN_THRESHOLD_25);
        
        Serial.println("[MQTT] ========== VERIFICARE ANTI-LOOP ==========");
        Serial.print("[MQTT] SSID actual: '");
        Serial.print(oldSSID);
        Serial.print("' vs SSID nou: '");
        Serial.print(newSSID);
        Serial.println("'");
        Serial.print("[MQTT] Pass actual: '");
        Serial.print(oldPass.length() > 0 ? "***" : "(gol)");
        Serial.print("' (lungime: ");
        Serial.print(oldPass.length());
        Serial.print(") vs Pass nou: '");
        Serial.print(newPass.length() > 0 ? "***" : "(gol)");
        Serial.print("' (lungime: ");
        Serial.print(newPass.length());
        Serial.println(")");
        Serial.print("[MQTT] Threshold actual: ");
        Serial.print(oldThreshold);
        Serial.print(" vs Threshold nou: ");
        Serial.println(newThreshold);
        
        // Compară datele: dacă sunt identice, NU face restart
        bool ssidChanged = (newSSID != oldSSID);
        bool passChanged = (newPass != oldPass);
        bool thresholdChanged = (abs(newThreshold - oldThreshold) > 0.01); // Toleranță pentru float
        
        if (!ssidChanged && !passChanged && !thresholdChanged) {
          Serial.println("[MQTT] ╔═══════════════════════════════════════════════════════╗");
          Serial.println("[MQTT] ║ ⚠️ DATE IDENTICE DETECTATE - ANTI-LOOP ACTIVAT! ║");
          Serial.println("[MQTT] ╚═══════════════════════════════════════════════════════╝");
          Serial.println("[MQTT] >>> Datele primite sunt identice cu cele salvate. <<<");
          Serial.println("[MQTT] >>> Nu este nevoie de restart - evităm loop infinit! <<<");
          Serial.println("[MQTT] >>> Mesajul va fi ignorat pentru a preveni restartul. <<<");
          return; // Ieșire - nu face restart
        }
        
        // Dacă datele sunt diferite, continuă cu salvare și restart
        Serial.println("[MQTT] ╔═══════════════════════════════════════════════════════╗");
        Serial.println("[MQTT] ║ ✓✓✓ DATE NOI DETECTATE - SALVARE ȘI RESTART ✓✓✓ ║");
        Serial.println("[MQTT] ╚═══════════════════════════════════════════════════════╝");
        if (ssidChanged) Serial.println("[MQTT] → SSID s-a schimbat!");
        if (passChanged) Serial.println("[MQTT] → Parola s-a schimbat!");
        if (thresholdChanged) Serial.println("[MQTT] → Threshold s-a schimbat!");
        
        // Salvează și prin funcția existentă pentru compatibilitate
        saveWiFiCredentials(newSSID, newPass);
        
        Serial.println("[MQTT] ✓✓✓ Configurare nouă salvată în Preferences! ✓✓✓");
        Serial.print("[MQTT] SSID: ");
        Serial.println(newSSID);
        Serial.print("[MQTT] Threshold: ");
        Serial.println(newThreshold);
        
        // Publică confirmare la status
        publishMQTTStatus("config_saved");
        
        // Publică confirmare și la comands pentru vizibilitate
        String confirmMsg = "config_saved:SSID=" + newSSID;
        if (client.publish(MQTT_TOPIC_COMMANDS, confirmMsg.c_str())) {
          Serial.print("[MQTT] ✓ Confirmare trimisă la COMANDS: ");
          Serial.println(confirmMsg);
        } else {
          Serial.println("[MQTT] ✗ Eroare la trimitere confirmare la COMANDS");
        }
        
        // Forțează sincronizare
        client.loop();
        delay(500);
        
        // Restart pentru a aplica noile setări
        Serial.println("[MQTT] ⏳ Restart în 3 secunde pentru a aplica configurarea...");
        Serial.println("[MQTT] După restart, placa ar trebui să se conecteze la: " + newSSID);
        delay(3000);
        ESP.restart();
      } else {
        Serial.println("[MQTT] ✗ SSID invalid sau gol în configurare");
      }
    } else {
      Serial.println("[MQTT] ✗ Configurarea nu a putut fi parsată");
    }
    return;  // Ieșire - nu procesează alte comenzi
  }
  
  // Dacă nu este mesaj de configurare, verifică dacă topic-ul nu se potrivește
  if (topicStr.indexOf("config") >= 0 && !shouldProcess) {
    Serial.print("[MQTT DEBUG] Topic conține 'config' dar nu s-a procesat. Topic: '");
    Serial.print(topicStr);
    Serial.println("'");
  }
  
  // Procesare update OTA (topic esp-code)
  if (String(topic) == MQTT_TOPIC_ESP_CODE) {
    Serial.println("\n[MQTT] ╔════════════════════════════════════════╗");
    Serial.println("[MQTT] ║ 🔄 UPDATE OTA DETECTAT!              ║");
    Serial.println("[MQTT] ╚════════════════════════════════════════╝\n");
    Serial.print("[MQTT] URL primit: ");
    Serial.println(message);
    
    // Verifică dacă este un URL valid
    if (message.startsWith("http://") || message.startsWith("https://")) {
      // Verifică dacă nu este același URL procesat anterior (anti-loop)
      if (message != lastProcessedOTAUrl) {
        lastProcessedOTAUrl = message;
        Serial.println("[MQTT] ✓ URL nou detectat - inițiere update OTA...");
        performOTAUpdate(message);
      } else {
        Serial.println("[MQTT] ⚠️ Același URL a fost deja procesat - ignorat (anti-loop)");
      }
    } else {
      Serial.println("[MQTT] ✗ URL invalid - trebuie să înceapă cu http:// sau https://");
    }
    return;  // Ieșire - nu procesează alte mesaje
  }
  
  // Procesare comenzi (buzzer, fan, calibrare)
  if (String(topic) == MQTT_TOPIC_COMMANDS) {
    Serial.print("[MQTT] Mesaj primit pe COMANDS: '");
    Serial.print(message);
    Serial.print("' (lungime: ");
    Serial.print(message.length());
    Serial.println(")");
    
    // Comandă pentru configurare WiFi (NOU - trimisă prin "comands" pentru a funcționa prin MQTT)
    if (message.startsWith("wifi_config:")) {
      Serial.println("\n[MQTT] ╔════════════════════════════════════════╗");
      Serial.println("[MQTT] ║ 📝✓✓✓ CONFIGURARE WIFI DETECTATĂ! ✓✓✓ ║");
      Serial.println("[MQTT] ╚════════════════════════════════════════╝\n");
      
      String configJson = message.substring(12);  // Elimină "wifi_config:"
      Serial.print("[MQTT] Prefix eliminat, JSON rămas: '");
      Serial.print(configJson);
      Serial.println("'");
      
      StaticJsonDocument<256> configDoc;
      DeserializationError configError = deserializeJson(configDoc, configJson);
      
      if (!configError) {
        String newSSID = configDoc["ssid"] | "";
        String newPass = configDoc["pass"] | "";
        float newThreshold = configDoc["threshold"] | OXYGEN_THRESHOLD_25;
        
        Serial.print("[MQTT] SSID extras: '");
        Serial.print(newSSID);
        Serial.print("', Pass: '");
        Serial.print(newPass);
        Serial.print("', Threshold: ");
        Serial.println(newThreshold);
        
        if (newSSID.length() > 0) {
          // ========== LOGICA ANTI-LOOP: Compară setările actuale cu cele primite ==========
          String oldSSID = prefs.getString("ssid", "");
          String oldPass = prefs.getString("pass", "");
          float oldThreshold = prefs.getFloat("threshold", OXYGEN_THRESHOLD_25);
          
          Serial.println("[MQTT] ========== VERIFICARE ANTI-LOOP ==========");
          Serial.print("[MQTT] SSID actual: '");
          Serial.print(oldSSID);
          Serial.print("' vs SSID nou: '");
          Serial.print(newSSID);
          Serial.println("'");
          Serial.print("[MQTT] Pass actual: '");
          Serial.print(oldPass.length() > 0 ? "***" : "(gol)");
          Serial.print("' (lungime: ");
          Serial.print(oldPass.length());
          Serial.print(") vs Pass nou: '");
          Serial.print(newPass.length() > 0 ? "***" : "(gol)");
          Serial.print("' (lungime: ");
          Serial.print(newPass.length());
          Serial.println(")");
          Serial.print("[MQTT] Threshold actual: ");
          Serial.print(oldThreshold);
          Serial.print(" vs Threshold nou: ");
          Serial.println(newThreshold);
          
          // Compară datele: dacă sunt identice, NU face restart
          bool ssidChanged = (newSSID != oldSSID);
          bool passChanged = (newPass != oldPass);
          bool thresholdChanged = (abs(newThreshold - oldThreshold) > 0.01); // Toleranță pentru float
          
          if (!ssidChanged && !passChanged && !thresholdChanged) {
            Serial.println("[MQTT] ╔═══════════════════════════════════════════════════════╗");
            Serial.println("[MQTT] ║ ⚠️ DATE IDENTICE DETECTATE - ANTI-LOOP ACTIVAT! ║");
            Serial.println("[MQTT] ╚═══════════════════════════════════════════════════════╝");
            Serial.println("[MQTT] >>> Datele primite sunt identice cu cele salvate. <<<");
            Serial.println("[MQTT] >>> Nu este nevoie de restart - evităm loop infinit! <<<");
            Serial.println("[MQTT] >>> Mesajul va fi ignorat pentru a preveni restartul. <<<");
            return; // Ieșire - nu face restart
          }
          
          // Dacă datele sunt diferite, continuă cu salvare și restart
          Serial.println("[MQTT] ╔═══════════════════════════════════════════════════════╗");
          Serial.println("[MQTT] ║ ✓✓✓ DATE NOI DETECTATE - SALVARE ȘI RESTART ✓✓✓ ║");
          Serial.println("[MQTT] ╚═══════════════════════════════════════════════════════╝");
          if (ssidChanged) Serial.println("[MQTT] → SSID s-a schimbat!");
          if (passChanged) Serial.println("[MQTT] → Parola s-a schimbat!");
          if (thresholdChanged) Serial.println("[MQTT] → Threshold s-a schimbat!");
          
          // Salvează configurarea în Preferences
          prefs.putString("ssid", newSSID);
          prefs.putString("pass", newPass);
          prefs.putFloat("threshold", newThreshold);
          
          // Salvează și prin funcția existentă pentru compatibilitate
          saveWiFiCredentials(newSSID, newPass);
          
          Serial.println("[MQTT] ✓✓✓ Configurare nouă salvată în Preferences! ✓✓✓");
          Serial.print("[MQTT] SSID: ");
          Serial.println(newSSID);
          Serial.print("[MQTT] Threshold: ");
          Serial.println(newThreshold);
          
          // Publică confirmare
          publishMQTTStatus("config_saved");
          
          // Restart pentru a aplica noile setări
          Serial.println("[MQTT] ⏳ Restart în 3 secunde pentru a aplica configurarea...");
          Serial.println("[MQTT] După restart, placa ar trebui să se conecteze la: " + newSSID);
          delay(3000);
          ESP.restart();
        } else {
          Serial.println("[MQTT] ✗ SSID invalid sau gol în configurare");
        }
      } else {
        Serial.print("[MQTT] ✗ Eroare la parsare JSON: ");
        Serial.println(configError.c_str());
      }
      return;  // Ieșire - nu procesează alte comenzi
    }
    
    // Comenzi pentru control manual buzzer
    if (message == "buzzer_on") {
      // Dacă buzzerul a fost oprit prin buton, nu permite reactivarea din aplicație
      // până când oxigenul revine la normal SAU până când se trimite explicit buzzer_off
      if (buzzerMuted) {
        Serial.println("[MQTT] ⚠️ Buzzer oprit prin buton - nu se poate activa din aplicație");
        Serial.println("[MQTT] ⚠️ Apasă butonul din nou sau așteaptă până oxigen < 21%");
        publishMQTTStatus("buzzer_blocked_by_button");
        // Forțează buzzerul să rămână oprit - IMPORTANT!
      digitalWrite(BUZZER_PIN, LOW);
        manualBuzzer = false;  // Dezactivează modul manual
        buzzerEnabled = false; // Dezactivează buzzerul
        return;  // Ieșire imediată - nu permite reactivarea
      }
      manualBuzzer = true;   // Activează modul manual
      buzzerMuted = false;   // Dezactivează mute (dacă nu era deja oprit)
      buzzerEnabled = true;  // Activează și flag-ul global
      digitalWrite(BUZZER_PIN, HIGH);
      Serial.println("[MQTT] ✓ Buzzer ACTIVAT MANUAL (mod manual activ)");
      publishMQTTStatus("buzzer_on_manual");
    } 
    else if (message == "buzzer_off" || message == "buzzer_off_button") {
      manualBuzzer = false;  // Dezactivează modul manual, revine la automat
      buzzerEnabled = false;  // Dezactivează buzzerul complet
      buzzerMuted = true;     // Activează mute
      digitalWrite(BUZZER_PIN, LOW);
      Serial.println("[MQTT] ✓ Buzzer DEZACTIVAT (revine la mod automat)");
      publishMQTTStatus("buzzer_off_manual");
    }
    // Comenzi pentru control manual ventilator (fan)
    else if (message == "fan_on") {
      manualFan = true;  // Activează modul manual
      digitalWrite(RELAY_PIN, HIGH);
      relayState = true;
      Serial.println("[MQTT] ✓ Ventilator ACTIVAT MANUAL (mod manual activ)");
      publishMQTTStatus("fan_on_manual");
    }
    else if (message == "fan_off") {
      manualFan = false;  // Dezactivează modul manual, revine la automat
      digitalWrite(RELAY_PIN, LOW);
      relayState = false;
      Serial.println("[MQTT] ✓ Ventilator DEZACTIVAT (revine la mod automat)");
      publishMQTTStatus("fan_off_manual");
    }
    // Comenzi calibrare
    else if (message == "calibrate") {
      // Calibrare automată cu valoare default 20.9%
      startCalibration(20.9);
    } else if (message.startsWith("calibrate:")) {
      // Calibrare cu valoare specificată: "calibrate:21.0"
      float refValue = message.substring(10).toFloat();
      if (refValue > 0 && refValue <= 100) {
        startCalibration(refValue);
      } else {
        Serial.println("[CALIBRARE] ✗ Valoare de referință invalidă!");
      }
    } else if (message.startsWith("set_offset:")) {
      // Setează offset manual: "set_offset:7.4"
      float offset = message.substring(11).toFloat();
      setCalibrationOffset(offset);
    } else if (message.startsWith("set_factor:")) {
      // Setează factor manual: "set_factor:1.05"
      float factor = message.substring(11).toFloat();
      if (factor > 0 && factor <= 2.0) {
        setCalibrationFactor(factor);
      } else {
        Serial.println("[CALIBRARE] ✗ Factor invalid (trebuie să fie între 0.1 și 2.0)!");
      }
    } else if (message == "reset_calibration") {
      resetCalibration();
    } else if (message == "calibration_status") {
      // Afișează statusul calibrării
      Serial.println("[CALIBRARE] ========================================");
      Serial.print("[CALIBRARE] Offset: ");
      Serial.print(calibrationOffset);
      Serial.println("%");
      Serial.print("[CALIBRARE] Factor: ");
      Serial.println(calibrationFactor);
      Serial.print("[CALIBRARE] Valoare de referință: ");
      Serial.print(referenceValue);
      Serial.println("%");
      Serial.print("[CALIBRARE] Status: ");
      Serial.println(calibrationActive ? "ACTIVĂ" : "INACTIVĂ");
      Serial.println("[CALIBRARE] ========================================");
    }
  }
}

// ==================== PUBLICARE DATE MQTT ====================
void publishMQTTData() {
  if (!client.connected()) {
    Serial.println("[MQTT] Nu este conectat - nu se pot publica date");
    return;
  }
  
  // Nu publică 0.00 dacă nu avem date valide de la senzor
  if (oxygenLevel == 0.0 && (lastSensorDataReceived == 0 || (millis() - lastSensorDataReceived > 10000))) {
    Serial.println("[MQTT] ⚠️ Nu se publică - fără date valide de la senzor");
    return;
  }
  
  // Adafruit IO așteaptă doar valoarea numerică, nu JSON
  // Trimite doar valoarea oxigenului ca string
  String valueStr = String(oxygenLevel, 2);  // 2 zecimale
  
  Serial.print("[MQTT] Publicare date: ");
  Serial.print(valueStr);
  Serial.print("% la topic: ");
  Serial.println(MQTT_TOPIC_DATA);
  
  if (client.publish(MQTT_TOPIC_DATA, valueStr.c_str())) {
    Serial.println("[MQTT] ✓ Date publicate cu succes la Adafruit IO");
  } else {
    Serial.println("[MQTT] ✗ Eroare la publicare date");
  }
}

// ==================== PUBLICARE STATUS MQTT ====================
void publishMQTTStatus(String status) {
  if (!client.connected()) {
    return;
  }
  
  StaticJsonDocument<100> doc;
  doc["status"] = status;
  doc["oxygen_level"] = oxygenLevel;
  doc["timestamp"] = millis();
  
  char jsonBuffer[100];
  serializeJson(doc, jsonBuffer);
  
  client.publish(MQTT_TOPIC_STATUS, jsonBuffer);
}

// ==================== FUNCȚII CALIBRARE ====================

// Salvează calibrarea în EEPROM
void saveCalibration() {
  EEPROM.put(EEPROM_OFFSET_ADDR, calibrationOffset);
  EEPROM.put(EEPROM_FACTOR_ADDR, calibrationFactor);
  EEPROM.write(EEPROM_MAGIC_ADDR, (EEPROM_MAGIC_VALUE >> 8) & 0xFF);
  EEPROM.write(EEPROM_MAGIC_ADDR + 1, EEPROM_MAGIC_VALUE & 0xFF);
  EEPROM.commit();
  
  Serial.println("[CALIBRARE] ✓ Calibrare salvată în EEPROM");
  Serial.print("[CALIBRARE] Offset: ");
  Serial.print(calibrationOffset);
  Serial.print("%, Factor: ");
  Serial.println(calibrationFactor);
}

// Încarcă calibrarea din EEPROM
void loadCalibration() {
  // Verifică magic number pentru validitate
  uint8_t magicHigh = EEPROM.read(EEPROM_MAGIC_ADDR);
  uint8_t magicLow = EEPROM.read(EEPROM_MAGIC_ADDR + 1);
  uint16_t magic = (magicHigh << 8) | magicLow;
  
  if (magic == EEPROM_MAGIC_VALUE) {
    EEPROM.get(EEPROM_OFFSET_ADDR, calibrationOffset);
    EEPROM.get(EEPROM_FACTOR_ADDR, calibrationFactor);
    
    Serial.println("[CALIBRARE] ✓ Calibrare încărcată din EEPROM");
    Serial.print("[CALIBRARE] Offset: ");
    Serial.print(calibrationOffset);
    Serial.print("%, Factor: ");
    Serial.println(calibrationFactor);
  } else {
    Serial.println("[CALIBRARE] ⚠️ Nu există calibrare salvată, folosind valori default");
    calibrationOffset = 0.0;
    calibrationFactor = 1.0;
  }
}

// Colectează eșantion pentru calibrare
void collectCalibrationSample(float value) {
  if (calibrationSampleCount < CALIBRATION_SAMPLES) {
    calibrationSamples[calibrationSampleCount] = value;
    calibrationSampleCount++;
    
    Serial.print("[CALIBRARE] Eșantion ");
    Serial.print(calibrationSampleCount);
    Serial.print("/");
    Serial.print(CALIBRATION_SAMPLES);
    Serial.print(": ");
    Serial.print(value);
    Serial.println("%");
  }
  
  // Când am colectat toate eșantioanele, calculează offset-ul
  if (calibrationSampleCount >= CALIBRATION_SAMPLES) {
    finishCalibration();
  }
}

// Finalizează calibrarea calculând offset-ul
void finishCalibration() {
  // Calculează media eșantioanelor
  float average = 0.0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    average += calibrationSamples[i];
  }
  average /= CALIBRATION_SAMPLES;
  
  // Calculează offset-ul necesar pentru a ajunge la valoarea de referință
  // offset = referenceValue - average
  calibrationOffset = referenceValue - average;
  
  Serial.println("[CALIBRARE] ========================================");
  Serial.print("[CALIBRARE] Media eșantioanelor: ");
  Serial.print(average);
  Serial.println("%");
  Serial.print("[CALIBRARE] Valoare de referință: ");
  Serial.print(referenceValue);
  Serial.println("%");
  Serial.print("[CALIBRARE] Offset calculat: ");
  Serial.print(calibrationOffset);
  Serial.println("%");
  Serial.println("[CALIBRARE] ========================================");
  
  // Salvează calibrarea
  saveCalibration();
  
  // Resetează modul calibrare
  calibrationActive = false;
  calibrationSampleCount = 0;
  
  Serial.println("[CALIBRARE] ✓ Calibrare finalizată!");
}

// Pornește procesul de calibrare
void startCalibration(float reference) {
  if (calibrationActive) {
    Serial.println("[CALIBRARE] ⚠️ Calibrare deja în curs!");
    return;
  }
  
  referenceValue = reference;
  calibrationActive = true;
  calibrationSampleCount = 0;
  calibrationStartTime = millis();
  
  Serial.println("[CALIBRARE] ========================================");
  Serial.println("[CALIBRARE] 🎯 CALIBRARE PORNITĂ");
  Serial.print("[CALIBRARE] Valoare de referință: ");
  Serial.print(referenceValue);
  Serial.println("%");
  Serial.print("[CALIBRARE] Colectare ");
  Serial.print(CALIBRATION_SAMPLES);
  Serial.println(" eșantioane...");
  Serial.println("[CALIBRARE] Asigură-te că senzorul este în aer normal!");
  Serial.println("[CALIBRARE] ========================================");
}

// Setează manual offset-ul
void setCalibrationOffset(float offset) {
  calibrationOffset = offset;
  saveCalibration();
  
  Serial.print("[CALIBRARE] ✓ Offset setat manual: ");
  Serial.print(calibrationOffset);
  Serial.println("%");
}

// Setează manual factorul de corecție
void setCalibrationFactor(float factor) {
  calibrationFactor = factor;
  saveCalibration();
  
  Serial.print("[CALIBRARE] ✓ Factor setat manual: ");
  Serial.println(calibrationFactor);
}

// Resetează calibrarea la valori default
void resetCalibration() {
  calibrationOffset = 0.0;
  calibrationFactor = 1.0;
  saveCalibration();
  
  Serial.println("[CALIBRARE] ✓ Calibrare resetată la valori default");
}

// ==================== OTA UPDATE FUNCTIONS ====================

// Verifică feed-ul "esp-code" prin HTTP API pentru update-uri OTA
void checkOTAUpdate() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  
  Serial.println("[OTA] Verificare update disponibil...");
  
  HTTPClient http;
  String url = "https://io.adafruit.com/api/v2/" + String(MQTT_USER) + "/feeds/esp-code/data/last";
  http.begin(url);
  http.addHeader("X-AIO-Key", MQTT_KEY);
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    http.end();
    
    Serial.print("[OTA] Payload primit: ");
    Serial.println(payload);
    
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (!error) {
      String messageId = doc["id"] | "";
      String firmwareUrl = doc["value"] | "";
      
      Serial.print("[OTA] Message ID: ");
      Serial.println(messageId);
      Serial.print("[OTA] Firmware URL: ");
      Serial.println(firmwareUrl);
      
      // Verifică dacă este un URL valid și nu este același procesat anterior
      if ((firmwareUrl.startsWith("http://") || firmwareUrl.startsWith("https://")) && 
          firmwareUrl != lastProcessedOTAUrl) {
        Serial.println("[OTA] ╔════════════════════════════════════════╗");
        Serial.println("[OTA] ║ 🔄 UPDATE OTA DISPONIBIL!            ║");
        Serial.println("[OTA] ╚════════════════════════════════════════╝");
        Serial.print("[OTA] URL: ");
        Serial.println(firmwareUrl);
        
        lastProcessedOTAUrl = firmwareUrl;
        performOTAUpdate(firmwareUrl);
      } else if (firmwareUrl == lastProcessedOTAUrl) {
        Serial.println("[OTA] ⚠️ Același URL a fost deja procesat - ignorat (anti-loop)");
      } else {
        Serial.println("[OTA] ⚠️ URL invalid sau gol");
      }
    } else {
      Serial.print("[OTA] ✗ Eroare la parsare JSON: ");
      Serial.println(error.c_str());
    }
  } else {
    http.end();
    Serial.print("[OTA] ⚠️ Eroare HTTP: ");
    Serial.println(httpCode);
  }
}

// Efectuează update-ul OTA descărcând și instalând binarul
bool performOTAUpdate(String firmwareUrl) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA] ✗ WiFi nu este conectat - nu se poate efectua update!");
    return false;
  }
  
  Serial.println("\n[OTA] ╔═══════════════════════════════════════════════════════╗");
  Serial.println("[OTA] ║ 🚀 ÎNCEPE UPDATE OTA                              ║");
  Serial.println("[OTA] ╚═══════════════════════════════════════════════════════╝");
  Serial.print("[OTA] URL firmware: ");
  Serial.println(firmwareUrl);
  Serial.println("[OTA] Descărcare și instalare în curs...");
  Serial.println("[OTA] ⚠️ NU OPRI ALIMENTAREA ÎN TIMPUL UPDATE-ULUI!");
  Serial.println("[OTA] ⚠️ Procesul poate dura 30-60 secunde...");
  
  // Publică status înainte de update
  publishMQTTStatus("ota_update_starting");
  
  // Mesaje de progres
  httpUpdate.onStart([]() {
    Serial.println("[OTA] ═════════════════════════════════════════");
    Serial.println("[OTA] 🔄 Update început - descărcare binar...");
    Serial.println("[OTA] ═════════════════════════════════════════");
  });
  
  httpUpdate.onEnd([]() {
    Serial.println("[OTA] ═════════════════════════════════════════");
    Serial.println("[OTA] ✅ Update finalizat cu succes!");
    Serial.println("[OTA] ═════════════════════════════════════════");
  });
  
  httpUpdate.onProgress([](int current, int total) {
    int progress = (current * 100) / total;
    Serial.print("[OTA] Progres: ");
    Serial.print(progress);
    Serial.print("% (");
    Serial.print(current);
    Serial.print("/");
    Serial.print(total);
    Serial.println(" bytes)");
  });
  
  httpUpdate.onError([](int error) {
    Serial.print("[OTA] ✗ Eroare la update: ");
    switch(error) {
      case HTTP_UE_TOO_LESS_SPACE:
        Serial.println("Spațiu insuficient");
        break;
      case HTTP_UE_SERVER_NOT_REPORT_SIZE:
        Serial.println("Serverul nu raportează dimensiunea");
        break;
      case HTTP_UE_SERVER_FILE_NOT_FOUND:
        Serial.println("Fișierul nu a fost găsit pe server");
        break;
      case HTTP_UE_SERVER_FORBIDDEN:
        Serial.println("Acces interzis (403)");
        break;
      case HTTP_UE_SERVER_WRONG_HTTP_CODE:
        Serial.println("Cod HTTP greșit");
        break;
      case HTTP_UE_SERVER_FAULTY_MD5:
        Serial.println("MD5 greșit");
        break;
      case HTTP_UE_BIN_VERIFY_HEADER_FAILED:
        Serial.println("Verificare header eșuată");
        break;
      case HTTP_UE_BIN_FOR_WRONG_FLASH:
        Serial.println("Binar pentru flash greșit");
        break;
      default:
        Serial.print("Eroare necunoscută: ");
        Serial.println(error);
    }
  });
  
  // Creează HTTPClient pentru descărcare
  HTTPClient http;
  http.begin(firmwareUrl);
  
  // Efectuează update-ul folosind HTTPClient
  t_httpUpdate_return ret = httpUpdate.update(http);
  
  http.end();
  
  if (ret == HTTP_UPDATE_OK) {
    Serial.println("[OTA] ╔════════════════════════════════════════╗");
    Serial.println("[OTA] ║ ✅ UPDATE COMPLET CU SUCCES!         ║");
    Serial.println("[OTA] ║ Sistemul va reporni automat...        ║");
    Serial.println("[OTA] ╚════════════════════════════════════════╝");
    
    // Publică status de succes
    publishMQTTStatus("ota_update_success");
    delay(1000);
    
    // Repornește pentru a aplica noul firmware
    ESP.restart();
    return true;
  } else {
    Serial.println("[OTA] ✗ Update eșuat!");
    publishMQTTStatus("ota_update_failed");
    return false;
  }
}
