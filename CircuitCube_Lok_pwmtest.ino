/*
 * CircuitCube_Lok_pwmtest_v0.2.1
 * PWM-Testsketch: BLE-Verbindung, Poti → Geschwindigkeit → PWM → Kanal A
 * NEU v0.2.0: Akkustandabfrage über TX-Characteristic (Notify)
 * FIX v0.2.1: Akkuantwort ist ASCII-Spannungswert (z.B. "3.73"), kein Rohbyte
 *
 * Zweck: Anlauf-PWM-Schwellwert empirisch ermitteln + Akkuprotokoll verifizieren
 *
 * Autor: Claude (Anthropic) – claude-sonnet-4-6
 *
 * Pinbelegung:
 *   GPIO 34 – Fahr-Potentiometer (ADC, 0–4095)
 *   GPIO 32 – Richtungsschalter (INPUT_PULLUP, LOW = rückwärts)
 *
 * Serielle Ausgabe (115200 Baud):
 *   Geschw: 0.00  PWM:   0  Richtung: V  Akku: ---
 *   Geschw: 0.47  PWM: 119  Richtung: V  Akku:  72%
 *
 * Akkuabfrage-Protokoll (Circuit Cube / Tenka):
 *   Anfrage:  ASCII "b" auf RX-Characteristic (6E400002) schreiben
 *   Antwort:  4-Byte ASCII-String via Notify auf TX-Characteristic (6E400003)
 *             Format: "X.XX" = Akkuspannung in Volt (z.B. "3.73")
 *             Vollladung ~4.20 V, Entladegrenze ~3.30–3.50 V
 *   Hinweis:  Hex-Dump der Rohantwort wird immer ausgegeben (Protokollverifikation)
 *   Ergebniss: Der ESP32 gibt 3,73V aus, die Android App meldet 73%
 *
 * PWM-Messwerte – CircuitCube Lok / Motor-Anlaufverhalten
 * Ermittelt mit: CircuitCube_Lok_pwmtest v0.1
 * Hardware: ESP32 DevKit V1, Circuit Cube (TenkaC256), Köf II (Stoneheap)
 *
 * Richtung   Bedingung    Minimalgeschw. (PWM)   Anlauf (PWM)
 * ---------  -----------  ---------------------  ------------
 * Vorwärts   ohne Last     55                     70
 * Vorwärts   mit Last     110                    120
 * Rückwärts  ohne Last     50                     60
 * Rückwärts  mit Last     100                    100
 *
 * Hinweise:
 * - Vorwärts benötigt durchgehend mehr PWM als Rückwärts
 *   (asymmetrische Lagerreibung oder Vorzugswicklungswinkel des Ankers)
 * - Anlauf-PWM liegt nur knapp über Minimalgeschwindigkeit
 * - Worst Case: Vorwärts mit Last, PWM 120
 * - Empfohlener PWM_KICK-Ausgangswert: 130 (etwas oberhalb Worst Case)
 * - Werte variieren je nach Aufdreh-Geschwindigkeit, Last, Temperatur,
 *   Steigung – Bereich ist wichtiger als Einzelwert
 */

#define SKETCH_NAME    "CircuitCube_Lok_pwmtest"
#define SKETCH_VERSION "v0.2.1"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>

// ── Pins ────────────────────────────────────────────────────────────────────
#define PIN_POTI        34
#define PIN_RICHTUNG    32

// ── BLE UUIDs ───────────────────────────────────────────────────────────────
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ── Parameter ───────────────────────────────────────────────────────────────
#define PWM_MAX              255    // Maximalwert Kanal A
#define TOTZONE              0.05f  // Poti-Totzone um Null (±5%)
#define SEND_INTERVALL       100    // BLE-Sendeintervall Motor in ms
#define SERIAL_INTERVALL     500    // Serielle Ausgabe in ms
#define AKKU_INTERVALL     10000    // Akkuabfrage-Intervall in ms (alle 10 s)

// ── BLE Zustand ─────────────────────────────────────────────────────────────
static BLEAddress*              pServerAddress = nullptr;
static BLEClient*               pClient        = nullptr;
static BLERemoteCharacteristic* pRxChar        = nullptr;
static BLERemoteCharacteristic* pTxChar        = nullptr;
static bool                     verbunden      = false;
static bool                     scanLaeuft     = false;

// ── Akkustand ────────────────────────────────────────────────────────────────
static float akku_volt     = 0.0f;  // Akkuspannung in Volt (z.B. 3.73)
static bool  akku_empfangen = false;

// ── Notify-Callback: TX-Characteristic (Cube → ESP32) ───────────────────────
// Wird vom BLE-Stack asynchron aufgerufen, wenn der Circuit Cube antwortet.
// Erwartetes Format: 1 Rohbyte, Wert = Akkustand in Prozent (0–100)
void onNotify(BLERemoteCharacteristic* pChar,
              uint8_t* pData, size_t length, bool isNotify) {

  // Hex-Dump der Rohantwort (immer, zur Protokollverifikation)
  Serial.printf("[AKKU] Notify empfangen, %d Byte(s): ", (int)length);
  for (size_t i = 0; i < length; i++) {
    Serial.printf("0x%02X ", pData[i]);
  }
  Serial.println();

  // Auswertung: ASCII-String "X.XX" = Spannung in Volt (z.B. "3.73")
  // Null-Terminator anhängen und mit atof() parsen
  if (length > 0 && length <= 8) {
    char buf[9];
    memcpy(buf, pData, length);
    buf[length] = '\0';
    float volt = atof(buf);
    if (volt > 0.0f) {
      akku_volt      = volt;
      akku_empfangen = true;
      Serial.printf("[AKKU] Akkuspannung: %.2f V\n", akku_volt);
    } else {
      Serial.printf("[AKKU] Parsen fehlgeschlagen: \"%s\"\n", buf);
    }
  } else {
    Serial.printf("[AKKU] Unerwartete Antwortlaenge: %d Bytes\n", (int)length);
  }
}

// ── Scan-Callback ────────────────────────────────────────────────────────────
class MeinScanCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if (dev.getName().indexOf("Tenka") >= 0) {
      Serial.printf("[BLE] Gefunden: %s (%s)\n",
                    dev.getName().c_str(),
                    dev.getAddress().toString().c_str());
      BLEDevice::getScan()->stop();
      pServerAddress = new BLEAddress(dev.getAddress());
      scanLaeuft = false;
    }
  }
};

// ── Verbindungsaufbau ────────────────────────────────────────────────────────
bool verbinden() {
  if (!pServerAddress) return false;

  pClient = BLEDevice::createClient();
  if (!pClient->connect(*pServerAddress)) {
    Serial.println("[BLE] Verbindung fehlgeschlagen.");
    return false;
  }
  Serial.println("[BLE] Verbunden.");

  BLERemoteService* pService = pClient->getService(NUS_SERVICE_UUID);
  if (!pService) {
    Serial.println("[BLE] NUS-Service nicht gefunden.");
    pClient->disconnect();
    return false;
  }

  // RX-Characteristic (Befehle senden)
  pRxChar = pService->getCharacteristic(NUS_RX_UUID);
  if (!pRxChar) {
    Serial.println("[BLE] RX-Characteristic nicht gefunden.");
    pClient->disconnect();
    return false;
  }
  Serial.println("[BLE] RX-Characteristic bereit.");

  // TX-Characteristic (Notify, Akkuantwort empfangen)
  pTxChar = pService->getCharacteristic(NUS_TX_UUID);
  if (!pTxChar) {
    Serial.println("[BLE] TX-Characteristic nicht gefunden – Akkuabfrage nicht moeglich.");
    // Kein Abbruch: Motorsteuerung funktioniert auch ohne TX
  } else if (pTxChar->canNotify()) {
    pTxChar->registerForNotify(onNotify);
    Serial.println("[BLE] TX-Characteristic Notify registriert.");
  } else {
    Serial.println("[BLE] TX-Characteristic unterstuetzt kein Notify.");
  }

  return true;
}

// ── BLE-Befehl senden ───────────────────────────────────────────────────────
void sendCommand(char dir, int pwm, char kanal) {
  if (!verbunden || !pRxChar) return;
  char cmd[6];
  snprintf(cmd, sizeof(cmd), "%c%03d%c", dir, pwm, kanal);
  pRxChar->writeValue((uint8_t*)cmd, strlen(cmd));
}

// ── Akkuabfrage auslösen ─────────────────────────────────────────────────────
// Sendet ASCII "b" auf RX; Antwort kommt asynchron über onNotify()
void akkuAnfragen() {
  if (!verbunden || !pRxChar) return;
  const char req[] = "b";
  pRxChar->writeValue((uint8_t*)req, 1);
  Serial.println("[AKKU] Anfrage gesendet (\"b\")");
}

// ── Scan starten ─────────────────────────────────────────────────────────────
void scanStarten() {
  Serial.println("[BLE] Starte Scan nach Tenka...");
  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MeinScanCallback());
  pScan->setActiveScan(true);
  pScan->start(10, false);   // 10 s, nicht blockierend
  scanLaeuft = true;
}

// ════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("\n%s %s\n", SKETCH_NAME, SKETCH_VERSION);
  Serial.println("PWM-Testsketch – Kanal A + Akkuabfrage");

  pinMode(PIN_RICHTUNG, INPUT_PULLUP);
  // GPIO 34 ist input-only, kein pinMode nötig

  BLEDevice::init("ESP32-Lok");
  scanStarten();
}

// ════════════════════════════════════════════════════════════════════════════
void loop() {
  static unsigned long tSend   = 0;
  static unsigned long tSerial = 0;
  static unsigned long tAkku   = 0;

  // ── Verbindungsmanagement ────────────────────────────────────────────────
  if (!verbunden && !scanLaeuft) {
    if (!pServerAddress) {
      scanStarten();
    } else {
      akku_volt      = 0.0f;   // Akkustand bei Neuverbindung zurücksetzen
      akku_empfangen = false;
      verbunden = verbinden();
      if (!verbunden) {
        delete pServerAddress;
        pServerAddress = nullptr;
        delay(1000);
        scanStarten();
      } else {
        // Sofort erste Akkuabfrage nach Verbindungsaufbau
        tAkku = millis();
        akkuAnfragen();
      }
    }
    return;
  }

  if (verbunden && !pClient->isConnected()) {
    Serial.println("[BLE] Verbindung verloren.");
    verbunden      = false;
    pRxChar        = nullptr;
    pTxChar        = nullptr;
    akku_volt      = 0.0f;
    akku_empfangen = false;
    delete pServerAddress;
    pServerAddress = nullptr;
    return;
  }

  if (!verbunden) return;   // Noch beim Scannen

  // ── Eingaben lesen ───────────────────────────────────────────────────────
  int   adcRaw  = analogRead(PIN_POTI);
  float rohwert = adcRaw / 4095.0f;           // 0.0 … 1.0

  float geschw;
  if (rohwert < TOTZONE) {
    geschw = 0.0f;
  } else {
    geschw = (rohwert - TOTZONE) / (1.0f - TOTZONE);   // 0.0 … 1.0
  }

  bool rueckwaerts = (digitalRead(PIN_RICHTUNG) == LOW);
  char dir = rueckwaerts ? '-' : '+';

  int pwm = (int)(geschw * PWM_MAX);

  unsigned long jetzt = millis();

  // ── BLE Motor senden ─────────────────────────────────────────────────────
  if (jetzt - tSend >= SEND_INTERVALL) {
    tSend = jetzt;
    sendCommand(dir, pwm, 'a');
  }

  // ── Periodische Akkuabfrage ───────────────────────────────────────────────
  if (jetzt - tAkku >= AKKU_INTERVALL) {
    tAkku = jetzt;
    akkuAnfragen();
  }

  // ── Serielle Ausgabe ─────────────────────────────────────────────────────
  if (jetzt - tSerial >= SERIAL_INTERVALL) {
    tSerial = jetzt;
    if (akku_empfangen) {
      Serial.printf("Geschw: %.2f  PWM: %3d  Richtung: %c  Akku: %.2f V\n",
                    geschw, pwm, rueckwaerts ? 'R' : 'V', akku_volt);
    } else {
      Serial.printf("Geschw: %.2f  PWM: %3d  Richtung: %c  Akku: ---\n",
                    geschw, pwm, rueckwaerts ? 'R' : 'V');
    }
  }
}
