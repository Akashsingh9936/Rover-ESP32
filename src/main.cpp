/*
  ROVER — Step 1: CRSF Channel Reader
  =====================================
  Hardware : ESP32 DevKitV1
  Receiver : RadioMaster RP1 (ExpressLRS)
  Protocol : CRSF via UART2

  Wiring:
    RP1 TX  →  ESP32 GPIO16 (RX2)
    RP1 RX  →  ESP32 GPIO17 (TX2)  [optional — only needed for telemetry back]
    RP1 5V  →  ESP32 5V
    RP1 GND →  ESP32 GND

  What this does:
    - Reads all 16 CRSF channels from RP1
    - Prints channel values to Serial Monitor every 100ms
    - Nothing else — no mixing, no motor output

  Channel values range: 172 to 1811 (CRSF raw)
  Midpoint (stick center): 992

  Next step: add mixing + motor output
*/

#include <Arduino.h>
#include <CRSFforArduino.hpp>

// ─── PIN DEFINITIONS ────────────────────────────────────────────────
#define CRSF_RX_PIN   16    // GPIO16 = RX2 — connects to RP1 TX
#define CRSF_TX_PIN   17    // GPIO17 = TX2 — connects to RP1 RX (telemetry)

// ─── CRSF INSTANCE ──────────────────────────────────────────────────
CRSFforArduino crsf(&Serial2, CRSF_RX_PIN, CRSF_TX_PIN);

// ─── TIMING ─────────────────────────────────────────────────────────
#define PRINT_INTERVAL_MS   100
unsigned long lastPrint = 0;

// ─── TOTAL CHANNELS TO READ ─────────────────────────────────────────
#define NUM_CHANNELS   16

// ─── LINK STATUS ────────────────────────────────────────────────────
// isLinkUp() isn't exposed publicly — track it ourselves via callbacks.
volatile bool linkIsUp = false;
void onLinkUp() { linkIsUp = true; }
void onLinkDown() { linkIsUp = false; }

// ────────────────────────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("=================================");
  Serial.println("ROVER Step 1 — CRSF Channel Read");
  Serial.println("=================================");

  // initialise CRSF — pass UART2 baud (420000 handled internally)
  if (!crsf.begin())
  {
    Serial.println("[ERROR] CRSF init failed — check wiring on GPIO16/17");
    while (true) { delay(1000); }
  }

  crsf.setLinkUpCallback(onLinkUp);
  crsf.setLinkDownCallback(onLinkDown);

  Serial.println("[OK] CRSF initialised — waiting for RC link...");
}

// ────────────────────────────────────────────────────────────────────
void loop()
{
  // must call update() every loop — parses incoming CRSF packets
  crsf.update();

  unsigned long now = millis();
  if (now - lastPrint >= PRINT_INTERVAL_MS)
  {
    lastPrint = now;

    // check if receiver is actually connected and sending data
    if (!linkIsUp)
    {
      Serial.println("[WARN] No RC link — check RP1 power and TX binding");
      return;
    }

    // print all 16 channels
    Serial.println("--- Channels ---");
    for (int ch = 1; ch <= NUM_CHANNELS; ch++)
    {
      // getChannel() returns raw CRSF value: 172 (min) to 1811 (max), center 992
      int val = crsf.getChannel(ch);
      Serial.printf("  CH%02d : %4d", ch, val);

      // every 4 channels go to new line for readability
      if (ch % 4 == 0) Serial.println();
    }
    Serial.println();
    delay(20);  // small delay to avoid flooding Serial Monitor
  }
}
