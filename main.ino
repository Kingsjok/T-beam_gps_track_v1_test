
#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>
#include <Wire.h>

#include <SPI.h>
#include <LoRa.h>

// UPDATE the config.h file in the same folder WITH YOUR TTN KEYS AND ADDR.
#include "gps.h"

// T-Beam specific hardware
#define BUILTIN_LED 14


String LoraStatus;

char s[32]; // used to sprintf for Serial output
uint8_t txBuffer[9];
uint16_t txBuffer2[5];
uint8_t mydata[]="Please Wait!"; 
gps gps;
static osjob_t sendjob;

static const u1_t PROGMEM DEVEUI[8]={ 0xDD, 0x4C, 0xB5, 0x4E, 0x42, 0xF4, 0x33, 0x00 } ; // Device EUI, hex, lsb
static const u1_t PROGMEM APPEUI[8]={ 0x34, 0x5F, 0x03, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // Application EUI, hex, lsb
static const u1_t PROGMEM APPKEY[16] = { 0xAA, 0xD2, 0x49, 0x7A, 0xAD, 0x6D, 0x37, 0x0C, 0x8F, 0x6F, 0xE0, 0xF1, 0xA2, 0x70, 0xA7, 0x10 }; // App Key, hex, msb

void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}


// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 120;

// For battery mesurement
//const uint8_t vbatPin = 35;
//float VBAT;  // battery voltage from ESP32 ADC read

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN, // was "14,"
  .dio = {26, 33, 32},
};

void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      LoraStatus = "EV_SCAN_TIMEOUT";
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      LoraStatus = "EV_BEACON_FOUND";
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      LoraStatus = "EV_BEACON_MISSED";
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      LoraStatus = "EV_BEACON_TRACKED";
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      LoraStatus = "EV_JOINING";
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      LoraStatus = "EV_JOINED";
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      LMIC.dn2Dr = DR_SF9;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      LoraStatus = "EV_RFU1";
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      LoraStatus = "EV_JOIN_FAILED";
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      LoraStatus = "EV_REJOIN_FAILED";
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      LoraStatus = "EV_TXCOMPLETE";
      digitalWrite(BUILTIN_LED, LOW);  
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received Ack"));
        LoraStatus = "Received Ack";
      }
      if (LMIC.dataLen) {
        sprintf(s, "Received %i bytes of payload", LMIC.dataLen);
        Serial.println(s);
        sprintf(s, "RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
        Serial.println(s);
      }
      // Schedule next transmission
      do_send(&sendjob);
      //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      LoraStatus = "EV_LOST_TSYNC";
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      LoraStatus = "EV_RESET";
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      LoraStatus = "EV_RXCOMPLETE";
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      LoraStatus = "EV_LINK_DEAD";
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      LoraStatus = "EV_LINK_ALIVE";
      break;
    default:
      Serial.println(F("Unknown event"));
      LoraStatus = "Unknown event";
      break;
  }
}

void do_send(osjob_t* j) {  

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
    LoraStatus = "OP_TXRXPEND, not sending";
  }
  else
  {     

      //if(gps.checkGpsFix()){
      // Prepare upstream data transmission at the next possible time.
      gps.buildPacket(txBuffer);
      LMIC_setTxData2(1,txBuffer, sizeof(txBuffer), 0);
      Serial.println(F("Packet queued"));
      LoraStatus = "Packet queued";
     // }
     //else
     //
      //try again in 3 seconds
     // os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
     //}
    }
    
    
  
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("TTN Mapper Start"));
  
  //pinMode(vbatPin, INPUT);// Battery mesurement

  
  //Turn off WiFi and Bluetooth
  WiFi.mode(WIFI_OFF);
  btStop();

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14); 

  do_send(&sendjob);
  

  
}

void loop() {
    
    os_runloop_once();



}
