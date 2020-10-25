const char *devEui = "";
const char *appEui = "";
const char *appKey = "";

const lmic_pinmap lmic_pins = {
   .nss = 10,
   .rxtx = LMIC_UNUSED_PIN,
   .rst = LMIC_UNUSED_PIN,
   .dio = {2, 3, LMIC_UNUSED_PIN},
};

#define GPSSerial Serial2
//#define GPS_RX_PIN 34
//#define GPS_TX_PIN 12