#include <Arduino.h>
#include <TinyGPS++.h>
#include <SimpleLMIC.h>
#include "./TTN-configuration.h"
#ifdef USE_DISPLAY
#include "ssd1306.h"
#endif

SimpleLMIC ttn;

int GPSDebugLoopInterval = 1000;
uint32_t GPSDebugLoopMillis = millis() - GPSDebugLoopInterval;
int TTNDebugLoopInterval = 1000;
uint32_t TTNDebugLoopMillis = millis() - TTNDebugLoopInterval;

int TTNSendLoopInterval = 10000;
uint32_t TTNSendLoopMillis = millis() - TTNSendLoopInterval;

int DisplayUpdateLoopInterval = 1000;
uint32_t DisplayUpdateLoopMillis = millis() - DisplayUpdateLoopInterval;

TinyGPSPlus gps;
bool newGPSDataThisLoop = false;

uint8_t coords[9];
uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;

void setup()
{
  Serial.begin(115200);
  delay(10000);
  Serial.println();
  Serial.println("Begin...");
  #if defined GPS_RX_PIN && defined GPS_TX_PIN
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  #else
  GPSSerial.begin(9600);
  #endif //defined GPS_RX_PIN && defined GPS_TX_PIN
  GPSSerial.setTimeout(2);
  
#ifdef USE_DISPLAY
  setupDisplay();
#endif
  ttn.begin();
  ttn.setSubBand(2);
  ttn.onMessage(message);
  ttn.join(devEui, appEui, appKey);
}

void loop()
{
  ttn.loop();
  newGPSDataThisLoop = false;
  poll_gps();

  if(newGPSDataThisLoop == true) get_coords();
  if(newGPSDataThisLoop && millis() - GPSDebugLoopMillis > GPSDebugLoopInterval) {
    GPSDebugLoopMillis = millis();
    dislayGPSInfo();
  }
  if(millis() - TTNDebugLoopMillis > TTNDebugLoopInterval) {
    TTNDebugLoopMillis = millis();
    printTTNDebug();
  }
  if (!ttn.isBusy() && ttn.isLink() && millis() - TTNSendLoopMillis > TTNSendLoopInterval && gps.location.isValid())
  {
    TTNSendLoopMillis = millis();
    Serial.println("Not Busy!");
    ttn.write((uint8_t*) coords, sizeof(coords));

    ttn.send();
  }
#ifdef USE_DISPLAY
  updateDisplay();
#endif
}

void message(uint8_t *payload, size_t size, uint8_t port)
{
  Serial.println("Received " + String(size) + " bytes on port " + String(port));
  switch (port) {
    case 1:
      break;
  }
}

void dislayGPSInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F(","));
    Serial.print(gps.altitude.meters(), 6);
    Serial.print(F(","));
    Serial.print(gps.hdop.value(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void get_coords ()
{
  if (gps.location.isValid()) {

    LatitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
    LongitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;

    coords[0] = ( LatitudeBinary >> 16 ) & 0xFF;
    coords[1] = ( LatitudeBinary >> 8 ) & 0xFF;
    coords[2] = LatitudeBinary & 0xFF;

    coords[3] = ( LongitudeBinary >> 16 ) & 0xFF;
    coords[4] = ( LongitudeBinary >> 8 ) & 0xFF;
    coords[5] = LongitudeBinary & 0xFF;

    altitudeGps = gps.altitude.meters();
    coords[6] = ( altitudeGps >> 8 ) & 0xFF;
    coords[7] = altitudeGps & 0xFF;

    hdopGps = gps.hdop.value() / 10;
    coords[8] = hdopGps & 0xFF;
  }
}

void poll_gps() {
  while (GPSSerial.available())
  {
    char c = GPSSerial.read();
    gps.encode(c);
    newGPSDataThisLoop = true;
  }
}

void printTTNDebug() {
  Serial.print("ttn.isBusy(): ");
  Serial.print(ttn.isBusy());
  Serial.print(" ttn.isLink(): ");
  Serial.print(ttn.isLink());

  Serial.println();
}
#ifdef USE_DISPLAY
void setupDisplay() {
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  //ssd1306_setFixedFont(courier_new_font11x16_digits);
  ssd1306_128x64_i2c_init();
  ssd1306_clearScreen();
  ssd1306_flipVertical(1);
  ssd1306_flipHorizontal(1);
  updateDisplay();
}

void updateDisplay() {
  if(millis() - DisplayUpdateLoopMillis > DisplayUpdateLoopInterval) {
    DisplayUpdateLoopMillis = millis();

    char displayTextTTNIsBusy[256];
    sprintf(displayTextTTNIsBusy, "TTN.isBusy(): %d", ttn.isBusy());
    ssd1306_printFixed(0,  0, displayTextTTNIsBusy, STYLE_NORMAL);
    char displayTextTTNIsLink[256];
    sprintf(displayTextTTNIsLink, "TTN.isLink(): %d", ttn.isLink());
    ssd1306_printFixed(0,  8, displayTextTTNIsLink, STYLE_NORMAL);
    char displayTextGPSIsValid[256];
    sprintf(displayTextGPSIsValid, "GPS.isValid(): %d", gps.location.isValid());
    ssd1306_printFixed(0,  16, displayTextGPSIsValid, STYLE_NORMAL);
    char displayTextLat[256];
    sprintf(displayTextLat, "lat: %10.6f", gps.location.lat());
    ssd1306_printFixed(0,  24, displayTextLat, STYLE_NORMAL);
    char displayTextLng[256];
    sprintf(displayTextLng, "lng: %10.6f", gps.location.lng());
    ssd1306_printFixed(0,  32, displayTextLng, STYLE_NORMAL);
    char gpsNumSats[256];
    sprintf(gpsNumSats, "Satellites: %3u", gps.satellites.value());
    ssd1306_printFixed(0,  40, gpsNumSats, STYLE_NORMAL);
  }
}
#endif