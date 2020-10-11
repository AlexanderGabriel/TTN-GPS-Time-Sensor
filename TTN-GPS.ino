//#define USE_DISPLAY

#include "time.h"
#include <Timezone.h>
#ifdef USE_DISPLAY
#include "ssd1306.h"
#endif
#include <TinyGPS++.h>

#include <SimpleLMIC.h>
#include "./TTN-IDs.h"

SimpleLMIC ttn;

const lmic_pinmap lmic_pins = {
   .nss = 10,
   .rxtx = LMIC_UNUSED_PIN,
   .rst = LMIC_UNUSED_PIN,
   .dio = {2, 3, LMIC_UNUSED_PIN},
};

uint32_t lastMillisGPSDebugLoop = millis() - 1000;
uint32_t lastMillis10SecLoop = millis() - 1000;
uint32_t lastMillisTTNSent = millis() - 1000;
uint32_t lastMillis60SecLoop = millis() - 60000;

TinyGPSPlus gps;
bool gpsdatasetonce = false;

String msg = "falsch";
char vlat[30];
char vlng[30];
char valt[30];
char vhdop[30];

// Change these two rules corresponding to your timezone, see https://github.com/JChristensen/Timezone
//Central European Time (Frankfurt, Paris)  120 = +2 hours in daylight saving time (summer).
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};
//Central European Time (Frankfurt, Paris)  60  = +1 hour in normal time (winter)
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};
Timezone CE(CEST, CET);
// time variables
time_t local, utc, prev_set;
int timesetinterval = 60; //set microcontroller time every 60 seconds
bool timesetonce = false;

#ifdef USE_DISPLAY
char displayTextDate[256], displayTextTime[256];
#endif

void setup()
{
  Serial.begin(115200);
  while(!Serial) {delay(100);}
  Serial.println();
  Serial.println("Begin...");
  Serial1.begin(9600);
  
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
  double altitude;
  double latitude;
  double longitude;
  double hdop;

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  while (Serial1.available()) {
    if (gps.encode(Serial1.read())) newData = true;
  }

  if (newData && gps.location.isValid() && millis() - lastMillisGPSDebugLoop > 20*1000 )
  {
    lastMillisGPSDebugLoop = millis();
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    altitude = gps.altitude.meters();
    hdop = gps.hdop.value();

    setthetime();

    sprintf(vlat, "%010.6f", (float)latitude);
    sprintf(vlng, "%010.6f", (float)longitude);
    sprintf(valt, "%06.2f", (float)altitude);
    sprintf(vhdop, "%04.1f", (float)hdop);
    msg = String(vlat) + String(vlng) + String(valt) + String(vhdop);
    Serial.println(msg);

    gpsdatasetonce = true;

    if(!timesetonce) {
      setthetime();
      prev_set = now();
      timesetonce = true;
    }
    else if (now() - prev_set > timesetinterval)
    {
      setthetime();
      prev_set = now();
    }
  }

  ttn.loop();
  if (!ttn.isBusy() && millis() - lastMillisTTNSent > 60*1000 && gpsdatasetonce && msg != "falsch")
  {
    lastMillisTTNSent = millis();
    Serial.println("Not Busy!");
    ttn.print(msg);
    ttn.send();
  }
  //10sec-Loop
  if ((millis() - lastMillis10SecLoop) > 10000)
  {
    lastMillis10SecLoop = millis();

    //Show Time and Location on Serial Interface every 10 Seconds
    serialPrintTime();
    serialPrintLocation();
  }

#ifdef USE_DISPLAY
  //Update Time and Date on Display every Loop
  if(timesetonce) {
    updateDisplay();
  }
#endif
}

void setthetime(void)
{
  // Set Time from GPS data string
  setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());  // set the time of the microcontroller to the UTC time from the GPS
}

void serialPrintTime(void)
{
    Serial.print(gps.date.day());
    Serial.print(".");Serial.print(gps.date.month());Serial.print(".");Serial.print(gps.date.year());Serial.print(" ");Serial.print(gps.time.hour());Serial.print(":");Serial.print(gps.time.minute());Serial.print(".");Serial.println(gps.time.second());
}

void message(uint8_t *payload, size_t size, uint8_t port)
{
  Serial.println("Received " + String(size) + " bytes on port " + String(port));
  switch (port) {
    case 1:
      break;
  }
}

#ifdef USE_DISPLAY
void setupDisplay() {
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_setFixedFont(courier_new_font11x16_digits);
  ssd1306_128x32_i2c_init();
  ssd1306_clearScreen();
  ssd1306_flipVertical(1);
  ssd1306_flipHorizontal(1);
  sprintf(displayTextDate, "00.00.0000", day(local), month(local), year(local));
  sprintf(displayTextTime, "00:00.00", hour(local), minute(local), second(local));
  ssd1306_printFixed(0,  0, displayTextDate, STYLE_NORMAL);
  ssd1306_printFixed(0,  16, displayTextTime, STYLE_NORMAL);
}

void updateDisplay() {
  utc = now();
  local = CE.toLocal(utc);

  sprintf(displayTextDate, "%02d.%02d.%d", day(local), month(local), year(local));
  sprintf(displayTextTime, "%02d:%02d.%02d", hour(local), minute(local), second(local));
  ssd1306_printFixed(0,  0, displayTextDate, STYLE_NORMAL);
  ssd1306_printFixed(0,  16, displayTextTime, STYLE_NORMAL);
}
#endif

void serialPrintLocation() {
  if(!gps.location.isValid()) return;
  Serial.print("LAT=");
  Serial.print(gps.location.lat(), 6);
  Serial.print(" LON=");
  Serial.print(gps.location.lng(), 6);
  Serial.print(" ALT=");
  Serial.print(gps.altitude.meters());
  Serial.print(" HDOP=");
  Serial.print(gps.hdop.value());
  Serial.println();
}
