# TTN GPS Sensor
Log GPS to ttn with float-precistion

I use this to feed ttnmapper :)

Works on Arduino Due with RFM95W and SSD1306-Display

Had to use a Due because i could not figure out how to get this running with less memory

Parts-List:
- Arduino Due
- RFM95 RFM95w SX1276-Module
- Neo6G-GPS
- SSD1306-Display (optional)

Required Libs:
- TinyGPS++
- SSD1306 (optional)
- SimpleLMIC
- MCCI_LoRaWAN_LMIC_library
- Time
- Timezone

Don't forget to adjust project_config.h from LMIC-Library to your needs

RENAME TTN-IDs-Template.h to TTN-IDs.h and add your EUIDS, etc.