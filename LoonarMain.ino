
/********************************************************
Loonar Technologies Full Systems Code

Written by Aria Tedjarati 2017-18

********************************************************/

/***********************************************************************************************************************************************************************/

/********* LIBRARIES *********/
#include <stdio.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <IridiumSBD.h>
#include <RH_RF24.h>
#include <Adafruit_MCP23008.h>

/***********************************************************************************************************************************************************************/

/********** PINOUT **********/
const int CS_BMP =                     2;    // Chip Select Pin for the BMP280 Sensor
const int BREAKOUT_6 =                 3;    // Breakout pin #6
const int BREAKOUT_1 =                 4;    // Breakout pin #1
const int GFSK_GATE =                  5;    // Gate pin for power to the radio
const int GFSK_SDN =                   6;    // Shutdown pin for the radio
const int GFSK_GPIO2 =                 7;    // GPIO #2 pin for the radio 
const int GFSK_GPIO3 =                 8;    // GPIO #3 pin for the radio
const int GFSK_IRQ =                   14;   // IRQ pin for the radio
const int VCAP_SENSE =                 A1;   // Analog input to read supercapacitor voltage
const int BREAKOUT_3 =                 16;   // Breakout pin #3
const int RB_SLEEP =                   17;   // Iridium modem sleep pin
const int GFSK_GPIO1 =                 20;   // GPIO #1 pin for the radio
const int GFSK_GPIO0 =                 21;   // GPIO #0 pin for the radio
const int GFSK_CS =                    22;   // Chip Select Pin for the radio
const int SD_CS =                      23;   // Chip Select pin for the SD Card
const int BREAKOUT_2 =                 A10;  // Breakout pin #2
const int VBAT_SENSE =                 A11;  // Analog input to read battery voltage
const int BREAOUT_4 =                  A12;  // Breakout pin #4

// Part of the GPIO Extender chip
const int GPS_GATE =                   0;    // Gate pin for power to the GPS
const int EN_5V =                      2;    // Enable pin for the 5V Line
const int RB_GATE =                    3;    // Gate pin for power to the RockBlock
const int CAM_GATE =                   4;    // Gate pin for power to the camera
const int CAM_CTRL =                   5;    // Enable pin for the camera
const int CUT_GATE =                   6;    // Gate pin for power to cutdown
const int BREAKOUT_5 =                 7;    // Breakout pin #5   

/***********************************************************************************************************************************************************************/

/********** OBJECTS **********/
Adafruit_BMP280           bmp(CS_BMP);                         // BMP Pressure/Temperature Sensor
File                      logfile;                             // SD Card Logfile
#define hsGPS             Serial1                              // GPS Object
TinyGPSPlus               tinygps;                             // GPS Parser
IridiumSBD                isbd(Serial2, RB_SLEEP);             // iridium Module
RH_RF24                   rf24(GFSK_CS, GFSK_IRQ, GFSK_SDN);   // Radio Object
Adafruit_MCP23008         GPIO_chip;                           // GPIO Expander Chip

/***********************************************************************************************************************************************************************/

/********** GLOBAL VARIABLES **********/
float          latitude =                                 0.0;
float          longitude =                                0.0;
float          altitude =                                 0.0;
float          bmp_altitude =                             0.0;
float          bmp_temperature =                          0.0;
float          temperature =                              0.0;
double         minutes =                                  0.0;
byte           gps_set_sucess =                           0;

const uint8_t MSG[] = { // Command to send to GPS for high altitude mode
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
    
/***********************************************************************************************************************************************************************/

/********** CONSTANTS **********/  
const double     freq =               145.0;  // Radio Frequency
uint8_t          finaldata[50];
    
/***********************************************************************************************************************************************************************/

void setup(){
  init_GPIO_chip();
  setPinmodes();
  RadioOff();
  IridiumOff();
  CameraOff();
  FiveVOff();
  GPSOff();
  CutdownOff();
  CameraDeTrigger();
  analogReadResolution(12);
  analogReference(EXTERNAL);
  setupSDCard();
  printLogfileHeaders();
  init_bmp();
  GPSOn();
  setGPSFlightMode();
  initRF();
  initExpansion();
  initIridium();
  initCamera();
}

/***********************************************************************************************************************************************************************/

void loop(){
  getGPS();
  getBMP();
  getTemp();
  sendRF();
  sendIridium();
  delay(1000);
}

/***********************************************************************************************************************************************************************/

void sendRF() { 
  char data[50] = "";
  sprintf(data, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f", latitude, longitude, altitude, bmp_altitude, bmp_temperature, temperature);

  for (int i = 0; i < 50; i++){
    finaldata[i] = data[i];    
  }
  
  rf24.send(finaldata, sizeof(finaldata));
  rf24.waitPacketSent();
}

/***********************************************************************************************************************************************************************/

void sendIridium() { 
  size_t bufferSize = sizeof(finaldata);
  isbd.sendReceiveSBDBinary(finaldata, 50, finaldata, bufferSize);
}

/***********************************************************************************************************************************************************************/

void getTemp() {
  temperature = rf24.get_temperature();
}

/***********************************************************************************************************************************************************************/

void initCamera() {
  CameraOn();
  delay(1000);
  CameraTrigger();
  delay(1000);
  CameraDeTrigger();
}

/***********************************************************************************************************************************************************************/

void initExpansion() {
  while(superCapVoltage() <= 4.75) {
    delay(500);
    Serial.println("Supercap voltage:");
    Serial.print(superCapVoltage()); Serial.println(" V");
  }
}

/***********************************************************************************************************************************************************************/

void initIridium(){
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.setPowerProfile(1);
  FiveVOn();
  IridiumOn();
  delay(500);
  isbd.begin();
}

/***********************************************************************************************************************************************************************/

void initRF() {
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);  // Setting clock speed to 8mhz, as 10 is the max for the rfm22
  SPI.begin(); 
  RadioOn();
  rf24.init(50);
  uint8_t buf[8];
  if (!rf24.command(RH_RF24_CMD_PART_INFO, 0, 0, buf, sizeof(buf))) {
    Serial.println("SPI ERROR");
  } else {
    Serial.println("SPI OK");
  }
  if (!rf24.setFrequency(freq)) {
    Serial.println("setFrequency failed");
  } else {
    Serial.print(freq);
    Serial.println(" MHz.");
  }
  rf24.setModemConfig(rf24.GFSK_Rb0_5Fd1);
  rf24.setTxPower(0x7f);
}

/***********************************************************************************************************************************************************************/

void init_GPIO_chip(){
  GPIO_chip.begin();
}

/***********************************************************************************************************************************************************************/

void printLogfileHeaders() {
  logfile.print("Time (min), ");
  logfile.print("Lat,");
  logfile.print("Long, ");
  logfile.print("Alt (m), ");
  logfile.print("Alt_Bar (m), ");
  logfile.print("Temp_Bar (C), ");
  logfile.print("Temp (C), ");
  logfile.print("BatV (V), ");
  logfile.println("CapV (V)");
  logfile.flush();
}

/***********************************************************************************************************************************************************************/

void logStuff() {
  logfile.print(minutes);
  logfile.print(",");
  logfile.print(latitude);
  logfile.print(",");
  logfile.print(longitude);
  logfile.print(",");
  logfile.print(altitude);
  logfile.print(",");
  logfile.print(bmp_altitude);
  logfile.print(",");
  logfile.print(bmp_temperature);
  logfile.print(",");
  logfile.print(temperature);
  logfile.print(",");
  logfile.print(batteryVoltage());
  logfile.print(",");
  logfile.println(superCapVoltage());
  logfile.flush();
}

/***********************************************************************************************************************************************************************/

void init_bmp() {
  bmp.begin();
}

/***********************************************************************************************************************************************************************/

void setupSDCard() {
  SD.begin(SD_CS);
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {   // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }
  if (!logfile) {
    Serial.println ("SD ERROR");
  } else {
    Serial.print("Logging to: "); Serial.println(filename);
  }
}

/***********************************************************************************************************************************************************************/

void setPinmodes(){
  pinMode(GFSK_GATE, OUTPUT);
  pinMode(CS_BMP, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(RB_SLEEP, OUTPUT);
  GPIO_chip.pinMode(GPS_GATE,OUTPUT);
  GPIO_chip.pinMode(EN_5V,OUTPUT);
  GPIO_chip.pinMode(RB_GATE,OUTPUT);
  GPIO_chip.pinMode(CAM_GATE,OUTPUT);
  GPIO_chip.pinMode(CAM_CTRL,OUTPUT);
  GPIO_chip.pinMode(CUT_GATE,OUTPUT);
  GPIO_chip.pinMode(BREAKOUT_5,OUTPUT);
  pinMode(VCAP_SENSE, INPUT);
  pinMode(VBAT_SENSE, INPUT);
  hsGPS.begin(9600);
  Serial2.begin(19200);
}

/***********************************************************************************************************************************************************************/

float superCapVoltage(){
  return (float)analogRead(VCAP_SENSE) * 3.3 * 2 / (double)pow(2, 12);
}

/***********************************************************************************************************************************************************************/

float batteryVoltage(){
  return (float)analogRead(VBAT_SENSE) * 3.3 * 2 / (double)pow(2, 12);
}

/***********************************************************************************************************************************************************************/

void IridiumOn(){
  GPIO_chip.digitalWrite(RB_GATE, HIGH);
}

/***********************************************************************************************************************************************************************/

void IridiumOff(){
  GPIO_chip.digitalWrite(RB_GATE, LOW);
}

/***********************************************************************************************************************************************************************/

void RadioOn(){
  digitalWrite(GFSK_GATE, LOW);
}

/***********************************************************************************************************************************************************************/

void RadioOff(){
  digitalWrite(GFSK_GATE, HIGH);
}

/***********************************************************************************************************************************************************************/

void GPSOn(){
  GPIO_chip.digitalWrite(GPS_GATE, LOW);
}

/***********************************************************************************************************************************************************************/

void GPSOff(){
  GPIO_chip.digitalWrite(GPS_GATE, HIGH);
}

/***********************************************************************************************************************************************************************/

void CameraOn(){
  GPIO_chip.digitalWrite(CAM_GATE, LOW);
}

/***********************************************************************************************************************************************************************/

void CameraOff(){
  GPIO_chip.digitalWrite(CAM_GATE, HIGH); 
}

/***********************************************************************************************************************************************************************/

void CutdownOn(){
  GPIO_chip.digitalWrite(CUT_GATE, HIGH);
}

/***********************************************************************************************************************************************************************/

void CutdownOff(){
  GPIO_chip.digitalWrite(CAM_GATE, LOW); 
}

/***********************************************************************************************************************************************************************/

void FiveVOn(){
  GPIO_chip.digitalWrite(EN_5V, HIGH);
}

/***********************************************************************************************************************************************************************/

void FiveVOff(){
  GPIO_chip.digitalWrite(EN_5V, LOW);
}

/***********************************************************************************************************************************************************************/

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t len) {
  for(int i=0; i<len; i++) {
    hsGPS.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  hsGPS.println();
}

/***********************************************************************************************************************************************************************/

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK() {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	 // header
  ackPacket[1] = 0x62;	 // header
  ackPacket[2] = 0x05;	 // class
  ackPacket[3] = 0x01;	 // id
  ackPacket[4] = 0x02;	 // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2]; // ACK class
  ackPacket[7] = MSG[3]; // ACK id
  ackPacket[8] = 0;	 // CK_A
  ackPacket[9] = 0;	 // CK_B
 
  // Calculate the checksums
  for (uint8_t i = 2; i < 8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println("GPS GOOD!");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println("GPS BAD");
      return false;
    }
 
    // Make sure data is available to read
    if (hsGPS.available()) {
      b = hsGPS.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }
    }
  }
}

/***********************************************************************************************************************************************************************/

void setGPSFlightMode(){
  while(!gps_set_sucess) {
    sendUBX(sizeof(MSG)/sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK();
  }
  gps_set_sucess = 0;  
}

/***********************************************************************************************************************************************************************/

void getBMP(){
  bmp_temperature = bmp.readTemperature();
  bmp_altitude = bmp.readAltitude();
}

/***********************************************************************************************************************************************************************/

void getGPS() {
  smartdelay(500);
  latitude = tinygps.location.lat();
  longitude = tinygps.location.lng();
  altitude = tinygps.altitude.meters();
}

/***********************************************************************************************************************************************************************/

static void smartdelay(unsigned long ms) {
  unsigned long startt = millis();
  do {
    while (hsGPS.available()){
      char c = hsGPS.read();
      tinygps.encode(c);
      Serial.print(c);}
  } while (millis() - startt < ms);
}

/***********************************************************************************************************************************************************************/

void CameraDeTrigger() {
  GPIO_chip.digitalWrite(CAM_CTRL, HIGH); 
}

/***********************************************************************************************************************************************************************/

void CameraTrigger() {
  GPIO_chip.digitalWrite(CAM_CTRL, LOW); 
}

/***********************************************************************************************************************************************************************/

bool isbdCallback(){
  getGPS();
  getBMP();
  getTemp();
  sendRF();
  delay(1000);
  return true;
}

/***********************************************************************************************************************************************************************/

