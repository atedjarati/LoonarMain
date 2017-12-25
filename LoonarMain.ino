
/********************************************************
Loonar Technologies Full Systems Code
 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

********************************************************/

/***********************************************************************************************************************************************************************/

/********* LIBRARIES *********/
#include <stdio.h>                 // Standard library
#include <SD.h>                    // SD Card library
#include <TinyGPS++.h>             // GPS Parser Library
#include <SPI.h>                   // Serial Peripheral Interface Library
#include <i2c_t3.h>                // Teensy I2C Library
#include <Adafruit_Sensor.h>       // Adafruit sensor libraries
#include <Adafruit_BMP280.h>       // BMP280 Library
#include <IridiumSBD.h>            // Iridium module library
#include <RH_RF24.h>               // Radio module library
#include <Adafruit_MCP23008.h>     // MCP23008 GPIO Expansion chip library
#include "Configuration.h"         // Loonar Technologies Configuration file. 
#include "UserConfiguration.h"

/***********************************************************************************************************************************************************************/

/********** OBJECTS **********/
Adafruit_BMP280           bmp(CS_BMP);                         // BMP280 Pressure/Temperature Sensor
File                      logfile;                             // SD Card Logfile
#define hsGPS             Serial1                              // GPS Object
TinyGPSPlus               tinygps;                             // GPS Parser
IridiumSBD                isbd(Serial2, RB_SLEEP);             // Iridium Module
RH_RF24                   rf24(GFSK_CS, GFSK_IRQ, GFSK_SDN);   // Si4463 Radio Object
Adafruit_MCP23008         GPIO_chip;                           // GPIO Expander Chip, i2c
IntervalTimer             loonarInterrupt;                     // Interrupt to run all flight code independent of user code

/***********************************************************************************************************************************************************************/

/********** GLOBAL STRUCT **********/

struct FlightData{
  float     latitude;
  float     longitude;
  float     altitude;
  float     bmp_altitude;
  float     bmp_temperature;
  float     temperature;
  float     battery_voltage;
  float     supercap_voltage;
  boolean   cutdown;
  double    minutes;
  double    startTime;
  uint8_t   finaldata[BUF_SIZE];
} flightData;
    
/***********************************************************************************************************************************************************************/

void setup()
{
  init_GPIO_chip();                                  // Initialize the GPIO Expansion chip over I2C with the teensy i2c library integrated already into this.
  setPinmodes();                                     // Initialize all the pinModes for every pin (i.e. input, output, etc).
  RadioOff();                                        // Shut off power to the radio.
  IridiumOff();                                      // Shut off power to the Iridium modem
  CameraOff();                                       // Shut off power to the camera.
  FiveVOff();                                        // Shut off the 5 volt line.
  GPSOff();                                          // Shut off power to the GPS.
  CutdownOff();                                      // Shut off power to cutdown. 
  CameraDeTrigger();                                 // De-trigger the camera (drive high).
  analogReadResolution(12);                          // Set the ADC resolution to 12 bits (0-4095) for maximum resolution
  analogReference(EXTERNAL);                         // Set the ADC reference voltage to the externally supplied 3.3V reference. 
  setupSDCard();                                     // Configure the SD card and set up the log file. 
  printLogfileHeaders();                             // Write headers to the log file.
  init_bmp();                                        // Initialize the BMP 
  GPSOn();                                           // Turn on power to the GPS.
  setGPSFlightMode();                                // Configure the GPS for flight mode to work at high altitudes.
  initRF();                                          // Turn on and initialize the Radio module.
  chargeSuperCapacitor();                            // Charge the supercapacitor up.  
  initIridium();                                     // Turn on the 5 volt line, give power to the Iridium module, and begin talking to it.
  initCameraVideo();                                 // Give power to the camera, then trigger the camera line to start video. 
  userSetupCode();                                   // Call the user setup function code. 
  loonarInterrupt.begin(loonarCode, INTERVAL_TIME);  // Start the interrupt timer. 
  flightData.startTime = millis();                   // Initializes the start time of the entire program. 
}

/***********************************************************************************************************************************************************************/

void loop()
{
  userLoopCode();
}

/***********************************************************************************************************************************************************************/

/*--------------------------------------------------------------------------------------------------------------
   Function:
     loonarCode
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Completes all necessary flight code, including acquiring all data from the GPS and sensors, then sending 
     this data to the radio module and the iridium module, and finally logging all data onboard.  This function
     gets called on an interrupt using the IntervalTimer function built into the Teensy LC. 
--------------------------------------------------------------------------------------------------------------*/
static void loonarCode ()
{
  flightData.minutes = getTime();
  flightData.latitude = getLatitude();
  flightData.longitude = getLongitude();
  flightData.altitude = getAltitude();
  flightData.bmp_temperature = bmp.readTemperature();
  flightData.bmp_altitude = bmp.readAltitude();
  flightData.temperature = getTemp();                     // Acquire the temperature from the built in sensor from the radio chip.
  flightData.battery_voltage = getBatteryVoltage();
  flightData.supercap_voltage = getSuperCapVoltage();
  configureData();                                        // Configure the data we want to transmit via Iridium and/or RF.
  logToSDCard();                                          // Log all data to the SD Card.
  checkCutdown();                                         // Check to see if the conditions call for cutting down the balloon.
  transceiveRF();                                         // Transmit and receive telemetry via the radio module. 
  transceiveIridium();                                    // Transmit and receive telemetry via the Iridium modem. 
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     getTime
   Parameters:
     None
   Returns:
     Current time since startup of the Teensy in double format. 
   Purpose: 
     Acquires the current time since startup of the teensy. 
--------------------------------------------------------------------------------------------------------------*/
static double getTime(){
  return (double) (millis() - flightData.startTime)/1000.0/60.0;  // Acquire the current time since startup of the Teensy.
}

/*--------------------------------------------------------------------------------------------------------------
   Function:
     checkCutdown
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Checks to see if the conditions are appropriate for cutdown, and if so, will cut down the balloon. 
--------------------------------------------------------------------------------------------------------------*/
static void checkCutdown() 
{
  if (!flightData.cutdown) 
  {
    if (CUTDOWN_CONFIG == 1)
    {
      if (tinygps.location.isValid())
      {
        if (flightData.altitude >= CUTDOWN_ALTITUDE) 
        {
          cutdownBalloon();
        }
      }
    } 
    else if (CUTDOWN_CONFIG == 2)
    {
      if (tinygps.location.isValid())
      {
        if (flightData.minutes >= CUTDOWN_TIME) 
        {
          cutdownBalloon();
        }
      }
    } 
    else if (CUTDOWN_CONFIG == 3)
    {
      if (tinygps.location.isValid())
      {
        if ( (flightData.latitude > CUTDOWN_LATITUDE_MAX) || 
             (flightData.latitude < CUTDOWN_LATITUDE_MIN) || 
             (flightData.longitude > CUTDOWN_LONGITUDE_MAX) || 
             (flightData.longitude < CUTDOWN_LONGITUDE_MIN) ) 
        {
          cutdownBalloon();
        }
      }
    }
  }
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     cutdownBalloon
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Turns on the power to the cutdownr resistor and bleeds energy from the supercap until it hits 2.9V.
--------------------------------------------------------------------------------------------------------------*/
static void cutdownBalloon()
{
  noInterrupts();
  CutdownOn();
  double startCutdown = millis();
  while (getSuperCapVoltage() > SUPERCAP_CUTDOWN_VOLTAGE_MIN) 
  {
    if ((millis() - startCutdown) > CUTDOWN_MAX_TIME)
    {
      break; 
    }
  }
  CutdownOff(); 
  flightData.cutdown = true;
  interrupts();
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     configureData
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Configures all the current global variable data and puts it in the global data array.   
--------------------------------------------------------------------------------------------------------------*/
static void configureData()
{
  char data[BUF_SIZE] = "";
  sprintf(data, "%.4f,%.4f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d", 
    flightData.latitude, 
    flightData.longitude, 
    flightData.altitude, 
    flightData.bmp_altitude, 
    flightData.bmp_temperature, 
    flightData.temperature,
    flightData.battery_voltage, 
    flightData.supercap_voltage, 
    flightData.cutdown);
    
  Serial.print("Final configured data: ");
  for (int i = 0; i < BUF_SIZE; i++)
  {
    Serial.print(data[i]);
    flightData.finaldata[i] = data[i];    
  } 
  Serial.println();
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     transceiveRF
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Sends the current data stream to the Radio module for transmission.  Also reads in any data. 
--------------------------------------------------------------------------------------------------------------*/
static void transceiveRF() 
{   
  rf24.send(flightData.finaldata, sizeof(flightData.finaldata));
  rf24.waitPacketSent();
  
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     transceiveIridium
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Sends the current data stream to the Iridium module for transmission. 
--------------------------------------------------------------------------------------------------------------*/
static void transceiveIridium() 
{ 
  noInterrupts(); // Turns off interrupts so we can transmit our message via Iridium.
  if (flightData.minutes > IRIDIUM_LOOP_TIME) 
  {
    size_t bufferSize = sizeof(flightData.finaldata);
    isbd.sendReceiveSBDBinary(flightData.finaldata, BUF_SIZE, flightData.finaldata, bufferSize);
    char rxBuf [bufferSize];
    IRIDIUM_LOOP_TIME += IRIDIUM_LOOP_TIME;
    if (bufferSize > 0) 
    {
      boolean shouldCutdown = true;
      for (size_t i = 0; i < bufferSize; i++)
      {
        rxBuf[i] = flightData.finaldata[i];
        if(rxBuf[i] != CUTDOWN_COMMAND[i]) 
        {
          shouldCutdown = false;
        }
      }
      if (shouldCutdown) 
      {
        cutdownBalloon();
      }
      messageReceived(flightData.finaldata);
    }
  }
  interrupts(); // Turn interrupts back on. 
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     getTemp
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Acquires the temperature reading in celsius from the radio module and stores it in respectibe global variable. 
--------------------------------------------------------------------------------------------------------------*/
static float getTemp() 
{
 return rf24.get_temperature();
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     initCameraVideo
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Triggers the camera into video mode. 
--------------------------------------------------------------------------------------------------------------*/
static void initCameraVideo() 
{
  CameraOn();
  delay(1000);
  CameraTrigger();
  delay(1000);
  CameraDeTrigger();
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     chargeSuperCapacitor
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Does nothing until the supercapacitor voltage hits a predetermined limit.  
--------------------------------------------------------------------------------------------------------------*/
static void chargeSuperCapacitor() 
{
  while(getSuperCapVoltage() <= SUPERCAP_MIN_LIMIT) 
  {
    delay(500);
    Serial.println("Supercap voltage:");
    Serial.print(getSuperCapVoltage()); 
    Serial.println(" V");
  }
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     initIridium
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Initializes the iridium module by attaching the serial monitor to the iridium modem, turning on the 5 volt 
     line as well as delivering power ot the iridium module, and then calling the begin function. 
--------------------------------------------------------------------------------------------------------------*/
static void initIridium()
{
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.setPowerProfile(1);
  FiveVOn();
  IridiumOn();
  delay(500);
  isbd.begin();
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     initRF
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Initializes the RF module.  Sets the SPI data mode and clock speed.  Delivers power to the radio module. 
     Initializes the radio module buffer size.  Sets the center frequency of the radio module. Sets the 
     modulation configuration to GFSK at a speed of 500bps and a bandwidth of approximately 1kHz.  Finally,
     it sets the output transmit power to +20dBm or 100mW output RF power. 
--------------------------------------------------------------------------------------------------------------*/
static void initRF() 
{
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);  // Setting clock speed to 8mhz, as 10 is the max for the rfm22
  SPI.begin(); 
  RadioOn();
  rf24.init(BUF_SIZE);
  uint8_t buf[8];
  if (!rf24.command(RH_RF24_CMD_PART_INFO, 0, 0, buf, sizeof(buf))) 
  {
    Serial.println("SPI ERROR");
  } 
  else 
  {
    Serial.println("SPI OK");
  }
  if (!rf24.setFrequency(FREQ)) 
  {
    Serial.println("setFrequency failed");
  } 
  else 
  {
    Serial.print(FREQ);
    Serial.println(" MHz.");
  }
  rf24.setModemConfig(rf24.GFSK_Rb0_5Fd1);
  rf24.setTxPower(0x7f);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     init_GPIO_chip
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Initialize the GPIO Expander chip. 
--------------------------------------------------------------------------------------------------------------*/
static void init_GPIO_chip()
{
  GPIO_chip.begin();
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     printLogfileHeaders
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Write the headers to the SD Card logfile. 
--------------------------------------------------------------------------------------------------------------*/
static void printLogfileHeaders() 
{
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


/*--------------------------------------------------------------------------------------------------------------
   Function:
     logToSDCard
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Logs al relevant variables to the SD card log file.  
--------------------------------------------------------------------------------------------------------------*/
static void logToSDCard() 
{
  logfile.print(flightData.minutes);
  logfile.print(",");
  logfile.print(flightData.latitude);
  logfile.print(",");
  logfile.print(flightData.longitude);
  logfile.print(",");
  logfile.print(flightData.altitude);
  logfile.print(",");
  logfile.print(flightData.bmp_altitude);
  logfile.print(",");
  logfile.print(flightData.bmp_temperature);
  logfile.print(",");
  logfile.print(flightData.temperature);
  logfile.print(",");
  logfile.print(flightData.battery_voltage);
  logfile.print(",");
  logfile.println(flightData.supercap_voltage);
  logfile.flush();
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     init_bmp
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Initializes the BMP280 barometric pressure sensor. 
--------------------------------------------------------------------------------------------------------------*/
static void init_bmp() 
{
  bmp.begin();
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     setupSDCard
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Creates the SD Card log file.  
--------------------------------------------------------------------------------------------------------------*/
static void setupSDCard() 
{
  SD.begin(SD_CS);
  char filename[] = "LOGGER000.CSV";
  for (uint8_t i = 0; i < 1000; i++) 
  {
    filename[6] = (i / 100) + '0';
    filename[7] = (i / 10) % 10 + '0';
    filename[8] = (i % 10) + '0';
    if (! SD.exists(filename)) // only open a new file if it doesn't exist
    {   
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }
  if (!logfile) 
  {
    Serial.println ("SD ERROR");
  } 
  else 
  {
    Serial.print("Logging to: "); Serial.println(filename);
  }
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     setPinmodes
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Sets up the pinmodes for all of the control pins, ADC's, and also begins all the Serial Communication. 
--------------------------------------------------------------------------------------------------------------*/
static void setPinmodes()
{
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


/*--------------------------------------------------------------------------------------------------------------
   Function:
     superCapVoltage
   Parameters:
     None
   Returns:
     float representing the supercapacitor voltage in volts. 
   Purpose: 
     Reads the ADC to determine the current supercapacitor voltage. 
--------------------------------------------------------------------------------------------------------------*/
static float getSuperCapVoltage()
{
  return (float)analogRead(VCAP_SENSE) * 3.3 * 2 / (double)pow(2, 12);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     batteryVoltage
   Parameters:
     None
   Returns:
     float representing the battery voltage in volts. 
   Purpose: 
     Reads the ADC to determine the current battery voltage. 
--------------------------------------------------------------------------------------------------------------*/
static float getBatteryVoltage()
{
  return (float)analogRead(VBAT_SENSE) * 3.3 * 2 / (double)pow(2, 12);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     IridiumOn
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Delivers power to the Iridium modem.
--------------------------------------------------------------------------------------------------------------*/
static void IridiumOn()
{
  GPIO_chip.digitalWrite(RB_GATE, HIGH);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     IridiumOff
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Shuts off power to the Iridium modem. 
--------------------------------------------------------------------------------------------------------------*/
static void IridiumOff()
{
  GPIO_chip.digitalWrite(RB_GATE, LOW);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     RadioOn
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Delivers power to the radio module. 
--------------------------------------------------------------------------------------------------------------*/
static void RadioOn()
{
  digitalWrite(GFSK_GATE, LOW);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     RadioOff
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Shuts off power to the radio module.
--------------------------------------------------------------------------------------------------------------*/
static void RadioOff()
{
  digitalWrite(GFSK_GATE, HIGH);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     GPSOn
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Delivers power to the GPS module.
--------------------------------------------------------------------------------------------------------------*/
static void GPSOn()
{
  GPIO_chip.digitalWrite(GPS_GATE, LOW);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     GPSOff
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Shuts off power to the GPS module. 
--------------------------------------------------------------------------------------------------------------*/
static void GPSOff()
{
  GPIO_chip.digitalWrite(GPS_GATE, HIGH);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     CameraOn
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Delivers power to the camera.
--------------------------------------------------------------------------------------------------------------*/
static void CameraOn()
{
  GPIO_chip.digitalWrite(CAM_GATE, LOW);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     CameraOff
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Shuts off power to the camera
--------------------------------------------------------------------------------------------------------------*/
static void CameraOff()
{
  GPIO_chip.digitalWrite(CAM_GATE, HIGH); 
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     CutdownOn
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     enables Cutdown. 
--------------------------------------------------------------------------------------------------------------*/
static void CutdownOn()
{
  GPIO_chip.digitalWrite(CUT_GATE, HIGH);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     CutdownOff
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Disables cutdown
--------------------------------------------------------------------------------------------------------------*/
static void CutdownOff()
{
  GPIO_chip.digitalWrite(CAM_GATE, LOW); 
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     FiveVOn
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Turn on the 5 volt line on the expansion board.
--------------------------------------------------------------------------------------------------------------*/
static void FiveVOn()
{
  GPIO_chip.digitalWrite(EN_5V, HIGH);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     FiveVOff
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Turn off the 5 volt line on the expansion board.
--------------------------------------------------------------------------------------------------------------*/
static void FiveVOff()
{
  GPIO_chip.digitalWrite(EN_5V, LOW);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     sendUBX
   Parameters:
     Unsigned 8 bit integer denoting the length of the sent transmission to the GPS.  
   Returns:
     Nothing
   Purpose: 
     Send a byte array of UBX protocol to the GPS.
--------------------------------------------------------------------------------------------------------------*/
static void sendUBX(uint8_t len) 
{
  for(int i = 0; i < len; i++) 
  {
    hsGPS.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  hsGPS.println();
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     getUBX_ACK
   Parameters:
     None
   Returns:
     Boolean representing a successful message transmission to the GPS.
   Purpose: 
     Calculate expected UBX ACK packet and parse UBX response from GPS.
--------------------------------------------------------------------------------------------------------------*/
static boolean getUBX_ACK() 
{
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
  for (uint8_t i = 2; i < 8; i++) 
  {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) 
  {
 
    // Test for success
    if (ackByteID > 9) 
    {
      // All packets in order!
      Serial.println("GPS GOOD!");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) 
    { 
      Serial.println("GPS BAD");
      return false;
    }
 
    // Make sure data is available to read
    if (hsGPS.available()) 
    {
      b = hsGPS.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) 
      { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else 
      {
        ackByteID = 0;	// Reset and look again, invalid order
      }
    }
  }
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     setGPSFlightMode
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Sends a message to the GPS upon intialization to set it to flight mode so it works at high altitude. 
--------------------------------------------------------------------------------------------------------------*/
void setGPSFlightMode()
{
  static byte gps_set_success = 0;
  while(!gps_set_success) 
  {
    sendUBX(sizeof(MSG)/sizeof(uint8_t));
    gps_set_success = getUBX_ACK();
  }
  gps_set_success = 0;  
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     getLatitude
   Parameters:
     None
   Returns:
     Current GPS Latitude in float format
   Purpose: 
     Acquires the current GPS latitude.
--------------------------------------------------------------------------------------------------------------*/
static float getLatitude()
{
  smartdelay(200);
  return tinygps.location.lat(); 
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     getLongitude
   Parameters:
     None
   Returns:
     Current GPS Longitude in float format
   Purpose: 
     Acquires the current GPS longitude.
--------------------------------------------------------------------------------------------------------------*/
static float getLongitude()
{
  smartdelay(200);
  return tinygps.location.lng(); 
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     getAltitude
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Acquires the current GPS Altitude in meters.  
--------------------------------------------------------------------------------------------------------------*/
static float getAltitude()
{
  smartdelay(200);
  return tinygps.altitude.meters(); 
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     smartdelay
   Parameters:
     Unsigned long 'ms' to designate milliseconds to run this function for.
   Returns:
     Nothing
   Purpose: 
     Reads and parses the GPS datastream for the allotted time passed in.
--------------------------------------------------------------------------------------------------------------*/
static void smartdelay(unsigned long ms) 
{
  unsigned long timing = millis();
  do 
  {
    while (hsGPS.available()) 
    {  
      char a = hsGPS.read();
      tinygps.encode(a);
      Serial.print(a);
    }
  } while ((millis() - timing) < ms);
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     CameraDeTrigger
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     To drive the camera control pin high to detrigger the camera control pin. After this, the camera begins
     either a photo or a video based on how long it took from the CameraTrigger function to this function.
--------------------------------------------------------------------------------------------------------------*/
static void CameraDeTrigger() 
{
  GPIO_chip.digitalWrite(CAM_CTRL, HIGH); 
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     CameraTrigger
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     To drive the camera control pin low to trigger a response from the camera to begin taking photos/videos.  
     The function does not handle timing.  Any parent function that calls this function is responsible for the
     tining of the control pin, in order to choose between taking a photo or a video.
--------------------------------------------------------------------------------------------------------------*/
static void CameraTrigger() 
{
  GPIO_chip.digitalWrite(CAM_CTRL, LOW); 
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     isbdCallback
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     During the iridium send function, this function will be called on an interrupt basis. 
--------------------------------------------------------------------------------------------------------------*/
bool isbdCallback()
{
 
  delayMicroseconds(INTERVAL_TIME);
  return true;
}



