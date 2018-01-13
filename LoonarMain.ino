/*
Loonar Technologies Full Systems Code
 
Copyright 2018 Loonar Technologies, LLC

      THIS SOFTWARE IS PRESENTED WITH THE MIT LICENCE:

* Permission is hereby granted, free of charge, to any person 
* obtaining a copy of this software and associated documentation 
* files (the "Software"), to deal in the Software without 
* restriction, including without limitation the rights to use, 
* copy, modify, merge, publish, distribute, sublicense, and/or 
* sell copies of the Software, and to permit persons to whom the 
* Software is furnished to do so, subject to the following conditions:

* The above copyright notice and this permission notice shall be included 
* in all copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR 
* ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
* CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION 
* WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

/***********************************************************************************************************************************************************************/

/********* LIBRARIES *********/
#include <SD.h>                    // SD Card library
#include <SPI.h>                   // Serial Peripheral Interface Library
#include <stdio.h>                 // Standard library
#include <TinyGPS.h>       // GPS Parser Library
//#include "i2ct3.h"                 // Teensy I2C Library
#include "AdafruitSensor.h"        // Adafruit sensor libraries
#include "AdafruitBMP280.h"        // BMP280 Library
#include "Iridium_SBD.h"           // Iridium module library
#include "RHRF24.h"                // Radio module library
#include "AdafruitMCP23008.h"      // MCP23008 GPIO Expansion chip library
#include "ConfigSettings.h"        // Loonar Technologies Configuration file. 
#include "Configuration_Main.h"    // Loonar Technologies User Configuration file. 

/***********************************************************************************************************************************************************************/

/********** OBJECTS **********/
Adafruit_BMP280           bmp(CS_BMP);                         // BMP280 Pressure/Temperature Sensor
File                      logfile;                             // SD Card Logfile
HardwareSerial hsGPS      =       Serial1;                              // GPS Object
TinyGPS               tinygps;                             // GPS Parser
IridiumSBD                isbd(Serial2, RB_SLEEP);             // Iridium Module
RH_RF24                   rf24(GFSK_CS, GFSK_IRQ, GFSK_SDN);   // Si4463 Radio Object
Adafruit_MCP23008         GPIO_chip;                           // GPIO Expander Chip, i2c
//IntervalTimer             loonarInterrupt;                     // Interrupt to run all flight code independent of user code

/***********************************************************************************************************************************************************************/

/********** GLOBAL DATA STRUCT **********/

  float     flightDataLatitude;
  float     flightDataLongitude;
  float     flightDataLastAltitude;
  float     flightDataAltitude;
  float     flightDataMaxAltitude;
  float     flightDataAscentRate;
  float     flightDataBMPAltitude;
  float     flightDataBMPTemperature;
  float     flightDataTemperature;
  float     flightDataBatteryVoltage;
  float     flightDataSupercapVoltage;
  boolean   flightDataCutdown;
  boolean   flightDataLanded;
  double    flightDataMinutes;
  double    flightDataStartTime;
  double    flightDataIridiumTimer;
  uint8_t   flightDataFinalData[BUF_SIZE];
  long      flightDataCounter;

    
/***********************************************************************************************************************************************************************/

/********** FUNCTIONS **********/

/*--------------------------------------------------------------------------------------------------------------
   Function:
     userSetupCode
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     This is where the user can input his/her own setup functions.  You should treat this function as you would
     a regular Arduino setup() function.  DO NOT add code to the regular setup() function.  
--------------------------------------------------------------------------------------------------------------*/
void userSetupCode(){

 // YOUR SETUP CODE HERE!!!! 
  
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     userLoopCode()
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     This is where the user can input his/her own loop functions.  You should treat this function as you would
     a regular Arduino loop() function.  DO NOT add code to the regular loop() function.  
--------------------------------------------------------------------------------------------------------------*/
void userLoopCode(){
  
 // YOUR LOOP CODE HERE!!!! 
  
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     messageReceived
   Parameters:
     finaldata[] array that represents the received message 
   Returns:
     Nothing
   Purpose: 
     If there is a received message from the ground to either the radio module or the satellite modem, this 
     function will be called.  Therefore, place any code that you would like to respond to a received message
     in this function. Right now, all it does is print the received data. 
--------------------------------------------------------------------------------------------------------------*/
void messageReceived (uint8_t finaldata[], uint8_t leng){
 
 // YOUR CODE BASED ON RECEIVED DATA HERE
 
 for (uint16_t i = 0; i < leng; i++)
 {
   Serial.print((char)finaldata[i]); 
 }
 Serial.println();
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     setup
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Completes all necessary setup functions in order to prepare the electronics for flight. 
--------------------------------------------------------------------------------------------------------------*/
void setup()
{
  delay(1000);
  //Serial.println("GPIO Chip Init");
  init_GPIO_chip();                                  // Initialize the GPIO Expansion chip over I2C with the teensy i2c library integrated already into this.
  Serial.println("Initializing");
  setPinmodes();                                     // Initialize all the pinModes for every pin (i.e. input, output, etc).
  RadioOff();                                        // Shut off power to the radio.
  //IridiumOff();                                      // Shut off power to the Iridium modem
  //CameraOff();                                       // Shut off power to the camera.
  //FiveVOff();                                        // Shut off the 5 volt line.
  //CutdownOff();                                      // Shut off power to cutdown. 
  //CameraDeTrigger();                                 // De-trigger the camera (drive high).
  analogReadResolution(ADC_RESOLUTION);              // Set the ADC resolution to appropriate number of bits for maximum resolution
  analogReference(EXTERNAL);                         // Set the ADC reference voltage to the externally supplied 3.3V reference. 
  checkCallsign();                                   // Make sure user entered a correct FCC Callsign. 
  flightDataLatitude = LAUNCH_LATITUDE;             // Initialize flight data structure latitude float to the launch site latitude. 
  flightDataLongitude = LAUNCH_LONGITUDE;           // Initialize flight data structure longitude float to the launch site longitude. 
  flightDataLastAltitude = 0.0;                    // Initialize flight data structure last altitude float to 0 meters.
  flightDataAltitude = 0.0;                         // Initialize flight data structure altitude float to 0 meters. 
  flightDataAscentRate = 0.0;                       // Initialize flight data structure ascent rate float to 0 m/s.
  flightDataMaxAltitude = 0.0;                     // Initialize flight data structure maximum altitude float to 0 meters.
  flightDataTemperature = 0.0;                      // Initialize flight data structure temperature float to 0 celsius.
  flightDataBatteryVoltage = 0.0;                  // Initialize flight data structure battery voltage float to 0.0 V.
  flightDataSupercapVoltage = 0.0;                 // Initialize flight data structure supercapacitor voltage float to 0.0 V.
  flightDataCutdown = false;                        // Initialize flight data structure cutdown boolean to false.
  flightDataLanded = false;                         // Initialize flight data structure landed boolean to false.
  flightDataMinutes = 0.0;                          // Initialize flight data structure minutes double to 0 minutes.
  flightDataIridiumTimer = 1.0;                      // Initialize flight data structure first iridium time to 1 minute. 
  flightDataCounter = 0;                            // Initialize the transmit counter to 0.
  setupSDCard();                                     // Configure the SD card and set up the log file. 
  printLogfileHeaders();                             // Write headers to the log file.
  init_bmp();                                        // Initialize the BMP 280 Pressure/Temperature sensor.
  setGPSFlightMode();                                // Configure the GPS for flight mode to work at high altitudes.
  initRF();                                          // Turn on and initialize the Radio module.
  //chargeSuperCapacitor();                            // Charge the supercapacitor up.  
  //initIridium();                                     // Turn on the 5 volt line, give power to the Iridium module, and begin talking to it.
  initCameraVideo();                                 // Give power to the camera, then trigger the camera line to start video. 
  //userSetupCode();                                   // Call the user setup function code. 
  flightDataStartTime = millis();                   // Initializes the start time of the entire program. 
  //loonarInterrupt.begin(loonarCode, INTERVAL_TIME);  // Start the interrupt timer to run the entire loonar technologies program. 
  //Serial.println("Got here");
}

/***********************************************************************************************************************************************************************/

/*--------------------------------------------------------------------------------------------------------------
   Function:
     loop
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Only calls the loop function provided by the user. If the user loop function is empty, nothing will be called.
     All Loonar technologies code is operated on an interrupt and not in the loop. 
--------------------------------------------------------------------------------------------------------------*/
void loop()
{
  //userLoopCode();
  loonarCode();
  delay(100);
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
void loonarCode ()
{
  smartdelay(75);
  flightDataMinutes = getTime();                         // Acquire the current time since startup of the electronics. 
  flightDataLatitude = getLatitude();                    // Parse the latitude data from the GPS.
  flightDataLongitude = getLongitude();                  // Parse the longitude data from the GPS. 
  flightDataLastAltitude = flightDataAltitude;         // Store the last altitude float variable. 
  flightDataAltitude = getAltitude();                    // Parse the altitude data from the GPS.
  if (flightDataAltitude > flightDataMaxAltitude)
  {
    flightDataMaxAltitude = flightDataAltitude;        // Get the max altitude achieved. 
  }
  flightDataBMPTemperature = bmp.readTemperature();     // Read the temperature from the BMP280 sensor. 
  flightDataBMPAltitude = bmp.readPressure();           // Read the altitude from the BMP280 pressure sensor.
  //flightDataTemperature = rf24.get_temperature();                     // Acquire the temperature from the built in sensor from the radio chip.
  flightDataBatteryVoltage = getBatteryVoltage();       // Measure the voltage of the batteries. 
  flightDataSupercapVoltage = getSuperCapVoltage();     // Measure the voltage of the expansion board supercapacitor. 
  flightDataAscentRate = getAscentRate();
  checkIfLanded();                                        // Check if the balloon has landed.
  getConfiguredData();                                    // Configure the data we want to transmit via Iridium and/or RF.
  logToSDCard();                                          // Log all data to the SD Card.
  printToSerial();                                        // Print everything to the serial monitor. 
  //checkCutdown();                                         // Check to see if the conditions call for cutting down the balloon.
  //transceiveRF();                                         // Transmit and receive telemetry via the radio module. 
  //transceiveIridium();                                    // Transmit and receive telemetry via the Iridium modem. 
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     getAscentRate
   Parameters:
     None
   Returns:
     Float representing ascent rate of the balloon.
   Purpose: 
     Calculates the averaged ascent rate of the payload. 
--------------------------------------------------------------------------------------------------------------*/
 float getAscentRate()
{
   float ascent_rate_array[25] = {0.0};
   uint8_t ctr = 0;
  ascent_rate_array[ctr] = (float)(flightDataAltitude - flightDataLastAltitude)/((float)INTERVAL_TIME/1000000.0);
  ctr++;
  if (ctr >= 25) 
  {
    ctr = 0;
  }
  float ascent_rate = 0.0;
  for (int i = 0; i < 25; i++)
  {
    ascent_rate += ascent_rate_array[i];
  }
  ascent_rate /= 25.0;
  return ascent_rate;
}



/*--------------------------------------------------------------------------------------------------------------
   Function:
     checkIfLanded
   Parameters:
     None
   Returns:
     None.
   Purpose: 
     Checks to see if the payload has landed, and then changes loop time for Iridium to conserve power. 
--------------------------------------------------------------------------------------------------------------*/
 void checkIfLanded() 
{
  if (!flightDataLanded)
  {
    if ( (flightDataAltitude < 7000.0) 
    &&   (flightDataAltitude < (flightDataMaxAltitude - 5000.0)) 
    &&   (flightDataAscentRate < abs(0.20)))
    {
      flightDataLanded = true;
    }
  }

  if (flightDataLanded)
  {
    IRIDIUM_LOOP_TIME = 15.0;
    INTERVAL_TIME = 100000000; 
  }
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
 double getTime()
{
  return (double) (millis() - flightDataStartTime)/1000.0/60.0;  // Acquire the current time since startup of the electronics. 
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     checkCallsign
   Parameters:
     None
   Returns:
     None
   Purpose: 
     Makes sure user has inputted a FCC Callsign other than the default. 
--------------------------------------------------------------------------------------------------------------*/
 void checkCallsign()
{
  if (FCCID[0] == 'A' &&
      FCCID[1] == 'B' &&
      FCCID[2] == 'C' &&
      FCCID[3] == 'D' &&
      FCCID[4] == 'E' &&
      FCCID[5] == 'F')
  {
    Serial.println("Please enter a valid FCC Callsign ID in the user configuration file and reupload the code.");
  }
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
 void checkCutdown() 
{
  if (!flightDataCutdown) 
  {
    if (CUTDOWN_CONFIG == 1 || CUTDOWN_CONFIG == 5)
    {
      //if (tinygps.location.isValid())
      //{
        if (flightDataAltitude >= CUTDOWN_ALTITUDE) 
        {
          cutdownBalloon();
        }
      //}
    } 
    if (CUTDOWN_CONFIG == 2 || CUTDOWN_CONFIG == 5)
    {
      //if (tinygps.location.isValid())
      //{
        if (flightDataMinutes >= CUTDOWN_TIME) 
        {
          cutdownBalloon();
        }
      //}
    } 
    if (CUTDOWN_CONFIG == 3 || CUTDOWN_CONFIG == 5)
    {
      //if (tinygps.location.isValid())
      //{
        if ( (flightDataLatitude > CUTDOWN_LATITUDE_MAX) || 
             (flightDataLatitude < CUTDOWN_LATITUDE_MIN) || 
             (flightDataLongitude > CUTDOWN_LONGITUDE_MAX) || 
             (flightDataLongitude < CUTDOWN_LONGITUDE_MIN) ) 
        {
          cutdownBalloon();
        }
      //}
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
 void cutdownBalloon()
{
  if (!flightDataCutdown) {
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
    flightDataCutdown = true;
  }
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
 void getConfiguredData()
{
  char data[BUF_SIZE] = "";
  sprintf(data, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", 
    (int)(flightDataLatitude*10000), 
    (int)(flightDataLongitude*10000), 
    (int)(flightDataAltitude), 
    (int)(flightDataTemperature),
    (int)(flightDataBatteryVoltage*100), 
    (int)(flightDataSupercapVoltage*100), 
    (int)(flightDataCutdown),
    (int)(flightDataLanded),
    (int)(flightDataCounter),
    (int)(flightDataBMPAltitude), 
    (int)(flightDataBMPTemperature));
  
  Serial.print("Data: ");
  for (int i = 0; i < BUF_SIZE; i++)
  {
    Serial.print(data[i]);
    flightDataFinalData[i] = data[i];    
  } 
  Serial.println();
  flightDataCounter++;
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     transceiveRF
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Sends the current data stream to the Radio module for transmission.  Also reads in any data and calls the
     messageReceived function whenever incoming data is detected.   
--------------------------------------------------------------------------------------------------------------*/
 void transceiveRF() 
{   
  // If it is time to send our FCC ID as per law, send it
  if ((flightDataCounter % FCC_ID_INTERVAL) == 0)
  {
    rf24.send(FCCID, arr_len(FCCID));
    rf24.waitPacketSent(); 
  }

  // Send the contents of the flight data array
  rf24.send(flightDataFinalData, arr_len(flightDataFinalData));
  rf24.waitPacketSent();
/*
  // Time to parse messages. 
  uint8_t data[BUF_SIZE] = {0};
  uint8_t leng = BUF_SIZE;

  // Check to see if there is a received message. 
  if (rf24.recv(data, &leng))
  {
    // Check to see if the incoming message is a cutdown command. 
    boolean shouldCutdown = true;
    Serial.println("Incoming Potential Cutdown Message: ");
    for (size_t i = 0; i < arr_len(CUTDOWN_COMMAND); i++)
    {
      Serial.print(data[i]);
      if(data[i] != CUTDOWN_COMMAND[i]) 
      {
        shouldCutdown = false;
      }
    }
    
    Serial.println();

    // If the cutdown command has been parsed correctly, cutdown the balloon.
    if (shouldCutdown && (CUTDOWN_CONFIG == 4 || CUTDOWN_CONFIG == 5)) 
    {
      cutdownBalloon();
    }

    

    // If the message is not a loonar message, send it to the user messageReceived function
    if (!shouldCutdown)
    {
      messageReceived(data, leng); 
    }
  }*/
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
 void transceiveIridium() 
{ 
  noInterrupts(); // Turns off interrupts so we can transmit our message via Iridium.
  
  // If it is time to send and receive iridium messages
  if (flightDataMinutes > flightDataIridiumTimer) 
  {
    // Get the array length of our flight data. 
    size_t bufferSize = arr_len(flightDataFinalData);

    // Send and receive data through the Iridium network.
    isbd.sendReceiveSBDBinary(flightDataFinalData, BUF_SIZE, flightDataFinalData, bufferSize);
    char rxBuf [bufferSize];

    // Increment time for next Iridium transmission. 
    flightDataIridiumTimer += IRIDIUM_LOOP_TIME;

    // If there is an incoming message
    if (bufferSize > 0) 
    {
      // Parse the received message to see if there was a cutdown command
      boolean shouldCutdown = true;
      Serial.println("Incoming Potential Cutdown Message: ");
      for (size_t i = 0; i < bufferSize; i++)
      {
        rxBuf[i] = flightDataFinalData[i];
        Serial.print(rxBuf[i]);
        if(rxBuf[i] != CUTDOWN_COMMAND[i]) 
        {
          shouldCutdown = false;
        }
      }
      Serial.println();

      // If the cutdown command has been parsed correctly, cutdown the balloon.
      if (shouldCutdown && (CUTDOWN_CONFIG == 4 || CUTDOWN_CONFIG == 5)) 
      {
        cutdownBalloon();
      }

      // Parse the received message to see if there was a "landed" command
      boolean landed_copy_0 = true;
      boolean landed_copy = flightDataLanded;
      flightDataLanded = true;
      Serial.println("Incoming Potential 'Landed' Message: ");
      for (size_t i = 0; i < bufferSize; i++)
      {
        rxBuf[i] = flightDataFinalData[i];
        Serial.print(rxBuf[i]);
        if(rxBuf[i] != LANDED_COMMAND[i]) 
        {
          flightDataLanded = false;
          landed_copy_0 = false; 
        }
      }
      Serial.println();
      
      if (!flightDataLanded)
      {
        flightDataLanded = landed_copy;
      }

     
      if (!shouldCutdown && !landed_copy_0)
      {
        messageReceived(flightDataFinalData, (uint8_t)bufferSize);
      }
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
 float getTemp() 
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
 void initCameraVideo() 
{
  delay(5000);
  CameraOn();
  CameraDeTrigger();
  delay(5000);
  CameraTrigger();
  delay(1000);
  CameraDeTrigger();
  delay(10000);
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
 void chargeSuperCapacitor() 
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
 void initIridium()
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
 void initRF() 
{
  //SPI.setDataMode(SPI_MODE0);
  //SPI.setClockDivider(SPI_CLOCK_DIV2);  // Setting clock speed to 8mhz, as 10 is the max for the rfm22
  //SPI.begin(); 
  RadioOn();
  boolean ok = rf24.init(BUF_SIZE);
  if (!ok) Serial.println("Radio Err");
  rf24.setFrequency(FREQ); 
  delay(1000);
  rf24.send(FCCID, arr_len(FCCID));
  rf24.waitPacketSent();

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
void init_GPIO_chip()
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
void printLogfileHeaders() 
{
  logfile.print("Time, ");
  logfile.print("Lat,");
  logfile.print("Lng, ");
  logfile.print("Alt, ");
  logfile.print("Vel, ");
  logfile.print("MaxA, ");
  logfile.print("Alt2, ");
  logfile.print("Tmp2, ");
  logfile.print("Tmp1, ");
  logfile.print("BatV, ");
  logfile.print("CapV, ");
  logfile.print("Cut, ");
  logfile.print("Lnd, ");
  logfile.println("Cnt");
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
     Logs all relevant variables to the SD card log file.  
--------------------------------------------------------------------------------------------------------------*/
void logToSDCard() 
{ 
  logfile.print(flightDataMinutes);
  logfile.print(",");
  logfile.print(flightDataLatitude);
  logfile.print(",");
  logfile.print(flightDataLongitude);
  logfile.print(",");
  logfile.print(flightDataAltitude);
  logfile.print(",");
  logfile.print(flightDataAscentRate);
  logfile.print(",");
  logfile.print(flightDataMaxAltitude);
  logfile.print(",");
  logfile.print(flightDataBMPAltitude);
  logfile.print(",");
  logfile.print(flightDataBMPTemperature);
  logfile.print(",");
  logfile.print(flightDataTemperature);
  logfile.print(",");
  logfile.print(flightDataBatteryVoltage);
  logfile.print(",");
  logfile.print(flightDataSupercapVoltage);
  logfile.print(",");
  logfile.print(flightDataCutdown);
  logfile.print(",");
  logfile.print(flightDataLanded);
  logfile.print(",");
  logfile.println(flightDataCounter);
  logfile.flush();
}


/*--------------------------------------------------------------------------------------------------------------
   Function:
     pral
   Parameters:
     None
   Returns:
     Nothing
   Purpose: 
     Prints all relevant flight data to the Serial Monitor. 
--------------------------------------------------------------------------------------------------------------*/
void printToSerial() {/*
  Serial.print("Time: ");
  Serial.print(flightDataMinutes);
  Serial.print(", ");
  Serial.print("Lat: ");
  Serial.print(flightDataLatitude);
  Serial.print(", ");
  Serial.print("Long: ");
  Serial.print(flightDataLongitude);
  Serial.print(", ");
  Serial.print("Alt: ");
  Serial.print(flightDataAltitude);
  Serial.print(", ");
  Serial.print("Ascent Rate: ");
  Serial.print(flightDataAscentRate);
  Serial.print(", ");
  Serial.print("Max Alt: ");
  Serial.print(flightDataMaxAltitude);
  Serial.print(", ");
  Serial.print("BMP Alt: ");
  Serial.print(flightDataBMPAltitude);
  Serial.print(", ");
  Serial.print("BMP Temp: ");
  Serial.print(flightDataBMPTemperature);
  Serial.print(", ");
  Serial.print("Temp: ");
  Serial.print(flightDataTemperature);
  Serial.print(", ");
  Serial.print("BatV: ");
  Serial.print(flightDataBatteryVoltage);
  Serial.print(", ");
  Serial.print("CapV: ");
  Serial.print(flightDataSupercapVoltage);
  Serial.print(", ");
  Serial.print("Cut: ");
  Serial.print(flightDataCutdown);
  Serial.print(", ");
  Serial.print("Land: ");
  Serial.print(flightDataLanded);
  Serial.print(", ");
  Serial.print("Start: ");
  Serial.print(flightDataStartTime);
  Serial.print(", ");
  Serial.print("Iridium: ");
  Serial.print(flightDataIridiumTimer);
  Serial.print(", ");
  Serial.print("RF Count: ");
  Serial.println(flightDataCounter);*/
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
void init_bmp() 
{
  bmp.begin();
  //Serial.println(bmp.readTemperature()); 
  //Serial.println(bmp.readAltitude()); 
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
void setupSDCard() 
{
  if (!SD.begin(SD_CS)) {
    Serial.println("SD ERR");
    // don't do anything more:
    return;
  }
  char filename[] = "LOG000.csv";
  for (uint8_t i = 0; i < 1000; i++) 
  {
    filename[3] = (i / 100) + '0';
    filename[4] = (i / 10) % 10 + '0';
    filename[5] = (i % 10) + '0';
    if (! SD.exists(filename)) // only open a new file if it doesn't exist
    {   
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }
  if (!logfile) 
  {
    Serial.println ("SD ERR");
  } 
  else 
  {
    Serial.println(filename);
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
void setPinmodes()
{
  pinMode(GFSK_GATE, OUTPUT);
  //pinMode(CS_BMP, OUTPUT);
  //pinMode(SD_CS, OUTPUT);
  pinMode(RB_SLEEP, OUTPUT);
  pinMode(10,OUTPUT);
  //GPIO_chip.pinMode(GPS_GATE,OUTPUT);
  //GPIO_chip.pinMode(EN_5V,OUTPUT);
  //GPIO_chip.pinMode(RB_GATE,OUTPUT);
  GPIO_chip.pinMode(CAM_GATE,OUTPUT);
  GPIO_chip.pinMode(CAM_CTRL,OUTPUT);
  GPIO_chip.digitalWrite(CAM_GATE,HIGH);
  GPIO_chip.digitalWrite(CAM_CTRL,LOW);
  //GPIO_chip.pinMode(CUT_GATE,OUTPUT);
  //GPIO_chip.pinMode(BREAKOUT_5,OUTPUT);
  pinMode(VCAP_SENSE, INPUT);
  pinMode(VBAT_SENSE, INPUT);
  hsGPS.begin(9600);
  //Serial2.begin(19200);
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
float getSuperCapVoltage()
{
  return (float)analogRead(VCAP_SENSE) * 3.3 * 2.0 / (double)pow(2, ADC_RESOLUTION);
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
float getBatteryVoltage()
{
  return (float)analogRead(VBAT_SENSE) * 3.3 * 2.0 / (double)pow(2, ADC_RESOLUTION);
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
void IridiumOn()
{
  GPIO_chip.digitalWrite(RB_GATE, HIGH);
  delay(1000);
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
 void IridiumOff()
{
  GPIO_chip.digitalWrite(RB_GATE, LOW);
  delay(1000);
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
 void RadioOn()
{
  digitalWrite(GFSK_GATE, LOW);
  delay(1000);
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
 void RadioOff()
{
  digitalWrite(GFSK_GATE, HIGH);
  delay(1000);
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
 void CameraOn()
{
  //GPIO_chip.digitalWrite(CAM_CTRL,HIGH);
  GPIO_chip.digitalWrite(CAM_GATE, LOW);
  delay(1000);
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
 void CameraOff()
{
  GPIO_chip.digitalWrite(CAM_GATE, HIGH); 
  delay(1000);
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
 void CutdownOn()
{
  GPIO_chip.digitalWrite(CUT_GATE, HIGH);
  delay(1000);
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
 void CutdownOff()
{
  GPIO_chip.digitalWrite(CAM_GATE, LOW); 
  delay(1000);
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
 void FiveVOn()
{
  GPIO_chip.digitalWrite(EN_5V, HIGH);
  delay(1000);
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
 void FiveVOff()
{
  GPIO_chip.digitalWrite(EN_5V, LOW);
  delay(1000);
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
 void sendUBX(uint8_t len) 
{
  for(int i = 0; i < len; i++) 
  {
    hsGPS.write(MSG[i]);
    //Serial.print(MSG[i], HEX);
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
 boolean getUBX_ACK() 
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
        //Serial.print(b, HEX);
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
   byte gps_set_success = 0;
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
float getLatitude()
{
  float flat,flon;
  unsigned long age;
  tinygps.f_get_position(&flat, &flon, &age);
  if (flat == TinyGPS::GPS_INVALID_F_ANGLE) return flightDataLatitude;
  return flat; 
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
float getLongitude()
{
  float flat,flon;
  unsigned long age;
  tinygps.f_get_position(&flat, &flon, &age);
  if (flon == TinyGPS::GPS_INVALID_F_ANGLE) return flightDataLongitude;
  return flon; 
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
float getAltitude()
{
  float alti = tinygps.f_altitude();
  if (alti == TinyGPS::GPS_INVALID_F_ALTITUDE) return flightDataAltitude;
  return alti;
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
void smartdelay(unsigned long ms) 
{
  for (unsigned int start = millis(); millis() - start < 1000;)
  {
    while (hsGPS.available())
    {
      char c = hsGPS.read();
       Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (tinygps.encode(c)) // Did a new valid sentence come in?
        {//Serial.println("New Data");
          
        }
    }
  }
  /*
  Serial.println("SmartDelay");
  unsigned long timing = millis();
  do 
  {
    if (hsGPS.available()) 
    {  
      char a = hsGPS.read();
      tinygps.encode(a);
      Serial.print(a);
    }
  } while ((millis() - timing) < ms);*/
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
 void CameraDeTrigger() 
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
 void CameraTrigger() 
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



