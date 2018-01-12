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
#include "TinyGPSPlusPlus.h"       // GPS Parser Library
#include "i2ct3.h"                 // Teensy I2C Library
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
#define hsGPS             Serial1                              // GPS Object
TinyGPSPlus               tinygps;                             // GPS Parser
IridiumSBD                isbd(Serial2, RB_SLEEP);             // Iridium Module
RH_RF24                   rf24(GFSK_CS, GFSK_IRQ, GFSK_SDN);   // Si4463 Radio Object
Adafruit_MCP23008         GPIO_chip;                           // GPIO Expander Chip, i2c
IntervalTimer             loonarInterrupt;                     // Interrupt to run all flight code independent of user code

/***********************************************************************************************************************************************************************/

/********** GLOBAL DATA STRUCT **********/
struct FlightData{
  float     latitude;
  float     longitude;
  float     last_altitude;
  float     altitude;
  float     max_altitude;
  float     ascent_rate;
  float     bmp_altitude;
  float     bmp_temperature;
  float     temperature;
  float     battery_voltage;
  float     supercap_voltage;
  boolean   cutdown;
  boolean   landed;
  double    minutes;
  double    startTime;
  double    iridiumTime;
  uint8_t   finaldata[BUF_SIZE];
  long      counter;
} flightData;
    
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
  Serial.begin(9600);
  //init_GPIO_chip();                                  // Initialize the GPIO Expansion chip over I2C with the teensy i2c library integrated already into this.
  Serial.println("Stuck here");
  setPinmodes();                                     // Initialize all the pinModes for every pin (i.e. input, output, etc).
  RadioOff();                                        // Shut off power to the radio.
  //IridiumOff();                                      // Shut off power to the Iridium modem
  //CameraOff();                                       // Shut off power to the camera.
  //FiveVOff();                                        // Shut off the 5 volt line.
  //GPSOff();                                          // Shut off power to the GPS.
  //CutdownOff();                                      // Shut off power to cutdown. 
  //CameraDeTrigger();                                 // De-trigger the camera (drive high).
  analogReadResolution(ADC_RESOLUTION);              // Set the ADC resolution to appropriate number of bits for maximum resolution
  analogReference(EXTERNAL);                         // Set the ADC reference voltage to the externally supplied 3.3V reference. 
  //checkCallsign();                                   // Make sure user entered a correct FCC Callsign. 
  flightData.latitude = LAUNCH_LATITUDE;             // Initialize flight data structure latitude float to the launch site latitude. 
  flightData.longitude = LAUNCH_LONGITUDE;           // Initialize flight data structure longitude float to the launch site longitude. 
  flightData.last_altitude = 0.0;                    // Initialize flight data structure last altitude float to 0 meters.
  flightData.altitude = 0.0;                         // Initialize flight data structure altitude float to 0 meters. 
  flightData.ascent_rate = 0.0;                       // Initialize flight data structure ascent rate float to 0 m/s.
  flightData.max_altitude = 0.0;                     // Initialize flight data structure maximum altitude float to 0 meters.
  flightData.temperature = 0.0;                      // Initialize flight data structure temperature float to 0 celsius.
  flightData.battery_voltage = 0.0;                  // Initialize flight data structure battery voltage float to 0.0 V.
  flightData.supercap_voltage = 0.0;                 // Initialize flight data structure supercapacitor voltage float to 0.0 V.
  flightData.cutdown = false;                        // Initialize flight data structure cutdown boolean to false.
  flightData.landed = false;                         // Initialize flight data structure landed boolean to false.
  flightData.minutes = 0.0;                          // Initialize flight data structure minutes double to 0 minutes.
  flightData.iridiumTime = 1.0;                      // Initialize flight data structure first iridium time to 1 minute. 
  flightData.counter = 0;                            // Initialize the transmit counter to 0.
  setupSDCard();                                     // Configure the SD card and set up the log file. 
  printLogfileHeaders();                             // Write headers to the log file.
  init_bmp();                                        // Initialize the BMP 280 Pressure/Temperature sensor.
  //GPSOn();                                           // Turn on power to the GPS.
  //setGPSFlightMode();                                // Configure the GPS for flight mode to work at high altitudes.
  initRF();                                          // Turn on and initialize the Radio module.
  //chargeSuperCapacitor();                            // Charge the supercapacitor up.  
  //initIridium();                                     // Turn on the 5 volt line, give power to the Iridium module, and begin talking to it.
  //initCameraVideo();                                 // Give power to the camera, then trigger the camera line to start video. 
  //userSetupCode();                                   // Call the user setup function code. 
  flightData.startTime = millis();                   // Initializes the start time of the entire program. 
  //loonarInterrupt.begin(loonarCode, INTERVAL_TIME);  // Start the interrupt timer to run the entire loonar technologies program. 
  Serial.println("Got here");
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
  userLoopCode();
  loonarCode();
  delay(1000);
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
  flightData.minutes = getTime();                         // Acquire the current time since startup of the electronics. 
  flightData.latitude = getLatitude();                    // Parse the latitude data from the GPS.
  flightData.longitude = getLongitude();                  // Parse the longitude data from the GPS. 
  flightData.last_altitude = flightData.altitude;         // Store the last altitude float variable. 
  flightData.altitude = getAltitude();                    // Parse the altitude data from the GPS.
  if (flightData.altitude > flightData.max_altitude)
  {
    flightData.max_altitude = flightData.altitude;        // Get the max altitude achieved. 
  }
  flightData.bmp_temperature = bmp.readTemperature();     // Read the temperature from the BMP280 sensor. 
  flightData.bmp_altitude = bmp.readAltitude();           // Read the altitude from the BMP280 pressure sensor.
  flightData.temperature = getTemp();                     // Acquire the temperature from the built in sensor from the radio chip.
  flightData.battery_voltage = getBatteryVoltage();       // Measure the voltage of the batteries. 
  flightData.supercap_voltage = getSuperCapVoltage();     // Measure the voltage of the expansion board supercapacitor. 
  flightData.ascent_rate = getAscentRate();
  //checkIfLanded();                                        // Check if the balloon has landed.
  //getConfiguredData();                                    // Configure the data we want to transmit via Iridium and/or RF.
  //logToSDCard();                                          // Log all data to the SD Card.
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
static float getAscentRate()
{
  static float ascent_rate_array[25] = {0.0};
  static uint8_t ctr = 0;
  ascent_rate_array[ctr] = (float)(flightData.altitude - flightData.last_altitude)/((float)INTERVAL_TIME/1000000.0);
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
static void checkIfLanded() 
{
  if (!flightData.landed)
  {
    if ( (flightData.altitude < 7000.0) 
    &&   (flightData.altitude < (flightData.max_altitude - 5000.0)) 
    &&   (flightData.ascent_rate < abs(0.20)))
    {
      flightData.landed = true;
    }
  }

  if (flightData.landed)
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
static double getTime()
{
  return (double) (millis() - flightData.startTime)/1000.0/60.0;  // Acquire the current time since startup of the electronics. 
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
static void checkCallsign()
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
static void checkCutdown() 
{
  if (!flightData.cutdown) 
  {
    if (CUTDOWN_CONFIG == 1 || CUTDOWN_CONFIG == 5)
    {
      if (tinygps.location.isValid())
      {
        if (flightData.altitude >= CUTDOWN_ALTITUDE) 
        {
          cutdownBalloon();
        }
      }
    } 
    if (CUTDOWN_CONFIG == 2 || CUTDOWN_CONFIG == 5)
    {
      if (tinygps.location.isValid())
      {
        if (flightData.minutes >= CUTDOWN_TIME) 
        {
          cutdownBalloon();
        }
      }
    } 
    if (CUTDOWN_CONFIG == 3 || CUTDOWN_CONFIG == 5)
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
  if (!flightData.cutdown) {
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
static void getConfiguredData()
{
  char data[BUF_SIZE] = "";
  sprintf(data, "%.4f, %.4f, %.1f, %.1f, %.1f, %.1f, %d, %d, %ld, %.1f, %.1f,,", 
    flightData.latitude, 
    flightData.longitude, 
    flightData.altitude, 
    flightData.temperature,
    flightData.battery_voltage, 
    flightData.supercap_voltage, 
    flightData.cutdown,
    flightData.landed,
    flightData.counter,
    flightData.bmp_altitude, 
    flightData.bmp_temperature);
  
  Serial.print("Final configured data before transmission: ");
  for (int i = 0; i < BUF_SIZE; i++)
  {
    Serial.print(data[i]);
    flightData.finaldata[i] = data[i];    
  } 
  Serial.println();
  flightData.counter++;
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
static void transceiveRF() 
{   
  // If it is time to send our FCC ID as per law, send it
  if ((flightData.counter % FCC_ID_INTERVAL) == 0)
  {
    rf24.send(FCCID, arr_len(FCCID));
    rf24.waitPacketSent(); 
  }

  // Send the contents of the flight data array
  rf24.send(flightData.finaldata, arr_len(flightData.finaldata));
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
static void transceiveIridium() 
{ 
  noInterrupts(); // Turns off interrupts so we can transmit our message via Iridium.
  
  // If it is time to send and receive iridium messages
  if (flightData.minutes > flightData.iridiumTime) 
  {
    // Get the array length of our flight data. 
    size_t bufferSize = arr_len(flightData.finaldata);

    // Send and receive data through the Iridium network.
    isbd.sendReceiveSBDBinary(flightData.finaldata, BUF_SIZE, flightData.finaldata, bufferSize);
    char rxBuf [bufferSize];

    // Increment time for next Iridium transmission. 
    flightData.iridiumTime += IRIDIUM_LOOP_TIME;

    // If there is an incoming message
    if (bufferSize > 0) 
    {
      // Parse the received message to see if there was a cutdown command
      boolean shouldCutdown = true;
      Serial.println("Incoming Potential Cutdown Message: ");
      for (size_t i = 0; i < bufferSize; i++)
      {
        rxBuf[i] = flightData.finaldata[i];
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
      boolean landed_copy = flightData.landed;
      flightData.landed = true;
      Serial.println("Incoming Potential 'Landed' Message: ");
      for (size_t i = 0; i < bufferSize; i++)
      {
        rxBuf[i] = flightData.finaldata[i];
        Serial.print(rxBuf[i]);
        if(rxBuf[i] != LANDED_COMMAND[i]) 
        {
          flightData.landed = false;
          landed_copy_0 = false; 
        }
      }
      Serial.println();
      
      if (!flightData.landed)
      {
        flightData.landed = landed_copy;
      }

     
      if (!shouldCutdown && !landed_copy_0)
      {
        messageReceived(flightData.finaldata, (uint8_t)bufferSize);
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
  //SPI.setDataMode(SPI_MODE0);
  //SPI.setClockDivider(SPI_CLOCK_DIV2);  // Setting clock speed to 8mhz, as 10 is the max for the rfm22
  //SPI.begin(); 
  RadioOn();
  boolean ok = rf24.init(BUF_SIZE);
  if (!ok) Serial.println("Unplug and replug the Loonar Mainboard");
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
static void init_GPIO_chip()
{
  GPIO_chip.begin(0);
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
  logfile.print("Asc Rate (m/s), ");
  logfile.print("Max Alt (m), ");
  logfile.print("Alt_Bar (m), ");
  logfile.print("Temp_Bar (C), ");
  logfile.print("Temp (C), ");
  logfile.print("BatV (V), ");
  logfile.print("CapV (V)");
  logfile.print("Cutdown");
  logfile.print("Landed");
  logfile.print("Start Time (ms)");
  logfile.print("Irid Time (min)");
  logfile.println("RF Count");
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
  logfile.print(flightData.ascent_rate);
  logfile.print(",");
  logfile.print(flightData.max_altitude);
  logfile.print(",");
  logfile.print(flightData.bmp_altitude);
  logfile.print(",");
  logfile.print(flightData.bmp_temperature);
  logfile.print(",");
  logfile.print(flightData.temperature);
  logfile.print(",");
  logfile.print(flightData.battery_voltage);
  logfile.print(",");
  logfile.print(flightData.supercap_voltage);
  logfile.print(",");
  logfile.print(flightData.cutdown);
  logfile.print(",");
  logfile.print(flightData.landed);
  logfile.print(",");
  logfile.print(flightData.startTime);
  logfile.print(",");
  logfile.print(flightData.iridiumTime);
  logfile.print(",");
  logfile.println(flightData.counter);
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
static void printToSerial() {
  double minutes = flightData.minutes;
  float latitude = flightData.latitude;
  float longitude = flightData.longitude;
  float altitude = flightData.altitude;
  float ascent_rate = flightData.ascent_rate;
  float max_altitude = flightData.max_altitude;
  float bmp_altitude = flightData.bmp_altitude;
  float bmp_temperature = flightData.bmp_temperature;
  float temperature = flightData.temperature;
  float battery_voltage = flightData.battery_voltage;
  float supercap_voltage = flightData.supercap_voltage;
  boolean cutdown = flightData.cutdown;
  boolean landed = flightData.landed;
  double startTime = flightData.startTime;
  double iridiumTime = flightData.iridiumTime;
  long counter = flightData.counter;
  Serial.print("Time: ");
  Serial.print(minutes);
  Serial.print(", ");
  Serial.print("Lat: ");
  Serial.print(latitude);
  Serial.print(", ");
  Serial.print("Long: ");
  Serial.print(longitude);
  Serial.print(", ");
  Serial.print("Alt: ");
  Serial.print(altitude);
  Serial.print(", ");
  Serial.print("Ascent Rate: ");
  Serial.print(ascent_rate);
  Serial.print(", ");
  Serial.print("Max Alt: ");
  Serial.print(max_altitude);
  Serial.print(", ");
  Serial.print("BMP Alt: ");
  Serial.print(bmp_altitude);
  Serial.print(", ");
  Serial.print("BMP Temp: ");
  Serial.print(bmp_temperature);
  Serial.print(", ");
  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.print(", ");
  Serial.print("BatV: ");
  Serial.print(battery_voltage);
  Serial.print(", ");
  Serial.print("CapV: ");
  Serial.print(supercap_voltage);
  Serial.print(", ");
  Serial.print("Cut: ");
  Serial.print(cutdown);
  Serial.print(", ");
  Serial.print("Land: ");
  Serial.print(landed);
  Serial.print(", ");
  Serial.print("Start: ");
  Serial.print(startTime);
  Serial.print(", ");
  Serial.print("Iridium: ");
  Serial.print(iridiumTime);
  Serial.print(", ");
  Serial.print("RF Count: ");
  Serial.println(counter);
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
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
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
void setPinmodes()
{
  pinMode(GFSK_GATE, OUTPUT);
  //pinMode(CS_BMP, OUTPUT);
  //pinMode(SD_CS, OUTPUT);
  //pinMode(RB_SLEEP, OUTPUT);
  pinMode(10,OUTPUT);
  /*GPIO_chip.pinMode(GPS_GATE,OUTPUT);
  GPIO_chip.pinMode(EN_5V,OUTPUT);
  GPIO_chip.pinMode(RB_GATE,OUTPUT);
  GPIO_chip.pinMode(CAM_GATE,OUTPUT);
  GPIO_chip.pinMode(CAM_CTRL,OUTPUT);
  GPIO_chip.pinMode(CUT_GATE,OUTPUT);*/
  //GPIO_chip.pinMode(BREAKOUT_5,OUTPUT);
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
static float getBatteryVoltage()
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
static void IridiumOn()
{
  GPIO_chip.digitalWrite(RB_GATE, HIGH);
  delay(100);
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
  delay(100);
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
  delay(100);
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
  delay(100);
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
  delay(100);
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
  delay(100);
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
  delay(100);
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
  delay(100);
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
  delay(100);
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
  delay(100);
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
  delay(100);
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
  delay(100);
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
  smartdelay(GPS_ACQUISITION_TIME);
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
  smartdelay(GPS_ACQUISITION_TIME);
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
  smartdelay(GPS_ACQUISITION_TIME);
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



