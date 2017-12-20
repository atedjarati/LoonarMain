
/********************************************************
Loonar Technologies Configuration Header File
 
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
#ifndef Configuration_h
#define Configuration_h

/********** LIBRARIES **********/



/***********************************************************************************************************************************************************************/

/********** PINOUT **********/
const int BREAKOUT_1 =                 4;    // Breakout pin #1 - Digital input/output only
const int BREAKOUT_2 =                 A10;  // Breakout pin #2 - Analog input only
const int BREAKOUT_3 =                 16;   // Breakout pin #3 - Digital input/output or analog input only. 
const int BREAKOUT_4 =                 A12;  // Breakout pin #4 - Digital input/output or analog input or DAC output only. 
const int BREAKOUT_6 =                 3;    // Breakout pin #6 - Digital input/output only 

// GPIO Expansion Chip
const int BREAKOUT_5 =                 7;    // Breakout pin #5 - Digital input/output only.  To use, put the prefix "GPIO_chip" before arduino functions.  For example:
                                             //                                               GPIO_chip.pinMode(1,OUTPUT);
                                             //                                               GPIO_chip.digitalWrite(1,HIGH);
                                             //                                               GPIO_chip.digitalRead(1);

/***********************************************************************************************************************************************************************/

/********** USER DEFINED CONSTANTS **********/
const double   FREQ =                  145.0;                        // Center frequency in MHz for the radio module.
const char     FCCID[6] =              {'K','K','6','M','I','S'};    // FCC Radio License Callsign.
const char     CUTDOWN_COMMAND[] =     {"CutdownBalloon"};           // Cutdown Command to be sent over radio/iridium communications.
const uint8_t  CUTDOWN_CONFIG =        1;                            // 1 for altitude cutdown, 2 for time cutdown, 3 for GPS fencing cutdown, 4 for Iridium cutdown.
const float    CUTDOWN_ALTITUDE =      25000.0;                      // Altitude at which cutdown will occur. 
const float    CUTDOWN_TIME =          100.0;                        // Time after initialization at which cutdown will occur.
const float    CUTDOWN_LATITUDE_MIN =  -90.0000;                     // Minimum southern latitude boundary at which cutdown will occur. 
const float    CUTDOWN_LATITUDE_MAX =  90.0000;                      // Maximum northern latitude boundary at which cutdown will occur.
const float    CUTDOWN_LONGITUDE_MIN = -180.0000;                    // Minimum western longitude boundary at which cutdown will occur.
const float    CUTDOWN_LONGITUDE_MAX = 180.0000;                     // Maximum eastern longitude boundary at which cutdown will occur. 


/***********************************************************************************************************************************************************************/

void userSetupCode(){

 // YOUR SETUP CODE HERE!!!! 
  
}

/***********************************************************************************************************************************************************************/

void userLoopCode(){
  
 // YOUR LOOP CODE HERE!!!! 
  
}

/***********************************************************************************************************************************************************************/

void messageReceived(uint8_t finaldata[]){
 
 // YOUR CODE BASED ON RECEIVED DATA HERE
  
}


#endif /* Configuration_h */
