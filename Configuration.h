
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

/********** PINOUT **********/
const int CS_BMP =                     2;    // Chip Select Pin for the BMP280 Sensor 
const int GFSK_GATE =                  5;    // Gate pin for power to the radio
const int GFSK_SDN =                   6;    // Shutdown pin for the radio
const int GFSK_GPIO2 =                 7;    // GPIO #2 pin for the radio 
const int GFSK_GPIO3 =                 8;    // GPIO #3 pin for the radio
const int GFSK_IRQ =                   14;   // IRQ pin for the radio
const int VCAP_SENSE =                 A1;   // Analog input to read supercapacitor voltage
const int RB_SLEEP =                   17;   // Iridium modem sleep pin
const int GFSK_GPIO1 =                 20;   // GPIO #1 pin for the radio
const int GFSK_GPIO0 =                 21;   // GPIO #0 pin for the radio
const int GFSK_CS =                    22;   // Chip Select Pin for the radio
const int SD_CS =                      23;   // Chip Select pin for the SD Card
const int VBAT_SENSE =                 A11;  // Analog input to read battery voltage


// Part of the GPIO Extender chip running on i2c
const int GPS_GATE =                   0;    // Gate pin for power to the GPS
const int EN_5V =                      2;    // Enable pin for the 5V Line
const int RB_GATE =                    3;    // Gate pin for power to the RockBlock
const int CAM_GATE =                   4;    // Gate pin for power to the camera
const int CAM_CTRL =                   5;    // Enable pin for the camera
const int CUT_GATE =                   6;    // Gate pin for power to cutdown

/***********************************************************************************************************************************************************************/

/********** CONSTANTS **********/  
const long     INTERVAL_TIME =                                   5000000;                      // Loop time for entire program in microseconds. 
      double   IRIDIUM_LOOP_TIME =                               6.0;                          // Loop time for Iridium in minutes. 
const float    SUPERCAP_MIN_LIMIT =                              4.75;                         // Minimum number of volts to charge the supercapacitor to during startup. 
const uint8_t  BUF_SIZE =                                        100;                          // Data array size for telemetry in bytes. 
const float    SUPERCAP_CUTDOWN_VOLTAGE_MIN =                    2.9;                          // Voltage to drain the supercapacitor to during cutdown in volts. 
const long     CUTDOWN_MAX_TIME =                                3000;                         // Maximum time to keep the cutdown active in milliseconds. 
const uint8_t  MSG[] = {                                                                       // Command to send to GPS for high altitude mode upon startup.
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
    

#endif /* Configuration_h */
