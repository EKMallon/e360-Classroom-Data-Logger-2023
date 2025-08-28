// 2-module logger code by Edward Mallon - modified 2023 for the e360 course at Northwestern University
// https://thecavepearlproject.org/2023/12/01/the-e360-a-classroom-data-logger-for-science/
/*
This program supports an ongoing series of DIY 'Classroom Logger' tutorials from the Cave Pearl Project. 
The goal is to provide a starting point for self-built student projects in environmental monitoring.
This low power 2-module iteration runs from a CR2032 coin cell and uses EEprom memory to store sensor readings. 
Data download & logger control are managed through the IDE's serial monitor window at 500000 baud. 
The logger WILL NOT START until those serial handshakes are completed via a UART connection.
The most important rule to follow when adding new sensors is that this code can only accept 1, 2, 4, 8 or 16 sensorBytesPerRecord.
These 'powers of 2' fit in the I2C buffer AND divide evenly into the EEproms hardware page size to prevent wrap-around.

The only library you MUST INSTALL for the minimum configuration of Promini & RTC module is LowPower [by LowPowerLab]
--------------------------------------------------------------------------------------------------------------------
  This can be INSTALLED via the Library Manager or from https://github.com/LowPowerLab/LowPower

Three sensors are supported by this code natively so do not need a library:
  readNTC, readLDR, and Si7051  (the reference sensor we use for NTC calibrations)

The following sensors require library installations before they can be used:
---------------------------------------------------------------------------
  BH1750 LUX senso uses hp_BH1750  [by Stefan Armborst] via Lib Manager OR from https://github.com/Starmbi/hp_BH1750

  Sht30 Humidity sensor uses SHT85  [by Rob Tillaart] via Lib Manager OR from https://github.com/RobTillaart/SHT85/tree/master

  BMP280 uses BMP280_DEV  [by Martin Lindupp] which is available through the Lib Manager although the repo seems to have disappeared from github(?)

  BME280 (includes RH%) uses forcedBMX280  [by soylentOrange] which is available through the Lib Manager
  OR from https://github.com/soylentOrange/Forced-BMX280/tree/master NOTE that this library also works with the BMP280
  if you enable only recordBMEtemp_2byteInt and/or recordBMEpressure_2byteInt

*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// CURRENT 328p internal eeprom memory locations in use for start-up information
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
(0-3)     logger START TIME in unixtime (for reconstructiong timestamps later in sendSerial)
(4)       SampleIntervalMinutes
(5)       SampleIntervalSeconds
(6-9)     InternalReferenceConstant for calc. rail voltage from ADC in readRailVoltage
(10)      RTCagingOffset [adjusting by 3 compensates for ~ 1second of drift per month]
(12-15)   4byte UNIXtime of last time RTC was set by startMenu_setRTCtime - for drift tracking
(20-23)   Sensor starting paramaters usually stored from (24) onwards

FOUR 100-CHARACTER text info FIELDS also get stored in 328p internal eeprom:
Logger info:            64 to 164
Deployment info:        165 to 265
Calibration constants:  266 to 366
Site info:              367 to 467

These are output to screen by void setup_sendboilerplate2serialMonitor() when logger starts up
BUT you can store 'any' text in these fields, notes to self, normalization constants, etc
Note: we still have upper 512 bytes of 328p 1k eeprom availible for OLED screen fonts, etc.
// https://thecavepearlproject.org/2020/11/15/adding-two-oled-displays-to-your-arduino-logger-with-no-library/
*/

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// STEP0 :
// Create SENSOR definitions HERE to match the sensors you added to the logger
// Use these as global controls to enable/disable sensor code with #ifdef & #endif
// sensorBytesPerRecord for your sensor combination MUST TOTAL 1,2,4,8 or 16
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// LowestBattery & RTC_Temperature are the 2 byte 'base values' which are usually recorded with every sensor reading (as they require no extra sensor hardware beyond the logger itself)
#define logLowestBattery_1byte           // 1-byte (compressed): saves LowestBattery voltage recorded during EEprom saves
#define logRTC_Temperature_1byte         // 1-byte: the RTC's internal 0.25°C resolution temperature sensor
//#define logCurrentBattery_2byte          // RARELY USED - not 1byte compressed like LowestBattery, primarily included as a powers-of-2 balancing option
//#define logFreeVariableMemory_2byte      // RARELY USED - primarily included as a powers-of-2 rule balancing option that does not rely on any external sensors to be present

//#define readD7resistorwD8pullup_2byte      // NTC // 2-bytes: ohms // for explanation of the method for reading analog resistance with digital pins see
//#define readD6ResistorwD8pullup_2byte    // LDR // 2-bytes: ohms // https://thecavepearlproject.org/2019/03/25/using-arduinos-input-capture-unit-for-high-resolution-sensor-readings/
                                           // these have to match the connections shown in the build lab!
//#define readSi7051_Temp_2byte            // 2-bytes: often used for NTC calibration - does not require a library, functions for si7051 at end of program

//#define readBh1750_LUX_2byte             // 2-bytes: raw sensor output: gets converted to Lux during download

//#define readSht3x_Temp_2byte             // 2-bytes
//#define readSht3x_Humidity_2byte         // 2-bytes

// IF you enable all three BMP or BME outputs
// you will need two more bytes for an 8-byte record: try adding logLowestBattery_1byte & logRTC_Temperature_1byte 
//#define readBMP280_Temp_2byte            // 2-bytes
//#define readBMP280_Pressure_2byte        // 2-bytes
//#define recordBMP280_Altitude_2byte       // 2-bytes: calculated by library

//#define recordBMEtemp_2byteInt            // 2-byte NOTE: works with both BMP & BME
//#define recordBMEpressure_2byteInt        // 2-byte NOTE: works with both BMP & BME
//#define recordBMEhumidity_2byteInt        // 2-byte ONLY if BME 280 connected!

//#define OLED_64x32_SSD1306                // not a sensor, but enabled with define to include needed library - requires 1000uF rail capacitor!-

//r9_b10_g11_gnd12 is the DEFAULT on the e360 logger for the Pulse Width Modulation lab
#define LED_r9_b10_g11_gnd12               // enables code for RGB indicator LED // 1k limit resistor on shared GND line!
//#define LED_GndGB_A0_A2                  // For 2022 2-module build with NO breadboards: red channel leg on led cut, A0gnd Green A1, blue A2, default Red on d13 left in place
// Red LED on D13 gets used if both of the LED #define statements are commented out


//#define countPIReventsPerSampleInterval   // 2-bytes:  saves # of PIR HIGH events in a specified sample interval. Do not enable this with PIRtriggersSensorReadings - choose one or the other
//#define PIRtriggersSensorReadings         // 4-bytes: Still in beta!   Do not enable this with countPIReventsPerSampleInterval - choose one or the other
// does NOT use the regular RTC-alarm based sampling interval but instead records the seconds elapsed between EVERY PIR trigger event in a uint32_t long variable [uint16_t would overflow at ~18 hours]
// WARNING this can use alot of memory very quickly! - recommend use with larger eeprom memory attached
// PIRtriggersSensorReadings could be enabled with four other bytes of sensor data [for a total of 8 bytes per record] OR with another 12 bytes of sensor data for a total of 16 bytes per record.


#include <Wire.h>       // I2C bus coms library: RTC, EEprom & Sensors
#include <EEPROM.h>     // note: requires default promini bootloader (ie NOT optiboot)
#include <avr/power.h>  // library for shutting down 328p chip peripherals to lower runtime current
#include <avr/sleep.h>  // provides SLEEP_MODE_ADC to lower current during ADC readings in readRailVoltage() function
#include <LowPower.h>   // for interval & battery recovery sleeps
                        // from LowPowerLab (https://github.com/LowPowerLab/LowPower)

// Ref, Interval & Echo are reset via serial monitor input - so the values here don't matter
//-------------------------------------------------------------------------------------------------
int32_t InternalReferenceConstant = 1126400;  // default = 1126400L = 1100mV internal vref * 1024 // gets changed in setup via serial menu input option later
                                              // adding/subtracting 400 from the constant raises/lowers the 'calculated' result from readRailVoltage() by ~1 millivolt,
                                              // simply read the rail with a DVM while running on UART power and change the constant until the calculation is accurate
int8_t RTCagingOffset = 0;                    // stores -127 to +128 in the same two complement format that the register in the RTC uses
uint8_t SampleIntervalMinutes = 15;           // Allowed values: 1,2,3,5,10,15,20,30 for both - must divide equally into 60!
uint8_t SampleIntervalSeconds = 0;            // minutes must be zero for intervalseconds, seconds must be zero for intervalMinutes
                                              // NOTE: Make sure your sensor readings don't take longer than your sample interval!
                                              // If you over-run your alarm because the sensor took too long you will have to wait 24hours for next wakeup

bool ECHO_TO_SERIAL = false;                  // true enables multiple print statements throughout the code via if(ECHO_TO_SERIAL){} // also starts the run with no interval sync delay so timestamps are misaligned
bool displayMoreOptions = false;              // Flag toggle between Setup and Runtime Menus in void setup_displayStartMenu()
bool settingsChanged = false;                 // Flag forces backup copy of operating parameters from 1st 64bytes 328p to 1st 64bytes Ext. EEprom

// most VARIABLES below this point stay the SAME on all machines:
//---------------------------------------------------------------------------
#define fileNAMEonly (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__) //from: https://stackoverflow.com/questions/8487986/file-macro-shows-full-path
const char compileDate[] PROGMEM = __DATE__;  //  built-in function in C++ makes text string: Jun 29 2023
const char compileTime[] PROGMEM = __TIME__;  //  built-in function in C++ makes text string: 10:04:18

#define EEpromI2Caddr 0x57                    // Run a bus scanner to check where your eeproms are https://github.com/RobTillaart/MultiSpeedI2CScanner
#define totalBytesOfStorage 4096              // Default: 0x57 / 4096     bytes to use the 4k eeprom on the RTC module 
// 32k I2C EEprom Module: use 0x50 & 32768    // for 64k eeprom (soldered on top of 4k) usually at 0x50 & 65536 with no address pins pulled high
uint8_t sensorBytesPerRecord = 0;             // INCREMENTED at the beginning of setup to match #defined sensors. MUST divide evenly into EEprom Page buffer AND fit inside I2C buffer
uint32_t EEmemPointer = 64;                   // first 64 bytes reserved for backup, a counter that advances through the EEprom memory locations by sensorBytesPerRecord at each pass through the main loop

#ifdef logFreeVariableMemory_2byte
uint16_t freeVariableMemory = 0;              // for debugging or powers of two memory balancing
#endif

//defines & variables for ADC & readRailVoltage() function
//------------------------------------------------------------------------------
uint16_t CurrentBattery = 0;
uint16_t LowestBattery = 5764;                                  
uint16_t systemShutdownVoltage = 2795;        // MUST be > BrownOutDetect default of 2775mv (which is also the EEprom voltage limit)
byte default_ADCSRA,default_ADMUX;            // stores default ADC controll register settings for peripheral shut down
byte set_ADCSRA_2readRailVoltage, set_ADMUX_2readRailVoltage; // stores custom settings for readRailVoltage() via 1.1 internal band gap reference
volatile uint8_t adc_interrupt_counter;       // incremented in readADCLowNoise ISR to calculate average of multiple ADC readings

//defines & variables for DS3231 RTC
//------------------------------------------------------------------------------

#define RTC_TempRangeOffset 80                // 80 shifts 63degree range of 1byte encoding by +(80/4)C to handle COLD temps: -20C to +43.5C
                                              // change this Offset to 40 for WARMER climate deployments to set the range to: -10C to +53.5C
#define rtcAlarmInputPin 2                    // DS3231's SQW output is connected to interrupt0 pin D2 on the ProMini
#define DS3231_ADDRESS     0x68               // this is the I2C bus address of our RTC chip
#define DS3231_STATUS_REG  0x0F               // reflects status of internal operations
#define DS3231_CONTROL_REG 0x0E               // enables or disables clock functions
#define DS3231_TMP_UP_REG  0x11               // temperature registers (upper byte 0x11 & lower 0x12) gets updated every 64sec
#define DS3231_AGING_OFFSET_REG 0x10          // Aging offset register
uint8_t AlarmSelectBits;                      // sets which parts of time to use or ignore for nxt alarm // ALRM1_SET bits and ALRM2_SET are 0b1000 and 0b0111 respectively.
//uint32_t loggerStartTime;                     // uint32_t is large enough to hold the 10-digit unixtime number
char CycleTimeStamp[] = "0000/00/00,00:00:00";   //16 character array to store human readble time (with seconds)
uint8_t t_second,t_minute,t_hour,t_day,t_month; // current time variables populated by calling RTC_DS3231_getTime()
uint16_t t_year;                              //current year //note: yOff = raw year to which you need to add 2000
uint8_t Alarmday,Alarmhour,Alarmminute,Alarmsecond; // calculated variables for setting next alarm
volatile boolean rtc_INT0_Flag = false;       // used in startup time sync delay //volatile because it's changed in an ISR // rtc_d2_alarm_ISR() sets this boolean flag=true when RTC alarm wakes the logger
float rtc_TEMP_degC = 0.0;
bool DS3231_PowerLossFlag = false;

// temporary 'buffer' variables only used during calculations
//------------------------------------------------------------------------------
bool booleanBuffer;                           // boolean for functions that return a true/false or 1/0
uint8_t byteBuffer1 = 9;                      // 1-byte (8 bit) type = unsigned number from 0 to 255
uint8_t byteBuffer2 = 9;                      // note: uint8_t is the same as byte variable type
int16_t int16_Buffer = 9999;                 // 2-byte from -32,768 to 32,767
uint16_t uint16_Buffer= 9999;                 // 2-byte from 0 to 65535
int32_t int32_Buffer = 9999;                // 4-byte from -2,147,483,648 to 2,147,483,647
uint32_t uint32_Buffer= 9999;                 // 4-byte from 0 to 4,294,967,295 //used for millis() timing
float floatBuffer = 9999.99;                   // for float calculations
uint8_t hiByte,loByte;                        // for splitting 16-byte integers during EEprom save

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// STEP1 : #include sensor libraries, create GLOBAL variables, #defines etc. HERE
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef countPIReventsPerSampleInterval
//--------------------------
uint16_t d3_INT1_eventCounter = 0;
volatile boolean d3_INT1_Flag = false; 
#endif

#ifdef PIRtriggersSensorReadings
//--------------------------------
uint32_t currentPIRtriggerTime;
uint32_t previousPIRtriggerTime;
uint32_t d3_INT1_elapsedSeconds = 0;
uint16_t d3_INT1_eventCounter = 0;   //not used in this case but left in for compatiblity with countPIReventsPerSampleInterval
volatile boolean d3_INT1_Flag = false; 
#endif

#ifdef readD7resistorwD8pullup_2byte 
//----------------------------------
  uint32_t D7resistor_NewReading;                // max of 65535 limits our ability to measure 10kNTC at temps below zero C!
#endif

#ifdef readD6ResistorwD8pullup_2byte 
//----------------------------------
  uint32_t D6resistor_NewReading;                // NOTE this OVERFLOWS if resistance > 65535
#endif
#if defined(readD7resistorwD8pullup_2byte) || defined(readD6ResistorwD8pullup_2byte)
//------------------------------------------------
  #define referenceResistorValue 32768UL    //  ARBITRARY value that should be 'close' to actual but precise value is not required  //https://hackingmajenkoblog.wordpress.com/2016/08/12/measuring-arduino-internal-pull-up-resistors/
  volatile boolean triggered;               //  volatiles needed for all digital pin reading of resistance:        
  volatile uint16_t timer1CounterValue;     //  prepareForInterrupts(), ISR (TIMER1_OVF_vect), ISR (TIMER1_CAPT_vect),ReadD6riseTimeOnD8
#endif

#ifdef readBh1750_LUX_2byte 
//------------------------------------------------------------------------------
  #include <hp_BH1750.h>                    // by Stefan Armborst via Lib Manager OR from https://github.com/Starmbi/hp_BH1750 returns the sensor to sleep automatically after each read & supports auto-ranging.
  hp_BH1750 bh1750;                         // Instantiate a BH1750FVI library object
  uint16_t lux_BH1750_RawInt;               // raw reading before conversion to lux // 2-byte, 0 to 65535
  #define Bh1750_Address 0x23
#endif

#if defined(readSht3x_Temp_2byte) || defined(readSht3x_Humidity_2byte)
//---------------------------------------------------------------
// https://sensirion.com/media/documents/213E6A3B/63A5A569/Datasheet_SHT3x_DIS.pdf
// SHT30 Accuracy ±0.2°C, ±2%RH, Sht30 resolution is only: 0.01°C and 0.01 %RH
// idle state(single shot mode@3v) 0.2uA,  Supply current Measuring 600uA, Heater power 33mW 
// Soft reset time 1.5 ms, Measurement duration 15 ms (slow)  4 ms (fast)
// The SHT85 is protocol in this library is compatible with the SHT3x series of sensors

  #include <SHT85.h> // https://github.com/RobTillaart/SHT85/tree/master
  #define ShtRH_ADDRESS   0x44              // two address options: 0x44 & 0x45
  float Sht3x_Temp_degC,Sht3x_RH_percent;
  SHT85 sht3x(ShtRH_ADDRESS);
  //uint16_t Sht3x_Temp_RawInt,Sht3x_RH_RawInt; //library also supports raw integer output
#endif

#if defined(readBMP280_Temp_2byte) || defined(readBMP280_Pressure_2byte) || defined(recordBMP280_Altitude_2byte)
//-----------------------------------------------------------------------------------
  #include <BMP280_DEV.h>                 // Include the BMP280_DEV.h library  // NOTE: this library has disappeared from github?
  BMP280_DEV bmp280;                      // Instantiate (create) a BMP280_DEV object and set-up for I2C operation
  #define BMP280_Address 0x76
  float Bmp280_Temp_degC, Bmp280_Pr_mBar, Bmp280_altitude_m;  // Variables for sensor output
#endif

#if defined(recordBMEtemp_2byteInt) || defined(recordBMEpressure_2byteInt) || defined(recordBMEhumidity_2byteInt)
//-----------------------------------------------------------------------------------
  #include <forcedBMX280.h>  // install this through the library manager in the IDE
  // from https://github.com/soylentOrange/Forced-BMX280/blob/master/examples/BME280_full_example/BME280_full_example.ino
  // ctrl_meas - see datasheet section 5.4.5 // forced mode: ctrl_meas[0..1] 0b01 (0b11 for normal and 0b00 for sleep mode)
  // pressure oversampling x 1: ctrl_meas[4..2] 0b001
  // temperature oversampling x 1: ctrl_meas[7..5] 0b001
  // humidity does not have oversampling
  
  ForcedBME280 climateSensor = ForcedBME280();
  int32_t g_temperature;  // current temperature with 2 decimal places embedded in the integer
  uint32_t g_pressure;    // current pressure             "
  uint32_t g_humidity;    // current humidity             "
#endif

#ifdef readSi7051_Temp_2byte 
//------------------------------------------------------------------------------
  // we are not using a library - init and read functions for si7051 at end of program
  uint16_t TEMP_si7051=0;                  //NOTE sensors output overruns this uint16_t at 40C!
  #define Si7051_Resolution 0b00000000       // 0b00000000 = 14-bit (0.01 C rez), 13 bit = 0b10000000 (0.02 C rez), 12 bit = 0b00000001 (0.04 C rez),11 bit = 0b10000001 (0.08 C rez)
  #define Si7051_Address 0x40
#endif

#ifdef OLED_64x32_SSD1306
//------------------------------------------------------------------------------
#include <SSD1306Ascii.h>       // https://github.com/greiman/SSD1306Ascii
#include <SSD1306AsciiWire.h>   // tells SSD1306Ascii.h to use default wire library
#define oled_I2C_Address 0x3C   // 0X3C+SA0 can be 0x3C or 0x3D
SSD1306AsciiWire oled;          // declare oled object
#endif


//======================================================================================================================
//======================================================================================================================
//======================================================================================================================
//*   *   *   *   *   *   *   *   *   *   *   SETUP   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   * 
//======================================================================================================================
//======================================================================================================================
//======================================================================================================================
// NOTE: problems in setup usually call the error_shutdown() function which shuts down the logger

void setup () {

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// STEP2 : Adjust the sensorBytesPerRecord variable to match the #bytes in your
// sensor variables that will be saved to EEprom at each sampling interval
// sensorBytesPerRecord must be 1,2,4,8 or 16 bytes because of EEprom page boundaries
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  
  #ifdef logLowestBattery_1byte
    sensorBytesPerRecord = sensorBytesPerRecord + 1;            // NOW INDEX-compressed to 1-Byte [with slight loss of resolution]
  #endif
  #ifdef logRTC_Temperature_1byte
    sensorBytesPerRecord = sensorBytesPerRecord + 1;            // NOW INDEX-compressed to 1-Byte of eeprom storage
  #endif

  #ifdef countPIReventsPerSampleInterval
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            //  two-byte integer counts Rising of output channel of PIR sensor
  #endif

  #ifdef PIRtriggersSensorReadings
    sensorBytesPerRecord = sensorBytesPerRecord + 4;            //  4-byte integer: d3_INT1_elapsedSeconds
  #endif
  
  #ifdef readD7resistorwD8pullup_2byte 
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            //  two-byte integer: NTC ohms
  #endif
  #ifdef readD6ResistorwD8pullup_2byte
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            //  two-byte integer: LDR ohms
  #endif

  #ifdef readBh1750_LUX_2byte
    sensorBytesPerRecord = sensorBytesPerRecord + 2;           // two-byte integer for RAW reading before conversion to lux
  #endif

  #ifdef readSht3x_Temp_2byte
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            // two-byte integer 
  #endif
  #ifdef readSht3x_Humidity_2byte
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            // two-byte integer 
  #endif
  
  #ifdef readBMP280_Temp_2byte
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            // two-byte integer 
  #endif
  #ifdef readBMP280_Pressure_2byte
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            // two-byte integer 
  #endif
  #ifdef recordBMP280_Altitude_2byte
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            // two-byte integer 
  #endif
 
  #ifdef recordBMEtemp_2byteInt
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            // two-byte integer 
  #endif
  #ifdef recordBMEpressure_2byteInt
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            // two-byte integer 
  #endif
  #ifdef recordBMEhumidity_2byteInt
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            // two-byte integer 
  #endif
  
  #ifdef logCurrentBattery_2byte
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            // two-byte integer
  #endif
  #ifdef logFreeVariableMemory_2byte
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            // two-byte integer
  #endif
  
  #ifdef readSi7051_Temp_2byte
    sensorBytesPerRecord = sensorBytesPerRecord + 2;            // two-byte integer  
  #endif
  

// General Startup housekeeping: Set UNUSED digital pins to a known state at startup to reduce current & noise
//------------------------------------------------------------------------------------------------------------
   
    pinMode(13, INPUT);       // turn of D13 onboard red LED by setting D13 to INPUT & LOW
  #ifdef LED_r9_b10_g11_gnd12                       // we will use INPUT & PULLUP resistor to PIP the leds to reduce current
    for (int i = 9; i <=12; i++) { digitalWrite(i, LOW);  pinMode(i, INPUT); }
    pinMode(12, OUTPUT);                            // the common ground line on our RGB led must OUTPUT to allow current
  #endif

  #ifdef LED_GndGB_A0_A2
    bitClear(PORTC,1); bitClear(DDRC,1);       // A1 [green] LED LOW & INPUT
    bitClear(PORTC,2); bitClear(DDRC,2);       // A2 [Blue] LED LOW & INPUT
    bitClear(PORTC,0); bitSet(DDRC,0);         // A0 GND pin LOW & OUTPUT
#endif //LED_GndGB_A0_A2

  // set UNUSED digital pins to LOW & OUTPUT so EMI noise does not toggle the pin & draw current
    for (int i = 3; i <=8; i++) { digitalWrite(i, LOW);  pinMode(i, OUTPUT); } //Note: if an interrupt source connected to D3 then the pin must be reset to INPUT

  //Disable UNUSED Peripherals to save power - restart them later when needed - At 3V @ 25°C, the ADC consumes ~87µA so we disable it to save power
    SPCR = 0; power_spi_disable();                  // stop the peripheral clock with SPCR = 0 BEFORE calling power_spi_disable(); -same applies to the ADC
    power_timer1_disable(); power_timer2_disable(); // NOTE: DON'T mess with timer0! - other peripherals like the I2C bus require Timer0 operating
    bitSet(ACSR,ACD);                               // Disables the analog comparator on AIN0(PD6) and AIN1(PD7)by setting the ACD bit (bit 7) of the ACSR register to one. analog comparator draws ~51µA.


// ADC Configuration: default & modified control register settings saved into storage variables
//------------------------------------------------------------------------------------------------------------
// A3..A0 - make sure pullups are OFF so they dont interefere with ADC readings // ignore A4/5 because I2C bus has hardware pullups (on RTC module)
    digitalWrite(A0,LOW);digitalWrite(A1,LOW);digitalWrite(A3,LOW);
  #ifndef LED_GndGB_A0_A2                           // diconnects the DIGITAL inputs sharing analog pins 0..3 (but NOT on 4&5 which are used by I2C as digital pins) 
    DIDR0 = 0x0F;                                   // Once disabled, a digitalRead on those pins will always return zero.
  #endif                                            // Digital input circuits can 'leak' a relatively high amount of current if the analog input is approximately half-Vcc 

  analogReference(DEFAULT); analogRead(A3);         // sets the ADC channel to A3 input pin
  default_ADCSRA = ADCSRA; default_ADMUX = ADMUX;   // Saves the DEFAULT ADC control registers into byte variables so we can restore those ADC control settings later

  bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,1);bitWrite(ADCSRA,ADPS0,0);     // 64 (default) prescalar @ 8MHz/64 = 125 kHz, =~104uS/ADC reading
  set_ADCSRA_2readRailVoltage = ADCSRA;             // store the modified ADCSRA register values for use in readRailVoltage() function & while saving eeprom data

  // modify ADC settings to reading the battery/rail voltage using the internal 1.1v reference inside the 328p chip 
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0); // from https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  set_ADMUX_2readRailVoltage = ADMUX;               // store modified ADMUX register values in a variable for use in readRailVoltage()

  ADMUX = default_ADMUX;                            //restore the default
  ADCSRA = 0; power_adc_disable();                  //turn off the ADC to save power At 3V @ 25°C, the ADC consumes ~87µA
  

// Configure the DS3231 Real Time Clock control registers for coincell powered operation
//------------------------------------------------------------------------------------------------------------
  Wire.begin();   // Start the I2C bus // enables internal 30-50k pull-up resistors on SDA & SCL by default

  if(totalBytesOfStorage==4096){ Wire.setClock(100000UL);}
    else {Wire.setClock(400000UL);}
  // Set the I2C bus speed to 100 kHz - but only because the default 4k eeprom is so OLD
  // NOTE: you can speed up to 400khz for the DS3231 and other I2C device coms
  // later on the in the main loop

  byteBuffer1 = i2c_readRegisterByte(DS3231_ADDRESS, DS3231_STATUS_REG) >> 7; //0Fh bit 7 is OSF: Oscillator Stopped Flag
  if(byteBuffer1){DS3231_PowerLossFlag = true;}                 // use this later to warn user via start menu 
  // i2c_setRegisterBit function requires: (deviceAddress, registerAddress, bitPosition, 1 or 0)
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_STATUS_REG, 3, 0);  // disable the 32khz output  pg14-17 of datasheet  // This does not reduce the sleep current but can't run because we cut VCC
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 6, 1); // 0Eh Bit 6 (Battery power ALARM Enable) - MUST set to 1 for wake-up alarms when running from the coincell bkup battery
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 7, 0); // Enable Oscillator (EOSC).  This bit is clear (logic 0) when power is first applied.  EOSC = 0 IS REQUIRED with OUR LOGGER DESIGN:
                                                                // when EOSC (bit7) is 0, the RTC oscillator continues running during battery powered operation. Otherwise it would stop.
  RTC_DS3231_ResetBothAlarmFlags();                               // stops RTC from holding the D2 interrupt line low if system reset just occured 
  digitalWrite(2, LOW);  pinMode(2, INPUT);                     // D2 INPUT & D2 pullup off because it is not requried with 4k7 hardware pullups on the RTC module
  bitSet(EIFR,INTF0); bitSet(EIFR,INTF1);                       // clears any previous trigger-flags inside the 328p processor for interrupt 0 (D2) &  interrupt 1 (D3)

  //DS3231 does not have any non-volatile memory so all of the internal registers reset to default if power is lost - so we store this in the 328p eeprom
  RTCagingOffset = EEPROM.read(10);                             // int8_t can store from -127 to +128
      if((RTCagingOffset<-128) || (RTCagingOffset>127)){        // if value stored in eeprom is outside normal operating parameters
        RTCagingOffset=0;                                       // then resets it to the 0 default 
        EEPROM.update(10,0);                                    // and store that default back in the eeprom
        }
  i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_AGING_OFFSET_REG,RTCagingOffset); delay(15);
  RTCagingOffset = i2c_readRegisterByte(DS3231_ADDRESS,DS3231_AGING_OFFSET_REG);  //RTCagingOffset variable in matching twos complement format as register
  // NOTE the RTCagingOffset value gets displayed as part of the serial menu

// Check Previous run Parameters stored in 328p eeprom [ update happens only on 1st run of a brand new logger]
//------------------------------------------------------------------------------------------------------------
  EEPROM.get(6,InternalReferenceConstant);                      //in this case .get is reading 4 consecutive bytes into the long (32bit) integer InternalReferenceConstant
  if((InternalReferenceConstant<1000000) || (InternalReferenceConstant>1228800)){ //if value stored in eeprom is outside normal operating parameters
    InternalReferenceConstant=1126400;                          // then re-sets it to the default 1126400
  EEPROM.put(6,InternalReferenceConstant);}                     // and store that default back in the eeprom
  
  SampleIntervalMinutes = EEPROM.read(4);                       // retrieve 'previous' sampling interval data stored in the CPU's internal 1024 bytes of eeprom space
  SampleIntervalSeconds = EEPROM.read(5);                       // these numbers will be random the first time the logger is run because the EEprom memory locations are empty
  if((SampleIntervalMinutes>60) || (SampleIntervalSeconds>30))  //if values read from eeprom are outside allowed maximums then reset to 'safe' default values
    { SampleIntervalMinutes=15;SampleIntervalSeconds=0;
      EEPROM.update(4,SampleIntervalMinutes); 
      EEPROM.update(5,SampleIntervalSeconds); }                 // .update is the same as .put, except that it only writes the data if there is a change - eeprom has a limited # of write cycles

//===============================================================================================
// Logger Configuration Menu via Serial Monitor  [loops for 8 minutes or until START is selected]
//===============================================================================================
  // NOTE: Opening the serial monitor always "restarts" the Arduino
  Serial.begin(500000);                                         // 500000 baud is max possible with 8Mhz processor http://wormfood.net/avrbaudcalc.php
  setup_sendboilerplate2serialMonitor();                        // send currently running code fileNAME, deployment details to monitor
//----------------------------------------------------------------------------------------

  setup_displayStartMenu();                                     // see this function & linked sub functions after end of MAIN loop
//----------------------------------------------------------------------------------------

// check if ECHO is on & confirm with user that's OK
  if(ECHO_TO_SERIAL){                                           //confirmation check so we don't waste battery power
  Serial.println(F("Serial Output ON! - NOT compatible w bat. powered operation. Disable? y/n"));
                                                                //DO NOT ENABLE ECHO_TO_SERIAL during battery powered logger operation or system wastes a lot of power waiting for a serial handshake that never arrives
  booleanBuffer = true; byteBuffer1 = 0;
    while (booleanBuffer) {
        if (Serial.available()) {byteBuffer1 = Serial.read();}  //.read captures only character at a time from the serial monitor window
        
        switch (byteBuffer1) {
          case 'y': 
            ECHO_TO_SERIAL = !ECHO_TO_SERIAL; booleanBuffer = false; break;
          case 'n': 
            booleanBuffer = false; break;
          default: break;
              }       // terminates switch case
        }             // terminates while(booleanBuffer)
   }                  // terminates if(ECHO_TO_SERIAL)
   
  Serial.print(F("Serial "));Serial.println(ECHO_TO_SERIAL ? "ON" : "OFF");  // ? here is short-form of if..else statement which checks the boolean and prints first text if true, second option if false
  Serial.println();

//Final confirmation check before erasing EEprom & starting the logger
  Serial.println(F("Type 'start' (& enter) to clear the EEprom & begin logging"));
  //Serial.println(F("Any other input will shut down the logger"));
  Serial.println(F("PROCEEDING at **this** point will ERASE ALL previous DATA!"));
//------------------------------------------------------------------------------------------------------------
  Serial.println();

  String command="";                                          // any variables delcared in the setup function get deleted at end of setup function
  boolean goFlagReceived = false;                             // so these variables will 'disapear' when we reach the main loop
  Serial.setTimeout(100000);                                  // need to set timeout or .readStringUntil would wait for input forever...
  unsigned long startMillis = millis();

do { command = Serial.readStringUntil('\n');                  // read serial monitor data into the string until carridge return = \n character

    // here we are using if statements to check the input instead of the switch / case method used above, goFlagReceived only becomes 'true' with valid input
    if(command == "start"){ 
      Serial.println(F("Erasing EEprom: Stay on UART power until done"));
      //------------------------------------------------------------------------------
                
      for (uint32_t memoryLocation=64; memoryLocation<totalBytesOfStorage; memoryLocation+=16){  // loop writes 16-bytes at a time into the I2C buffer
          Wire.beginTransmission(EEpromI2Caddr);
          Wire.write(highByte(memoryLocation));               // sends only the HiByte of the 2-byte integer address
          Wire.write(lowByte(memoryLocation));                // send only the LowByte of the address
          for (byte k=0; k<16; k++) { Wire.write(0);}         // we are loading the I2C buffer with 16 'zeros' each time
          Wire.endTransmission();                             // only at this command does the I2C bus transmission actually occcur

          // here we use modulo to print a progress dots in the serial monitor as the memory erase proceeds // also generates a return when memoryLocation=0
          if ((memoryLocation % 64) == 0){Serial.print(F("."));} // progress bar dot every 32 memory locations // %(modulo) = reminder after division 
          if ((memoryLocation % 4096) == 0){Serial.println();}   // serial printing takes approximately 10 seconds divided by the baud rate, per character
          
          // while loop which polls the eeprom to see if its ready for the next bytes to be written - can't progress past this point until EEprom says Yes
          do{ Wire.beginTransmission(EEpromI2Caddr); }while (Wire.endTransmission() != 0x00);  // endTransmission returns ZERO for successfully ACKnowledgement ONLY when EEprom is READ for more data
          
      }   //terminates: for (int memoryLocation=0; memoryLocation<totalBytesOfStorage; memoryLocation+=16){
     
          Serial.println(); goFlagReceived=true;
          
    } else if(command == "test")                // 'test' is a HIDDEN option not on the displayed menu
           {goFlagReceived=true;}               // 'test' simply skips erasing eeprom: I only use this to make debugging faster
 
  if(goFlagReceived) break;                     // breaks out of the while loop when goFlagReceived=true;
 
}while ((millis() - startMillis) < 200000);     // terminates the do-while after 200 seconds BEFORE loop times out with goFlagReceived=false which leads to logger shutdown

if (!goFlagReceived){                           // if goflag=false then the loop timed out so shut down the logger
      Serial.print(F("Menu Timeout with NO command! "));
      Serial.flush();error_shutdown();          // shut down the logger
}


Serial.print(F("Initializing sensors: "));Serial.flush();
//------------------------------------------------------------------------------------------------------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// STEP3 : Initialize your sensor (if needed)
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// this is where you could configure your control registers & usually take a first reading
// often wrapped with    #ifdef Sensor_Address  ...  #endif statements
// RTC is already initialized in setup!

// no initialization needed for readD7resistorwD8pullup_2byte or readD6ResistorwD8pullup_2byte

// BH1750 initialization
//-----------------------
#ifdef readBh1750_LUX_2byte                 // using library:  https://github.com/Starmbi/hp_BH1750  
  bh1750.begin(Bh1750_Address);       // set address & initialize
      //bh1750.calibrateTiming();     // NOTE: ambient light must be at least 140 lux when calibrateTiming runs!
      //Serial.println(F("BH1750 light sensor MUST be exposed to >150 lux @ startup")); 
      //Serial.println(F("for self-cal or it may freeze randomly. 15 sec = minimum interval for this sensor"));Serial.println();Serial.flush();
  bh1750.start(BH1750_QUALITY_LOW, BH1750_MTREG_LOW); // Quality LOW = fastest readings
      // QUALITY_HIGH -Resolution Mode Measurement Time 120-180 ms depending on light levels
      // QUALITY_LOW  -Resolution Mode Measurement Time 16-24 msec //LOW MTreg:31  resolution lux:7.4, 121557 is highest lux
  Serial.print(F("BH1750 started,"));Serial.flush();
#endif // terminates bh1750 init.

// Sht3x initialization
//-----------------------
#if defined(readSht3x_Temp_2byte) || defined(readSht3x_Humidity_2byte)
//---------------------------------------------------------------
  sht3x.begin(); 
  Serial.print(F("Starting Sht3x"));Serial.flush();
  
  // first reading should be done slowly with (false)
  sht3x.requestData(false);  //default () = fast = true blocks 4 (fast) or 15 (slow) milliseconds 
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF); // + an extra 2msec to wakeup
  //do{ booleanBuffer = sht3x.dataReady(false); }while(!booleanBuffer); //wait for data ready - this only checks if enough time has passed to read the data. (15 milliseconds)
  sht3x.readData(false);      
  // default () = fast = true blocks 4 (fast) or 15 milliseconds if false = slow
  // The parameter true/false should be the same in requestData() and dataReady()
  Sht3x_Temp_degC = sht3x.getTemperature();
  Sht3x_RH_percent = sht3x.getHumidity();
  Serial.print(F(" RH%: "));Serial.print(Sht3x_RH_percent,2);  
  Serial.print(F(" T[°C]: "));Serial.print(Sht3x_Temp_degC,2);
  Serial.println();Serial.flush();

  //alternatively:
  //Sht3x_RH_RawInt= sht3x.getRawHumidity();  //raw two-byte representation of humidity directly from the sensor
  //Sht3x_Temp_RawInt = sht3x.getRawTemperature(); //raw two-byte representation of temperature directly from the sensor
  //Sht3x_RH_percent = Sht3x_RH_RawInt * (100.0 / 65535);
  //Sht3x_Temp_degC = Sht3x_Temp_RawInt * (175.0 / 65535) - 45;

  // OTHER commands: from https://github.com/RobTillaart/SHT85/tree/master
  // sht3x.setTemperatureOffset( );  // Offset is defined in degrees Celsius.
  // sht3x.setHumidityOffset( );     // Default offsets are zero for both temperature and humidity. 
  // These functions allows one to adjust them a little. Note there is no limit to the offset so one can use huge values. This allows to use an offset of 273.15 effectively creating °Kelvin instead of Celsius.
  // sht3x.reset(); //(bool hard = false) resets the sensor, soft reset by default. Returns false if it fails.
  // sht3x.heatOff();   
  // uint16_t stat = sht3x.readStatus();Serial.print(F("Status: \t"));Serial.println(stat, HEX);
  // uint32_t ser = sht3x.GetSerialNumber();Serial.print(F("Serial Num: \t"));Serial.println(ser, HEX);
  // Serial.print(F("LIB_VERSION: \t"));Serial.println(SHT_LIB_VERSION); Serial.flush();
#endif

// BMP280 initialization
//-----------------------
#if defined(readBMP280_Temp_2byte) || defined(readBMP280_Pressure_2byte) || defined(recordBMP280_Altitude_2byte)
  bmp280.begin(BMP280_Address);                             // or bmp280.begin(BMP280_I2C_ALT_ADDR); for sensors at 0x76
      //Options are OVERSAMPLING_SKIP, _X1, _X2, _X4, _X8, _X16 // pg 15 datasheet One millibar = 100 Pa
  bmp280.setPresOversampling(OVERSAMPLING_X4);
      //X2 Low power              17 bit / 1.31 Pa     7.5-8.7msec    3-4µA @ 1 Hz forced mode
      //X4 Standard resolution    18 bit / 0.66 Pa     11.5-13.3ms  7-10µA @ 1 Hz forced mode
      //X8 High resolution        19 bit / 0.33 Pa     19.5-22.5ms  12-18µA @ 1 Hz forced mode
      //X16 Ultra high resolution 20 bit / 0.16 Pa     37.5-43.2ms  24-37µA @ 1 Hz forced mode
  bmp280.setTempOversampling (OVERSAMPLING_X2);
      //X2 Low power              17 bit / 0.0025 °C   // NOTE: Higher levels of Temperature oversampling
      //X4 Standard resolution    18 bit / 0.0012 °C   // provides NO improvement to ACCURACY of pressure readings
      //X8 High resolution        19 bit / 0.0006 °C
      //X16 Ultra high resolution 20 bit / 0.0003 °C
  bmp280.setSeaLevelPressure (1013.25f);                      // default value for altitude calculations
  bmp280.setIIRFilter(IIR_FILTER_OFF); 

//take the first reading (just a throw away reading to load the output registers)
  bmp280.startForcedConversion();                             // time needed here depends on oversampling settings
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);            // 60MSEC = long enough for max rezolution settings
  bmp280.getCurrentMeasurements(Bmp280_Temp_degC,Bmp280_Pr_mBar,Bmp280_altitude_m);

  Serial.print(F("BMP280 started,"));Serial.flush();
#endif // terminates BMP280 init.

// BME280 initialization
//-----------------------
#if defined(recordBMEtemp_2byteInt) || defined(recordBMEpressure_2byteInt) || defined(recordBMEhumidity_2byteInt)
  // NOTE this library defaults to lowest resoulution to save power
  // the NOISE depends on the oversampling and, for pressure and temperature on the filter setting used 
  // noise in temp at x1 oversampling: 0.005 deg C at 25C  [see chapter 3.6]
  // noise in pressure at x1 oversampling: 3.3mbar @x1 but 1.3mbar at x16 at 25C
  // IIR filtering can't be used with one-shot because it requires you to take multiple readings in succession
  
  while (climateSensor.begin()) {Serial.println(F("Waiting for sensor..."));delay(1000);}
  Serial.print("BmE280 ID:0x");
  Serial.print(climateSensor.getChipID(), HEX);
  Serial.println();Serial.flush(); 
#endif // terminates BME280 init.

#ifdef readSi7051_Temp_2byte
//--------------------
  initializeSI7051(); // we are not using a library so you can scroll down to read this function at the end of this program
  // if that function was in an #included library it would usually have an object .prefix something like:  si7051.initialize()
  Serial.print(F("Si7051 started"));Serial.flush();  // if you see this message in the serial monitor you know the sensor did not hang the machine...
#endif

#ifdef OLED_64x32_SSD1306       // using library:  https://github.com/greiman/SSD1306Ascii/blob/master/src/SSD1306Ascii.cpp
  oled.begin(&Adafruit128x64,oled_I2C_Address);
  oled.setFont(System5x7);      // list of other availiable fonts: https://github.com/greiman/SSD1306Ascii/tree/master/src/fonts
  oled.setContrast(64); //The contrast level -range 0 to 255. // LOWER CONTRAST = DIMMER SCREEN = LESS CURRENT
  oled.clear();
  oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF); // switch display off until it is needed
  // NOTE 0.49 inch OLED Display is only 64x32 pixels which ONLY OCCUPIES THE CENTER SQUARE of the SSD1306's 128x64 memory locations
#endif

Serial.println(F("& Starting the logger:"));Serial.flush();

//========================================================================================
// FINAL STARTUP PROCEDURE: is to DELAY the logger until 1st sampling alarm is synchronized
// otherwise you might get a "clipped interval" at the first hour rollover
//========================================================================================
// ALSO saves logger startup time to 328p internal EEprom WHILE tethered to UART for power
// the saved startup time is used later when startMenu_sendData2Serial reconstructs TimeStamps during data download
//========================================================================================

RTC_DS3231_getTime();                     // populates the global variables t_day, t_hour, etc.

  Alarmday = t_day; Alarmhour = t_hour;
  //AlarmSelectBits = 0b00001000;               // A1 Alarm when hours, minutes and seconds match
  //AlarmSelectBits = 0b00000000;               // A1 Alarm when day of month, hours, minutes and seconds match

  if(SampleIntervalSeconds>0){    // NOTE: if the sample interval is in seconds this delay can be VERY short
      Alarmminute = t_minute;  Alarmsecond = t_second + 1;    
      do{ Alarmsecond = Alarmsecond + 1;
      }while(Alarmsecond % SampleIntervalSeconds); // all non-zero results considered true // forces alignment
        
      byteBuffer2 = Alarmsecond-t_second; byteBuffer1=0; //byteBuffer1 = Alarmminute - t_minute;
      if(Alarmsecond>59){
        Alarmsecond = 0; Alarmminute=Alarmminute+1; byteBuffer2 =60-t_second; byteBuffer1=0;}
      
      }else{ // for minute alarms

      Alarmminute = t_minute; Alarmsecond = 0; byteBuffer2 =0;  
      if(t_second>=58){Alarmminute = Alarmminute + 1;} // and extra buffer if we are too close to rollover
         
      do{ 
        Alarmminute = Alarmminute + 1;
      }while(Alarmminute % SampleIntervalMinutes); // all non-zero results considered true // forces alignment
      byteBuffer1 = Alarmminute-t_minute;
      }

 Serial.print(F("Start-up Sync Delay: "));
  if(SampleIntervalMinutes>0){Serial.print(byteBuffer1);Serial.print(F("min"));}
  if(SampleIntervalSeconds>0){Serial.print(byteBuffer2);Serial.print(F("sec"));}
  Serial.println();Serial.flush();
  
  if (Alarmminute > 59 ){ Alarmhour = Alarmhour+1; Alarmminute = 0;}  // alt Alarmminute = SampleIntervalMinutes? for longer delay if started at rollover
  if (Alarmhour > 23)   { Alarmday = Alarmday+1; Alarmhour = 0;}

  // NOW set the 1st -ALIGNED- wakeup alarm:
  AlarmSelectBits = 0b00001100;             // A1 Alarm when minutes AND seconds match, ignores days, hours
  RTC_DS3231_setA1Time(0, 0, Alarmminute, Alarmsecond, AlarmSelectBits, 0, 0, 0);
  RTC_DS3231_turnOnAlarm(1);                // alarm will break the logger out of flashing RED&BLUE light sync delay that follows
  noInterrupts ();                          // make sure we don't get interrupted before we sleep
  bitSet(EIFR,INTF0);                       // clear any previous flags for interrupt 0 (D2) see https://gammon.com.au/interrupts
  attachInterrupt(0,rtc_d2_alarm_ISR, LOW); // RTC SQW alarms LOW and is connected to pin D2 which is interupt channel 0
  rtc_INT0_Flag=false;                      // Flag gets set TRUE only inside rtc_d2_alarm_ISR ISR
  interrupts ();
//---------------------------------------------------------------------------------------------- 
// calculate and save UNIXtime for the wakup alarm we just set // same calculation used in function: RTC_DS3231_unixtime():
// we are doing it here because UART is still connected to supply power for the internal eeprom saving which draws alot of current (8mA for EEprom + 5mA for ProMini)
  uint16_Buffer = rtc_date2days(t_year, t_month, Alarmday); // the days calculation     // don't start your logger at the midnight rollover!
  uint32_Buffer = rtc_time2long(uint16_Buffer, Alarmhour, Alarmminute, Alarmsecond); // convert that to seconds
  uint32_Buffer += 946684800;               // this will be the unixtime when we wake AFTER the sync delay is over // add # seconds from 1970 to 2000 = delta between Unixtime start & our RTC's internal start time
  EEPROM.put(0,uint32_Buffer);              // store loggerStartTime so it can be used reconstructing each records timestamp in the startMenu_sendData2Serial() later during download

  #ifdef PIRtriggersSensorReadings
    previousPIRtriggerTime = uint32_Buffer;
  #endif

//------------------------------------------------------------------------------  
// Transfer BACKUP COPY of starting parameters in bytes 0-64 internal 328p eeprom into 0-64 bytes of external eeprom
// done in four steps because wire buffer can only transfer 16 bytes at a time

if(settingsChanged){ // any time an operating parameter is changed by one of the startup menu options backup all 64 bytes

  Wire.beginTransmission(EEpromI2Caddr);    // physicalEEpromAddr = block[0];   
  Wire.write(0); Wire.write(0);             // two bytes to specify the external eeprom address
  for (uint8_t p = 0; p < 16; p++) { byteBuffer1 = EEPROM.read(p); Wire.write(byteBuffer1);}
  Wire.endTransmission(); 
  delay(11); // AT24c32 Self-Timed Write Cycle (10 ms max)
  
  Wire.beginTransmission(EEpromI2Caddr);          
  Wire.write(0); Wire.write(16);       
  for (uint8_t q = 16; q < 32; q++) { byteBuffer1 = EEPROM.read(q); Wire.write(byteBuffer1);}
  Wire.endTransmission(); delay(11);

  Wire.beginTransmission(EEpromI2Caddr);          
  Wire.write(0); Wire.write(32);       
  for (uint8_t r = 32; r < 48; r++) { byteBuffer1 = EEPROM.read(r); Wire.write(byteBuffer1);}
  Wire.endTransmission(); delay(11);  

  Wire.beginTransmission(EEpromI2Caddr);          
  Wire.write(0); Wire.write(48);      
  for (uint8_t s = 48; s < 64; s++) { byteBuffer1 = EEPROM.read(s); Wire.write(byteBuffer1);}
  Wire.endTransmission(); delay(11); 

} else{   // ALWAYS copy the logger start time & sampling intervals (used for timestamp reconstruction)

  Wire.beginTransmission(EEpromI2Caddr);   // physicalEEpromAddr = block[0];   
  Wire.write(0); Wire.write(0);            // two bytes to specify the external eeprom address
  for (uint8_t p = 0; p < 6; p++) { byteBuffer1 = EEPROM.read(p); Wire.write(byteBuffer1);}
  Wire.endTransmission();
  
} //if(settingsChanged){ 

//------------------------------------------------------------------------------  
  Serial.println(F("B&G LEDs 'flicker' for SyncDelay til 1stRead, then GREEN pips @sampleTime")); 
  Serial.println(F("--- If you don't observe the 'flicker' then restart the logger again ---"));
  if(!ECHO_TO_SERIAL){          // if it's not being used, shut down the UART peripheral now to save power
    Serial.println(F("Disconnect UART now - NO additional messages will be sent over serial.")); Serial.flush();
        // power_usart0_disable();          // we waited until this point because the startup input menu requires serial input via the UART
        // NOTE: TOO many students were forgetting to wrap the Serial.print statements in the main loop with if(ECHO_TO_SERIAL){ } so we commented usart0_disable out
                                            // digital pins 0(RX) and 1(TX) are connected to the UART peripheral inside the 328p chip
                                            // Connecting anything to these pins may interfere with serial communication, including causing failed uploads
      }
   Serial.flush();
//------------------------------------------------------------------------------
// FIRST sampling wakeup alarm is already set But instead of simply going to sleep we will use LowPower.powerDown SLEEP_500MS
// to wake the logger once per second to toggle the LEDs ON/Off so user can tell we are in the sync-delay period before logging starts
// RED & BLUE leds start in opposite states so they ALTERNATE when toggled by the PIN register

  turnOnGreenLED(); 
  do{ toggleBlueAndGreenLEDs(); LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
      }while(!rtc_INT0_Flag);                 // sends you back to do{ unless the RTC alarm has triggered
  
  RTC_DS3231_ResetBothAlarmFlags();           // writes 0 to the alarm flags A1F and A2F in the STATUS_REG  but does not disable the next alarm       
  // Note: detachInterrupt(0); already done inside the rtc_d2_alarm_ISR 
  rtc_INT0_Flag=false; 
  turnOffAllindicatorLEDs();  //terminates synchronization time delay -we are now ready to start logging!

//terminates synchronization time delay -we are now ready to start logging!
//------------------------------------------------------------------------------------------------------------

  #ifndef logCurrentBattery_2byte          //readRailVoltage(); Must be at the end of setup because it disables Serial if ECHO is off
    LowestBattery = readRailVoltage();         //sets starting value for LowBat, but only needed if not logging CurrentBat
  #endif

  #ifdef logFreeVariableMemory_2byte              
    freeVariableMemory = freeRam();
  #endif

  Wire.setClock(400000L);  //I2C bus speed = 400 kHz
  // You can speed up the I2C bus speed to 400 kHz for RTC comms (and most sensors)
  // However you must slow the clock down to 100khz when working with the 4k eeprom
  
}     // terminator for void setup()
//==========================================================================================


//==========================================================================================
//==========================================================================================
//========================== START OF MAIN loop()===========================================
//==========================================================================================
//==========================================================================================
//================================================== START OF MAIN loop()===================
//==========================================================================================
//==========================================================================================

void loop(){

//----------------------------------------------------------------------------------
// HEARTBEAT LED pip by leaving LED on during sleep delays in next RTC Alarm setting
//----------------------------------------------------------------------------------
turnOffAllindicatorLEDs();

//------------------------------------------------------------------------------- 
//  *  *  *  *  Set the next RTC wakeup alarm  *  *  *  *  *  *
//-------------------------------------------------------------------------------    
  RTC_DS3231_getTime();                     // populates global t_minute,t_second variables

#ifdef PIRtriggersSensorReadings
  currentPIRtriggerTime = RTC_DS3231_unixtime();  // special case where we preload a variable for a delta calculation
  d3_INT1_elapsedSeconds = currentPIRtriggerTime - previousPIRtriggerTime;
  previousPIRtriggerTime = currentPIRtriggerTime;
#else
    
  if (SampleIntervalSeconds > 0){           // then our alarm is in (SampleInterval) seconds
      Alarmsecond = (t_second + SampleIntervalSeconds) %60;      // gives seconds from 0-59 sec e.g. 50s+15 = 65s  65%60 = 5s
                                                                 // if (Alarmsecond < SampleIntervalSeconds){Alarmsecond=0;} // would force alignment at first minute rollover
      Alarmminute = 0;
      AlarmSelectBits = 0b00001110;         // A1 Alarm when seconds match, ignores others
        
        //if (SampleIntervalSeconds == 1){  // A1 Alarm once per second = Special case
        //  Alarmminute = 0;
        //  AlarmSelectBits = 0b00001111;   // A1 Alarm once per second = Special case
        //}
      RTC_DS3231_setA1Time(0, 0, 0, Alarmsecond, AlarmSelectBits, 0, 0, 0);     
    } else {                                // use SampleIntervalMinutes
      Alarmsecond = 0;                      // forces matching on even min
      Alarmminute = (t_minute + SampleIntervalMinutes) % 60;    // gives from 0-59
                                            // if (Alarmminute< SampleIntervalMinutes){Alarmminute=0;} // would force alignment at first hour rollover
      AlarmSelectBits = 0b00001100;         // A1 Alarm when minutes and seconds match, ignore days, hours
      RTC_DS3231_setA1Time(0, 0, Alarmminute, Alarmsecond, AlarmSelectBits, 0, 0, 0);
    }

    turnOnGreenAndBlueLED();
      LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);  // RTC memory register WRITING time & battery recovery time
    turnOnGreenLED();
    
    // RTC_DS3231_turnOnAlarm(1); // sets both INTCN & A1IE bits of control register
    // however this only needs to be done once in setup and alarm 1 will always fire, so we don't need to call it here again!
    // All we need do in sleepNwait is clear the stat register A2F,A1F flags 

#endif //terminates #ifdef PIRtriggersSensorReadings    
 
  if(ECHO_TO_SERIAL){
      Serial.println();Serial.println();Serial.print(F(">Wake: ")); 
      Serial.print(t_year,DEC);Serial.print(F("/"));Serial.print(t_month,DEC);Serial.print(F("/"));Serial.print(t_day,DEC);
      Serial.print(F(" "));Serial.print(t_hour,DEC);Serial.print(F(":"));Serial.print(t_minute,DEC);Serial.print(F(":"));Serial.print(t_second,DEC);

        #ifndef PIRtriggersSensorReadings
          Serial.print(F(" Next Alarm in: "));
          if (SampleIntervalSeconds > 0){ Serial.print(SampleIntervalSeconds); Serial.print(F("sec")); 
          }else{ Serial.print(SampleIntervalMinutes);Serial.print(F("min")); }
        #endif //#ifndef PIRtriggersSensorReadings
      Serial.println();Serial.flush();
    }


#ifdef logRTC_Temperature_1byte
      rtc_TEMP_degC = RTC_DS3231_getTemp();               // moved this code into its own function
      if(ECHO_TO_SERIAL){
        Serial.print(F(", RTC temp[°C]:"));Serial.print(rtc_TEMP_degC,2);Serial.flush();
      } 
#endif

    LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
    turnOffAllindicatorLEDs();
//--------END of RTC activity-----------------

#ifdef logCurrentBattery_2byte                              // ADC reads use significant power - only read CurrentBat if saving the data
    CurrentBattery = readRailVoltage();                   // Note: a SLEEP_15MS is embedded in the readRailVoltage function, processor draws about 1mA in sleep mode ADC
    if(ECHO_TO_SERIAL){
        Serial.print(F(", Current Bat[mV]:"));Serial.print(CurrentBattery);Serial.flush();
        }
#endif

#ifdef logFreeVariableMemory_2byte
    freeVariableMemory = freeRam();                   
    if(ECHO_TO_SERIAL){
        Serial.print(F(", Variable Memory (bytes):"));Serial.print(freeVariableMemory);Serial.flush();
        }
#endif
  
#ifdef logLowestBattery_1byte
      if(ECHO_TO_SERIAL){ 
      Serial.print(F(", Lowest Bat[mV]:"));Serial.print(LowestBattery);Serial.flush();
      }
 #endif

  // if you are only logging LowestBattery, it might be useful to record the unloaded battery voltage once per day (?)
  //if(t_hour==0 && t_minute==0 && t_second==0){          // midnight reset prevents 'occasional' low readings from permanently resetting the lobat record
  //  LowestBattery = readRailVoltage();                      // no-load readRailVoltage() calls are usually 20-100mv higher than Lobat reads during high drain EEsave events
  //  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  //  }
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// STEP4 : READ your sensors here
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#if defined(readD6ResistorwD8pullup_2byte) || defined(readD7resistorwD8pullup_2byte)
//----------------------------------------------------------
  if(ECHO_TO_SERIAL){Serial.println();Serial.flush();}
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  ConditionCapacitorOnD8();                             // ConditionCapacitor only needs to be called ONCE before other ICU resistor readings
  uint32_Buffer = ReadD8riseTimeOnD8();                 // charge cycles the cap through D8 to standardize condition // AND it sets all pins with resistor/sensors connected to that common capacitor to input
#endif

#ifdef readD7resistorwD8pullup_2byte
//------------------
  D7resistor_NewReading = ReadD7riseTimeOnD8();                // a complex function at the end of this program
  D7resistor_NewReading = (referenceResistorValue * D7resistor_NewReading) / uint32_Buffer;
  if(ECHO_TO_SERIAL){
    Serial.print(F(",  D7r[Ω]:"));Serial.print(D7resistor_NewReading);Serial.flush();
    }  
#endif

#ifdef readD6ResistorwD8pullup_2byte   
//-------------------
  D6resistor_NewReading = ReadD6riseTimeOnD8();
  D6resistor_NewReading = (referenceResistorValue * D6resistor_NewReading) / uint32_Buffer;
      if(ECHO_TO_SERIAL){
        Serial.print(F(" D6r[Ω]:"));Serial.println(D6resistor_NewReading);Serial.flush();
      }  
#endif

#ifdef readBh1750_LUX_2byte
//-------------------
  bh1750.start(BH1750_QUALITY_LOW, BH1750_MTREG_LOW);   // triggers a new sensor reading
                                                        // LOW MTreg:31  resolution lux:7.4, 121557 is highest lux
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);      // L-Resolution Mode Measurement Time 16-24 msec                   
  //LowPower.powerDown(SLEEP_30MS, ADC_ON, BOD_OFF);    // H-Resolution Mode Measurement Time is much longer: 120-180 ms

  lux_BH1750_RawInt =bh1750.getRaw();                   // reading can reach 120,000
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);      // battery recovery after I2C - not really needed with this low current sensor
    
  if(ECHO_TO_SERIAL){  
      Serial.println();Serial.print(F(", Bh1750(rawInt): "));Serial.print(lux_BH1750_RawInt);
      Serial.print(F(", Lux(calc): "));Serial.print(bh1750.calcLux(lux_BH1750_RawInt,BH1750_QUALITY_LOW,BH1750_MTREG_LOW),2);Serial.flush();
      //if you call calcLux() without Quality & MTreg, the parameters from the last measurement are used for the calculation
    }
#endif //readBh1750_LUX_2byte


//read Sht3x RH sensor
#if defined(readSht3x_Temp_2byte) || defined(readSht3x_Humidity_2byte)
//---------------------------------------------------------------
  sht3x.requestData(); //defaults() to fast 4msec read
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF); //15.5ms listed max in datasheet
  //do{ booleanBuffer = sht3x.dataReady(); }while(!booleanBuffer);
  //dataReady check uses excessive amount of power
  sht3x.readData();
#endif
  
#ifdef readSht3x_Humidity_2byte
  Sht3x_RH_percent = sht3x.getHumidity();
  //OR: Sht3x_RH_RawInt= getRawHumidity();  //raw two-byte representation of humidity directly from the sensor
  //Sht3x_RH_percent = Sht3x_RH_RawInt * (100.0 / 65535);
  if(ECHO_TO_SERIAL){
    Serial.print(F(", Sht3x RH: ")); Serial.print(Sht3x_RH_percent,2); Serial.print(F(" %")); }
#endif
  
#ifdef readSht3x_Temp_2byte 
  Sht3x_Temp_degC = sht3x.getTemperature();
  //OR: Sht3x_Temp_RawInt = getRawTemperature(); //raw two-byte representation of temperature directly from the sensor
  //Sht3x_Temp_degC = Sht3x_Temp_RawInt * (175.0 / 65535) - 45;
  if(ECHO_TO_SERIAL){
    Serial.print(F(", Sht3x Temp: ")); Serial.print(Sht3x_Temp_degC,2); Serial.print(F(" °C")); }
#endif

#if defined(readSht3x_Temp_2byte) || defined(readSht3x_Humidity_2byte)
    if(ECHO_TO_SERIAL){Serial.println();Serial.flush();}
#endif


// read bmp280 sensor
//-------------------
#if defined(readBMP280_Temp_2byte) || defined(readBMP280_Pressure_2byte) || defined(recordBMP280_Altitude_2byte) //  '||' means 'OR'
  bmp280.startForcedConversion(); 
  LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF); //NOTE: sleep time needed here depends on your oversampling settings
  if(ECHO_TO_SERIAL){ Serial.println();}
#endif

#ifdef readBMP280_Temp_2byte
  bmp280.getCurrentTemperature(Bmp280_Temp_degC);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  if(ECHO_TO_SERIAL){ Serial.print(F(", b280 Temp: ")); Serial.print(Bmp280_Temp_degC,2); Serial.print(F(" °C")); }
#endif

#ifdef readBMP280_Pressure_2byte
  bmp280.getCurrentPressure(Bmp280_Pr_mBar);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  if(ECHO_TO_SERIAL){ Serial.print(F(", b280 Pr. "));Serial.print(Bmp280_Pr_mBar,2); Serial.print(F(" hPa"));}
#endif

#ifdef recordBMP280_Altitude_2byte
  bmp280.getCurrentAltitude(Bmp280_altitude_m);
    if(ECHO_TO_SERIAL){ Serial.print(F(", b280 Alt. ")); Serial.print(Bmp280_altitude_m,2); Serial.print(F(" m,")); }
#endif
// to read all three at the same time: bmp280.getCurrentMeasurements(Bmp280_Temp_degC, Bmp280_Pr_mBar, Bmp280_altitude_m); //function returns 1 if readings OK

#if defined(readBMP280_Temp_2byte) || defined(readBMP280_Pressure_2byte) || defined(recordBMP280_Altitude_2byte) //  '||' means 'OR'
  if(ECHO_TO_SERIAL){ Serial.flush();}
#endif

// read BME280 sensor
//-------------------
#if defined(recordBMEtemp_2byteInt) || defined(recordBMEpressure_2byteInt) || defined(recordBMEhumidity_2byteInt)
   climateSensor.takeForcedMeasurement();
   LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);//At lowest resoluton this should be plenty of time 
   #if defined(recordBMEtemp_2byteInt)
    g_temperature = climateSensor.getTemperatureCelsius();
    if(ECHO_TO_SERIAL){Serial.print(F(" BmeT: "));Serial.print(g_temperature/100);Serial.print(F("."));Serial.print(g_temperature%100);Serial.print(F("°C"));}
   #endif
   #if defined(recordBMEpressure_2byteInt)
    g_pressure = climateSensor.getPressure();
    if(ECHO_TO_SERIAL){Serial.print(F(" BmePr: "));Serial.print(g_pressure/100);Serial.print(F("."));Serial.print(g_pressure%100);Serial.print(F("hPa"));}
   #endif
   #if defined(recordBMEhumidity_2byteInt)
    g_humidity = climateSensor.getRelativeHumidity();
    if(ECHO_TO_SERIAL){Serial.print(F(" BmeRh: "));Serial.print(g_humidity/100);Serial.print(F("."));Serial.print(g_humidity%100);Serial.print(F("%"));}
   #endif
    if(ECHO_TO_SERIAL){Serial.println();Serial.flush();}
#endif

#ifdef readSi7051_Temp_2byte
//------------------------------------------------------------------------------
    TEMP_si7051 = readSI7051();                       // see functions at end of this program
        if(ECHO_TO_SERIAL){ 
          Serial.print(F(", SI7051 temp: "));Serial.print(((175.26*TEMP_si7051)/65536.0)-46.85 ,3);Serial.flush();//print 3 decimals
          }
    LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);  //  battery recovery time
#endif

//-----------------------------------------------------
// Sensor readings finished: turn off the indicator LED 
//-----------------------------------------------------
turnOffAllindicatorLEDs();
    
//---------------------------------------------------------------------------------
// Setup ADC to read the rail voltage DURING the EEprom data save:
//---------------------------------------------------------------------------------

    //bitSet(ACSR,ACD);  //disable analog comparator already done in setup
    SPCR = 0; ADCSRA =0; power_all_disable();  
    power_timer0_enable(); 
    power_twi_enable(); 
    power_adc_enable();
    ADMUX = set_ADMUX_2readRailVoltage; ADCSRA = set_ADCSRA_2readRailVoltage;
//bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,1);bitWrite(ADCSRA,ADPS0,0);   // 64 (default) prescalar @ 8MHz/64 = 125 kHz, =~104uS/ADC reading
    bitSet(ADCSRA,ADSC);                                                          // triggers a 1st throw-away ADC reading to engauge the Aref cap //1st read takes 20 ADC clock cycles instead of usual 13  
    bitClear(DDRB,5);bitSet(PORTB,5);  // pip the RED d13 LED with INPUT_PULLUP to indicate EEprom save event
    LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); // NOTE: Aref cap settling only needs ~5 milliseconds
    bitClear(PORTB,5);                // D13 red LED: pullup OFF

  // NOTE: Aref capacitor Rise time can take 5-10 milliseconds after starting ADC so 15ms of ADC_ON powerDown sleep works 

//---------------------------------------------------------------------------------
//  SAVE NEW SENSOR READINGS into EEprom & READ battery after
//---------------------------------------------------------------------------------
// the number of bytes you transfer here must match the number in sensorBytesPerRecord
// AND bytes written to per cycle MUST divide evenly into the eeproms pagesize
// So each cycle can only add 1,2,4,8 or 16 bytes - not 3 , not 5, not 7 etc. 
// or you bork the 'powers of 2' math required by page boundaries in the EEprom
//---------------------------------------------------------------------------------
//  the general pattern when sending bytes to store in an I2C eeprom:

//  Wire.beginTransmission(EEpromAddressonI2Cbus);  // first byte in I2C buffer
//  Wire.write(highByte(memoryAddress));            // MSB is second byte
//  Wire.write(lowByte(memoryAddress));             // LSB is third byte
//    Wire.write(byte);                             // adds 1st byte of SAVED data to buffer
//    Wire.write(byte);                             // adds 2nd byte of SAVED data to buffer
//   -more wire.writes-                             // CONTINUE adding up to 16 DATA bytes per record - divide the sensor variables up into individual bytes for sending
//  Wire.endTransmission();                         // Only when this command executes does the I2C memory buffer get sent over the I2C bus

  // beginTransmission() and write() are slightly misleading terms, they do NOT send commands/packets to the I2C device. 
  // They are simply queuing commands, which means they are adding bytes to an internal buffer in the Wire/TWI library. 
  // This internal buffer is not sent to the I2C device on the bus until end.Transmission() is called 
  // estimate about 100us per byte at 100khz bus = 0.7milliseconds for 3(adr)+4(payload) bytes
//---------------------------------------------------------------------------------

  if(totalBytesOfStorage==4096){
  Wire.setClock(100000UL); // Set I2C bus speed to DEFAULT 100 kHz
  //when working with the 4K eeprom on the RTC module
  //NOTE: larger eeproms and most other I2c devices work just fine at 400khz
  //but if your wires get too long you may have to slow the bus
  //for them too.
  }
  
//---------------------------------------------------------------------------------
  Wire.beginTransmission(EEpromI2Caddr);            // STARTS filling the I2C transmission buffer with the eeprom I2C bus address
  Wire.write(highByte(EEmemPointer));             // send the HighByte of the EEprom memory location we want to write to
  Wire.write(lowByte(EEmemPointer));              // send the LowByte of the EEprom memory location   // Note: we add  'bytes per record' to EEmemPointer at the end of the main loop  
//---------------------------------------------------------------------------------
// NOW load the sensor variables - one byte at a time - into the I2C transmission buffer with Wire.write statements
// ORDER of loading here MUST EXACTLY MATCH the order of the bytes retrieved in the startMenu_sendData2Serial function
// ALSO NOTE we are using ZEROs as our END OF FILE marker to stop the download ( this is why we filled the eeprom with zeros at startup)
// So we must implement a 'zero trap' but ONLY ON THE FIRST byte of each record, bumping it to '1' if it actualy was zero - this occasionally causes a data error

#ifdef PIRtriggersSensorReadings  
  //how to slice a 4-byte LONG uint32_t into individual bytes:  & 0b11111111   extracts only the  lowest eight bits
  d3_INT1_elapsedSeconds++;   //++ as a zero-trap for first byte of the record
  
  loByte = d3_INT1_elapsedSeconds & 0b11111111; 
    Wire.write(loByte);       // byte 1 (the lowest byte)
  byteBuffer1 = (d3_INT1_elapsedSeconds >> 8) & 0b11111111;
    Wire.write(byteBuffer1);  // byte 3
  byteBuffer2 = (d3_INT1_elapsedSeconds >> 16) & 0b11111111;
    Wire.write(byteBuffer2);  // byte 2
  hiByte = (d3_INT1_elapsedSeconds >> 24) & 0b11111111;
    Wire.write(hiByte);       // byte 4 (the highest byte)
#endif

#ifdef logLowestBattery_1byte                           // INDEX compression converts lowBattery reading to # less than 255 which can be stored in one byte eeprom memory location
//-------------------------------------------------------------------------------------------------------
  int16_Buffer = (LowestBattery-1700)/16;        // index compression looses some data due to rounding error  - so record is reduced to only 16mv/bit resolution
  byteBuffer1 = lowByte(int16_Buffer);
  if(byteBuffer1<1){byteBuffer1=1;}               // ONLY THE FIRST data byte in each record must have a ZERO TRAP to preserve End Of Data indicator in EEprom
        Wire.write(byteBuffer1);                  // write that single compressed byte to eeprom (we will have to expand it back later !) 
#endif //logLowestBattery_1byte

#ifdef logRTC_Temperature_1byte                         // NOTE: this 1-byte COMPRESSED encoding limits our temperature range to 0-63 degrees 
//--------------------------------------------------------------------------------------------------------
  floatBuffer = (rtc_TEMP_degC + 10.0)*4;         // *4 converts the RTC temperature into a small integer (63*4= 252 - within the range of one byte!)
  int16_Buffer = (int)(floatBuffer);             // adding 10 shifts the 63 degree range, so we can record -10C to +53C  // there is no loss of information because the resolution is only 0.25C 
       
  if(int16_Buffer>255){int16_Buffer=255;}       // temps above 53 C get clipped because a byte cant represent them
  if(int16_Buffer<1){int16_Buffer=1;}           // Zero Trap to preserve End of Data check, this sets lower cutoff temp to -9.75C (but our coincell is dead before that)

  byteBuffer1 = lowByte(int16_Buffer);
  //if(byteBuffer1<1){byteBuffer1=1;}             // ONLY THE FIRST data byte saved in each record must have a ZERO TRAP to preserve zero EOF indicators in EEprom
        Wire.write(byteBuffer1); 
#endif //logRTC_Temperature_1byte

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// STEP5 : add more sensor data bytes to the I2C buffer HERE as required ++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Note: better overflow catching could be added here - but leaving as a student exercise

#ifdef countPIReventsPerSampleInterval
  d3_INT1_eventCounter = d3_INT1_eventCounter+1;       // we 'add one' so that the 1st stored byte is never zero - even when the count actually is zero- this is our zero trap
  loByte = lowByte(d3_INT1_eventCounter);
    Wire.write(loByte);
  hiByte = highByte(d3_INT1_eventCounter);
    Wire.write(hiByte);
  d3_INT1_eventCounter = 0;                            // after saving the data we can reset our event counter to zero
#endif

#ifdef readD7resistorwD8pullup_2byte
//---------------------------------
// D7resistor_NewReading changed to 4-byte uint32 to avoid overflows in intermediate calculations
// but we only record the lowest two of those bytes 
// this requires bitmath extraction of high & low bytes for eeprom storage
  uint32_Buffer = D7resistor_NewReading; 
    if (uint32_Buffer>65534){uint32_Buffer=65534;}   // Catch uint16_t Overflow
  byteBuffer1   = uint32_Buffer & 0b11111111; // lobyte
  if(byteBuffer1<1){byteBuffer1=1;}           // -ONLY 1st byte of record -needs to preserve zero EOF indicator in 'empty' EEprom space
    Wire.write(byteBuffer1);
  
  uint32_Buffer = uint32_Buffer>>8;
  byteBuffer2   = uint32_Buffer & 0b11111111; // highByte
    Wire.write(byteBuffer2);
#endif // #ifdef readNTC_D6refD7ntc

#ifdef readD6ResistorwD8pullup_2byte
//------------------------------
// D6resistor_NewReading changed to 4-byte uint32 to avoid calculation overflows
// this requires bitmath extraction of high & low bytes for eeprom storage
  uint32_Buffer = D6resistor_NewReading; 
    if (uint32_Buffer>65534){uint32_Buffer=65534;}   // Catch uint16_t Overflow
  byteBuffer1   = uint32_Buffer & 0b11111111; //lobyte
  if(byteBuffer1<1){byteBuffer1=1;}           // -ONLY 1st byte of record -needs to preserve zero EOF indicator in 'empty' EEprom space
    Wire.write(byteBuffer1);
  
  uint32_Buffer = uint32_Buffer>>8;
  byteBuffer2   = uint32_Buffer & 0b11111111; //highByte
    Wire.write(byteBuffer2);
#endif // #ifdef readD6ResistorwD8pullup_2byte

#ifdef readBh1750_LUX_2byte
//--------------------
  loByte = lowByte(lux_BH1750_RawInt);          // first byte of record gets checked by 
  //if(loByte<1){loByte=1;}                     // -ONLY 1st byte of record -needs to preserve zero EOF indicator in 'empty' EEprom space
        Wire.write(loByte);
  hiByte = highByte(lux_BH1750_RawInt);
        Wire.write(hiByte);  
#endif // #ifdef readBh1750_LUX_2byte

#ifdef readSht3x_Humidity_2byte
  //Sht30 resolution is only: 0.01°C and 0.01 %RH so only need two decimal places
  Sht3x_RH_percent = Sht3x_RH_percent*100.00;   // convert float reading to integer preserving two decimal places
  int16_Buffer = (int16_t)Sht3x_RH_percent;  
  loByte = lowByte(int16_Buffer);               // first byte of record gets checked by 
  //if(loByte<1){loByte=1;}                     // if needed to preserve zero EOF indicator in 'empty' EEprom space if single sensor
        Wire.write(loByte);
  hiByte = highByte(int16_Buffer);
        Wire.write(hiByte);  
#endif

#ifdef readSht3x_Temp_2byte                    // Sht30 resolution is only: 0.01°C and 0.01%RH
  // SHIFT the temps +10C so that we can reconstruct with 65535 range of uint16_t
  // giving us a range of -10C to +55.534C before calc fails
  Sht3x_Temp_degC = Sht3x_Temp_degC + 10.0;
  Sht3x_Temp_degC = Sht3x_Temp_degC*100.00;     // converts float to integer preserving two decimal places
  uint16_Buffer = (uint16_t)Sht3x_Temp_degC;    // Resolution is only 0.01 °C, so only preserve two decimal places
  loByte = lowByte(uint16_Buffer);              // first byte of record gets checked by 
  //if(loByte<1){loByte=1;}                     // to preserve zero EOF indicator in 'empty' EEprom space if single sensor
        Wire.write(loByte);
  hiByte = highByte(uint16_Buffer);
        Wire.write(hiByte); 
#endif

#ifdef readBMP280_Temp_2byte
  Bmp280_Temp_degC = Bmp280_Temp_degC*100.00;   //convert float reading to integer preserving two decimal places
  int16_Buffer = (int16_t)Bmp280_Temp_degC;  
  loByte = lowByte(int16_Buffer);               // first byte of record gets checked by 
  //if(loByte<1){loByte=1;}                     // to preserve zero EOF indicator in 'empty' EEprom space
        Wire.write(loByte);
  hiByte = highByte(int16_Buffer);
        Wire.write(hiByte);  
#endif

#ifdef readBMP280_Pressure_2byte
  Bmp280_Pr_mBar = Bmp280_Pr_mBar*10.0;         //convert float reading to integer preserving ONE decimal place
  int16_Buffer = (int16_t)Bmp280_Pr_mBar;  
  loByte = lowByte(int16_Buffer);              // first byte of record gets checked by 
  //if(loByte<1){loByte=1;}                     // to preserve zero EOF indicator in 'empty' EEprom space
        Wire.write(loByte);
  hiByte = highByte(int16_Buffer);
        Wire.write(hiByte);     
#endif

#ifdef recordBMP280_Altitude_2byte
  Bmp280_altitude_m = Bmp280_altitude_m*100.00;     //convert float reading to integer preserving two decimal places
  int16_Buffer = (int16_t)Bmp280_altitude_m;   // Note: *100 overuns the int at higher altitudes - switch to *10
  loByte = lowByte(int16_Buffer);              // first byte of record gets checked by 
  //if(loByte<1){loByte=1;}                     // to preserve zero EOF indicator in 'empty' EEprom space
        Wire.write(loByte);
  hiByte = highByte(int16_Buffer);
        Wire.write(hiByte);  
#endif

#ifdef recordBMEtemp_2byteInt       // stored in int32_t g_temperature with 2 decimals 'embedded' in the integer
//---------------------------
  int16_Buffer = g_temperature;     // room in int16 to ±32,000 so no danger of overflow
  loByte = lowByte(int16_Buffer); 
  if(loByte<1){loByte=1;} // low byte first for zero trapping on sensor readings - this can introduce error on lowest bit
      Wire.write(loByte);
  hiByte = highByte(int16_Buffer);
      Wire.write(hiByte);
#endif

#ifdef recordBMEpressure_2byteInt       // stored in int32_t g_pressure with 2 decimals 'embedded' in the integer
//-------------------------------
  uint16_Buffer = g_pressure - 80000;  // -80000 for SCALING to fit into uint16 with two preserved decimal places
  loByte = lowByte(uint16_Buffer); 
  if(loByte<1){loByte=1;} // low byte first for zero trapping on sensor readings - this can introduce error on lowest bit
      Wire.write(loByte);
  hiByte = highByte(uint16_Buffer);
      Wire.write(hiByte);
#endif

#ifdef recordBMEhumidity_2byteInt       // stored in int32_t g_humidity with 3 decimals 'embedded' in the integer
//-------------------------------
  uint16_Buffer = g_humidity;
  loByte = lowByte(uint16_Buffer); 
  if(loByte<1){loByte=1;} // low byte first for zero trapping on sensor readings - this can introduce error on lowest bit
      Wire.write(loByte);
  hiByte = highByte(uint16_Buffer);
      Wire.write(hiByte);
#endif

#ifdef logCurrentBattery_2byte                      // stores the 'raw' 16-byte integer using two bytes (ie with no compression)
//-----------------------------------------------------------------------------------------------------------------------
  loByte = lowByte(CurrentBattery);           // note: we save the low byte first because it is almost never zero
        Wire.write(loByte);                   // first byte added to I2C buffer
  //if(loByte<1){loByte=1;}                   // ONLY THE FIRST data byte saved in each record must have a ZERO TRAP to preserve zero EOF indicators in EEprom
  hiByte = highByte(CurrentBattery);
        Wire.write(hiByte);                   // 2nd byte of data added to I2C buffer 
#endif //logCurrentBattery_2byte

#ifdef logFreeVariableMemory_2byte             // stores the 'raw' 16-byte integer using two bytes (ie with no compression)
//-----------------------------------------------------------------------------------------------------------------------
  loByte = lowByte(freeVariableMemory);
        Wire.write(loByte);              
  hiByte = highByte(freeVariableMemory);
        Wire.write(hiByte);  
#endif //logFreeVariableMemory_2byte

#ifdef readSi7051_Temp_2byte                         // stores the 'raw' 16-byte integer using two bytes (ie with no compression)
//-----------------------------------------------------------------------------------------------------------------------
  loByte = lowByte(TEMP_si7051);              //NOTE TEMP_si7051 overruns this uint16_t if temps >40C!
        Wire.write(loByte);
  //if(loByte<1){loByte=1;}                   // ONLY THE FIRST data byte saved in each record must have a ZERO TRAP to preserve zero EOF indicators in EEprom
  hiByte =  highByte(TEMP_si7051); 
        Wire.write(hiByte);
#endif //readSi7051_Temp_2byte

//-------------------------------------------------------------------------------
    bitSet(ADCSRA,ADSC); //trigger the next ADC during the I2C send takes .104 msec
    Wire.endTransmission(); // ONLY AT THIS POINT do the bytes accumulated in the buffer actually get sent
    // Wire.endTransmission(); is blocking so the next ADC read happens AFTER the send
//-------------------------------------------------------------------------------
// The EEPROM enters an internally-timed write cycle to memory which takes ~3-10ms [longer for larger eeproms because of page size]
// 4k AT24c32 write draws ~10mA for about 10ms @3mA, but newer eeproms can take only only 5ms @3mA
// the coincell battery experiences a SIGNIFICANT VOLTAGE DROP due to its internal resistance during this load
//-------------------------------------------------------------------------------

  // ignore the first ADC reading & take another:
  bitSet(ADCSRA,ADSC); while(bit_is_set(ADCSRA,ADSC)); uint16_Buffer=ADC;
  ADCSRA = 0; power_adc_disable();                      // turn off ADC

// -------------------------------------------------------------------------------
// CRITICAL: EEproms are sensitive to voltage fluctuations during the save and will HANG if you do too much
// WARNING: wrt EEprom Polling (which works!): any I2C exchange draws 3.5mA because cpu is active while EEprom write usually draws 1-1.5mA
// CRITICAL understanding: DO NOT TURN OFF THE I2C bus (or Timer0?) too soon after sending data to EEPROM
// I'm not sure why this is but some EEproms hang if you powerdown right after endTransmission(?)
// Idle mode stops the CPU but keeps peripherals like timers and serial communication active.
// -------------------------------------------------------------------------------

if(totalBytesOfStorage>4096){
    noInterrupts();TCNT0=0;interrupts(); // reset timer0 for 1st ovrflow to wake the processor in 2.048 ms [at 8mhz]
      // NOTE: could shorten overflow time to 1msec by preloading TCNT0 = 128;
      // 8,000,000 Hz / 64 = 125,000 Hz // 1 / 125,000 Hz = 0.000008 seconds = 8 µs/tick, // 8 µs/tick * 256 ticks = 2048 µs = 2.048 ms
      // NOTE: 2msec aligns well on scope with transition from erase to write. Also generally a bad idea to mess with Timer0 too much.

    LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_ON);
      // SLEEP_FOREVER works, but WTD on gives us a safety wakeup source if something else locks the cpu?)
      // 1mA IDLE + 1.4mA EEsave current is ~2.5mA - less than full 3.5mA during the I2C data send     
      // _FOREVER works, but leaving WTD on gives us a safety wakeup source & WTD gets reset in the NEXT sleep
    }  
      
    bitClear(DDRB,5);bitSet(PORTB,5);  // pip the RED d13 LED with INPUT_PULLUP to indicate EEprom save event
    LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF); // Cr2032 Battery recovery time: & larger eeproms seem to need some recovery time or RTC alarm wont set
    //Waking the 328p from powerdown takes 16,000 clock cycles (~2milliseconds @8MHz +60µS if BOD_OFF) and the ProMini draws ~250µA while waiting for the oscillator to stabilize.
    bitClear(PORTB,5);                // D13 red LED: pullup OFF
  uint16_Buffer=uint16_Buffer+1;// compensates for very short drop spikes observed on 'scope (with 200uF rail caps)
#ifdef logLowestBattery_1byte 
  LowestBattery = InternalReferenceConstant / uint16_Buffer;
  //if logging Lowest (say for NTC calibrations) let low battery get over-written every time it is generated
#else   // we are logging the lowbat only ONCE PER DAY
  CurrentBattery = InternalReferenceConstant / uint16_Buffer;
  if(t_hour==0 && t_minute==0 && t_second==0){  LowestBattery = 5764; }  // midnight reset to high value prevents 'occasional' low battery readings from permanently affecting the battery record                              
  if(CurrentBattery < LowestBattery){ LowestBattery = CurrentBattery;} 
#endif

if(ECHO_TO_SERIAL){
      power_usart0_enable();  // enabled to send text to serial window
      } 

  if(totalBytesOfStorage==4096){
  Wire.setClock(400000UL); // Set I2C bus speed to 400 kHz
  //now that we are finished working with the 4K eeprom on the RTC module
  //you can speed the bus to 400khz for almost all other I2c devices
  }
  
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// OLED DISPLAY of DATA    [ AFTER alarm set & Data save ]
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // NOTE: 64x32 screen displays only the CENTER pixels of the1306 controllers  128x64 pixel memory!
    // so first collumn/ pixel on micro oled screen is 32 collums from LEFT of memory space
    // when using the 128x64 driver the first row starts at (32,4), and entire screen occupies 8pixel heigh rows 4-5-6-7
    // 5x7 system font can display 10 characters accross the screen
//------------------------------------------------------------------------------
    // The SSD1306 controller operates at 1.65V to 3.3V but we may need a beefier capacitor for the 
    // OLEDs 1.5mA current draw while cpu is also running?
    // screen off: logger goes to about 4.5uA to sleep, 
    // display on with minimal text: about 900uA (quite jumpy - may need 1000uF cap?)
    // I did notice that with the larger 0.96" screens as well - voltage jumpy till 1000uF rail cap?
    // NOT worth reducing contrast with this small text! Its already hard to see?
    // START a run-duration test with one of these OLED loggers [and repeat test with larger 1kuF capacitor] 
//-------------------------------------------------------------------------------
  // oled.setInvertMode(1); // Toggle invert mode for next line of text.
  // oled.setLetterSpacing(uint8_t pixels) { m_letterSpacing = pixels; } //pixels letter-spacing in pixels before magnification, setFont() will restore default letter-spacing.
  // oled.skipColumns(uint8_t n) { m_skip = n; } //Skip leading pixels writing characters to display / display RAM
  // oled.clearField(col[i%2], rows*(i/2), 4);

#ifdef OLED_64x32_SSD1306   // no need to print the same thing overagain!
                            // the less you print the faster the I2C transaction goes at the start - big power lump
                            // I2C is when both logger & screen are on  - biggest power use
                            // a battery reading is needed during this operatino as it's a huge hit on the coincell??

//noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_8;interrupts(); //Current draw at 3.3V for the '328P is approximately 1mA for 1 MHz & 3 mA for 8 MHz
// Waking from powerdown now takes 16 milliseconds

// MUST SET the RTC alarm BEFORE adding display screen-time
// AND make sure you don't over-run your alarm interval!

  //You can 'program' the OLED controller with the screen pixels turned off!
  //oled.clear();  //better to use oled.clearField(col[i%2], rows*(i/2), 4); //clear screen blanks entire memory pixel by pixel
  oled.set1X(); // standard font size. //this is clearing the entire memory - can we just clear the display space?
  //oled.skipColumns(32);? to bump us past the first 32 hidden collumns?

  #ifdef logRTC_Temperature_1byte
  //oled.clearField(32,4,10);
  oled.setCursor(32,4); //oled.setCursor(Column, Row) -row is from the upper left corner
  oled.print(F("RTC    temp")); //5x7 system font can display 10 characters accross the screen
  LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON); //battery recovery after each oled.print statement
  oled.setCursor(32,5); //64x32 screen displays only the CENTER pixels of the 128x64 pixel wide memory!
  // so first collumn/ pixel on micro oled screen from LEFT is at 32 across
  // when using the 128x64 driver the first row starts at (32,4), and entire screen occupies 8pixel heigh rows 4-5-6-7
  oled.print(F("----------")); //5x7 system font can display 10 characters accross the screen
  LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON);
  oled.set2X();  //2x  5x7 system font can display 5 characters accross the screen
  //oled.clearField(32,6,6);
  oled.setCursor(32,6); //set2X can only start at rows 4 or 5 or 6 (because they take two horizontal rows to display)
  oled.print(rtc_TEMP_degC,2);
  oled.print(F(" ")); //blank spaces to clearing the rest of the row
  LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON);
  #endif

  #ifdef BMP280_Address
  oled.setCursor(32,4); //oled.setCursor(Column, Row) -row is from the upper left corner
  oled.print(F("Pr   mBar")); //5x7 system font can display 10 characters accross the screen
  oled.setCursor(32,5); //64x32 screen displays only the CENTER pixels of the 128x64 pixel wide memory!
  // so first collumn/ pixel on micro oled screen from LEFT is at 32 across
  // when using the 128x64 driver the first row starts at (32,4), and entire screen occupies 8pixel heigh rows 4-5-6-7
  oled.print(F("----------")); //5x7 system font can display 10 characters accross the screen
  oled.set2X();  //2x  5x7 system font can display 5 characters accross the screen
  oled.setCursor(32,6); //set2X can only start at rows 4 or 5 or 6 (because they take two horizontal rows to display)
  oled.print(bmp280_pressure,1);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); //the ProMini does not need to stay awake for the screen display time
  oled.clear();oled.set1X(); oled.setCursor(32,4); oled.print(F("Temp   Cel"));
  oled.setCursor(32,5);oled.print(F("----------")); //5x7 font =10 characters accross the screen
  oled.set2X();oled.setCursor(32,6);oled.print(bmp280_temp,2);
  #endif //BMP280_Address

  #ifdef readD7resistorwD8pullup_2byte
  oled.setCursor(32,4);oled.print(F("D7r"));
  //oled.setCursor(56,5);oled.print(D7resistor_NewReading);
  
  oled.setCursor(32,5); //64x32 screen displays only the CENTER pixels of the 128x64 pixel wide memory!
  // so first collumn/ pixel on micro oled screen from LEFT is at 32 across
  // when using the 128x64 driver the first row starts at (32,4), and entire screen occupies 8pixel heigh rows 4-5-6-7
  oled.print(F("----------")); //5x7 system font can display 10 characters accross the screen
  oled.set2X();  //2x  5x7 system font can display 5 characters accross the screen
  oled.setCursor(32,6); //set2X can only start at rows 4 or 5 or 6 (because they take two horizontal rows to display)
  oled.print(D7resistor_NewReading);
  #endif //ReadNTC
  
  #ifdef readD6ResistorwD8pullup_2byte
  oled.setCursor(32,6);oled.print(F("D6r"));
  oled.setCursor(56,7);oled.print(D6resistor_NewReading);
  #endif //ReadLDR

// NOTE OLED is a high drain device with >50msec sustained loads
// so I have wrapped it with a battery level check
  power_adc_enable();   // Aref rise takes ~1msec, while it takes 5msec or more for Aref to fall.
  ADMUX = set_ADMUX_2readRailVoltage; ADCSRA = set_ADCSRA_2readRailVoltage; //sets 2x normal ADC speed
  bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,1);bitWrite(ADCSRA,ADPS0,0); // 64 (default) prescalar
  bitSet(ADCSRA,ADSC); //while(bit_is_set(ADCSRA,ADSC));// trigger 1st throw-away ADC reading to engauge the Aref capacitor
  //high drain event here // must take more than 3msec for aref cap to stabilize
                              
  oled.ssd1306WriteCmd(SSD1306_DISPLAYON);  // To switch display PIXELS ON // a BIG load on the coincell - 100-200mv drop!

  bitSet(ADCSRA,ADSC);while(bit_is_set(ADCSRA,ADSC)); //throw away read
  bitSet(ADCSRA,ADSC);while(bit_is_set(ADCSRA,ADSC)); //read ADC again
  uint16_Buffer = ADC;  ADCSRA = 0;   power_adc_disable();  // turn off ADC after reading
  uint16_Buffer = InternalReferenceConstant / uint16_Buffer; // convert average ADC reading into railvoltage
  if (uint16_Buffer < LowestBattery) {LowestBattery = uint16_Buffer;}
   
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF); // To switch display OFF

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif //end of     OLED display commands
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

  if (LowestBattery <= systemShutdownVoltage){
      error_shutdown(); // shutdown down the logger
      } 
  
  EEmemPointer = EEmemPointer + sensorBytesPerRecord;     //advances our memory pointer for the next loop
  if( EEmemPointer >= totalBytesOfStorage){              // if eeprom memory is full
      error_shutdown();                                 // shutdown down the logger
      }

#ifdef countPIReventsPerSampleInterval                            //Logger can be woken by D2 AND D3 interrupt events
    rtc_INT0_Flag = false;                              //clear the flag we use to indicate the RTC alarm occurred
    do{                                                 //this do-while locks the processor into a 'counting loop' until the RTC alarm happens.
        sleepNwait4D3InterruptORrtcAlarm();             //counter gets incremented in sleepNwait4D3Interrupt() function //Logger can be woken by D2 AND D3 interrupt events
        if(ECHO_TO_SERIAL){ Serial.print(F("PIR wakeup #")); Serial.print(d3_INT1_eventCounter);Serial.flush();}
      }while (rtc_INT0_Flag == false);      // if the RTC alarm fires we break out of the PIR counting loop
      
#else
    
    #ifdef PIRtriggersSensorReadings
        sleepNwait4D3InterruptORrtcAlarm();             //Logger can be woken by D2 AND D3 interrupt events
        if(ECHO_TO_SERIAL){ Serial.print(F("PIR wakeup #")); Serial.print(d3_INT1_eventCounter);Serial.flush();}
    #else
        sleepNwait4RTCalarm(); //NORMAL RTC-only sleepNwait with no D3 interrupt-event-counters
    #endif
    
#endif  //terminates SUBloop: for #if defined(TipBucket_RainGauge) || defined(piezoDRIP) || defined(PIR_withLDR)

//==========================================================================================
//==========================================================================================
   }     // terminator for MAIN loop()======================================================
//==========================================================================================
//==========================================================================================
//==========================================================================================
//==========================================================================================
//==========================================================================================
//==========================================================================================
//==========================================================================================
//==========================================================================================

//==========================================================================================
//   *   *   *   *   *   *   *  Processor SLEEPING functions   *   *   *   *   *   *   *   *
//==========================================================================================
void sleepNwait4RTCalarm() {                          //NOTE all existing pin states are preserved during sleep

  turnOffAllindicatorLEDs();
  pinMode(2,INPUT);                                   //D2 pullup off - not needed with hardware pullups on RTC module
  noInterrupts();
  bitSet(EIFR,INTF0);                                 // clears interrupt 0's flag bit before attachInterrupt(0,isr,xxxx)
  attachInterrupt(0,rtc_d2_alarm_ISR,LOW);            //RTC alarm connected to pin D2 // LOW assures it will always respond if the RTC alarm is asserted
  interrupts();
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); // ADC_ON simply preserves whatever the current ADC status is (in our case it's already OFF...)
  
  //HERE AFTER WAKING  // note that detachInterrupt(0); happened inside the ISR
    
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(DS3231_STATUS_REG);
  Wire.write(0);                                      // clearing the entire status register turns Off (both) RTC alarms though technically only the last two bits need to be set
  Wire.endTransmission();
  rtc_INT0_Flag = false;                              // clear the flag we use to indicate the RTC alarm occurred 
  bitSet(EIFR,INTF0); bitSet(EIFR,INTF1);            // clear leftover processor interrupt flags - both just in case we have a noise trigger on the other pin
}  //terminator for sleepNwait4RTCalarm

void rtc_d2_alarm_ISR() {                             // this function gets called by attachInterrupt above
  rtc_INT0_Flag = true;                               // this flag only used with interrupt generating sensors on D3
  detachInterrupt(0);                                 // detaching inside the ISR itself makes sure it only triggers ONE time
}

#if defined(countPIReventsPerSampleInterval) || defined(PIRtriggersSensorReadings)
void sleepNwait4D3InterruptORrtcAlarm(){
  // we need to use two flag variables to keep track of which interrupt woke the logger
  // detachInterrupt(X); gets done inside the ISRx for each interrupt which also sets that flag true
  // but the other detachInterrupt(Y); has to be done manually based on those flags

  turnOffAllindicatorLEDs();
  pinMode(2,INPUT);                       //D2 INPUT & pullup not needed due to hardware pullups on RTC module
  pinMode(3,INPUT);                       //D3 INPUT & pullup off - the PIR module is controlling the HIGH/LOW status of this input line
  rtc_INT0_Flag = false;
  d3_INT1_Flag = false;
  
  noInterrupts();                         // waking from two interrupt sources requries them to be 'nested'
  
  bitSet(EIFR,INTF0);                     // before attachInterrupt(0,isr,xxxx) you must clear int0 flag inside 328p processor
  attachInterrupt(0,rtc_d2_alarm_ISR,LOW); // RTC alarm on D2  // LOW assures it will always respond if the RTC alarm is asserted

  bitSet(EIFR,INTF1);                     // clear int1 flag
  attachInterrupt(1,input_d3_interrupt_ISR,RISING);   //PIR drives its output line HIGH when it detects motion
  
  interrupts ();
  
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); // ADC_ON simply preserves whatever the current ADC status is (in our case it's already OFF...)

  if (d3_INT1_Flag) {        // d3_INT1_Flag is set true in input_d3_interrupt_ISR()
       if (d3_INT1_eventCounter < 65532) {d3_INT1_eventCounter++;}  // only increment our counter variable if it's below the uint16_t max
                                                                    // this is somewhat unnecessary given the sensor has a 2-second reset time...

   // Pip blue LED to indicate PIR wakeup event being counted
      #if defined(LED_r9_b10_g11_gnd12) 
        digitalWrite(10,HIGH); pinMode(10,OUTPUT);          // or pinMode(10,INPUT_PULLUP); // BLUE 
      #elif defined(LED_GndGB_A0_A2)
        bitClear(DDRC,2);bitSet(PORTC,2);                   // Blue LED = pinMode(A2,INPUT_PULLUP);
      #endif
        
        LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);   // time to view LED pip
      
      #if defined(LED_r9_b10_g11_gnd12) 
        pinMode(10,INPUT);                                  // D10 [blue] LED pullup OFF 
      #elif defined(LED_GndGB_A0_A2)
        bitClear(PORTC,2);                                  // A2 Blue LED: pullup OFF
      #endif   
         
    } else { // If d3_INT1_Flag is still false then we woke from the RTC alarm - not the PIR, so Int1 still needs detached
       detachInterrupt(1);
    }

  if (rtc_INT0_Flag){       // rtc_INT0_Flag is set true in rtc_d2_alarm_ISR()
      Wire.beginTransmission(DS3231_ADDRESS);
      Wire.write(DS3231_STATUS_REG);
      Wire.write(0);      // turns Off (both) RTC alarms
      Wire.endTransmission();
      // NOTE: do not set rtc_INT0_Flag = false; here! //must wait till AFTER we break out of counting while loop
      // because: while (rtc_INT0_Flag == false) { is the "sub-loop" that acts as the PIR event counter
  } else { // If rtc_INT0_Flag is still false then we woke from the PIR - not the RTC - so Int0 still needs to be detached
      detachInterrupt(0);
  }
  
} // terminates void sleepNwait4D3Interrupt(){

void input_d3_interrupt_ISR() {
  d3_INT1_Flag = true;
  detachInterrupt(1);
}
#endif  //#if defined(countPIReventsPerSampleInterval)

void setup_sendboilerplate2serialMonitor(){      
//-----------------------------------------------------------------------------------------
//NOTE:(__FlashStringHelper*) is needed to print variables is stored in PROGMEM instead of regular memory
//  internal eeprom stores 0-1023 bytes:
//  first 0-63 =64 bytes reserved for logger parameters & constants

  Serial.print(fileNAMEonly); // or use Serial.println((__FlashStringHelper*)codebuild); for the entire path + filename
  Serial.print(F("   "));Serial.print((__FlashStringHelper*)compileDate);
  Serial.print(F(" @")); Serial.println((__FlashStringHelper*)compileTime);
  
    char OnecharBuffer; // retrieve & send 100 character info fields stored in 328p internal eeprom 
    Serial.print(F("Logger:, ")); //Serial.println((__FlashStringHelper*)loggerConfiguration);
    for (uint16_t k = 64; k < 165; k++) { OnecharBuffer = EEPROM.read(k); Serial.print(OnecharBuffer); }

    Serial.println();
    Serial.print(F("Calibrated:, "));
    for (uint16_t k = 266; k < 367; k++) { OnecharBuffer = EEPROM.read(k); Serial.print(OnecharBuffer); }

    Serial.println();
    Serial.print(F("Deployment: "));
    for (uint16_t k = 165; k < 266; k++) { OnecharBuffer = EEPROM.read(k); Serial.print(OnecharBuffer); }

    Serial.println();   
    Serial.print(F("Site info: "));
    for (uint16_t k = 367; k < 468; k++) { OnecharBuffer = EEPROM.read(k); Serial.print(OnecharBuffer); }
    Serial.println();Serial.flush();

//  remaining 512 to 1023 for fonts(?) on loggers with OLED screens
}

void setup_displayStartMenu() {       
//-----------------------------------------------------------------------------------------

    clearSerialInputBuffer;                           // this just clears out any residual data in serial send buffer before starting our menu  
    Serial.setTimeout(1000);                          // 1000 milliseconds is the default timeout for the Serial.read(); command
    uint8_t inByte=0;
    boolean wait4input = true;
    boolean displayMenuAgain = true;
    uint32_t startMenuStart = millis();                          //Beginning of time-out period must be unsigned long variable

  do{ inByte=0;
    if (displayMenuAgain) { 
      startMenu_printMenuOptions(); displayMenuAgain=false;}

    if (Serial.available()) { 
      inByte = Serial.parseInt(); }                     //from https://forum.arduino.cc/t/simple-serial-menu-without-a-library/669556
    
    switch (inByte) {                                   //NOTE: switch can also accept 'letter inputs' with single quotes: case 'Z':
            case 99:
              displayMoreOptions = !displayMoreOptions; // toggles the boolean true/false variable  
              Serial.setTimeout(1000); displayMenuAgain=true;  break;
            case 1:
              startMenu_sendData2Serial(true);  
              displayMenuAgain=true;  break;
            case 2:
              startMenu_setRTCtime(); 
              Serial.setTimeout(1000); displayMenuAgain=true;  break;
            case 3:
              startMenu_setSampleInterval(); 
              Serial.setTimeout(1000); displayMenuAgain=true;  break;
            case 4:
              startMenu_updateLoggerInfoField(2,165,265); // updateDeploymentInfo  (SwitchText,EEstart,EEend)
              Serial.setTimeout(1000); displayMenuAgain=true; break;
            case 5:
              startMenu_updateLoggerInfoField(4,367,467);  // updateLoggerInfo 
              Serial.setTimeout(1000); displayMenuAgain=true; break;
            case 6:                                       // START logger operation
              wait4input=false; break;                    // wait4input=false breaks you out of the switch-case loop & sends you back to Setup function where displayStartMenu was first called
            case 7:
              ECHO_TO_SERIAL = !ECHO_TO_SERIAL;           // toggles SERIAL Output On/Off
              displayMenuAgain=true;  break;                             
            case 8:
              startMenu_updateLoggerInfoField(1,64,164);  // update Logger Hardware info 
              Serial.setTimeout(1000); displayMenuAgain=true; break;    
            case 9:
              startMenu_updateLoggerInfoField(3,266,366); // Set Cal. Constants //was updateCalibrationInfo 
              Serial.setTimeout(1000); displayMenuAgain=true; break;
            case 10:
              startMenu_setVrefConstant(); 
              Serial.setTimeout(1000); displayMenuAgain=true;  break; 
            case 11:
              startMenu_setRTCageOffset(); 
              Serial.setTimeout(1000); displayMenuAgain=true;  break; 
            case 12:
              startMenu_sendData2Serial(false);             // DLoad ALL RAW bytes from External EEprom
              displayMenuAgain=true;  break;                // outputs ALL eeprom memory as RAW bytes - useful for spotting code / pointer problems
            case 13:           
              startMenu_restoreStartValuesFromBackup(); Serial.setTimeout(1000);  // Restore operating parameters from EE backup to 328 internal eeprom
              displayMenuAgain=true;  break;                // restores startup parameters from 64byte backup on external eeprom - useful if you have to replace a dead promini                        
            case 14:           
              error_shutdown();                  
            default:                                      // Check milliseconds elapsed & send logger into shutdown if we've waited too long
                if ((millis() - startMenuStart) > 480000) {// start menu has an 480000 = 8 minute timeout
                Serial.println(F("Start Menu Timed out with NO commands!"));
                Serial.println(F("Logger shutting down...")); Serial.flush(); error_shutdown(); 
                }
              break;
         }    //  terminates switch-case cascade
      }while(wait4input);                   //  do-while loops back to check for input if wait4input=true;
  return;
}

void startMenu_printMenuOptions(){          
//-----------------------------------------------------------------------------------------
// note: setup_sendboilerplate2serialMonitor(); runs once on startup before this

  Serial.println();
  RTC_DS3231_getTime();                     // reads current clock time  and display it via CycleTimeStamp
  Serial.print(t_year,DEC);Serial.print(F("/"));Serial.print(t_month,DEC);Serial.print(F("/"));Serial.print(t_day,DEC);
    if(t_year==2000){Serial.print(F("(*)"));} // flags need for clock set
  Serial.print(F(" Time:")); Serial.print(t_hour,DEC);Serial.print(F(":"));Serial.print(t_minute,DEC);
  Serial.print(F(":"));Serial.print(t_second,DEC); //seconds separate because usually value is zero
  EEPROM.get(6,InternalReferenceConstant); // use .get for multi-byte variables
  Serial.print(F("  VREF:"));Serial.print(InternalReferenceConstant);
  #ifndef LowMemoryCompile
    if(InternalReferenceConstant==1126400){Serial.print(F("(Set?) "));}
  #endif
  RTCagingOffset = EEPROM.read(10);        // use .read for single-byte reads
  Serial.print(F("  RTCage:"));Serial.print(RTCagingOffset);
  #ifndef LowMemoryCompile
    if(RTCagingOffset==0){Serial.print(F("(?)"));}Serial.println();
  #endif
  Serial.print(F("Logging: "));
  startMenu_listEnabledSensors();
  Serial.println();
  
  SampleIntervalMinutes = EEPROM.read(4);
  SampleIntervalSeconds = EEPROM.read(5);
  Serial.print(F("RUNtime: "));Serial.print(totalBytesOfStorage-64);//64 bytes reserved for parameter backup
  Serial.print(F(" / "));Serial.print(sensorBytesPerRecord);Serial.print(F("recBytes @ "));
    if (SampleIntervalMinutes==0){Serial.print(SampleIntervalSeconds);Serial.print(F("sec"));}else{Serial.print(SampleIntervalMinutes);Serial.print(F("min"));}
    Serial.print(F(" ~ "));
    floatBuffer = ((totalBytesOfStorage-96)/sensorBytesPerRecord); // 96 =  64bytes reserved for backup // 32 byte buffer of ZEROS in eeprom
    if (SampleIntervalMinutes==0){
              floatBuffer = floatBuffer*SampleIntervalSeconds;
              Serial.print(floatBuffer/60,0);Serial.print(F("m or "));
              Serial.print(floatBuffer/3600,1);Serial.println(F("h"));
            } else {
              floatBuffer = floatBuffer*(60U*SampleIntervalMinutes); //was UL?
              uint16_Buffer = (int)(floatBuffer/86400);
              if (uint16_Buffer>1){
                Serial.print(uint16_Buffer);Serial.println(F("d"));
                } else {
                Serial.print(floatBuffer/3600,1);Serial.print(F("h"));
                }
            }
      
//POWER OF TWO error checks - any non zero result in if statement is interpreted as 'true' 
  if (sensorBytesPerRecord &(sensorBytesPerRecord-1)){ 
    Serial.println();Serial.print(sensorBytesPerRecord);Serial.println(F(" Sensor bytes not PowerOfTwo → CHANGE CONFIG!"));
    }
    Serial.println();
    
    if(displayMoreOptions){Serial.print(F("Setup & Testing:"));
      }else{Serial.print(F("Runtime Options:"));}
            
    sendMultiAscii2serial(8,32);// blank spaces
    Serial.print(F("Serial Output["));
    Serial.print(ECHO_TO_SERIAL ? "ON] " : "Off]"); 
    sendMultiAscii2serial(4,32);// blank spaces
    Serial.print(F("[99]→ "));  
    
    if(displayMoreOptions){Serial.print(F("Runtime"));
      }else{Serial.print(F("Setup"));}
    Serial.println(F(" Menu"));  
      
    sendMultiAscii2serial(65,45);Serial.println();      // line of -minus signs
    Serial.print(F(" [1]  DOWNLOAD Data     "));
    if((DS3231_PowerLossFlag) || (t_year==2000)){   //Oscillator Stop Flag (OSF). A logic 1 in bit7 indicates
    Serial.print      (F("[2]**Set RTC time**"));}//that the oscillator was stopped for some period due to power loss
    else {Serial.print(F("[2]  Set RTC Clock "));}
    Serial.println(F("   [3]  Set INTERVAL"));
    Serial.println(F(" [4]  Deployment info   [5]  Add Site info    [6]  START logging"));

if(displayMoreOptions){    
    sendMultiAscii2serial(65,45);Serial.println();      // line of -minus- signs
    Serial.println(F(" [7]  SERIAL On / Off   [8]  Add Logger info  [9]  Cal.Constants"));
    Serial.println(F(" [10] Set Internal VREF [11] Set RTC Aging    [12] Dload RAW EEdata"));
    Serial.println(F(" [13] Restore EE->328   [14] SHUTDOWN"));
    }
      Serial.println(); Serial.flush();
}   //terminates startMenu_printMenuOptions

void startMenu_listEnabledSensors(){ 
// to remove duplication in startMenu_printMenuOptions & startMenu_sendData2Serial
// ORDER here MUST MATCH the order of the sensor data in memory
// because these are the data collumn headers used in startMenu_sendData2Serial

  #ifdef logLowestBattery_1byte
        Serial.print(F("LoBat[mv], "));
        #endif
  #ifdef logRTC_Temperature_1byte
        Serial.print(F("RTC[°C], "));
        #endif
  #ifdef countPIReventsPerSampleInterval
        Serial.print(F("PIR count, "));
        #endif
  #ifdef PIRtriggersSensorReadings
        Serial.print(F("PIR Triggers Reading, "));
        #endif
  #if defined(readD7ResistorwD6ref_2byte) || defined(readD7resistorwD8pullup_2byte)
        Serial.print(F("D7r[Ω], "));  //e360 & 2part
        #endif 
  #ifdef readD9resistorwD6ref_2byte    // 2part
        Serial.print(F("D9r[Ω], "));
        #endif  
  #ifdef readD6ResistorwD8pullup_2byte // e360
      Serial.print(F("D6r[Ω], "));
      #endif  
  #ifdef readBh1750_LUX_2byte
        Serial.print(F("Bh1750[Lux], "));
        #endif
  #ifdef readSht3x_Humidity_2byte
        Serial.print(F("Sht3x[RH%], "));
        #endif
  #ifdef readSht3x_Temp_2byte
        Serial.print(F("Sht3x[T°C], "));
        #endif 
  #ifdef readBMP280_Temp_2byte
        Serial.print(F("b280[T°C], "));
      #endif
  #ifdef readBMP280_Pressure_2byte
        Serial.print(F("b280Pr[mbar], "));
        #endif
  #ifdef recordBMP280_Altitude_2byte
        Serial.print(F("b280Alt[m], "));
        #endif
  #ifdef recordBMEtemp_2byteInt
        Serial.print(F("[°C]bmE, "));
        #endif
  #ifdef recordBMEpressure_2byteInt
        Serial.print(F("[mbar]bmE, "));
        #endif
  #ifdef recordBMEhumidity_2byteInt
        Serial.print(F("[%rh]bmE, "));
        #endif        
  #ifdef logCurrentBattery_2byte
        Serial.print(F("C.Bat[mv], "));
        #endif
  #ifdef logFreeVariableMemory_2byte
        Serial.print(F("freeMem, "));
        #endif    
  #ifdef readSi7051_Temp_2byte
        Serial.print(F("SI7051[°C], "));
        #endif 
  }

void startMenu_setRTCageOffset(){                           //default = 0 
//-----------------------------------------------------------------------------------------
do {
    Serial.println(F("Input a new RTC aging Offset between -128 and +127:"));
    Serial.setTimeout(100000);                              // parseInt will normally “time out” after default set point is 1 second (1000 milliseconds).
    while (Serial.available() != 0 ) {Serial.read();}       // clears the serial buffer  
    RTCagingOffset = Serial.parseInt();          //parseInt() actually returns a long
     } while((RTCagingOffset<-128) || (RTCagingOffset>127));  // if condition fails & you have to re-enter the number

   // dont need to convert twos complement? because int8_t ALREADY IS??
   EEPROM.put(10,RTCagingOffset); // every time you run the logger it will retrieve this and load it into the RTC 
   i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_AGING_OFFSET_REG,RTCagingOffset);
   delay(15);
   RTCagingOffset = i2c_readRegisterByte(DS3231_ADDRESS,DS3231_AGING_OFFSET_REG);
   Serial.print(F("RTC Aging Offset set to: ")); Serial.println(RTCagingOffset); 
   settingsChanged = true; // forces backup of first 64bytes 328p eeprom to External EEprom at end of startup 
   return;
} // terminates startMenu_setRTCageOffset

void startMenu_setVrefConstant(){                           //default =1126400L = 1100mV * 1024 
//-----------------------------------------------------------------------------------------
do {
    Serial.println(F("Input a new Vref constant between 1000000 and 1228800:")); //1,126,400L = default for 1100mV * 1024
    Serial.setTimeout(100000);                              //parseInt will normally “time out” after default set point is 1 second (1000 milliseconds).
    while (Serial.available() != 0 ) {Serial.read();}       // clears the serial buffer  
    InternalReferenceConstant = Serial.parseInt();          //parseInt() actually returns a long
    }while((InternalReferenceConstant<1000000) || (InternalReferenceConstant>1228800)); // if condition fails & you have to re-enter the number
   Serial.print(F("Vref set to: ")); Serial.println(InternalReferenceConstant);
   EEPROM.put(6,InternalReferenceConstant);                 // every time you run the logger it will retrieve the interval from the previous run 
   settingsChanged = true; // forces backup of first 64bytes 328p eeprom to External EEprom at end of startup    
   return;
} // terminates startMenu_setVrefConstant

void startMenu_setSampleInterval(){
//-----------------------------------------------------------------------------------------
do {
    Serial.println();
    Serial.println(F("Don't change intervals until AFTER downloading any prexisting data!"));
    Serial.println(F("Input a sampling interval of 1,2,5,10,15,20,30,60 or [0] minutes:"));
    byteBuffer1 = 0;  while (Serial.available() != 0 ) {Serial.read();}   // clears the serial buffer  
    Serial.setTimeout(100000); SampleIntervalMinutes = Serial.parseInt(); 
    byteBuffer1 = SampleIntervalMinutes ? 60 % SampleIntervalMinutes : 0; // ERROR check: input must be valid divisor of 60 OR zero
    }while((byteBuffer1 !=0) || (SampleIntervalMinutes>60));              // or while condition fails & you have to re-enter the number
    
    Serial.print(F("  Sample Interval set to: ")); Serial.print(SampleIntervalMinutes);Serial.println(F(" min"));
    
    if (SampleIntervalMinutes>0){ SampleIntervalSeconds = 0; }
    // sub minute sampling ONLY for rapid burn tests so this is a HIDDEN OPTION for rapid debugging/testing (it's not displayed on the menu)
    // IF you enter a zero for minutes, then the logger assumes you want an interval in seconds:
    if (SampleIntervalMinutes==0){ 
        do {
         Serial.println(F("Enter sub-minute interval as 1,2,3,5,10,15,20, or 30 seconds: (sampling time may over-run the alarm!)"));
          byteBuffer2 =0;
         SampleIntervalSeconds = Serial.parseInt();
          byteBuffer2 = SampleIntervalSeconds ? 60 % SampleIntervalSeconds : 0; // ERROR check: input must be valid divisor of 30 OR zero
        }while ((byteBuffer2 !=0) || (SampleIntervalSeconds>30));               // or while condition fails & you have to re-enter the number
      Serial.print(F("  Sub-minute Sample Interval: ")); Serial.print(SampleIntervalSeconds);Serial.println(F(" seconds"));
      }
    if (SampleIntervalMinutes==0 && SampleIntervalSeconds==0){
      Serial.println(F("Invalid Entry: 15 min DEFAULT interval being SET"));Serial.flush();
      SampleIntervalMinutes = 15;SampleIntervalSeconds = 0;
      }
    EEPROM.update(4,SampleIntervalMinutes);             // newly entered values are now stored in the 329p's internal eeprom
    EEPROM.update(5,SampleIntervalSeconds);             // every time you run the logger it will retrieve the interval from the previous run 
    settingsChanged=true; // forces backup of first 64bytes 328p eeprom to External EEprom at end of startup 
    return;
} // startMenu_setSampleInterval


void startMenu_setRTCtime(){
//-----------------------------------------------------------------------------------------
  uint8_t set_t_second,set_t_minute,set_t_hour,set_t_day,set_t_month; 
  uint16_t set_t_year;

#ifndef LowMemoryCompile  
  int32_t previousRTCclockUpdate;
  EEPROM.get(12,previousRTCclockUpdate);
#endif

  Serial.println(F("Enter date/time:"));  // note Serial.parseInt will ONLY ACCEPT NUMBERS from the serial window!
  Serial.setTimeout(100000);    clearSerialInputBuffer;
  Serial.print(F("YYYY:"));     set_t_year = Serial.parseInt();      Serial.println(set_t_year);
  Serial.print(F("MM:"));       set_t_month = Serial.parseInt();     Serial.println(set_t_month);
  Serial.print(F("DD:"));       set_t_day = Serial.parseInt();       Serial.println(set_t_day);
  Serial.print(F("(24h) HH:")); set_t_hour = Serial.parseInt();      Serial.println(set_t_hour);
  Serial.print(F("MM:"));       set_t_minute = Serial.parseInt();    Serial.println(set_t_minute);
  Serial.print(F("SS:"));       set_t_second = Serial.parseInt();    Serial.println(set_t_second);

  if (set_t_month==0 && set_t_day==0){                          // this is a very crude error catch //this needs to be further developed
    Serial.println(F("Not valid input!"));
    return;                                             //shut down the logger - user will need to re-open the serial window to restart the logger
    } else {

    // Before updating the clock: load current RTC time in UnxTime seconds so delta can be calculated:
    RTC_DS3231_getTime();                               //updates all the t_variables
    int32_Buffer= RTC_DS3231_unixtime();                //current unix time on RTC clock _unixtime is just a calculation
    uint16_Buffer = t_year;                             //if current t_year==2000 then skip drift displays because of power fail

    // with the new time entered via the serial variables...
    t_year=set_t_year; t_second=set_t_second; t_minute=set_t_minute;
    t_hour=set_t_hour; t_day=set_t_day; t_month=set_t_month;      
    RTC_DS3231_setTime();         // NOW update the actual RTC registers 
    uint32_Buffer = RTC_DS3231_unixtime();
    EEPROM.put(12,uint32_Buffer); // Whenever an RTC update is done a new timestamp gets stored
    //in the 328p EEprom space for a future clock drift calculation

if(uint16_Buffer>2000){ //if existing t_year ==2000 then power failed 
                        //and last clockset is irrelevant
                        
    // limits here:  jan 1st 2025 to jan 1st 2038 [int32_t max: 2147483646]
    if ((uint32_Buffer >  1735711200) && (uint32_Buffer <  2145938400)) {
      Serial.print(F("Last ClockSet done @ "));Serial.println(previousRTCclockUpdate);
      // https://www.epochconverter.com/ converts to human readable
      }
}//if(uint16_Buffer>2000)

    int32_Buffer = uint32_Buffer - int32_Buffer;    // the current adjustment delta (in seconds)
    Serial.print(F("Current RTCtime updated by "));Serial.print(int32_Buffer);Serial.println(F("sec "));

if(uint16_Buffer>2000){ //if existing t_year == 2000 then power failed 
                        // so cant do any drift calc(s)
  
    // calculate the delta between current and previous RTC setting dates 
    uint32_Buffer = uint32_Buffer - previousRTCclockUpdate; 
    uint32_Buffer = uint32_Buffer /2678400;  // conversion delta in seconds to 'months' by straight division
    // NOTE: time calcs can be optimized w Tillart's divmod3, divmod5, divmod10, divmod12, divmod24, divmod60 [12,24&60 for time] 
if ((uint32_Buffer > 0) && (uint32_Buffer < 240)) {  // <20 years equivalent in months
    Serial.print(F("after "));Serial.print(uint32_Buffer);Serial.print(F("months "));
    Serial.println(F("[±3 RTCage fixes ~1sec drift/m]"));
    }  
}//if(uint16_Buffer>2000)
  
    i2c_setRegisterBit(DS3231_ADDRESS,DS3231_STATUS_REG,7,0); //clear the OSF flag after time is set
    DS3231_PowerLossFlag=false;
    }   //terminates if (set_t_month==0 && set_t_day==0){

    settingsChanged=true; // forces backup of first 64bytes 328p eeprom to External EEprom at end of startup 
} //terminates startMenu_setRTCtime



void startMenu_updateLoggerInfoField(uint8_t switchText,uint16_t EEstart,uint16_t EEend){
//----------------------------------------------------------------------------------------------------
// Serial input method from Example 3  https://forum.arduino.cc/t/serial-input-basics-updated/382007

Serial.print(F("Type < "));
switch (switchText) {
            case 1:
              Serial.print(F("Logger Config"));break;
            case 2:
              Serial.print(F("Deployment Location"));break;
            case 3:
              Serial.print(F("CalDate & constants"));break;
            case 4:
              Serial.print(F("Site info"));break;
            default:
              Serial.print(F("-Err-"));break;
             } // terminate switch-case cascade 
Serial.println(F(" > with '<'&'>' ends +[Enter]"));

Serial.println(F(" < MAX 100 chars >  [Timeout: 200sec]"));
clearSerialInputBuffer;

byteBuffer1 = 0; // counts the # of receivedChars // Char is signed and that Byte is unsigned.
boolean newData = false;
boolean recvInProgress = false;
char startMarker = '<';
char endMarker = '>';
char rc;              //incoming character
byte numChars = 101;  //sets maximum number of characters that can be recieved
char receivedChars[numChars];

uint32_t startMillis = millis();
do {   // timeout do-while loop

   while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
                if (rc != endMarker) {
                receivedChars[byteBuffer1] = rc;
                byteBuffer1++;
                    if (byteBuffer1 >= numChars) {
                    byteBuffer1 = numChars - 1;
                    }
                } else {
                receivedChars[byteBuffer1] = '\0'; // terminate the string
                recvInProgress = false;
                newData = true;
                }
         } else if (rc == startMarker) { recvInProgress = true;}
    } //while
    
    if (newData) break; // breaks out of the timeout dowhile-loop after data is input
    
}while ((millis() - startMillis) < 200000); // 200 seconds to respond?

if (!newData){   // if newData = false then return to menu
Serial.println(F("NO valid information received -> returning to start menu"));
Serial.flush(); return;
}

// erase previous description                 // NOTE I'm leaving LAST 512 eeprom memory locations for future use screen fonts
for (uint16_t h = EEstart; h <= EEend; h++){  // EEPROM.update does not write new data unless new content is different from old
    EEPROM.update(h,32);                      // writes ascii[32] to each memory location which is the 'blank space' character
    if ((h % 16) == 0){Serial.print(F("."));} // send progress indicator dot to the serial monitor window (every 16 characters)
    delay(4);                                 // writing to the internal eeprom needs 3.5 msec per byte and adds an additional 8mA to the ProMini’s normal 5mA operating current
    }

// save new 100 - char Hardware details to EEprom
  for (uint16_t i = 0; i < byteBuffer1; i++){
    EEPROM.update(i+EEstart,receivedChars[i]);
    Serial.print(F("."));
    delay(4);
    }
      
  Serial.println();Serial.print(F("New Info Saved: "));
  Serial.println(receivedChars);Serial.flush();
  return;
}// end startMenu_updateLoggerInfoField


void startMenu_sendData2Serial(boolean convertDataFlag){ // called at startup via serial window
//===============================================================================================
// NOTE: the ORDER of BYTES/SENSORS listed in startMenu_sendData2Serial MUST EXACTLY MATCH
// the order you ADDED those SENSOR READINGS when loading the EEprom write buffer in the main loop

 if(totalBytesOfStorage==4096){Wire.setClock(100000UL);}

// ADD HEADER information to top rows of serial output:
  setup_sendboilerplate2serialMonitor();  // a description of the deployment should be part of the data output
  //Serial.println(F("Convert Unixtime to Excel Dates with = UnixTime/#secondsInaDay + DATE(1970,1,1) "));

if (convertDataFlag){                     // don't print these headers if sending RAW bytes as output
  Serial.print(F("UnixTime,"));  
  startMenu_listEnabledSensors();

 }else{
  Serial.print(F("ALL memory locations from EEprom as RAW byte values:"));
  } //terminates if(convertDataFlag)
  Serial.println();Serial.flush();

//starting time value was stored in first four bytes of the 328p internal eeprom:
  uint32_t unix_timeStamp;
    EEPROM.get(0,unix_timeStamp);                       // the loggerStartTime saved at previous logger startup
    SampleIntervalMinutes = EEPROM.read(4);
    SampleIntervalSeconds = EEPROM.read(5);
    
  uint16_t secondsPerSampleInterval;
    if (SampleIntervalMinutes==0){                      // sub-minute alarms for accelerated run testing
        secondsPerSampleInterval = SampleIntervalSeconds;
        }else{                                          //  normal minute based alarms:
        secondsPerSampleInterval = 60UL*SampleIntervalMinutes;
        }

  uint32_t startMillis = millis();                      // track how long it takes for the data download
  EEmemPointer = 64;                                   // a counter that advances through the EEprom Memory in sensorBytesPerRecord increments
    

  do{    // this big do-while loop readback must EXACTLY MATCH the data saving pattern in our main loop:
  //---------------------------------------------------------------------------------------------------------- 
   
  byteBuffer2 = 0;  //if the 1st & 2nd bytes in the record readback as ZERO then we've reached our end of data in the EEprom
  byteBuffer1 = i2c_eeprom_read_byte(EEpromI2Caddr, EEmemPointer);    // uint8_t i2c_eeprom_read_byte
  if((EEmemPointer+1) < totalBytesOfStorage){
    byteBuffer2 = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer+1);
    }
  if(byteBuffer1==0 && byteBuffer2==0 && convertDataFlag){ break;}     // this breaks us out of the do-while readback loop


if (!convertDataFlag){    // then output raw bytes exactly as read from eeprom [with no timestamp] // this is ONLY used for debugging
        for (uint8_t j = 0; j < sensorBytesPerRecord; j++) {
        byteBuffer1 = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
        Serial.print(byteBuffer1); Serial.print(F(","));    // outputs raw bytes as read from eeprom:
        EEmemPointer++; 
        }      
} else { // if convertDataFlag is true then raw eeprom bytes get re-constituted back to variables

#ifdef PIRtriggersSensorReadings   //4-byte reconstruction of d3_INT1_elapsedSeconds // saved from low byte first to high byte last
      d3_INT1_elapsedSeconds = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //lo byte
      EEmemPointer++;
      uint32_Buffer = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;
      d3_INT1_elapsedSeconds = (uint32_Buffer << 8) | d3_INT1_elapsedSeconds;
      uint32_Buffer = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;
      d3_INT1_elapsedSeconds = (uint32_Buffer << 16) | d3_INT1_elapsedSeconds;
      uint32_Buffer = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //high byte
      EEmemPointer++; 
      d3_INT1_elapsedSeconds = (uint32_Buffer << 24) | d3_INT1_elapsedSeconds;
      
      d3_INT1_elapsedSeconds=d3_INT1_elapsedSeconds-1; // removes our zero trap on the first byte
      
      unix_timeStamp += d3_INT1_elapsedSeconds;   // then PIR triggered readings have an irregular number seconds between intervals    
#endif

  Serial.print(unix_timeStamp);Serial.print(",");
  
#ifndef PIRtriggersSensorReadings
  unix_timeStamp += secondsPerSampleInterval;               //increment unix timestamp for the NEXT record after printing
#endif
  // order of sensors & bytes listed here must EXACLTY MATCH the order in which you loaded the bytes into the eeprom in the main loop

#ifdef logLowestBattery_1byte  //  1 byte index encoded (slight loss of resolution compared to original two bytes)
      uint16_Buffer = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;
      uint16_Buffer =(uint16_Buffer*16)+1700;               // REVERSING the calculation we used to index-compress the data into one byte(note <<4 is the same as *16)
      Serial.print(uint16_Buffer);Serial.print(F(","));
#endif

#ifdef logRTC_Temperature_1byte                                   // RTC temperature 1-byte compressed: low side cutoff at 1 for minimum reading of -12.25C
      int16_Buffer = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;
      floatBuffer = int16_Buffer;
      floatBuffer = (floatBuffer/4.0)-10.0;                 // REVERSING the calculation we used to compress the data into one byte
      Serial.print(floatBuffer,2);Serial.print(F(","));     // ,2) specifies that you only print two decimal places
#endif  //#ifdef logRTC_Temperature_1byte 

#ifdef countPIReventsPerSampleInterval
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;             
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;                   
      uint16_Buffer = (uint16_t)((hiByte << 8) | loByte); 
      Serial.print(uint16_Buffer-1);Serial.print(F(","));   // 'minus 1' because we added one as our zero trap before the count was saved
#endif

#ifdef readD7resistorwD8pullup_2byte
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;             
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;                   
      uint16_Buffer = (uint16_t)((hiByte << 8) | loByte); 
      Serial.print(uint16_Buffer);Serial.print(F(","));
#endif

#ifdef readD6ResistorwD8pullup_2byte
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;             
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;                   
      uint16_Buffer = (uint16_t)((hiByte << 8) | loByte); 
      Serial.print(uint16_Buffer);Serial.print(F(","));
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// STEP6 : following the pattern shown directly ABOVE extract the sensor bytes from the EEprom
// in the SAME ORDER you saved them in during STEP5, then use the 'bitshift & bitwise OR' method to combine
// those bytes back into an integer variable. Don't forget to add a comma to separate the numbers on screen.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef readBh1750_LUX_2byte
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //low byte
      EEmemPointer++;
      lux_BH1750_RawInt = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //hi byte
      EEmemPointer++;
      lux_BH1750_RawInt = (lux_BH1750_RawInt << 8) | loByte;
      floatBuffer = bh1750.calcLux(lux_BH1750_RawInt,BH1750_QUALITY_LOW,BH1750_MTREG_LOW);
      Serial.print(floatBuffer,0); //the decimals are meaningless at this resolution 
      Serial.print(",");
#endif

#ifdef readSht3x_Humidity_2byte              // 2-bytes, low byte first
      //Sht30 resolution is only: 0.01°C and 0.01 %RH
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //low byte
      EEmemPointer++;
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //hi byte
      EEmemPointer++;
      int16_Buffer = (int16_t)((hiByte << 8) | loByte);
      floatBuffer  = (float)(int16_Buffer)/100.0;
      Serial.print(floatBuffer,2);Serial.print(",");
#endif

#ifdef readSht3x_Temp_2byte          //2-bytes, low byte first
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //low byte
      EEmemPointer++;
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //hi byte
      EEmemPointer++;
      uint16_Buffer = (uint16_t)((hiByte << 8) | loByte);    
      floatBuffer  = (float)(uint16_Buffer)/100.0;
      floatBuffer  = floatBuffer -10.0; // correction for the +10C shift we did before *100 to preserve the decimals
      Serial.print(floatBuffer,2);Serial.print(",");
#endif

#ifdef readBMP280_Temp_2byte           // Bmp280_Temp_degC, 2-bytes, low byte first
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //low byte
      EEmemPointer++;
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //hi byte
      EEmemPointer++;
      int16_Buffer = (int16_t)((hiByte << 8) | loByte);
      floatBuffer  = (float)(int16_Buffer)/100.0;
      Serial.print(floatBuffer,2);Serial.print(",");
#endif

#ifdef readBMP280_Pressure_2byte      // Bmp280_Pr_mBar, 2-bytes, low byte first
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //low byte
      EEmemPointer++;
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //hi byte
      EEmemPointer++;
      int16_Buffer = (int16_t)((hiByte << 8) | loByte);
      floatBuffer  = (float)(int16_Buffer)/10.0;
      Serial.print(floatBuffer,1);Serial.print(",");
#endif

#ifdef recordBMP280_Altitude_2byte            // Bmp280_Temp_degC, 2-bytes, low byte first
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //low byte
      EEmemPointer++;
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer); //hi byte
      EEmemPointer++;
      int16_Buffer = (int16_t)((hiByte << 8) | loByte);
      floatBuffer  = (float)(int16_Buffer)/100.0;  //Note: *100 over runs the integer at higher altitudes, in that case switch to *10
      Serial.print(floatBuffer,1);Serial.print(",");
#endif

#ifdef recordBMEtemp_2byteInt  // Bme280_Temp_degC, 2-bytes
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;
      int16_Buffer = (int16_t)((hiByte << 8) | loByte);
      floatBuffer  = (float)(int16_Buffer)/100.0;  // two decimal places were embedded in the original integer
      Serial.print(floatBuffer,2);Serial.print(",");
#endif

#ifdef recordBMEpressure_2byteInt // Bme280_Pr_mBar stored as two bytes
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;
      uint16_Buffer = (uint16_t)((hiByte << 8) | loByte);
      uint32_Buffer = uint16_Buffer + 80000;  // remvoe the -80000 scaling we did before storing the pressure with the embedded 2 decimal places
      floatBuffer  = (float)(uint32_Buffer) /100.0; // two decimal places were embedded in the original integer
      Serial.print(floatBuffer,2);Serial.print(",");
#endif

#ifdef recordBMEhumidity_2byteInt // originally uint32_t g_humidity but we truncated to 2-bytes
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;
      uint16_Buffer = (uint16_t)((hiByte << 8) | loByte);
      floatBuffer  = (float)(uint16_Buffer) /100.0; // two decimal places were embedded in the original integer
      Serial.print(floatBuffer,2);Serial.print(",");
#endif
      
#ifdef logCurrentBattery_2byte            // stored as two data bytes in the Main Loop, with lowByte first, then highByte
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;             
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;                   
      uint16_Buffer = (uint16_t)((hiByte << 8) | loByte); 
      Serial.print(uint16_Buffer);Serial.print(F(","));
#endif

#ifdef logFreeVariableMemory_2byte   // stored as two data bytes in the Main Loop, with lowByte first, then highByte
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;             
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
      EEmemPointer++;                   
      uint16_Buffer = (uint16_t)((hiByte << 8) | loByte); 
      Serial.print(uint16_Buffer);Serial.print(F(","));
#endif

#ifdef readSi7051_Temp_2byte 
        loByte = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
        EEmemPointer++;
        TEMP_si7051 = i2c_eeprom_read_byte(EEpromI2Caddr,EEmemPointer);
        EEmemPointer++;
        TEMP_si7051 = (TEMP_si7051 << 8) | loByte;           // NOTE: no casting needed with hiByte loaded into TEMP_si7051 which is an integer variable
        Serial.print(((175.26*TEMP_si7051)/65536.0)-46.85,3);Serial.print(F(","));  //calculation is promoted to float by the decimal places
        //integer converted to celcius (3 decimals output)   //or Serial.print(TEMP_si7051); to print raw integer 
#endif // #ifdef readSi7051_Temp_2byte

  } //terminators if(convertDataFlag)
  
  Serial.println();
  } while(EEmemPointer < totalBytesOfStorage); // terminates the readback loop when pointer reaches end of memory space
  //---------------------------------------------------------------------------------------
  Serial.println();
  Serial.print(F("Download took: "));Serial.print((millis() - startMillis));Serial.println(F(" msec"));
  Serial.flush();
  EEmemPointer = 64;                       //back to starup default value
}  // terminates sendData2Serial() function


void startMenu_restoreStartValuesFromBackup(){        // this option only used when fixing a dead logger!
//-------------------------------------------------------------------------------------------------------
    uint8_t tempDataBuffer[16];

    Serial.println(F("WARNING: this functionality is still in dev -not fully tested yet-")); 
 
    Serial.println(F("Parameters in 328p:"));
    Serial.print(F("Start Utime:"));EEPROM.get(0,uint32_Buffer);Serial.print(uint32_Buffer);
    Serial.print(F("  Interval:"));Serial.print(EEPROM.read(4));Serial.print(F("m, "));
    Serial.print(EEPROM.read(5));Serial.println(F("s"));
    Serial.print(F("Vref:"));EEPROM.get(6,uint32_Buffer);Serial.println(uint32_Buffer); 
    //Serial.print(F("  RTCage:"));Serial.print(EEPROM.read(10));
    Serial.println();Serial.println(F("Will be replaced by these Ext.EEprom values:"));
    byteBuffer2=0;
    Wire.beginTransmission(EEpromI2Caddr);
    Wire.write(0);Wire.write(0); Wire.endTransmission();
    Wire.requestFrom(EEpromI2Caddr,16);
    while (Wire.available()) {tempDataBuffer[byteBuffer2++] = Wire.read();}
    byteBuffer2=0;
    Serial.print(F("Start Utime:"));
    uint32_Buffer = tempDataBuffer[byteBuffer2++];
    uint32_Buffer = (uint32_Buffer << 8) | tempDataBuffer[byteBuffer2++];
    uint32_Buffer = (uint32_Buffer << 8) | tempDataBuffer[byteBuffer2++];
    uint32_Buffer = (uint32_Buffer << 8) | tempDataBuffer[byteBuffer2++];
    Serial.print(uint32_Buffer);
    Serial.print(F("  Interval:"));Serial.print(tempDataBuffer[byteBuffer2++]);
    Serial.print(F("m, "));Serial.print(tempDataBuffer[byteBuffer2++]);Serial.println(F("s"));
    Serial.print(F("  Vref:"));
    uint32_Buffer = tempDataBuffer[byteBuffer2++];
    uint32_Buffer = (uint32_Buffer << 8) | tempDataBuffer[byteBuffer2++];
    uint32_Buffer = (uint32_Buffer << 8) | tempDataBuffer[byteBuffer2++];
    uint32_Buffer = (uint32_Buffer << 8) | tempDataBuffer[byteBuffer2++];
    Serial.println(uint32_Buffer);
    Serial.println();  
   
    Serial.println(F("Restoring Logger Parameters from external BACKUP cannot be undone! Proceed? y/n"));
    clearSerialInputBuffer;   // clears any leftover bytes in serial buffer  
    Serial.setTimeout(1000);
    booleanBuffer = true;   byteBuffer1 = 0;
    while (booleanBuffer) {
        if (Serial.available()) byteBuffer1 = Serial.read();  //.read captures only ONE character at a time from the serial monitor window
        
        switch (byteBuffer1) {
          case 'y': 
            Wire.beginTransmission(EEpromI2Caddr);          
            Wire.write(0); Wire.write(0);               // two bytes to specify the external eeprom address
            Wire.endTransmission();

            Wire.requestFrom(EEpromI2Caddr,16);
            byteBuffer2=0;
            while (Wire.available()) {tempDataBuffer[byteBuffer2++] = Wire.read();}
            
            for (uint8_t h=0; h<16; h++){
              EEPROM.update(h, tempDataBuffer[h]); // EEPROM. BLOCKING each byte write is about 4ms
              // BLOCKING about 4ms per byte (addr, val);
             }
            Serial.println(F("328p startup values RESTORED from BACKUP on external EEprom")); Serial.flush();
            booleanBuffer = false; break;

          case 'n': 
            Serial.println(F("Restore NOT done.")); Serial.flush();
            booleanBuffer = false; break;  // return;
            
          default: 
            break;
          }       // terminates switch case
      }           // terminates while(booleanBuffer)     
}                 // terminatesvoid _restoreStartValuesFromBackup


//==========================================================================================
//==========================================================================================
//   *  *   *  *  *  *  * FUNCTIONS called by the Main loop()  *  *  *  *  *  *  *  *  *  *
//==========================================================================================
//==========================================================================================

// ======================================================================================
//   *  *   *  *  *  *  * I2C SENSOR & MEMORY REGISTER FUNCTIONS  *  *  *  *  *  *  *  * 
// ======================================================================================
// see: https://thecavepearlproject.org/2017/11/03/configuring-i2c-sensors-with-arduino/

bool i2c_getRegisterBit(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition) 
//-----------------------------------------------------------------------------------------
{
  byte registerByte;
  registerByte = i2c_readRegisterByte(deviceAddress, registerAddress);
  return bitRead(registerByte,bitPosition);     // alt: return ((registerByte >> bitPosition) & 0b00000001);
}

byte i2c_setRegisterBit(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition, bool state) 
//-----------------------------------------------------------------------------------------
{
  byte registerByte;
  byte result;
  registerByte = i2c_readRegisterByte(deviceAddress, registerAddress);  //load the existing register contents
  if (state) {    // when state = 1
    bitSet(registerByte,bitPosition);           // alt: registerByte |= (1 << bitPosition);  // bitPosition of registerByte now = 1
  }
  else {  // when state = 0
    bitClear(registerByte,bitPosition);         // alt: registerByte &= ~(1 << bitPosition); // bitPosition of registerByte now = 0
  }
  result = i2c_writeRegisterByte(deviceAddress, registerAddress, registerByte);
  return result;                                // result =0 if the writing the new data to the registry went ok
}

byte i2c_readRegisterByte(uint8_t deviceAddress, uint8_t registerAddress)
//-----------------------------------------------------------------------------------------
{
  byte registerData;
  Wire.beginTransmission(deviceAddress);        //set destination target
  Wire.write(registerAddress);                  //assumes register address is <255  - this is not the case for all sensors
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)1);
  registerData = Wire.read();
  return registerData;
}

byte i2c_writeRegisterByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t newRegisterByte){
//-----------------------------------------------------------------------------------------
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(newRegisterByte);
  byte result = Wire.endTransmission();

  if (result > 0)   //error checking
  {
    if(ECHO_TO_SERIAL){                           //NOTE: only call halt on error if in debug mode!
      Serial.print(F("FAIL in I2C register write! Result code: "));
      Serial.println(result); Serial.flush();
      error_shutdown();
    }
  }
  // LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);  // some sensors need this settling time after a register change?
  return result;
}

uint8_t i2c_eeprom_read_byte(uint8_t deviceAddress, uint16_t memoryAddress ) {  // called by void startMenu_sendData2Serial
//-----------------------------------------------------------------------------------------
// unlike i2c_readRegisterByte this function takes a TWO-byte memory address

  uint8_t rdata;
  Wire.beginTransmission(deviceAddress);
  Wire.write(highByte(memoryAddress));            // Address High Byte
  Wire.write(lowByte(memoryAddress));             // Address Low Byte
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress,(uint8_t)1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

// ======================================================================================
//   *  *   *  *  *  *  * BATTERY MONITORING function  *  *  *  *  *  *  *  *  *  *  *
// ======================================================================================
// see: http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
// long video exlainer: https://www.youtube.com/watch?v=G6dvDgCOyqk&ab_channel=JulianIlett
// for info on sleeping the 328p during ADC readings see:  https://www.gammon.com.au/adc
  
uint16_t readRailVoltage(){                             // reads 1.1vref as input against VCC as reference voltage
  ADCSRA = 0; SPCR = 0; // Disables ADC & SPI           // only use PRR after disabling the peripheral clocks, otherwise the ADC gets "frozen" in an active state drawing power
  power_all_disable();  // must call PRR after disabling ADC clock
  power_adc_enable(); 

  ADMUX = set_ADMUX_2readRailVoltage;   ADCSRA = set_ADCSRA_2readRailVoltage;
  // default 64 prescalar @ 8MHz/64 = 125 kHz ADC clock 
  // default ADC read takes 13 ADC clock cycles, so default speed is about 9615 Hz (or 0.104 milliseconds per reading).
  bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,0);bitWrite(ADCSRA,ADPS0,1);   // 32 ADC prescalar = 2x normal the speed of the ADC
  // prescalar to 32 so it operates at 2x the normal speed note: readings at 2x are nearly identical to 1x speed readings
  // but occasionally delivers an anomalously low read?
  bitSet(ADCSRA,ADSC);  // triggers a 1st THROW AWAY READING to engage 100nF AREF capacitor
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); // leave ADC_ON: adds ~110µA so the aref cap charges up (actually needs ~5ms)

  uint16_Buffer=0; adc_interrupt_counter = 0;           // reset our accumulator variables
  bitSet(ACSR,ADIF);   // clears any previous ADC interrupt flags
  bitSet(ADCSRA,ADIE); // tells ADC to generate processor interrupts when a new ADC reading is ready
  set_sleep_mode( SLEEP_MODE_ADC );                     // Enable ADC Noise Reduction Sleep Mode
    do{     // Note: Sleep_Mode_ADC AUTOMATICALY TRIGGERS a READING WHEN ENTERED
          do{ sleep_mode();                             // sleep_mode macro combines sleep enable & disable with sleep_cpu command     // Note: sleep_disable(); not needed with sleep_mode();    
          }while (bit_is_set(ADCSRA,ADSC)); // ADC resets ADSC bit to zero only when conversion is finished, otherwise bit stays 1 during ADC read
            uint16_Buffer += ADC;
    }while (adc_interrupt_counter<4);       // uint16_Buffer = sum of 4 ADC readings (so (4x0.104/2)msec total time)

  bitClear(ADCSRA,ADIE);                                // turn off the ADC interrupts
  bitSet(ACSR,ADIF);                                    // clears any ADC interrupt flags in the processor
  ADMUX = default_ADMUX; //restores default A3 channel? // but we can skip this unless ADC used elsewhere
  ADCSRA = 0; power_adc_disable();                      // turn off ADC

  uint16_t railvoltage = InternalReferenceConstant / (uint16_Buffer>>2); // convert average ADC reading into railvoltage in mV // bitshift >>2 same as divide by 4

// re-enable other peripherals we turned off with power_all_disable();
  power_timer0_enable();
  power_twi_enable();
  //TWI has its own independent system (CPU) clock divider to generate SCL
  // so not dependant on the internal timers like Timer0

  if(ECHO_TO_SERIAL){ power_usart0_enable();}

  if (railvoltage < LowestBattery) {LowestBattery = railvoltage;}  
  if (railvoltage < systemShutdownVoltage){
      if(ECHO_TO_SERIAL){Serial.print(railvoltage); Serial.println(F(" battery voltage too low!")); Serial.flush();}
      error_shutdown(); 
      }  // battery is near BOD trigger voltage, shut down the logger
  
  return railvoltage; 
}  // terminator for readRailVoltage()

ISR (ADC_vect){ adc_interrupt_counter++;}  // called by the readRailVoltage() FUNCTION above
// the ADC_vect ISR executes when the ADC generates interrupts - increments the adc_interrupt_counter variable with each new reading

// ======================================================================================
//   *  *   *  *  *  *  *  *  *  *  ERROR HANDLER   *  *  *  *  *  *  *  *  *  *  *  *  *
// ======================================================================================
void error_shutdown() {

  if(!bitRead(PRR,PRUSART0)){  // if the USART peripheral IS TURNED ON = the bit is a zero,  so use ! to invert
    Serial.println(F("→ logger SHUT-DOWN in 15sec")); Serial.flush(); // see datasheet 9.11.3 PRR – Power Reduction Register
    }

  pinMode(13, INPUT);                               // the built-in red led on the Arduino is on D13
   for (uint8_t CNTR = 0; CNTR < 64; CNTR++) {      // FLASH red indicator LED to indicate error state
     PINB = B00100000;                              // writing a bit to the pin register TOGGLES D13 LED pullup resistor On/Off
     LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
   }
  digitalWrite(13, LOW);

  if(bitRead(PRR,PRTWI)){      // if the I2C peripheral is TURNED OFF = the bit is a 1 // Writing a logic one to this bit shuts down the TWI by stopping the clock to the module.
    power_twi_enable();        // if the I2C off then must turn on to disable sensors & RTC before shutdown
  }
  
  // SLEEP ANY CONNECTED SENSORS before you disable I2C

  //disable RTC alarms, by setting to an INVALID TIME that can not be reached
  // when an alarm is triggered, the INT/SQW pin goes low, and it stays that way until the appropriate register is cleared in the DS3231
  // this results in ~700uA drain through the pullup AFTER shutdown which depletes the coincell battery

  RTC_DS3231_setA1Time(0,0,62,0,0b00001100,0,0,0);              // DISABLE AL1 by setting to an INVALID TIME that can never be reached: 61 for minutes
                                                                // 0b00001100 = A1 Alarm when minutes AND seconds match, ignores days, hours
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);             // give the RTC memory register some WRITING time 

  // clears (both) RTC alarms if they have fired
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(DS3231_STATUS_REG); Wire.write(0); 
  Wire.endTransmission();
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  
   bitSet(ACSR,ACD);                                 // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
   ADCSRA = 0; SPCR = 0;                             // 0 Disables ADC & SPI // only use PRR after disabling the peripheral clocks, otherwise the ADC gets "frozen" in an active state drawing power
   if(!bitRead(PRR,PRUSART0)){                       // if serial connection is ON, turn it off
      power_usart0_disable();
      }   
  
  // FLOAT all pins so no current can't leak after shutdown:
  for (uint8_t i = 0; i <=13; i++) { 
        pinMode(i,INPUT);
        }
  pinMode(A0,INPUT);pinMode(A1,INPUT);pinMode(A2,INPUT);pinMode(A3,INPUT);
  pinMode(A4,INPUT); pinMode(A5,INPUT);              //Note: A4 & A5 are still connected to 4k7 pullup resistors on RTC module

  noInterrupts ();
  bitSet(EIFR,INTF0);                               // clear flag for interrupt 0  see: https://gammon.com.au/interrupts
  bitSet(EIFR,INTF1);                               // clear flag for interrupt 1
  interrupts (); 

  LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);
  } //end  void error_shutdown()

//==========================================================================================
//==========================================================================================
//   *  *   *  *  Functions used for RTC control  *  *  *  *  *  *  *  *  *  *  *  *  *
//==========================================================================================
//==========================================================================================
// modified from JeeLab's fantastic real time clock library released into the public domain 
// =========================================================================================

float RTC_DS3231_getTemp(){             // from http://forum.arduino.cc/index.php?topic=22301.0  
//--------------------------------------------------------------------------------------------
// temp registers (upper byte 0x11 & lower 0x12) get updated automatically every 64s

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(DS3231_TMP_UP_REG);        // set the memory pointer inside the RTC to first temp register
  Wire.endTransmission(); 
  Wire.requestFrom(DS3231_ADDRESS, 2);  // request the two temperature register bytes
  if (Wire.available()) {
    hiByte = Wire.read();               // 2's complement int portion - If hiByte bit7 is a 1 then the temperature is negative
    loByte = Wire.read();               // fraction portion
    floatBuffer = ((((int16_t)hiByte << 8) | (int16_t)loByte) >> 6) / 4.0; // Allows for readings below freezing // to Convert Celcius to Fahrenheit: temp3231 = (temp3231 * 1.8) + 32.0; 
  } else { floatBuffer = 99.99;}        // use 'impossible' temperatue output to flag if sensor returns bad data
  return floatBuffer;
}

void RTC_DS3231_getTime(){
//------------------------
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0);
  Wire.endTransmission(); 
  Wire.requestFrom(DS3231_ADDRESS, 7);
  t_second = rtc_bcd2bin(Wire.read() & 0x7F);
  t_minute = rtc_bcd2bin(Wire.read());
  t_hour = rtc_bcd2bin(Wire.read());
  Wire.read();
  t_day = rtc_bcd2bin(Wire.read());
  t_month = rtc_bcd2bin(Wire.read());
  t_year = rtc_bcd2bin(Wire.read()) + 2000;
  return;
}

void RTC_DS3231_setTime(){
//------------------------
   Wire.beginTransmission(DS3231_ADDRESS);
   Wire.write((byte)0);
   Wire.write(rtc_bin2bcd(t_second));
   Wire.write(rtc_bin2bcd(t_minute));
   Wire.write(rtc_bin2bcd(t_hour));
   Wire.write(rtc_bin2bcd(0));
   Wire.write(rtc_bin2bcd(t_day));
   Wire.write(rtc_bin2bcd(t_month));
   Wire.write(rtc_bin2bcd(t_year - 2000));
   //Wire.write(0); //what is the last one for?
   Wire.endTransmission();
  }

//void RTC_DS3231_setAlarm1Simple(byte hour, byte minute) {
//-----------------------------------------------------------
//  RTC_DS3231_setA1Time(0, hour, minute, 00, 0b00001000, false, false, false);
//}

void RTC_DS3231_setA1Time(byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmBits, bool A1Dy, bool A1h12, bool A1PM) {
//-----------------------------------------------------------------------------------------------------------------------------------
//  Sets the alarm-1 date and time on the DS3231, using A1* information
  byte temp_buffer;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0x07);    // A1 starts at 07h
  // Send A1 second and A1M1
  Wire.write(rtc_bin2bcd(A1Second) | ((AlarmBits & 0b00000001) << 7));
  // Send A1 Minute and A1M2
  Wire.write(rtc_bin2bcd(A1Minute) | ((AlarmBits & 0b00000010) << 6));
  // Figure out A1 hour
  if (A1h12) {
    // Start by converting existing time to h12 if it was given in 24h.
    if (A1Hour > 12) {
      // well, then, this obviously isn't a h12 time, is it?
      A1Hour = A1Hour - 12;
      A1PM = true;
    }
    if (A1PM) {
      // Afternoon
      // Convert the hour to BCD and add appropriate flags.
      temp_buffer = rtc_bin2bcd(A1Hour) | 0b01100000;
    } else {
      // Morning
      // Convert the hour to BCD and add appropriate flags.
      temp_buffer = rtc_bin2bcd(A1Hour) | 0b01000000;
    }
  } else {
    // Now for 24h
    temp_buffer = rtc_bin2bcd(A1Hour);
  }
  temp_buffer = temp_buffer | ((AlarmBits & 0b00000100) << 5);
  // A1 hour is figured out, send it
  Wire.write(temp_buffer);
  // Figure out A1 day/date and A1M4
  temp_buffer = ((AlarmBits & 0b00001000) << 4) | rtc_bin2bcd(A1Day);
  if (A1Dy) {
    // Set A1 Day/Date flag (Otherwise it's zero)
    temp_buffer = temp_buffer | 0b01000000;
  }
  Wire.write(temp_buffer);
  Wire.endTransmission();
}

void RTC_DS3231_turnOnAlarm(byte Alarm) {
//---------------------------------------
// turns on alarm number "Alarm". Defaults to 2 if Alarm is not 1.
  byte temp_buffer = i2c_readRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG);
  // modify control byte
  if (Alarm == 1) {
    temp_buffer = temp_buffer | 0b00000101;       // bitwise OR //either or both
  } else {
    temp_buffer = temp_buffer | 0b00000110;       // Defaults to enable 2 if Alarm is not 1
  }
  i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG,temp_buffer);
}

void RTC_DS3231_ResetBothAlarmFlags() {             // from http://forum.arduino.cc/index.php?topic=109062.0
//-----------------------------------
  byteBuffer1=i2c_readRegisterByte(DS3231_ADDRESS,DS3231_STATUS_REG);
  byteBuffer1 &= B11111100;                       //change the target bits //with '&=' only the 0's affect the target byte // with '|=' only the 1's will set
  i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_STATUS_REG,byteBuffer1);
  rtc_INT0_Flag = false;                          //clear the flag we use to indicate the RTC alarm occurred
}

uint32_t RTC_DS3231_unixtime() { 
//-----------------------------------
// call this function AFTER RTC_DS3231_getTime updates the global t_ variables
  uint32_t t;
  uint16_t days = rtc_date2days(t_year, t_month, t_day);
  t = rtc_time2long(days, t_hour, t_minute, t_second);
  t += 946684800;                                 // add # seconds from 1970 to 2000 which we took away with y -= 2000
  return t;
}

//Calculation / Conversion functions called by RTC_DS3231 functions
//-----------------------------------------------------------------

const uint8_t rtc_days_in_month [12] PROGMEM = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

static uint8_t rtc_bcd2bin (uint8_t val) {
  return val - 6 * (val >> 4);
}

static uint8_t rtc_bin2bcd (uint8_t val) {
  return val + 6 * (val / 10);
}

static long rtc_time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
  return ((days * 24L + h) * 60 + m) * 60 + s;
}

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t rtc_date2days(uint16_t y, uint8_t m, uint8_t d) {
  if (y >= 2000)
    y -= 2000;
  uint16_t days = d;
  for (uint8_t i = 1; i < m; ++i)
    days += pgm_read_byte(rtc_days_in_month + i - 1);
    
  if (m > 2 && (y % 4 == 0)){++days;} // if LeapYear add extra day
  
  return days + 365 * y + (y + 3) / 4 - 1;
}

// Other RTC refs:
// also see https://github.com/sleemanj/DS3231_Simple/blob/master/DS3231_Simple.cpp for an alt library
// also see https://github.com/mizraith/RTClib
// or https://github.com/akafugu/ds_rtc_lib for more DS3231 specific libs
// https://github.com/MajicDesigns/MD_DS3231/blob/master/src/MD_DS3231.cpp
// https://github.com/JChristensen/DS3232RTC wrapper for the time.h lib
// alternate unixtime calculation: https://github.com/rodan/ds3231
// http://tronixstuff.com/2014/12/01/tutorial-using-ds1307-and-ds3231-real-time-clock-modules-with-arduino/


// ===========================================================================================================
// ============================================================================================================
//   *  *   *  *  *  *  *  *  *  *  SENSOR SPECIFIC FUNCTIONS  *  *  *  *  *  *  *  *  *  *  *  *  *
// ============================================================================================================
// ===========================================================================================================

// ============================================================================================================
// SI7051 TEMPERATURE SENSOR - here as an 'example' but we may not use this particular sensor in the course
// ============================================================================================================
// datasheet at : http://www.silabs.com/Support%20Documents/TechnicalDocs/Si7050-1-3-4-5-A20.pdf 
// code from https://github.com/closedcube/ClosedCube_Si7051_Arduino/blob/master/src/ClosedCube_Si7051.cpp
// and we usually buy these sensors from ClosedCube on Tindie: https://www.tindie.com/stores/closedcube/

#ifdef readSi7051_Temp_2byte  //compiler will include these functions up to the next #endif statement
void initializeSI7051() {
//---------------------------------------------------------------------------------------------
  if(ECHO_TO_SERIAL){
  Serial.println(F("INIT: SI7051 sensor..."));Serial.flush(); 
  }

  Wire.beginTransmission(Si7051_Address);
  Wire.write(0xFE); //sensor reset
  Wire.endTransmission();
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
 
  Wire.beginTransmission(Si7051_Address);
  Wire.write(0xE6);                         // Command Code:  Write User Register 0xE6  //Command Code:  Read User Register 0xE7
  Wire.write(Si7051_Resolution);            // Only bit 7 &  bit 0 can be set by user
  byteBuffer1 = Wire.endTransmission();
  
  if ( byteBuffer1 != 0) {
        if(ECHO_TO_SERIAL){                 //if echo is on, we are in debug mode, and errors force a halt.
        Serial.print (F("FAIL Initial control reg write:si7051"));Serial.flush(); 
        }
    error_shutdown();
    byteBuffer1=0;
  }
}

uint16_t readSI7051() {    //Conversion time: 14-bit temps = 10 ms @ 120 μA, peak current during I2C operations 4.0 mA
//---------------------------------------------------------------------------------------------

  //first transaction starts reading
  Wire.beginTransmission(Si7051_Address);
  Wire.write(0xF3);  //Device will NACK the slave address byte until conversion is complete.
  //The measure temperature commands 0xE3 and 0xF3 will perform a temperature measurement and return the measurement value.  
  byteBuffer1 = Wire.endTransmission();
  
    if ( byteBuffer1 != 0) {
      if(ECHO_TO_SERIAL){                   //if echo is on, we are in debug mode, and errors force a halt.
      Serial.println(F("FAIL request data:si7051")); Serial.flush(); 
      error_shutdown();
      }
    byteBuffer1=0;
  } 
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);     // some of my sensors need settling time after a register change.
  // actual conversion times: 14-bit 10.8 ms //13-bit 6.2 ms //12-bit 3.8 ms //11-bit 2.4 ms

  //second I2C transaction requests the data from those memory registers
  Wire.requestFrom(Si7051_Address, 2);       //this sensor stores its output in two memory registers
  byte msb = Wire.read();
  byte lsb = Wire.read();
  uint16_t val= msb << 8 | lsb;             // merge the two output register bytes into one integer number
  return val;                               // note:  to calculate TEMP_degC =(175.26*val) / 65536 - 46.85;
}

#endif // end of code for SI7051 sensor included via controlling #ifdef / #endif statements
// ========================================================================================


#if defined(readD6ResistorwD8pullup_2byte) || defined(readD7resistorwD8pullup_2byte)
// ============================================================================================================
// ===========================================================================================================
// Based on Nick Gammon's frequency counter at https://www.gammon.com.au/forum/?id=11504
// Adapted by Edward Mallon for ratiometric reading of resistive sensors with ICU on D8. For a details see 
// https://thecavepearlproject.org/2019/03/25/using-arduinos-input-capture-unit-for-high-resolution-sensor-readings/
// NewSensorReading =(elapsedTimeSensor * referenceResistorValue) / elapsedTimeReff;
// with [104] caps this calculation fits within uint32_t variables
// HOWEVER w 105 capacitors must change read functions & cast this calculation to higher uint64_t bit depth
// D7resistor_NewReading=((uint64_t)elapsedTimeSensor * (uint64_t)referenceResistorValue) / elapsedTimeReff;
// FAST PORT COMMANDS used throughout these functions to increase timing accuracy

//-----------------------------------------------------------------------------------------
void ConditionCapacitorOnD8(){            // 2023-06-20: internal pullup resistor on D8 as ref- D6=LDR (not D9), D7=10kNTC
//-----------------------------------------------------------------------------------------
  // Charge & discharge [104] 0.1uF capacitor through 300ohm resistor on D8 (5xRC takes about 0.15 milliseconds each direction)
  // RC disharge time calculator: https://www.digikey.com/en/resources/conversion-calculators/conversion-calculator-time-constant
  // this cycles the capacitor to bring it to a 'standardised' residual level/condition
  // AND floats [Input & Low] all pins with resistor/sensors connected to that common capacitor 

  //bitSet(ACSR,ACD);                     // Disable the analog comparator done in setup
  ADCSRA = 0; SPCR = 0;                   // Disable ADC & SPI
  power_all_disable();                    // must come after disabling ADC
  power_timer1_enable();                  // need Timer1 to clock the change on the D8 pin

  //MUST float ALL pins with resistor/sensors connected to the common capacitor
  //or you wont be able to read the others!
  bitClear(PORTB,0);bitClear(DDRB,0);     //D8 [300Ω] to LOW(0) & INPUT(0) 
  PORTD &= B00111111;DDRD &= B00111111;   //D6  & D7 to LOW(0) & INPUT(0)
  
  bitSet(DDRB,0); //pinMode(8,OUTPUT)

  set_sleep_mode(SLEEP_MODE_IDLE);// this mode leaves Timer1 running @<1mA draw but wakes in 6 clock cycles
  prepareForInterrupts(); // sets triggered=false
  noInterrupts ();  sleep_enable();
  bitSet(PORTB,0); //digitalWrite(8,HIGH);
  do{
    interrupts(); sleep_cpu(); noInterrupts(); 
    }while(!triggered); // NOTHING IS RECORDED during this 1st capacitor charge up
  interrupts ();
  // sleep_disable(); is inside ISR (TIMER1_CAPT_vect)
  
  // sampling cap is now at its 66% of Vcc HIGH trigger point -> now discharge the cap through 300Ω on D8
  bitClear(PORTB,0);  // digitalWrite(8,LOW); 
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF); // + extra 2msec for osc start! //ADC_ON leaves the already sleeping ACD alone as is
  // NOTE: 15MS is overkill:  5T with 300ohm&105(1uF) is 1.5 msec, with 300Ω&104(100nF) 5RC is only 0.15ms 
  bitClear(DDRB,0);  // pinMode(8,INPUT); 

    //re-enable timers
    power_timer1_disable();    // no longer need Timer1
    power_timer0_enable();     // MUST RE-ENABLE for any delay/serial statements,etc
    power_twi_enable();        // cant use I2C bus without interrupts enabled
    if(ECHO_TO_SERIAL){ power_usart0_enable();}
    
  }

//-----------------------------------------------------------------------------------------
uint16_t ReadD6riseTimeOnD8(){            //2023-06-20: internal pullup resistor on D8 as ref- D6=LDR (not D9), D7=10kNTC
//-----------------------------------------------------------------------------------------
  noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_1;interrupts(); //this function must run a max clock speed

  //bitSet(ACSR,ACD);                     // Disable the analog comparator done in setup
  ADCSRA = 0; SPCR = 0;                   // Disable ADC & SPI
  power_all_disable();                    // must come after disabling ADC
  power_timer1_enable();                  // need Timer1 to clock the change on the D8 pin

  //MUST float ALL pins with resistor/sensors connected to the common capacitor
  //or you wont be able to read the others!
  bitClear(PORTB,0);bitClear(DDRB,0);     //D8 [300Ω] to LOW(0) & INPUT(0) 
  PORTD &= B00111111;DDRD &= B00111111;   //D6 [LDR] & D7[NTC] to LOW(0) & INPUT(0)

//========== read resistor RISING on D6 ===========
  set_sleep_mode(SLEEP_MODE_IDLE);
  prepareForInterrupts();
  noInterrupts(); sleep_enable();
  TCNT1 = 0;                    // reset Timer 1 counter value to 0
  bitSet(DDRD,6);               // D6 OUTPUT
  bitSet(PORTD,6);              // D6 HIGH -> now charging the cap through ref
  do{
  interrupts(); sleep_cpu(); noInterrupts();
  }while(!triggered);         // trapped here till TIMER1_CAPT_vect changes triggered to true

  //elapsedTime = timer1CounterValue; // capped in ISR (TIMER1_OVF_vect) at 65534
  interrupts ();                // can't use I2C bus or powerDown without interrupts...
  // sleep_disable() inside ISR (TIMER1_CAPT_vect)
  
    bitClear(DDRD,6); bitClear(PORTD,6); // D6 INPUT & LOW -> stops the capacitor charge
    bitSet(DDRB,0); //bitClear(PORTB,0); //D8 OUTPUT & (already) LOW to discharge the capacitor through 300Ω on D8
      LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF); // sleep processor during the discharge
    bitClear(DDRB,0);                 // D8 INPUT 
    
//re-enable timers after Dpin read:
    power_timer1_disable();    // no longer need Timer1
    power_timer0_enable();     // MUST RE-ENABLE for any delay/serial statements,etc
    power_twi_enable();        // cant use I2C bus without interrupts enabled
    if(ECHO_TO_SERIAL){
      power_usart0_enable();   
      Serial.print(F(" D6 Timer1:"));Serial.print(timer1CounterValue);;Serial.flush(); 
      }

return timer1CounterValue;
} // terminates ReadD6riseTime 
//-----------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------
uint16_t ReadD7riseTimeOnD8(){
//-----------------------------------------------------------------------------------------
  noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_1;interrupts(); //this function must run a max clock speed

  //bitSet(ACSR,ACD);                     // Disable the analog comparator done in setup
  ADCSRA = 0; SPCR = 0;                   // Disable ADC & SPI
  power_all_disable();                    // must come after disabling ADC
  power_timer1_enable();                  // need Timer1 to clock the change on the D8 pin

  //MUST float ALL pins with resistor/sensors connected to the common capacitor
  //or you wont be able to read the others!
  bitClear(PORTB,0);bitClear(DDRB,0);     //D8 [300Ω] to LOW(0) & INPUT(0) 
  PORTD &= B00111111;DDRD &= B00111111;   //D6 [LDR] & D7[NTC] to LOW(0) & INPUT(0)
  
//=== read the NTC thermistor(?) RISING on D7 ===========
  set_sleep_mode (SLEEP_MODE_IDLE); 
  prepareForInterrupts(); 
  noInterrupts(); sleep_enable(); 
  TCNT1 = 0; // reset Timer 1
  bitSet(DDRD,7); bitSet(PORTD,7);  // Pin D7 OUTPUT & HIGH  //now charging the cap through NTC thermistor
    do{
    interrupts(); sleep_cpu(); noInterrupts();
    }while(!triggered);
    interrupts ();    // cant use I2C bus without interrupts!
    // sleep_disable() inside ISR (TIMER1_CAPT_vect)
  
    bitClear(DDRD,7);bitClear(PORTD,7);   //  D7 to INPUT & LOW //stops capacitor charging
    bitSet(DDRB,0);bitClear(PORTB,0);     //  D8 OUTPUT LOW // discharge the cap through D8
    LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
    bitClear(DDRB,0);                     //  D8 INPUT 

//re-enable timers after Dpin read:
    power_timer1_disable();    // no longer need Timer1
    power_timer0_enable();     // MUST RE-ENABLE for any delay/serial statements,etc
    power_twi_enable();        // cant use I2C bus without interrupts enabled
    if(ECHO_TO_SERIAL){
      power_usart0_enable();   
      Serial.print(F(", D7 Timer1:"));Serial.print(timer1CounterValue);Serial.flush(); 
      }
    
  return timer1CounterValue;
} // terminates uint32_t ReadD7riseTime 
//=========================================================================

//2023-06-20: 300Ω + internal pullup resistor on D8 as reference resistor, D6=LDR, D7=10kNTC
// 1xRC is about 3.5milliseconds with 35000 Ω & 104(0.1uF) = 66% of Vcc rise time which is our high trigger point
// RC calculator: https://www.digikey.com/en/resources/conversion-calculators/conversion-calculator-time-constant

//-----------------------------------------------------------------------------------------
uint16_t ReadD8riseTimeOnD8(){
//-----------------------------------------------------------------------------------------

  noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_1;interrupts(); //run @ max clock speed
  //uint16_t elapsedTime=0;

  //bitSet(ACSR,ACD);                     // Disable the analog comparator done in setup
  ADCSRA = 0; SPCR = 0;                   // Disable ADC & SPI
  power_all_disable();                    // AFTER disabling ADC!
  power_timer1_enable();                  // ICU uses Timer1 to clock change on D8 pin

  //MUST float ALL pins with resistor/sensors connected to the common capacitor
  //or you wont be able to read the others!
  bitClear(PORTB,0);bitClear(DDRB,0);     //D8 [300Ω] to LOW(0) & INPUT(0) 
  PORTD &= B00111111;DDRD &= B00111111;   //D6 [LDR] & D7[NTC] to LOW(0) & INPUT(0)

//=== read the D8 internal pullup resistor RISING on D8 ===========
// charging capacitor takes about 1msec through 36k pullup
  set_sleep_mode (SLEEP_MODE_IDLE);
  prepareForInterrupts();
  noInterrupts(); sleep_enable(); TCNT1 = 0;
  bitSet(PORTB,0);  // Pin D8 HIGH (D8 is already input mode so this just enables the pullup resistor)
     do{
     interrupts(); sleep_cpu(); noInterrupts();
     }while(!triggered);
  //elapsedTime now = timer1CounterValue;
  interrupts(); // NOTE:  sleep_disable() happens in ISR (TIMER1_CAPT_vect)

  bitClear(PORTB,0);bitSet(DDRB,0); // D8 LOW & OUTPUT // discharges the capacitor through D8
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF); 
  bitClear(DDRB,0);                     //  D8 INPUT 

  //re-enable timers after Dpin read:
    power_timer1_disable();    // no longer need Timer1
    power_timer0_enable();     // MUST RE-ENABLE for any delay/serial statements,etc
    power_twi_enable();        // cant use I2C bus without interrupts enabled
      if(ECHO_TO_SERIAL){
      power_usart0_enable();   
      Serial.print(F(", D8(32k pullup) Timer1:"));Serial.print(timer1CounterValue);Serial.flush(); 
      }
    
    return timer1CounterValue;
} // terminates ReadD8riseTimeOnD8 
//-----------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------
void prepareForInterrupts() {
//-----------------------------------------------------------------------------------------
// based on Nick Gammon's frequency counter at https://www.gammon.com.au/forum/?id=11504

    noInterrupts ();       // protected code
    triggered = false;     // re-arm for do-while loop
    TCCR1A = 0;            // set entire TCCR1A register to 0
    TCCR1B = 0;            // same for TCCR1B
    TCNT1 = 0;             // initialize counter value to 0
    bitSet(TIMSK1,TOIE1);  // interrupt on Timer 1 overflow
    bitSet(TIMSK1,ICIE1);  // enable input capture
    bitSet(TCCR1B,CS10);   // No prescaling on Timer1 // set prescaler to 1x system clock (F_CPU)
    bitSet(TCCR1B,ICNC1);  // Activates Input Capture Noise Canceler=filter function requires four successive equal valued samples of the ICP1 pin for changing its output. 
    bitSet(TCCR1B,ICES1);  // ICES1: = 1 for trigger on RISING edge on pin D8
    TIFR1 = bit (ICF1) | bit (TOV1);  // clear flags so we don't get a bogus interrupt
    interrupts ();
}  // end of prepareForInterrupts()

// a 10k resistor only with 104 capacitor counts ~6000 ticks @8MHz - so should never reach T1 overflow in normal temp ranges
ISR (TIMER1_CAPT_vect){ 
//----------------------------------------------------------------------------------------- 
    if (triggered){return;}       // error catch for overflow or multiple triggers(?)
    timer1CounterValue = ICR1;    // transfer value held in Input Capture register
    triggered = true;
    bitClear(TIMSK1,ICIE1);       // disable input capture unit
    bitClear(TIMSK1,TOIE1);       // also disable interrupts on Timer 1 overflow
    sleep_disable();
}  // end of TIMER1_CAPT_vect

// NOTE when powering from a coincell you don't want the sensor readings to take too long
// so here we throttle the system here to only ONE overflow as an error catch with large resistors (like CDS cells in the dark)
// this fails with larger 105 capacitors that need more than 65535 clocks, but works OK with 104 caps
ISR (TIMER1_OVF_vect) {           // timer1 overflows (every 65536 system clock ticks = 122 times a second @8MHz)
//-----------------------------------------------------------------------------------------
    timer1CounterValue = 65534;   // (=8.19msec in SLEEP_MODE_IDLE ~1mA) sometimes I modify this max limit slightly while debugging...
    triggered = true;
    bitClear(TIMSK1,ICIE1);       // disable input capture unit
    bitClear(TIMSK1,TOIE1);       // also disable interrupts on Timer 1 overflow
    sleep_disable();
}

// ============================================================================================================
// ============================================================================================================
#endif //end of #if defined(readD6ResistorwD8pullup_2byte) || defined(readD7resistorwD8pullup_2byte)
// ============================================================================================================
// ============================================================================================================

// Indicator LED functions
// =======================
// IF no LED is defined at startup, the default Red ProMini LED on pin13 gets used
// Colors can be combined for the LED pip but Red 5mm LED is usually too dim to see when lit with pullup(?)
// Blue mixes well with green but adds another 50uA // Green is brightest // each channel draws 30-50uA
// ~50uA to light RED onboard led through D13s internal pullup resistor
// Port B on an Arduino controls digital pins 8–13  //Port C: Controls analog input pins 0–5

void turnOnGreenAndBlueLED(){               // HIGHEST VISIBILITY 'blue-green' pip ~100uA
    #if defined(LED_r9_b10_g11_gnd12)
      bitClear(PORTB,3); bitSet(DDRB,3);    // Common GND       // same as digitalWrite(12,LOW); pinMode(12,OUTPUT);
      bitClear(DDRB,1);  bitClear(PORTB,1); // D9 [Red] OFF     // same as pinMode(9,INPUT);  digitalWrite(9,LOW); 
      bitClear(DDRB,2);  bitSet(PORTB,2);   // D10 [Blue] ON    // same as pinMode(10,INPUT); digitalWrite(10,HIGH);
      bitClear(DDRB,3);  bitSet(PORTB,3);   // D11 [Green] ON   // same as pinMode(11,INPUT); digitalWrite(11,HIGH);
    #elif defined(LED_GndGB_A0_A2)
      bitClear(PORTC,0); bitSet(DDRC,0);    // A0 Common GND    // same as digitalWrite(A0,LOW); pinMode(A0,OUTPUT);
      bitClear(PORTC,1); bitSet(DDRC,1);    // A1 [Green] ON    // same as pinMode(A1,INPUT); digitalWrite(A1,HIGH);
      bitClear(DDRC,2);  bitSet(PORTC,2);   // A2 [Blue] ON     // same as pinMode(A2,INPUT); digitalWrite(A2,HIGH);
    #else
      bitClear(DDRB,5); bitSet(PORTB,5);    // Red ProMini LED on pin13 = ON // same as pinMode(13,INPUT_PULLUP); light default red led on D13
    #endif
  } 
void toggleBlueAndGreenLEDs(){              // Writing a 1 to any bits in the PIN control register TOGGLES the PULLUP RESISTOR on the associated pins
    #if defined(LED_r9_b10_g11_gnd12)       
      PINB = B00001100;                     // TOGGLES D10[Blue] pullup & D11[green] PULLUP resistors
    #elif defined(LED_GndGB_A0_A2)
      PINC = B00000110;                     // TOGGLES A2 [Blue] & A1 [green] PULLUP resistors
    #else
      PINB = B00100000;                     // TOGGLES [Red] ProMini LED PULLUP resistor
    #endif
  }
  
void turnOnRedAndBlueLED(){
    #if defined(LED_r9_b10_g11_gnd12)       // 'purple' in this configuration
      bitClear(PORTB,3); bitSet(DDRB,3);    // Common GND       // same as digitalWrite(12,LOW); pinMode(12,OUTPUT);
      bitClear(DDRB,1);  bitSet(PORTB,1);   // D9 [Red] ON      // same as pinMode(9,INPUT);  digitalWrite(9,HIGH); 
      bitClear(DDRB,2);  bitSet(PORTB,2);   // D10 [Blue] ON    // same as pinMode(10,INPUT); digitalWrite(10,HIGH);
      bitClear(DDRB,3);  bitClear(PORTB,3); // D11 [Green] OFF  // same as pinMode(11,INPUT); digitalWrite(11,LOW);
    #elif defined(LED_GndGB_A0_A2)          // this configuration does not have RED channel embedded inside the 5mm LED!
      bitClear(PORTC,0); bitSet(DDRC,0);    // A0 Common GND    // same as digitalWrite(A0,LOW); pinMode(A0,OUTPUT);
      bitClear(PORTC,1); bitClear(DDRC,1);  // A1 [Green] OFF   // same as pinMode(A1,INPUT); digitalWrite(A1,LOW);
      bitClear(DDRC,2);  bitSet(PORTC,2);   // A2 [Blue] ON     // same as pinMode(A2,INPUT); digitalWrite(A2,HIGH);
      bitClear(DDRB,5);  bitSet(PORTB,5);   // Red ProMini LED on pin13 = ON // same as pinMode(13,INPUT_PULLUP); light default red led on D13
    #else
      bitClear(DDRB,5);  bitSet(PORTB,5);   // Red ProMini LED on pin13 = ON // same as pinMode(13,INPUT_PULLUP); light default red led on D13
    #endif
  }
void toggleRedandBlueLEDs(){                // Writing a 1 to any bit in PIN control register TOGGLES the PULLUP RESISTOR on the associated pins
    #if defined(LED_r9_b10_g11_gnd12)       
      PINB = B00000110;                     // TOGGLES D10[Blue] pullup & D9 [Red]PULLUP resistors
    #elif defined(LED_GndGB_A0_A2)
      PINC = B00000100;                     // TOGGLES A2 [Blue] PULLUP resistors [in 5mm RBG]
      PINB = B00100000;                     // TOGGLES [Red] ProMini LED PULLUP resistor [on ProMini]
    #else
      PINB = B00100000;                     // TOGGLES [Red] ProMini LED PULLUP resistor
    #endif
  }
   
void turnOnGreenLED(){
    #if defined(LED_r9_b10_g11_gnd12)
      bitClear(PORTB,3); bitSet(DDRB,3);    // Common GND       // same as digitalWrite(12,LOW); pinMode(12,OUTPUT);
      bitClear(DDRB,1);  bitClear(PORTB,1); // D9 [Red]   OFF   // same as pinMode(9,INPUT);  digitalWrite(9,LOW);
      bitClear(DDRB,2);  bitClear(PORTB,2); // D10 [Blue] OFF   // same as pinMode(10,INPUT); digitalWrite(10,LOW );
      bitClear(DDRB,3);  bitSet(PORTB,3);   // D11 [Green] ON   // same as pinMode(11,INPUT); digitalWrite(11,HIGH);
    #elif defined(LED_GndGB_A0_A2)
      bitClear(PORTC,0); bitSet(DDRC,0);    // A0 Common GND    // same as digitalWrite(A0,LOW); pinMode(A0,OUTPUT);
      bitClear(DDRC,1);  bitSet(PORTC,1);   // A1 [Green] ON    // same as pinMode(A1,INPUT); digitalWrite(A1,HIGH);
      bitClear(DDRC,2);  bitClear(PORTC,2); // A2 [Blue] OFF    // same as pinMode(A1,INPUT); digitalWrite(A1,LOW);
    #else
      bitClear(DDRB,5);  bitSet(PORTB,5);   // Red ProMini LED on pin13 = ON // same as pinMode(13,INPUT_PULLUP); light default red led on D13
    #endif
  } 
  
void turnOnBlueLED(){
    #if defined(LED_r9_b10_g11_gnd12)
      bitClear(PORTB,3); bitSet(DDRB,3);    // Common GND       // same as digitalWrite(12,LOW); pinMode(12,OUTPUT);
      bitClear(DDRB,1);  bitClear(PORTB,1); // D11 [Red] OFF    // same as pinMode(9,INPUT);  digitalWrite(9,LOW);
      bitClear(DDRB,2);  bitSet(PORTB,2);   // D10 [Blue] ON    // same as pinMode(10,INPUT); digitalWrite(10,HIGH);
      bitClear(DDRB,3);  bitClear(PORTB,3); // D11 [Green] OFF  // same as pinMode(11,INPUT); digitalWrite(11,LOW);
 
    #elif defined(LED_GndGB_A0_A2)
      bitClear(PORTC,0); bitSet(DDRC,0);    // A0 Common GND    // same as digitalWrite(A0,LOW); pinMode(A0,OUTPUT);
      bitClear(DDRC,1);  bitClear(PORTC,1); // A1 [green] OFF   // same as pinMode(A1,INPUT); digitalWrite(A1,LOW);
      bitClear(DDRC,2);  bitSet(PORTC,2);   // A2 [Blue] ON     // same as pinMode(A2,INPUT); digitalWrite(A2,HIGH);
    #else
      bitClear(DDRB,5); bitSet(PORTB,5);    // Red ProMini LED on pin13 = ON // same as pinMode(13,INPUT_PULLUP); light default red led on D13
    #endif
  }    

void turnOnRedLED(){
    #if defined(LED_r9_b10_g11_gnd12)
      DDRB &= B11110001;                    // set 3 pins to INPUT 
      bitClear(PORTB,2);                    // D10 [Blue] PULLUP OFF
      bitClear(PORTB,3);                    // D11 [Green] PULLUP OFF
      bitSet(PORTB,1);                      // D9 [Red] PULLUP ON 
      bitClear(PORTB,4); bitSet(DDRB,4);    // D12 GND pin LOW & OUTPUT
    #elif defined(LED_GndGB_A0_A2)
      DDRC &= B11111001;                    // set both A1 A2 pins to INPUT
      bitClear(PORTC,1);                    // A1 [Green] PULLUP OFF
      bitClear(PORTC,2);                    // A2 [Blue] PULLUP OFF
      bitClear(DDRB,5); bitSet(PORTB,5);    // same as pinMode(13,INPUT_PULLUP); light default red led on D13
    #else
      bitClear(DDRB,5); bitSet(PORTB,5);    // same as pinMode(13,INPUT_PULLUP); light default red led on D13 
    #endif
  }
void turnOffAllindicatorLEDs(){
    #if defined(LED_r9_b10_g11_gnd12)
      bitClear(PORTB,1); bitClear(DDRB,1);  // D11 [Red]    OFF  // same as pinMode(9,INPUT);
      bitClear(PORTB,2); bitClear(DDRB,2);  // D10 [Blue]   OFF  // same as pinMode(10,INPUT);
      bitClear(PORTB,3); bitClear(DDRB,3);  // D11 [Green]  OFF  // same as pinMode(11,INPUT);
    #elif defined(LED_GndGB_A0_A2)
      bitClear(PORTC,1); bitClear(DDRC,1);  // A1 [green] OFF // same as pinMode(A1,INPUT); digitalWrite(A1,LOW);
      bitClear(PORTC,2); bitClear(DDRC,2);  // A2 [Blue]  OFF // same as pinMode(A2,INPUT); digitalWrite(A2,LOW);
      bitClear(PORTB,5);                    // pin13 red promini LED pullup Off
   #endif
      bitClear(PORTB,5);                    // pin13 red promini LED pullup Off
  }
//=======================================================================
// Checks variable RAM availiable on 328p processor
//=======================================================================
// from: http://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory

#ifdef logFreeVariableMemory_2byte
int freeRam ()     
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
#endif // logFreeVariableMemory_2byte

// DRY: Don't Repeat Yourself! -> use a function for anything you do often:
//=========================================================================
void clearSerialInputBuffer(){  // clears leftover bytes from incoming serial buffer
  while(Serial.available()){Serial.read();} 
  }
void sendMultiAscii2serial(uint8_t repeats,uint8_t asciiCode){
      for (uint8_t t = 0; t < repeats; t++){
      Serial.write(asciiCode);
      }
  }
