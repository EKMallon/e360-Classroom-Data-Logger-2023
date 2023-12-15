// 2-module logger code by Edward Mallon - modified 2023 for the e360 course at Northwestern University
// https://thecavepearlproject.org/2023/12/01/the-e360-a-classroom-data-logger-for-science/
/*
This program supports an ongoing series of DIY 'Classroom Logger' tutorials from the Cave Pearl Project. 
The goal is to provide a starting point for self-built student projects in environmental monitoring.
This low power 2-module iteration runs from a CR2032 coin cell and uses EEprom memory to store sensor readings. 
Data download & logger control are managed through the IDE's serial monitor window at 500000 baud. 
The logger WILL NOT START until those serial handshakes are completed via a UART connection.
The most important rule to follow when adding new sensors is that this code can only accept 1, 2, 4, 8 or 16 bytesPerRecord.
These 'powers of 2' fit in the I2C buffer AND divide evenly into the EEproms hardware page size to prevent wrap-around.
*/

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// STEP0 :
// Create SENSOR definitions HERE to match the sensors you added to the logger
// Use these as global controls to enable/disable sensor code with #ifdef & #endif
// bytesPerRecord for your sensor combination MUST TOTAL 1,2,4,8 or 16
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#define PIRtriggersSensorReadings         // 4-bytes: Still in beta!   Do not enable this with countPIReventsPerSampleInterval - choose one or the other
// does NOT use the regular RTC-alarm based sampling interval but instead records the seconds elapsed between EVERY PIR trigger event in a uint32_t long variable [uint16_t would overflow at ~18 hours]
// WARNING this can use alot of memory very quickly! - recommend use with larger eeprom memory attached
// PIRtriggersSensorReadings could be enabled with four other bytes of sensor data [for a total of 8 bytes per record] OR with another 12 bytes of sensor data for a total of 16 bytes per record.

// LowestBattery & RTC_Temperature are the 2 byte 'base values' which are generally recorded with every sensor reading (as they require no extra sensor hardware beyond the logger itself)
#define logLowestBattery                    // 1-byte (compressed): saves LowestBattery recorded during operation
#define logRTC_Temperature                  // 1-byte: the RTC's internal 0.25°C resolution temperature sensor

//#define countPIReventsPerSampleInterval   // 2-bytes:  saves # of PIR HIGH events in a specified sample interval. Do not enable this with PIRtriggersSensorReadings - choose one or the other

//#define readNTC_onD7                      // 2-bytes: ohms // for explanation of the method for reading analog resistance with digital pins see
//#define readLDR_onD6                      // 2-bytes: ohms // https://thecavepearlproject.org/2019/03/25/using-arduinos-input-capture-unit-for-high-resolution-sensor-readings/
                                            // these have to match the connections shown in the build lab!

//#define bh1750_Address 0x23               // 2-bytes: raw sensor output: gets converted to Lux during download

//#define recordBMPtemp                     // 2-bytes
//#define recordBMPpressure                 // 2-bytes
//#define recordBMPaltitude                 // 2-bytes

//#define logCurrentBattery on              // 2-byte: saves CurrentBattery in sensor records - could be compressed...
//#define Si7051_Address 0x40               // 2-bytes: si7051 is often used for NTC calibration

//#define OLED_64x32_SSD1306      // not a sensor, but enabled with define to include needed library - requires 1000uF rail capacitor!-

#define EEpromI2Caddr 0x57                  // Run a bus scanner to check where your eeproms are https://github.com/RobTillaart/MultiSpeedI2CScanner
#define EEbytesOfStorage 4096               // Default: 0x57 / 4096 bytes for 4k on RTC module // 32k I2C EEprom Module: set to 0x50 & 32768   // note EEmemoryPointr would have to got be uint32_t for 64k eeproms

// Adjust the following to match your logger:
const char loggerConfiguration[] PROGMEM = "YOUR NAME HERE e360/es390 logger,1126400(change this to match yours),4kEEprom,NTC[7]&LDR[6]NOref104cap,LED_r9_b10_g11_gnd12";
const char deploymentDetails[] PROGMEM =   "Lab09: PIR sensor";

//#ifdef readNTC_onD7                         // also enable matching define in setup_sendboilerplate2serialMonitor() to print this info with your data
// const char sensorCalibrationInfo[] PROGMEM = "No cal. for NTC sensor yet...";
// after calbration something like: PROGMEM = "YourName_03,Calibrated 202202:,A=,0.001246979,B=,0.000216248,C=,0.0000001537444523,R(25°C)=9834.81Ω,β=3850.83K,RTCy=0.9744x -0.4904";
//#endif

#define LED_r9_b10_g11_gnd12 installed      // enables code for RGB indicator LED //1k limit resistor on shared GND line! // Red LED on D13 gets used as indicator if this #define is commented out

#include <Wire.h>       // I2C bus coms library: RTC, EEprom & Sensors
#include <EEPROM.h>     // note: requires default promini bootloader (ie NOT optiboot)
#include <avr/power.h>  // library for shutting down 328p chip peripherals to lower runtime current
#include <avr/sleep.h>  // provides SLEEP_MODE_ADC to lower current during ADC readings in readBattery() function
#include <LowPower.h>   // for interval & battery recovery sleeps

// Ref, Interval & Echo are reset via serial monitor input - so the values here don't matter
//-------------------------------------------------------------------------------------------------
int32_t InternalReferenceConstant = 1126400;  // default = 1126400L = 1100mV internal vref * 1024 // gets changed in setup via serial menu input option later
                                              // adding/subtracting 400 from the constant raises/lowers the 'calculated' result from readBattery() by ~1 millivolt,
                                              // simply read the rail with a DVM while running on UART power and change the constant until the calculation is accurate
uint8_t SampleIntervalMinutes = 15;           // Allowed values: 1,2,3,5,10,15,20,30 for both - must divide equally into 60!
uint8_t SampleIntervalSeconds = 0;            // minutes must be zero for intervalseconds, seconds must be zero for intervalMinutes
                                              // NOTE: Make sure your sensor readings don't take longer than your sample interval!
                                              // If you over-run your alarm because the sensor took too long you will have to wait 24hours for next wakeup

bool ECHO_TO_SERIAL = false;                  // true enables multiple print statements throughout the code via if(ECHO_TO_SERIAL){} // also starts the run with no interval sync delay so timestamps are misaligned

// most VARIABLES below this point stay the SAME on all machines:
//---------------------------------------------------------------------------
#define fileNAMEonly (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__) //from: https://stackoverflow.com/questions/8487986/file-macro-shows-full-path
const char compileDate[] PROGMEM = __DATE__;  //  built-in function in C++ makes text string: Jun 29 2023
const char compileTime[] PROGMEM = __TIME__;  //  built-in function in C++ makes text string: 10:04:18

uint8_t bytesPerRecord = 0;                   // INCREMENTED at the beginning of setup to match #defined sensors. MUST divide evenly into EEprom Page buffer AND fit inside I2C buffer
uint32_t EEmemoryPointr = 0;                  // counter that advances through the EEprom memory locations by bytesPerRecord at each pass through the main loop

//defines & variables for ADC & readbattery() function
//------------------------------------------------------------------------------
uint16_t CurrentBattery = 0;
uint16_t LowestBattery = 5764;                                  
uint16_t systemShutdownVoltage = 2795;        // MUST be > BrownOutDetect default of 2775mv (which is also the EEprom voltage limit)
byte default_ADCSRA,default_ADMUX;            // stores default ADC controll register settings for peripheral shut down
byte set_ADCSRA_2readRailVoltage, set_ADMUX_2readRailVoltage; // stores custom settings for readbattery() via 1.1 internal band gap reference
volatile uint8_t adc_interrupt_counter;       // incremented in readADCLowNoise ISR to calculate average of multiple ADC readings

//defines & variables for DS3231 RTC
//------------------------------------------------------------------------------
#define rtcAlarmInputPin 2                    // DS3231's SQW output is connected to interrupt0 pin D2 on the ProMini
#define DS3231_ADDRESS     0x68               // this is the I2C bus address of our RTC chip
#define DS3231_STATUS_REG  0x0F               // reflects status of internal operations
#define DS3231_CONTROL_REG 0x0E               // enables or disables clock functions
#define DS3231_TMP_UP_REG  0x11               // temperature registers (upper byte 0x11 & lower 0x12) gets updated every 64sec
uint8_t AlarmSelectBits;                      // sets which parts of time to use or ignore for nxt alarm // ALRM1_SET bits and ALRM2_SET are 0b1000 and 0b0111 respectively.
//uint32_t loggerStartTime;                     // uint32_t is large enough to hold the 10-digit unixtime number
char CycleTimeStamp[] = "0000/00/00,00:00:00";   //16 character array to store human readble time (with seconds)
uint8_t t_second,t_minute,t_hour,t_day,t_month; // current time variables populated by calling RTC_DS3231_getTime()
uint16_t t_year;                              //current year //note: yOff = raw year to which you need to add 2000
uint8_t Alarmday,Alarmhour,Alarmminute,Alarmsecond; // calculated variables for setting next alarm
volatile boolean rtc_INT0_Flag = false;       // used in startup time sync delay //volatile because it's changed in an ISR // rtc_d2_alarm_ISR() sets this boolean flag=true when RTC alarm wakes the logger
float rtc_TEMP_degC = 0.0;

// temporary 'buffer' variables only used during calculations
//------------------------------------------------------------------------------
bool booleanBuffer;                           // boolean for functions that return a true/false or 1/0
uint8_t byteBuffer1 = 9;                      // 1-byte (8 bit) type = unsigned number from 0 to 255
uint8_t byteBuffer2 = 9;                      // note: uint8_t is the same as byte variable type
int16_t integerBuffer = 9999;                 // 2-byte from -32,768 to 32,767
uint16_t uint16_Buffer= 9999;                 // 2-byte from 0 to 65535
//int32_t int32_Buffer = 9999;                // 4-byte from -2,147,483,648 to 2,147,483,647
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

#ifdef readNTC_onD7 
//------------------
  uint32_t NTC_NewReading;                // max of 65535 limits our ability to measure 10kNTC at temps below zero C!
#endif

#ifdef readLDR_onD6 
//------------------
  uint32_t LDR_NewReading;                // 65535 limit
#endif
#if defined(readNTC_onD7) || defined(readLDR_onD6)
//------------------------------------------------
  #define referenceResistorValue 36000UL    //  ARBITRARY value that should be 'close' to actual but precise value is not required  //https://hackingmajenkoblog.wordpress.com/2016/08/12/measuring-arduino-internal-pull-up-resistors/
  volatile boolean triggered;               //  volatiles needed for all digital pin reading of resistance:        
  volatile uint16_t timer1CounterValue;     //  prepareForInterrupts(), ISR (TIMER1_OVF_vect), ISR (TIMER1_CAPT_vect),ReadD6riseTimeOnD8
#endif

#ifdef bh1750_Address 
//------------------------------------------------------------------------------
  #include <hp_BH1750.h>                    // from  https://github.com/Starmbi/hp_BH1750 returns the sensor to sleep automatically after each read & supports auto-ranging.
  hp_BH1750 bh1750;                         // Instantiate a BH1750FVI library object
  int16_t lux_BH1750_RawInt;               // raw reading before conversion to lux // 2-byte, 0 to 65535
#endif

#if defined(recordBMPtemp) || defined(recordBMPpressure) || defined(recordBMPaltitude)
//-----------------------------------------------------------------------------------
  #include <BMP280_DEV.h>                 // Include the BMP280_DEV.h library  // NOTE: this library has disappeared from github?
  BMP280_DEV bmp280;                      // Instantiate (create) a BMP280_DEV object and set-up for I2C operation
  #define BMP280_Address 0x76
  float Bmp280_Temp_degC, Bmp280_Pr_mBar, Bmp280_altitude_m;  // Variables for sensor output
#endif

#ifdef Si7051_Address 
//------------------------------------------------------------------------------
  // we are not using a library - init and read functions for si7051 at end of program
  uint16_t TEMP_si7051=0;                  //NOTE sensors output overruns this uint16_t at 40C!
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
// STEP2 : Adjust the bytesPerRecord variable to match the #bytes in your
// sensor variables that will be saved to EEprom at each sampling interval
// bytesPerRecord must be 1,2,4,8 or 16 bytes because of EEprom page boundaries
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  
  #ifdef logLowestBattery
    bytesPerRecord = bytesPerRecord + 1;            // NOW INDEX-compressed to 1-Byte [with slight loss of resolution]
  #endif
  #ifdef logRTC_Temperature
    bytesPerRecord = bytesPerRecord + 1;            // NOW INDEX-compressed to 1-Byte of eeprom storage
  #endif

  #ifdef countPIReventsPerSampleInterval
    bytesPerRecord = bytesPerRecord + 2;            //  two-byte integer counts Rising of output channel of PIR sensor
  #endif

  #ifdef PIRtriggersSensorReadings
    bytesPerRecord = bytesPerRecord + 4;            //  4-byte integer: d3_INT1_elapsedSeconds
  #endif
  
  #ifdef readNTC_onD7 
    bytesPerRecord = bytesPerRecord + 2;            //  two-byte integer: NTC ohms
  #endif
  #ifdef readLDR_onD6
    bytesPerRecord = bytesPerRecord + 2;            //  two-byte integer: LDR ohms
  #endif

  #ifdef bh1750_Address
    bytesPerRecord = bytesPerRecord + 2;           // two-byte integer for RAW reading before conversion to lux
  #endif
  
  #ifdef recordBMPtemp
    bytesPerRecord = bytesPerRecord + 2;            // two-byte integer 
  #endif
  #ifdef recordBMPpressure
    bytesPerRecord = bytesPerRecord + 2;            // two-byte integer 
  #endif
  #ifdef recordBMPaltitude
    bytesPerRecord = bytesPerRecord + 2;            // two-byte integer 
  #endif
  
  #ifdef logCurrentBattery
    bytesPerRecord = bytesPerRecord + 2;            // two-byte integer
  #endif
  #ifdef Si7051_Address
    bytesPerRecord = bytesPerRecord + 2;            // two-byte integer  
  #endif
  

// General Startup housekeeping: Set UNUSED digital pins to a known state at startup to reduce current & noise
//------------------------------------------------------------------------------------------------------------
   
    pinMode(13, INPUT);       // turn of D13 onboard red LED by setting D13 to INPUT & LOW
  #ifdef LED_r9_b10_g11_gnd12                       // we will use INPUT & PULLUP resistor to PIP the leds to reduce current
    for (int i = 9; i <=12; i++) { digitalWrite(i, LOW);  pinMode(i, INPUT); }
    pinMode(12, OUTPUT);                            //the common ground line on our RGB led must OUTPUT to allow current
  #endif
                         
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
   DIDR0 = 0x0F;                                    // diconnects the DIGITAL inputs sharing analog pins 0..3 (but NOT on 4&5 which are used by I2C as digital pins) //Once disabled, a digitalRead on those pins will always return zero.
                                                    // Digital input circuits can 'leak' a relatively high amount of current if the analog input is approximately half-Vcc 

  analogReference(DEFAULT); analogRead(A3);         // sets the ADC channel to A3 input pin
  default_ADCSRA = ADCSRA; default_ADMUX = ADMUX;   // Saves the DEFAULT ADC control registers into byte variables so we can restore those ADC control settings later

  // Set the ADC system clock prescalar to 32 so it operates at 2x the normal speed note: readings at 2x are nearly identical to 1x speed readings
  bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,0);bitWrite(ADCSRA,ADPS0,1); // https://www.arduino.cc/reference/en/language/functions/bits-and-bytes/bitwrite/
  set_ADCSRA_2readRailVoltage = ADCSRA;             // store the modified ADCSRA register values for use in readBattery() function & while saving eeprom data

  // modify ADC settings to reading the battery/rail voltage using the internal 1.1v reference inside the 328p chip 
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0); // from https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  set_ADMUX_2readRailVoltage = ADMUX;               // store modified ADMUX register values in a variable for use in readBattery()

  ADMUX = default_ADMUX;                            //restore the default
  ADCSRA = 0; power_adc_disable();                  //turn off the ADC to save power At 3V @ 25°C, the ADC consumes ~87µA
  

// Configure the DS3231 Real Time Clock control registers for coincell powered operation
//------------------------------------------------------------------------------------------------------------
  Wire.begin();                                     // Start the I2C bus // enables internal 30-50k pull-up resistors on SDA & SCL by default
  
  // i2c_setRegisterBit function requires: (deviceAddress, registerAddress, bitPosition, 1 or 0)
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_STATUS_REG, 3, 0);  // disable the 32khz output  pg14-17 of datasheet  // This does not reduce the sleep current but can't run because we cut VCC
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 6, 1); // 0Eh Bit 6 (Battery power ALARM Enable) - MUST set to 1 for wake-up alarms when running from the coincell bkup battery
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 7, 0); // Enable Oscillator (EOSC).  This bit is clear (logic 0) when power is first applied.  EOSC = 0 IS REQUIRED with OUR LOGGER DESIGN:
                                                                // when EOSC (bit7) is 0, the RTC oscillator continues running during battery powered operation. Otherwise it would stop.

  RTC_DS3231_turnOffBothAlarms();                               // stops RTC from holding the D2 interrupt line low if system reset just occured 
  digitalWrite(2, LOW);  pinMode(2, INPUT);                     // D2 INPUT & D2 pullup off because it is not requried with 4k7 hardware pullups on the RTC module
  bitSet(EIFR,INTF0); bitSet(EIFR,INTF1);                       // clears any previous trigger-flags inside the 328p processor for interrupt 0 (D2) &  interrupt 1 (D3)


// Check Previous run Parameters stored in 328p eeprom [ update happens only on 1st run of a brand new logger]
//------------------------------------------------------------------------------------------------------------
  EEPROM.get(4,InternalReferenceConstant);                      //in this case .get is reading 4 consecutive bytes into the long (32bit) integer InternalReferenceConstant
  if((InternalReferenceConstant<1000000) || (InternalReferenceConstant>1228800)){ //if value stored in eeprom is outside normal operating parameters
    InternalReferenceConstant=1126400;                          // then re-sets it to the default 1126400
  EEPROM.put(4,InternalReferenceConstant);}                     // and store that default back in the eeprom
  
  SampleIntervalMinutes = EEPROM.read(8);                       // retrieve 'previous' sampling interval data stored in the CPU's internal 1024 bytes of eeprom space
  SampleIntervalSeconds = EEPROM.read(9);                       // these numbers will be random the first time the logger is run because the EEprom memory locations are empty
  if((SampleIntervalMinutes>60) || (SampleIntervalSeconds>30))  //if values read from eeprom are outside allowed maximums then reset to 'safe' default values
    { SampleIntervalMinutes=15;SampleIntervalSeconds=0;
      EEPROM.update(8,SampleIntervalMinutes); 
      EEPROM.update(9,SampleIntervalSeconds); }                 // .update is the same as .put, except that it only writes the data if there is a change - eeprom has a limited # of write cycles

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
      Serial.print(F("Erasing EEprom: Stay on UART power until done"));
      //------------------------------------------------------------------------------
                
      for (uint32_t memoryLocation=0; memoryLocation<EEbytesOfStorage; memoryLocation+=16){  // loop writes 16-bytes at a time into the I2C buffer
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
          
      }   //terminates: for (int memoryLocation=0; memoryLocation<EEbytesOfStorage; memoryLocation+=16){
     
          Serial.println(); goFlagReceived=true;
          
    } else if(command == "test")                // 'test' is a HIDDEN option not on the displayed menu
           {goFlagReceived=true;}               // 'test' simply skips erasing eeprom: I only use this to make debugging faster
 
  if(goFlagReceived) break;                     // breaks out of the while loop when goFlagReceived=true;
 
}while ((millis() - startMillis) < 200000);     // terminates the do-while after 200 seconds BEFORE loop times out with goFlagReceived=false which leads to logger shutdown

if (!goFlagReceived){                           // if goflag=false then the loop timed out so shut down the logger
      Serial.println(F("Timeout with NO command recieved -> logger shutting down"));
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

// no initialization needed for readNTC_onD7 or readLDR_onD6

// BH1750 initialization
//-----------------------
#ifdef bh1750_Address             //using library:  https://github.com/Starmbi/hp_BH1750  
  bh1750.begin(bh1750_Address); // set address & initialize
      //bh1750.calibrateTiming();     // NOTE: ambient light must be at least 140 lux when calibrateTiming runs!
      //Serial.println(F("BH1750 light sensor MUST be exposed to >150 lux @ startup")); 
      //Serial.println(F("for self-cal or it may freeze randomly. 15 sec = minimum interval for this sensor"));Serial.println();Serial.flush();
  bh1750.start(BH1750_QUALITY_LOW, BH1750_MTREG_LOW); // Quality LOW = fastest readings
      // QUALITY_HIGH -Resolution Mode Measurement Time 120-180 ms depending on light levels
      // QUALITY_LOW  -Resolution Mode Measurement Time 16-24 msec //LOW MTreg:31  resolution lux:7.4, 121557 is highest lux

  Serial.print(F("BH1750 started,"));Serial.flush();
#endif // terminates bh1750 init.

// BMP280 initialization
//-----------------------
#if defined(recordBMPtemp) || defined(recordBMPpressure) || defined(recordBMPaltitude)
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
  LowPower.powerDown(SLEEP_30MS, ADC_ON, BOD_OFF);            // 60MSEC = long enough for max rezolution settings
  bmp280.getCurrentMeasurements(Bmp280_Temp_degC,Bmp280_Pr_mBar,Bmp280_altitude_m);

  Serial.print(F("BMP280 started,"));Serial.flush();

#endif // terminates BMP280 init.

#ifdef Si7051_Address
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
        
      byteBuffer2 = Alarmsecond-t_second; byteBuffer1 = Alarmminute - t_minute;
      if(Alarmsecond>59){Alarmsecond = 0; Alarmminute=Alarmminute+1; byteBuffer2 =60-t_second; byteBuffer1=0;}
      
      }else{ // for minute alarms

      Alarmminute = t_minute; Alarmsecond = 0; byteBuffer2 =0;  
      if(t_second>58){Alarmminute = Alarmminute + 1;} // and extra buffer if we are too close to rollover
         
      do{ 
        Alarmminute = Alarmminute + 1;
      }while(Alarmminute % SampleIntervalMinutes); // all non-zero results considered true // forces alignment
      byteBuffer1 = Alarmminute-t_minute;
      }
  Serial.print(F("Start-up Sync Delay: "));Serial.print(byteBuffer1);Serial.print(F("m "));Serial.print(byteBuffer2);Serial.println(F("s "));Serial.flush();
  if (Alarmminute > 59 ){ Alarmhour = Alarmhour+1; Alarmminute = 0;}  // alt Alarmminute = SampleIntervalMinutes? for longer delay if started at rollover
  if (Alarmhour > 23)   { Alarmday = Alarmday+1; Alarmhour = 0;}

  // NOW set the 1st -ALIGNED- wakeup alarm:
  AlarmSelectBits = 0b00001100;             // A1 Alarm when minutes AND seconds match, ignores days, hours
  RTC_DS3231_setA1Time(0, 0, Alarmminute, Alarmsecond, AlarmSelectBits, 0, 0, 0);
  RTC_DS3231_turnOnAlarm(1);                // alarm will break the logger out of flashing RED&BLUE light sync delay that follows
  noInterrupts ();                          // make sure we don't get interrupted before we sleep
  bitSet(EIFR,INTF0);                       // clear any previous flags for interrupt 0 (D2) see https://gammon.com.au/interrupts
  rtc_INT0_Flag=false;                      // Flag gets set TRUE only inside rtc_d2_alarm_ISR ISR
  attachInterrupt(0,rtc_d2_alarm_ISR, LOW); // RTC SQW alarms LOW and is connected to pin D2 which is interupt channel 0
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
  Serial.println(F("Red&Blue LEDs will light during SYNC DELAY until logger takes 1st reading.")); 
  Serial.println(F("Then Green/Blue LED pips will show when each sensor reading is collected."));
  Serial.flush();
  
  if(!ECHO_TO_SERIAL){          // if it's not being used, shut down the UART peripheral now to save power
    Serial.println(F("Disconnect UART now - NO additional messages will be sent over serial.")); Serial.flush();
        // power_usart0_disable();          // we waited until this point because the startup input menu requires serial input via the UART
        // NOTE: TOO many students were forgetting to wrap the Serial.print statements in the main loop with if(ECHO_TO_SERIAL){ } so we commented usart0_disable out
                                            // digital pins 0(RX) and 1(TX) are connected to the UART peripheral inside the 328p chip
                                            // Connecting anything to these pins may interfere with serial communication, including causing failed uploads
      }
//------------------------------------------------------------------------------
// FIRST sampling wakeup alarm is already set But instead of simply going to sleep we will use LowPower.powerDown SLEEP_500MS
// to wake the logger once per second to toggle the LEDs ON/Off so user can tell we are in the sync-delay period before logging starts
  #ifdef LED_r9_b10_g11_gnd12               // RED & BLUE leds start in opposite states so they ALTERNATE when toggled by the PIN register
        digitalWrite(10,LOW);pinMode(10,OUTPUT); // D10 [Blue] LED Output low   //pinMode(10,INPUT);           for lower current around 50uA
        digitalWrite(9,HIGH);pinMode(9,OUTPUT);  // D10 [Red] LED outputl high  //pinMode(9,INPUT_PULLUP);     OUTPUT HIGH PULLS 1.5 mA
  #else
        pinMode(13,INPUT);                    // D13 onboard red LED INPUT with PULLUP OFF  
  #endif

  byteBuffer1 = 2;
  do{   // see a ProMini pin map to understand why we are using PINB here for the LED controls https://images.theengineeringprojects.com/image/webp/2018/06/introduction-to-arduino-pro-mini-2.png.webp?ssl=1
        #ifdef LED_r9_b10_g11_gnd12         // setting any bits in the 328p PIN control registers to 1 TOGGLES the PULLUP RESISTOR on the associated pins
          PINB = B00000110;                 // this toggles blue, red led channels with one command
        #else
          PINB = B00100000;                 // this toggles ONLY the D13 led // ~50uA to light RED onboard led through D13s internal pullup resistor
        #endif 
        LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);   //ADC_ON preserves the existing ADC state - if its already off it stays off
    }while(!rtc_INT0_Flag);                 // sends you back to do{ unless the RTC alarm has triggered
  
  RTC_DS3231_turnOffBothAlarms();           // Note: detachInterrupt(0); already done inside the rtc_d2_alarm_ISR 
  rtc_INT0_Flag=false; 

//terminates synchronization time delay -we are now ready to start logging!
//------------------------------------------------------------------------------------------------------------

      pinMode(13,INPUT);                                      // D13 [Onboard Red] indicator LED PULLUP OFF   // alt:  bitClear(PORTB,5); would also do this job 
  #ifdef LED_r9_b10_g11_gnd12
      pinMode(11,INPUT);pinMode(10,INPUT);pinMode(9,INPUT);   // turn all indicators OFF at end of setup
      digitalWrite(12,LOW);pinMode(12,OUTPUT);                // the common ground line on our RGB led must OUTPUT to allow current
    #endif

  #ifndef logCurrentBattery                //readBattery(); Must be at the end of setup because it disables Serial if ECHO is off
    LowestBattery = readBattery();         //sets starting value for LowBat, but only needed if not logging CurrentBat
  #endif
  
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

//------------------------------------------------------------------------------- 
//  *  *  *  *  Set the next RTC wakeup alarm  *  *  *  *  *  *
//-------------------------------------------------------------------------------    
  RTC_DS3231_getTime();                     // populates global t_minute,t_second variables
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); // short sleeps for Cr2032 battery recovery after EVERY I2C exchange

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
    LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);  // RTC memory register WRITING time & battery recovery time
    
    RTC_DS3231_turnOnAlarm(1);
    LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);  // RTC memory register WRITING time & battery recovery time

#endif //terminates #ifdef PIRtriggersSensorReadings    
 
  if(ECHO_TO_SERIAL){
      sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d:%02d", t_year, t_month, t_day, t_hour, t_minute,t_second);
      Serial.print(F(">Logger woke at: ")); Serial.print(CycleTimeStamp); // sprintf ref:  http://www.perlmonks.org/?node_id=20519
        #ifndef PIRtriggersSensorReadings
          Serial.print(F(" Next Alarm Set in: "));
          if (SampleIntervalSeconds > 0){ Serial.print(SampleIntervalSeconds); Serial.print(F("sec")); 
          }else{ Serial.print(SampleIntervalMinutes);Serial.print(F("min")); }
        #endif //#ifndef PIRtriggersSensorReadings
      Serial.println();Serial.flush();
    }
  
#ifdef logCurrentBattery                              // ADC reads use significant power - only read CurrentBat if saving the data
    CurrentBattery = readBattery();                   // Note: a SLEEP_15MS is embedded in the readBattery function, processor draws about 1mA in sleep mode ADC
    if(ECHO_TO_SERIAL){
        Serial.print(F(", Current Bat[mV]: "));Serial.print(CurrentBattery);Serial.flush();
        }
#endif

  // if you are only logging LowestBattery, it might be useful to record the unloaded battery voltage once per day (?)
  //if(t_hour==0 && t_minute==0 && t_second==0){          // midnight reset prevents 'occasional' low readings from permanently resetting the lobat record
  //  LowestBattery = readBattery();                      // no-load readBattery() calls are usually 20-100mv higher than Lobat reads during high drain EEsave events
  //  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
  //  }
  
#ifdef logRTC_Temperature
//------------------------------------------------------------------------------
  rtc_TEMP_degC = RTC_DS3231_getTemp();               // moved this code into its own function
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);    // battery recovery pause after every I2C exchange
      if(ECHO_TO_SERIAL){
        Serial.print(F(", RTC temp[°C]: "));Serial.print(rtc_TEMP_degC,2);Serial.flush();
      } 
#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// STEP4 : READ your sensors here
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#if defined(readLDR_onD6) || defined(readNTC_onD7)
//------------------------------------------------
  ConditionCapacitorOnD8();                             // ConditionCapacitor only needs to be called ONCE before other ICU resistor readings
  uint32_Buffer = ReadD8riseTimeOnD8();                 // charge cycles the cap through D8 to standardize condition // AND it sets all pins with resistor/sensors connected to that common capacitor to input
#endif

#ifdef readNTC_onD7
//------------------
  NTC_NewReading = ReadD7riseTimeOnD8();                // a complex function at the end of this program
  NTC_NewReading = (referenceResistorValue * (uint32_t)NTC_NewReading) / uint32_Buffer;
  if(ECHO_TO_SERIAL){
    Serial.print(F(", NTC[Ω]: "));Serial.print(NTC_NewReading);Serial.flush();
    }  
#endif

#ifdef readLDR_onD6   
//-------------------
  LDR_NewReading = ReadD6riseTimeOnD8();
  LDR_NewReading = (referenceResistorValue * (uint32_t)LDR_NewReading) / uint32_Buffer;
      if(ECHO_TO_SERIAL){
        Serial.print(F(", LDR[Ω]: "));Serial.println(LDR_NewReading);Serial.flush();
      }  
#endif

#ifdef bh1750_Address
//-------------------
  bh1750.start(BH1750_QUALITY_LOW, BH1750_MTREG_LOW);   // triggers a new sensor reading
                                                        // LOW MTreg:31  resolution lux:7.4, 121557 is highest lux
  LowPower.powerDown(SLEEP_30MS, ADC_ON, BOD_OFF);      // L-Resolution Mode Measurement Time 16-24 msec                   
  //LowPower.powerDown(SLEEP_30MS, ADC_ON, BOD_OFF);    // H-Resolution Mode Measurement Time is much longer: 120-180 ms

  lux_BH1750_RawInt =bh1750.getRaw();                   // reading can reach 120,000
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);      // battery recovery after I2C - not really needed with this low current sensor
    
  if(ECHO_TO_SERIAL){  
      Serial.print(F(", Bh1750(raw): "));Serial.print(lux_BH1750_RawInt);
      Serial.print(F(", Lux(calc): "));Serial.print(bh1750.calcLux(lux_BH1750_RawInt,BH1750_QUALITY_LOW,BH1750_MTREG_LOW),2);Serial.flush();
      //if you call calcLux() without Quality & MTreg, the parameters from the last measurement are used for the calculation
    }
#endif //bh1750_Address

// read bmp280 sensor
//-------------------
#if defined(recordBMPtemp) || defined(recordBMPpressure) || defined(recordBMPaltitude) //  '||' means 'OR'
  bmp280.startForcedConversion(); 
  LowPower.powerDown(SLEEP_30MS, ADC_ON, BOD_OFF); //NOTE: sleep time needed here depends on your oversampling settings
#endif

#ifdef recordBMPtemp
  bmp280.getCurrentTemperature(Bmp280_Temp_degC);
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
  if(ECHO_TO_SERIAL){ Serial.print(F(" b280 Temp: ")); Serial.print(Bmp280_Temp_degC,2); Serial.print(F(" °C, ")); }
#endif

#ifdef recordBMPpressure
  bmp280.getCurrentPressure(Bmp280_Pr_mBar);
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
  if(ECHO_TO_SERIAL){ Serial.print(F(" b280 Pr. "));Serial.print(Bmp280_Pr_mBar,2); Serial.print(F(" hPa, ")); }
#endif

#ifdef recordBMPaltitude
  bmp280.getCurrentAltitude(Bmp280_altitude_m);
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
    if(ECHO_TO_SERIAL){ Serial.print(F(" b280 Alt. ")); Serial.print(Bmp280_altitude_m,2); Serial.print(F(" m,")); }
#endif
// to read all three at the same time: bmp280.getCurrentMeasurements(Bmp280_Temp_degC, Bmp280_Pr_mBar, Bmp280_altitude_m); //function returns 1 if readings OK

#ifdef Si7051_Address
//------------------------------------------------------------------------------
    TEMP_si7051 = readSI7051();                       // see functions at end of this program
        if(ECHO_TO_SERIAL){ 
          Serial.print(F(", SI7051 temp: "));Serial.print(((175.26*TEMP_si7051)/65536.0)-46.85 ,3);Serial.flush();//print 3 decimals
          }
    LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);  //  battery recovery time
#endif

//----------------------------------------------------------------------------------------
// HEARTBEAT pip of the LED at the end of the senor readings // each channel draws 30-50uA
//----------------------------------------------------------------------------------------
    #ifdef LED_r9_b10_g11_gnd12           // Colors can be combined for the LED pip!
      pinMode(11,INPUT_PULLUP);           // Green is brightest
      pinMode(10,INPUT_PULLUP);           // Blue mixes well with green
      //pinMode(9,INPUT_PULLUP);          // Red is too dim to see with pullup
    #else  //or use the default red led on D13
      pinMode(13,INPUT_PULLUP);
    #endif
    
      LowPower.powerDown(SLEEP_60MS, ADC_ON, BOD_OFF);
      
    #ifdef LED_r9_b10_g11_gnd12           // Colors can be combined for the LED pip!
    pinMode(10,INPUT);                    // D10 [blue] LED pullup Off 
    pinMode(11,INPUT);                    // D11 [Green] LED pullup Off
    //pinMode(9,INPUT);                   // D9 [red] LED pullup Off 
    #else  //or use the default red led on D13
      pinMode(13,INPUT);                    // pin13 indicator LED pullup Off
    #endif

//---------------------------------------------------------------------------------
//  SAVE NEW SENSOR READINGS into EEprom & READ battery after
//---------------------------------------------------------------------------------
// the number of bytes you transfer here must match the number in bytesPerRecord
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

//---------------------------------------------------------------------------------
// FIRST Setup ADC to read the coincell voltage DURING the EEprom data save:
//---------------------------------------------------------------------------------
  power_adc_enable();
    ADMUX = set_ADMUX_2readRailVoltage; ADCSRA = set_ADCSRA_2readRailVoltage;   //configure the 2 ADC control registers ADMUX & ADCSRA by loading the byte pattern from variables
    bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,1);bitWrite(ADCSRA,ADPS0,1); //ADC speed: 128 prescalar =67 kHz, this is slower than normal! ~208uS /ADC readings 
    bitSet(ADCSRA,ADSC);                            // triggers a 1st throw-away ADC reading to engauge the Aref capacitor //1st read takes 20 ADC clock cycles instead of usual 13  
    LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
  // NOTE: Aref capacitor Rise time can take 5-10 milliseconds after starting ADC so 15ms of ADC_ON powerDown sleep works // this could be done before the LED pip?
  // also provides some battery recovery time before data is saved to EEprom
  
//---------------------------------------------------------------------------------
  Wire.beginTransmission(EEpromI2Caddr);            // STARTS filling the I2C transmission buffer with the eeprom I2C bus address
  Wire.write(highByte(EEmemoryPointr));             // send the HighByte of the EEprom memory location we want to write to
  Wire.write(lowByte(EEmemoryPointr));              // send the LowByte of the EEprom memory location   // Note: we add  'bytes per record' to EEmemoryPointr at the end of the main loop  
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

#ifdef logLowestBattery                           // INDEX compression converts lowBattery reading to # less than 255 which can be stored in one byte eeprom memory location
//-------------------------------------------------------------------------------------------------------
  integerBuffer = (LowestBattery-1700)/16;        // index compression looses some data due to rounding error  - so record is reduced to only 16mv/bit resolution
  byteBuffer1 = lowByte(integerBuffer);
  if(byteBuffer1<1){byteBuffer1=1;}               // ONLY THE FIRST data byte in each record must have a ZERO TRAP to preserve End Of Data indicator in EEprom
        Wire.write(byteBuffer1);                  // write that single compressed byte to eeprom (we will have to expand it back later !) 
#endif //logLowestBattery

#ifdef logRTC_Temperature                         // NOTE: this 1-byte COMPRESSED encoding limits our temperature range to 0-63 degrees 
//--------------------------------------------------------------------------------------------------------
  floatBuffer = (rtc_TEMP_degC + 10.0)*4;         // *4 converts the RTC temperature into a small integer (63*4= 252 - within the range of one byte!)
  integerBuffer = (int)(floatBuffer);             // adding 10 shifts the 63 degree range, so we can record -10C to +53C  // there is no loss of information because the resolution is only 0.25C 
       
  if(integerBuffer>255){integerBuffer=255;}       // temps above 53 C get clipped because a byte cant represent them
  if(integerBuffer<1){integerBuffer=1;}           // Zero Trap to preserve End of Data check, this sets lower cutoff temp to -9.75C (but our coincell is dead before that)

  byteBuffer1 = lowByte(integerBuffer);
  //if(byteBuffer1<1){byteBuffer1=1;}             // ONLY THE FIRST data byte saved in each record must have a ZERO TRAP to preserve zero EOF indicators in EEprom
        Wire.write(byteBuffer1); 
#endif //logRTC_Temperature

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// STEP5 : add more sensor data bytes to the I2C buffer HERE as required ++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef countPIReventsPerSampleInterval
  d3_INT1_eventCounter = d3_INT1_eventCounter+1;       // we 'add one' so that the 1st stored byte is never zero - even when the count actually is zero- this is our zero trap
  loByte = lowByte(d3_INT1_eventCounter);
    Wire.write(loByte);
  hiByte = highByte(d3_INT1_eventCounter);
    Wire.write(hiByte);
  d3_INT1_eventCounter = 0;                            // after saving the data we can reset our event counter to zero
#endif

#ifdef readNTC_onD7
//------------------
  loByte = lowByte(NTC_NewReading);          // first byte of record gets checked by 
  //if(loByte<1){loByte=1;}                     // -ONLY 1st byte of record -needs to preserve zero EOF indicator in 'empty' EEprom space
        Wire.write(loByte);
  hiByte = highByte(NTC_NewReading);
        Wire.write(hiByte);  
#endif // #ifdef readNTC_onD7

#ifdef readLDR_onD6
//------------------
  loByte = lowByte(LDR_NewReading);          // first byte of record gets checked by 
  //if(loByte<1){loByte=1;}                     // -ONLY 1st byte of record -needs to preserve zero EOF indicator in 'empty' EEprom space
        Wire.write(loByte);
  hiByte = highByte(LDR_NewReading);
        Wire.write(hiByte);  
#endif // #ifdef readLDR_onD6

#ifdef bh1750_Address
//--------------------
  loByte = lowByte(lux_BH1750_RawInt);          // first byte of record gets checked by 
  //if(loByte<1){loByte=1;}                     // -ONLY 1st byte of record -needs to preserve zero EOF indicator in 'empty' EEprom space
        Wire.write(loByte);
  hiByte = highByte(lux_BH1750_RawInt);
        Wire.write(hiByte);  
#endif // #ifdef bh1750_Address

#ifdef recordBMPtemp
  Bmp280_Temp_degC = Bmp280_Temp_degC*100.00;   //convert float reading to integer preserving two decimal places
  integerBuffer = (int16_t)Bmp280_Temp_degC;  
  loByte = lowByte(integerBuffer);              // first byte of record gets checked by 
  //if(loByte<1){loByte=1;}                     // to preserve zero EOF indicator in 'empty' EEprom space
        Wire.write(loByte);
  hiByte = highByte(integerBuffer);
        Wire.write(hiByte);  
#endif

#ifdef recordBMPpressure
  Bmp280_Pr_mBar = Bmp280_Pr_mBar*10.0;         //convert float reading to integer preserving ONE decimal place
  integerBuffer = (int16_t)Bmp280_Pr_mBar;  
  loByte = lowByte(integerBuffer);              // first byte of record gets checked by 
  //if(loByte<1){loByte=1;}                     // to preserve zero EOF indicator in 'empty' EEprom space
        Wire.write(loByte);
  hiByte = highByte(integerBuffer);
        Wire.write(hiByte);     
#endif

#ifdef recordBMPaltitude
  Bmp280_altitude_m = Bmp280_altitude_m*100.00;     //convert float reading to integer preserving two decimal places
    integerBuffer = (int16_t)Bmp280_altitude_m;   // Note: *100 overuns the int at higher altitudes - switch to *10
  loByte = lowByte(integerBuffer);              // first byte of record gets checked by 
  //if(loByte<1){loByte=1;}                     // to preserve zero EOF indicator in 'empty' EEprom space
        Wire.write(loByte);
  hiByte = highByte(integerBuffer);
        Wire.write(hiByte);  
#endif

#ifdef logCurrentBattery                      // stores the 'raw' 16-byte integer using two bytes (ie with no compression)
//-----------------------------------------------------------------------------------------------------------------------
  loByte = lowByte(CurrentBattery);           // note: we save the low byte first because it is almost never zero
        Wire.write(loByte);                   // first byte added to I2C buffer
  //if(loByte<1){loByte=1;}                   // ONLY THE FIRST data byte saved in each record must have a ZERO TRAP to preserve zero EOF indicators in EEprom
  hiByte = highByte(CurrentBattery);
        Wire.write(hiByte);                   // 2nd byte of data added to I2C buffer 
#endif //logCurrentBattery

#ifdef Si7051_Address                         // stores the 'raw' 16-byte integer using two bytes (ie with no compression)
//-----------------------------------------------------------------------------------------------------------------------
  loByte = lowByte(TEMP_si7051);              //NOTE TEMP_si7051 overruns this uint16_t if temps >40C!
        Wire.write(loByte);
  //if(loByte<1){loByte=1;}                   // ONLY THE FIRST data byte saved in each record must have a ZERO TRAP to preserve zero EOF indicators in EEprom
  hiByte =  highByte(TEMP_si7051); 
        Wire.write(hiByte);
#endif //Si7051_Address
 

//-------------------------------------------------------------------------------
    Wire.endTransmission(); // ONLY AT THIS POINT do the bytes accumulated in the buffer actually get sent
//-------------------------------------------------------------------------------
// Then the EEPROM enters an internally-timed write cycle to memory which takes ~3-10ms
// 4k AT24c32 write draws ~10mA for about 10ms @3mA, but newer eeproms can take only only 5ms @3mA
// the coincell battery experiences a SIGNIFICANT VOLTAGE DROP due to its internal resistance during this load
// so we read the battery voltage during this load event for a true Li battery reading
//-------------------------------------------------------------------------------

    bitSet(ADCSRA,ADSC); while(bit_is_set(ADCSRA,ADSC));  // this is another throw away ADC reading to provide 200microseconds for the coincell volatage drop
    bitSet(ADCSRA,ADSC); while(bit_is_set(ADCSRA,ADSC));  uint16_Buffer=ADC; //ADC is a macro command that combines the ADC's two output memory registers into one integer
    ADMUX = default_ADMUX; ADCSRA = 0; power_adc_disable(); // restore defaults & turn off the ADC

    // default 4k & 32k eeproms can just put the logger straight to sleep during the EEprom data-save
    // but Larger eeproms sometimes need the I2C bus left on until the save completes
    if(EEbytesOfStorage == 4096 || EEbytesOfStorage == 32768 ){
        LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); // NOTE .powerDown ONLY works with the AT24c32(4K) EEproms
    }else{                                              // with larger eeproms you may need to keep the bus clock running -> but .idle FREEZES the 4k eeproms!
        LowPower.idle(SLEEP_15MS, ADC_ON, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_ON);  // NOTE _ON just leaves the peripheral in its existing state - if its already off it stays off
    }
  
  LowestBattery = InternalReferenceConstant / uint16_Buffer;   
  if(ECHO_TO_SERIAL){
      Serial.print(F(", Lowest Bat[mV]: "));Serial.println(LowestBattery);Serial.flush();
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

  #ifdef logRTC_Temperature
  //oled.clearField(32,4,10);
  oled.setCursor(32,4); //oled.setCursor(Column, Row) -row is from the upper left corner
  oled.print(F("RTC    temp")); //5x7 system font can display 10 characters accross the screen
  LowPower.powerDown(SLEEP_120MS, ADC_ON, BOD_ON); //battery recovery after each oled.print statement
  oled.setCursor(32,5); //64x32 screen displays only the CENTER pixels of the 128x64 pixel wide memory!
  // so first collumn/ pixel on micro oled screen from LEFT is at 32 across
  // when using the 128x64 driver the first row starts at (32,4), and entire screen occupies 8pixel heigh rows 4-5-6-7
  oled.print(F("----------")); //5x7 system font can display 10 characters accross the screen
  LowPower.powerDown(SLEEP_120MS, ADC_ON, BOD_ON);
  oled.set2X();  //2x  5x7 system font can display 5 characters accross the screen
  //oled.clearField(32,6,6);
  oled.setCursor(32,6); //set2X can only start at rows 4 or 5 or 6 (because they take two horizontal rows to display)
  oled.print(rtc_TEMP_degC,2);
  oled.print(F(" ")); //blank spaces to clearing the rest of the row
  LowPower.powerDown(SLEEP_120MS, ADC_ON, BOD_ON);
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
  LowPower.powerDown(SLEEP_8S, ADC_ON, BOD_OFF); //the ProMini does not need to stay awake for the screen display time
  oled.clear();oled.set1X(); oled.setCursor(32,4); oled.print(F("Temp   Cel"));
  oled.setCursor(32,5);oled.print(F("----------")); //5x7 font =10 characters accross the screen
  oled.set2X();oled.setCursor(32,6);oled.print(bmp280_temp,2);
  #endif //BMP280_Address

  #ifdef ReadNTC_onD7
  oled.setCursor(32,4);oled.print(F("NTC"));
  //oled.setCursor(56,5);oled.print(NTC_NewReading);
  
  oled.setCursor(32,5); //64x32 screen displays only the CENTER pixels of the 128x64 pixel wide memory!
  // so first collumn/ pixel on micro oled screen from LEFT is at 32 across
  // when using the 128x64 driver the first row starts at (32,4), and entire screen occupies 8pixel heigh rows 4-5-6-7
  oled.print(F("----------")); //5x7 system font can display 10 characters accross the screen
  oled.set2X();  //2x  5x7 system font can display 5 characters accross the screen
  oled.setCursor(32,6); //set2X can only start at rows 4 or 5 or 6 (because they take two horizontal rows to display)
  oled.print(NTC_NewReading);
  #endif //ReadNTC
  
  #ifdef ReadLDR_onD6
  oled.setCursor(32,6);oled.print(F("LDR"));
  oled.setCursor(56,7);oled.print(LDR_NewReading);
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
   
  LowPower.powerDown(SLEEP_8S, ADC_ON, BOD_OFF);
  oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF); // To switch display OFF

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif //end of     OLED display commands
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

  if (LowestBattery <= systemShutdownVoltage){
      error_shutdown(); // shutdown down the logger
      } 
  
  EEmemoryPointr = EEmemoryPointr + bytesPerRecord;     //advances our memory pointer for the next loop
  if( EEmemoryPointr >= EEbytesOfStorage){              // if eeprom memory is full
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

  pinMode(2,INPUT);                                  //D2 pullup off - not needed with hardware pullups on RTC module
  
  noInterrupts();
  bitSet(EIFR,INTF0);                                 // clears interrupt 0's flag bit before attachInterrupt(0,isr,xxxx)
  attachInterrupt(0,rtc_d2_alarm_ISR,LOW);            //RTC alarm connected to pin D2 // LOW assures it will always respond if the RTC alarm is asserted
  interrupts();
  LowPower.powerDown(SLEEP_FOREVER, ADC_ON, BOD_OFF); // ADC_ON simply preserves whatever the current ADC status is (in our case it's already OFF...)
  
  //HERE AFTER WAKING  // note that detachInterrupt(0); happened inside the ISR
    
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(DS3231_STATUS_REG);
  Wire.write(0);                                     // clearing the entire status register turns Off (both) RTC alarms though technically only the last two bits need to be set
  Wire.endTransmission();
  rtc_INT0_Flag = false;                             // clear the flag we use to indicate the RTC alarm occurred
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);   // coincell voltage recovery time from Wakup & I2C transaction
  
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
  
  LowPower.powerDown(SLEEP_FOREVER, ADC_ON, BOD_OFF); // ADC_ON simply preserves whatever the current ADC status is (in our case it's already OFF...)

  if (d3_INT1_Flag) {        // d3_INT1_Flag is set true in input_d3_interrupt_ISR()
       if (d3_INT1_eventCounter < 65532) {d3_INT1_eventCounter++;}  // only increment our counter variable if it's below the uint16_t max
                                                                    // this is somewhat unnecessary given the sensor has a 2-second reset time...

   // Pip blue LED to indicate PIR wakeup event being counted
        #ifdef LED_r9_b10_g11_gnd12 
        digitalWrite(10,HIGH); pinMode(10,OUTPUT);          // or pinMode(10,INPUT_PULLUP); // BLUE 
        #endif
        LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);    // time to view LED pip
        pinMode(10,INPUT);                                  // D10 [blue] LED pullup OFF
       
    } else { // If d3_INT1_Flag is still false then we woke from the RTC alarm - not the PIR, so Int1 still needs detached
       detachInterrupt(1);
    }

  if (rtc_INT0_Flag){       // rtc_INT0_Flag is set true in rtc_d2_alarm_ISR()
      Wire.beginTransmission(DS3231_ADDRESS);
      Wire.write(DS3231_STATUS_REG);
      Wire.write(0);      // turns Off (both) RTC alarms
      Wire.endTransmission();
      LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);   // coincell recovery time after I2C transaction
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

//==========================================================================================
//  *  *  *  *  *  *  *  *  *  FUNCTIONS called during Setup()  *  *  *  *  *  *  *  *  *  * 
//==========================================================================================
void setup_sendboilerplate2serialMonitor(){      
//-----------------------------------------------------------------------------------------
//NOTE:(__FlashStringHelper*) is needed to print variables is stored in PROGMEM instead of regular memory
    Serial.print(F("CodeBuild: ")); Serial.print(fileNAMEonly);         // or use Serial.println((__FlashStringHelper*)codebuild); //for the entire path + filename
    Serial.print(F(" Compiled: "));Serial.print((__FlashStringHelper*)compileDate);
    Serial.print(F(" @ ")); Serial.println((__FlashStringHelper*)compileTime);
    Serial.print(F("Hardware: ")); Serial.println((__FlashStringHelper*)loggerConfiguration);
    Serial.print(F("Last Deployment: ")); Serial.println((__FlashStringHelper*)deploymentDetails); 
    
    //#ifdef readNTC_onD7
    //  Serial.println((__FlashStringHelper*)sensorCalibrationInfo);
    //#endif
    
    Serial.flush();
}

void setup_displayStartMenu() {       
//-----------------------------------------------------------------------------------------

    while (Serial.available() != 0 ) {Serial.read();} // this just clears out any residual data in serial send buffer before starting our menu  
    Serial.setTimeout(1000);                          // 1000 milliseconds is the default timeout for the Serial.read(); command
    uint8_t inByte=0;
    boolean wait4input = true;
    boolean displayMenuAgain = true;
    uint32_Buffer = millis();                          //Beginning of time-out period must be unsigned long variable

  do{ inByte=0;
    if (displayMenuAgain) { 
      startMenu_printMenuOptions(); displayMenuAgain=false;}

    if (Serial.available()) { 
      inByte = Serial.parseInt(); }                     //from https://forum.arduino.cc/t/simple-serial-menu-without-a-library/669556
    
    switch (inByte) {                                   //NOTE: switch can also accept 'letter inputs' with single quotes: case 'Z':
            case 1:
              startMenu_sendData2Serial(true);  
              displayMenuAgain=true;  break;
            case 2:
              startMenu_setRTCtime(); Serial.setTimeout(1000); 
              displayMenuAgain=true;  break;
            case 3:
              startMenu_setSampleInterval(); Serial.setTimeout(1000); 
              displayMenuAgain=true;  break;
            case 4:
              ECHO_TO_SERIAL = !ECHO_TO_SERIAL;           // toggles the boolean true/false variable
              displayMenuAgain=true;  break;
            case 5:
              startMenu_setVrefConstant(); Serial.setTimeout(1000);
              displayMenuAgain=true;  break; 
            case 6:                                       // this is the 'start logger' option
              wait4input=false; break;                    // wait4input=false breaks you out of the switch-case loop & sends you back to Setup function where displayStartMenu was first called
            case 7:
            startMenu_sendData2Serial(false);             // a 'hidden' debugging option not displayed in the startmenu
            displayMenuAgain=true;  break;                // lets you see the RAW bytes stored in your eeprom
              
            default:                                      // Check milliseconds elapsed & send logger into shutdown if we've waited too long
                if ((millis() - uint32_Buffer) > 480000) {// start menu has an 480000 = 8 minute timeout
                Serial.println(F("Start Menu Timed out with NO commands!"));
                Serial.println(F("Logger shutting down...")); Serial.flush(); error_shutdown(); 
                }
              break;
         }    //  terminates switch-case cascade
      }while(wait4input);                   //  do-while loops back to check for input if wait4input=true;
  return;
}

void startMenu_printMenuOptions(){          // note: setup_sendboilerplate2serialMonitor(); runs once on startup before this
//-----------------------------------------------------------------------------------------

  Serial.println();
  RTC_DS3231_getTime();                     // reads current clock time  and display it via CycleTimeStamp
  sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", t_year, t_month, t_day, t_hour, t_minute);  //combines numeric time variables to a human readable text string
  Serial.print(F("Current Logger time:"));Serial.print(CycleTimeStamp);Serial.print(F(":"));Serial.print(t_second); //seconds separate because usually value is zero
  EEPROM.get(4,InternalReferenceConstant);
  Serial.print(F(", Current VREF const.="));Serial.println(InternalReferenceConstant);
  if(ECHO_TO_SERIAL){
  Serial.print(F("SERIAL output ON"));
  }else{
  Serial.print(F("SERIAL output OFF"));
  } 
  SampleIntervalMinutes = EEPROM.read(8);
  SampleIntervalSeconds = EEPROM.read(9);
  Serial.print(F(", Interval: "));Serial.print(SampleIntervalMinutes);Serial.print(F("m "));Serial.print(SampleIntervalSeconds);Serial.print(F("s"));
  Serial.print(F(" Logging: "));
  #ifdef logLowestBattery
        Serial.print(F("LoBat[mv],"));
        #endif
  #ifdef logRTC_Temperature
        Serial.print(F("RTC[°C],"));
        #endif
  #ifdef countPIReventsPerSampleInterval
        Serial.print(F("PIR count,"));
        #endif
  #ifdef PIRtriggersSensorReadings
        Serial.print(F("PIR Triggers Reading,"));
        #endif
  #ifdef readNTC_onD7
      Serial.print(F("NTC[Ω],"));
      #endif
  #ifdef readLDR_onD6
      Serial.print(F("LDR[Ω],"));
      #endif  
  #ifdef bh1750_Address
        Serial.print(F("Bh1750[Lux],"));
        #endif
  #ifdef recordBMPtemp
        Serial.print(F("b280[T°C],"));
      #endif
  #ifdef recordBMPpressure
        Serial.print(F("b280Pr[mbar],"));
        #endif
  #ifdef recordBMPaltitude
        Serial.print(F("b280Alt[m],"));
        #endif
  #ifdef logCurrentBattery
        Serial.print(F("C.Bat[mv],"));
        #endif
  #ifdef Si7051_Address
        Serial.print(F("SI7051[°C],"));
        #endif
    Serial.println();
    
    Serial.println();
    Serial.println(F("Select one of the following options:"));
    Serial.println(F("  [1] DOWNLOAD Data"));
    Serial.println(F("  [2] Set CLOCK        [3] Set INTERVAL"));
    Serial.println(F("  [4] Toggle SERIAL    [5] Change VREF"));
    Serial.println(F("  [6] START logging"));
    Serial.println(); Serial.flush();
}   //terminates startMenu_printMenuOptions

void startMenu_setVrefConstant(){                           //default =1126400L = 1100mV * 1024 
//-----------------------------------------------------------------------------------------
do {
    Serial.println(F("Input a new Vref constant between 1000000 and 1228800:")); //1,126,400L = default for 1100mV * 1024
    Serial.setTimeout(100000);                              //parseInt will normally “time out” after default set point is 1 second (1000 milliseconds).
    while (Serial.available() != 0 ) {Serial.read();}       // clears the serial buffer  
    InternalReferenceConstant = Serial.parseInt();          //parseInt() actually returns a long
    }while((InternalReferenceConstant<1000000) || (InternalReferenceConstant>1228800)); // if condition fails & you have to re-enter the number
   Serial.print(F("Vref set to: ")); Serial.println(InternalReferenceConstant);
   EEPROM.put(4,InternalReferenceConstant);                 // every time you run the logger it will retrieve the interval from the previous run 
   return;
} // terminates startMenu_setVrefConstant

void startMenu_setSampleInterval(){
//-----------------------------------------------------------------------------------------
do {
    Serial.println();Serial.println(F("Input a sampling interval of 1,2,5,10,15,30 or [0] minutes:"));
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
         Serial.println(F("Enter sub-minute interval as 1,2,3,5,10,15, or 30 seconds: (sampling time may over-run the alarm!)"));
          byteBuffer2 =0;
         SampleIntervalSeconds = Serial.parseInt();
          byteBuffer2 = SampleIntervalSeconds ? 30 % SampleIntervalSeconds : 0; // ERROR check: input must be valid divisor of 30 OR zero
        }while ((byteBuffer2 !=0) || (SampleIntervalSeconds>30));               // or while condition fails & you have to re-enter the number
      Serial.print(F("  Sub-minute Sample Interval: ")); Serial.print(SampleIntervalSeconds);Serial.println(F(" seconds"));
      }

    if (SampleIntervalMinutes==0 && SampleIntervalSeconds==0){
      Serial.println(F("Invalid Entry: 15 min DEFAULT interval being SET"));Serial.flush();
      SampleIntervalMinutes = 15;SampleIntervalSeconds = 0;
      }
    EEPROM.update(8,SampleIntervalMinutes);             // newly entered values are now stored in the 329p's internal eeprom
    EEPROM.update(9,SampleIntervalSeconds);             // every time you run the logger it will retrieve the interval from the previous run 
    
 return;
}//startMenu_setSampleInterval

void startMenu_setRTCtime(){
//-----------------------------------------------------------------------------------------
  Serial.println(F("Enter current date/time with digits as indicated:"));  // note Serial.parseInt will ONLY ACCEPT NUMBERS from the serial window!
  Serial.setTimeout(100000); while (Serial.available() != 0 ) {Serial.read();}   // clears any old leftover text out of the serial buffer 
  Serial.print(F("YYYY:"));     t_year = Serial.parseInt();      Serial.println(t_year);
  Serial.print(F("MM:"));       t_month = Serial.parseInt();     Serial.println(t_month);
  Serial.print(F("DD:"));       t_day = Serial.parseInt();       Serial.println(t_day);
  Serial.print(F("(24hour) HH:")); t_hour = Serial.parseInt();   Serial.println(t_hour);
  Serial.print(F("MM:"));       t_minute = Serial.parseInt();    Serial.println(t_minute);
  Serial.print(F("SS:"));       t_second = Serial.parseInt();    Serial.println(t_second);

  if (t_month==0 && t_day==0){                          // this is a very crude error catch //this needs to be further developed
    Serial.println(F("Not valid input to set RTC time!"));
    return;                                             //shut down the logger - user will need to re-open the serial window to restart the logger
    } else {
    RTC_DS3231_setTime(); delay(15);                    // give the RTC register memory write a little time
    i2c_setRegisterBit(DS3231_ADDRESS,DS3231_STATUS_REG,7,0); //clear the OSF flag after time is set
    }   //terminates if (t_month==0 && t_day==0){

}

void startMenu_sendData2Serial(boolean convertDataFlag){ // called at startup via serial window
//===============================================================================================
// NOTE: the ORDER of BYTES/SENSORS listed in startMenu_sendData2Serial MUST EXACTLY MATCH
// the order you ADDED those SENSOR READINGS when loading the EEprom write buffer in the main loop

// ADD HEADER information to top rows of serial output:
  setup_sendboilerplate2serialMonitor(); // a description of the deployment should be part of the data output
  //Serial.println(F("Convert Unixtime to Excel Dates with = UnixTime/#secondsInaDay + DATE(1970,1,1) "));

  Serial.print(F("UnixTime,"));  

  #ifdef logLowestBattery
        Serial.print(F("LoBat[mv],"));
        #endif
  #ifdef logRTC_Temperature
        Serial.print(F("RTC[°C],"));
        #endif
  #ifdef countPIReventsPerSampleInterval
        Serial.print(F("PIR count,"));
        #endif
  #ifdef readNTC_onD7
      Serial.print(F("NTC[Ω],"));
      #endif
  #ifdef readLDR_onD6
      Serial.print(F("LDR[Ω],"));
      #endif 
  #ifdef bh1750_Address
        Serial.print(F("Bh1750[Lux],"));
        #endif
  #ifdef recordBMPtemp
        Serial.print(F("b280[T°C],"));
      #endif
  #ifdef recordBMPpressure
        Serial.print(F("b280Pr[mbar],"));
        #endif
  #ifdef recordBMPaltitude
        Serial.print(F("b280Alt[m],"));
        #endif
  #ifdef logCurrentBattery
        Serial.print(F("C.Bat[mv],"));
        #endif
  #ifdef Si7051_Address
        Serial.print(F("SI7051[°C],"));
        #endif

 Serial.println();Serial.flush();

//starting time value was stored in first four bytes of the 328p internal eeprom:
  uint32_t unix_timeStamp;
    EEPROM.get(0,unix_timeStamp);                       // the loggerStartTime saved at previous logger startup
    SampleIntervalMinutes = EEPROM.read(8);
    SampleIntervalSeconds = EEPROM.read(9);
    
  uint16_t secondsPerSampleInterval;
    if (SampleIntervalMinutes==0){                      // sub-minute alarms for accelerated run testing
        secondsPerSampleInterval = SampleIntervalSeconds;
        }else{                                          //  normal minute based alarms:
        secondsPerSampleInterval = 60UL*SampleIntervalMinutes;
        }
    
  uint32_t EEmemPointr = 0;                             // a counter that advances through the EEprom Memory in bytesPerRecord increments
  uint32_t RecordMemoryPointer= 0;                      // uint32_t because overshoots uint16_t on systems with multiple eeproms
    

  do{    // this big do-while loop readback must EXACTLY MATCH the data saving pattern in our main loop:
  //---------------------------------------------------------------------------------------------------------- 
   
  RecordMemoryPointer = EEmemPointr;                        // a pointer for each byte within a given record
  
  //if the first byte readback process is ZERO then we've reached our end of data marker
  byteBuffer1 = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer); 
  if(byteBuffer1==0 & convertDataFlag){break;}              // this breaks us out of the do-while readback loop

  if (!convertDataFlag){    // then output raw bytes exactly as read from eeprom [with no timestamp] // this is ONLY used for debugging
        for (uint8_t j = 0; j < bytesPerRecord; j++) {
        byteBuffer1 = i2c_eeprom_read_byte(EEpromI2Caddr,(RecordMemoryPointer+j));
        Serial.print(byteBuffer1); Serial.print(F(","));    // outputs raw bytes as read from eeprom:
        }     
  } else { // if convertDataFlag is true then raw eeprom bytes get re-constituted back to variables

#ifdef PIRtriggersSensorReadings   //4-byte reconstruction of d3_INT1_elapsedSeconds // saved from low byte first to high byte last
      d3_INT1_elapsedSeconds = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer); //lo byte
      RecordMemoryPointer++;
      uint32_Buffer = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
      RecordMemoryPointer++;
      d3_INT1_elapsedSeconds = (uint32_Buffer << 8) | d3_INT1_elapsedSeconds;
      uint32_Buffer = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
      RecordMemoryPointer++;
      d3_INT1_elapsedSeconds = (uint32_Buffer << 16) | d3_INT1_elapsedSeconds;
      uint32_Buffer = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer); //high byte
      RecordMemoryPointer++; 
      d3_INT1_elapsedSeconds = (uint32_Buffer << 24) | d3_INT1_elapsedSeconds;
      
      d3_INT1_elapsedSeconds=d3_INT1_elapsedSeconds-1; // removes our zero trap on the first byte
      
      unix_timeStamp += d3_INT1_elapsedSeconds;   // then PIR triggered readings have an irregular number seconds between intervals    
#endif

  Serial.print(unix_timeStamp);Serial.print(",");
  
#ifndef PIRtriggersSensorReadings
  unix_timeStamp += secondsPerSampleInterval;               //increment unix timestamp for the NEXT record after printing
#endif
  // order of sensors & bytes listed here must EXACLTY MATCH the order in which you loaded the bytes into the eeprom in the main loop

#ifdef logLowestBattery  //  1 byte index encoded (slight loss of resolution compared to original two bytes)
      uint16_Buffer = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
      RecordMemoryPointer++;
      uint16_Buffer =(uint16_Buffer*16)+1700;               // REVERSING the calculation we used to index-compress the data into one byte(note <<4 is the same as *16)
      Serial.print(uint16_Buffer);Serial.print(F(","));
#endif

#ifdef logRTC_Temperature                                   // RTC temperature 1-byte compressed: low side cutoff at 1 for minimum reading of -12.25C
      integerBuffer = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
      RecordMemoryPointer++;
      floatBuffer = integerBuffer;
      floatBuffer = (floatBuffer/4.0)-10.0;                 // REVERSING the calculation we used to compress the data into one byte
      Serial.print(floatBuffer,2);Serial.print(F(","));     // ,2) specifies that you only print two decimal places
#endif  //#ifdef logRTC_Temperature 

#ifdef countPIReventsPerSampleInterval
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
      RecordMemoryPointer++;             
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
      RecordMemoryPointer++;                   
      uint16_Buffer = (uint16_t)((hiByte << 8) | loByte); 
      Serial.print(uint16_Buffer-1);Serial.print(F(","));   // 'minus 1' because we added one as our zero trap before the count was saved
#endif

#ifdef readNTC_onD7
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
      RecordMemoryPointer++;             
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
      RecordMemoryPointer++;                   
      uint16_Buffer = (uint16_t)((hiByte << 8) | loByte); 
      Serial.print(uint16_Buffer);Serial.print(F(","));
#endif

#ifdef readLDR_onD6
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
      RecordMemoryPointer++;             
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
      RecordMemoryPointer++;                   
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

#ifdef bh1750_Address
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer); //low byte
      RecordMemoryPointer++;
      lux_BH1750_RawInt = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer); //hi byte
      RecordMemoryPointer++;
      lux_BH1750_RawInt = (lux_BH1750_RawInt << 8) | loByte;
      floatBuffer = bh1750.calcLux(lux_BH1750_RawInt,BH1750_QUALITY_LOW,BH1750_MTREG_LOW);
      Serial.print(floatBuffer,0); //the decimals are meaningless at this resolution 
      Serial.print(",");
#endif


#ifdef recordBMPtemp           // Bmp280_Temp_degC, 2-bytes, low byte first
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer); //low byte
      RecordMemoryPointer++;
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer); //hi byte
      RecordMemoryPointer++;
      integerBuffer = (int16_t)((hiByte << 8) | loByte);
      floatBuffer  = (float)(integerBuffer)/100.0;
      Serial.print(floatBuffer,2);Serial.print(",");
#endif

#ifdef recordBMPpressure      // Bmp280_Pr_mBar, 2-bytes, low byte first
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer); //low byte
      RecordMemoryPointer++;
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer); //hi byte
      RecordMemoryPointer++;
      integerBuffer = (int16_t)((hiByte << 8) | loByte);
      floatBuffer  = (float)(integerBuffer)/10.0;
      Serial.print(floatBuffer,1);Serial.print(",");
#endif

#ifdef recordBMPaltitude            // Bmp280_Temp_degC, 2-bytes, low byte first
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer); //low byte
      RecordMemoryPointer++;
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer); //hi byte
      RecordMemoryPointer++;
      integerBuffer = (int16_t)((hiByte << 8) | loByte);
      floatBuffer  = (float)(integerBuffer)/100.0;  //Note: *100 over runs the integer at higher altitudes, in that case switch to *10
      Serial.print(floatBuffer,1);Serial.print(",");
#endif
      
#ifdef logCurrentBattery            // stored as two data bytes in the Main Loop, with lowByte first, then highByte
      loByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
      RecordMemoryPointer++;             
      hiByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
      RecordMemoryPointer++;                   
      uint16_Buffer = (uint16_t)((hiByte << 8) | loByte); 
      Serial.print(uint16_Buffer);Serial.print(F(","));
#endif

#ifdef Si7051_Address 
        loByte = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
        RecordMemoryPointer++;
        TEMP_si7051 = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);
        RecordMemoryPointer++;
        TEMP_si7051 = (TEMP_si7051 << 8) | loByte;           // NOTE: no casting needed with hiByte loaded into TEMP_si7051 which is an integer variable
        Serial.print(((175.26*TEMP_si7051)/65536.0)-46.85,3);Serial.print(F(","));  //calculation is promoted to float by the decimal places
        //integer converted to celcius (3 decimals output)   //or Serial.print(TEMP_si7051); to print raw integer 
#endif // #ifdef Si7051_Address

  } //terminators if(convertDataFlag)
  
  Serial.println();
  EEmemPointr = EEmemPointr + bytesPerRecord;
  
  } while(EEmemPointr < EEbytesOfStorage); // terminates the readback loop when pointer reaches end of memory space
  //---------------------------------------------------------------------------------------

  Serial.flush();
}  // terminates sendData2Serial() function


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
  
uint16_t readBattery(){                                 // reads 1.1vref as input against VCC as reference voltage
  power_all_disable();  power_adc_enable(); 
  ADMUX = set_ADMUX_2readRailVoltage;   ADCSRA = set_ADCSRA_2readRailVoltage;
  bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,1);bitWrite(ADCSRA,ADPS0,0); // 64 prescalar @ 8MHz/64 sets(default)125 kHz ADC clock 
                                                        // typical ADC read takes 13 ADC clock cycles, so default speed is about 9615 Hz (or 0.104 milliseconds per reading).
  bitSet(ADCSRA,ADSC); while(bit_is_set(ADCSRA,ADSC));  // triggers a 1st THROW AWAY READING to engage AREF capacitor
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);      // leaves ADC_ON: ~110µA so aref cap charges up during this sleep
    
  uint16_Buffer=0; adc_interrupt_counter = 0;           // reset our accumulator variables
  bitSet(ACSR,ADIF);                                    // clears any previous ADC interrupt flags
  bitSet(ADCSRA,ADIE);                                  // tells ADC to generate processor interrupts to wake the processor when a new reading is ready
  set_sleep_mode( SLEEP_MODE_ADC );                     // Enable ADC Noise Reduction Sleep Mode
    do{
          do{ sleep_mode();                             // sleep_mode macro combines sleep enable & disable with sleep_cpu command     // Note: sleep_disable(); not needed with sleep_mode();    
          }while (bit_is_set(ADCSRA,ADSC));             // ADC resets ADSC bit to zero ONLY when conversion is finished, otherwise bit is 1 while ADC is reading
            uint16_Buffer += ADC;
    }while (adc_interrupt_counter<4);                   // uint16_Buffer accumulates the sum of 4 ADC readings

  uint16_t railvoltage = InternalReferenceConstant / (uint16_Buffer>>2); // convert average ADC reading into railvoltage in mV // bitshift >>2 same as divide by 4

  bitClear(ADCSRA,ADIE);                                // turn off the ADC interrupts
  bitSet(ACSR,ADIF);                                    // clears any ADC interrupt flags in the processor
  ADMUX = default_ADMUX;                                // restore defaults
  ADCSRA = 0; power_adc_disable();                      // turn off ADC
  // re-enable the other peripherals we turned off with power_all_disable();
  power_timer0_enable(); power_twi_enable();
  if(ECHO_TO_SERIAL){power_usart0_enable();}

  if (railvoltage < LowestBattery) {LowestBattery = railvoltage;}
  
  if (railvoltage < systemShutdownVoltage){
        if(ECHO_TO_SERIAL){
          Serial.print(railvoltage); Serial.println(F(" battery voltage too low!")); Serial.flush();
        }
      error_shutdown();}                                // this shuts down the logger
         
  return railvoltage; 
}  // terminator for readBattery()

ISR (ADC_vect){ adc_interrupt_counter++;}  // called by the readBattery() FUNCTION above
// the ADC_vect ISR executes when the ADC generates interrupts - increments the adc_interrupt_counter variable with each new reading

// ======================================================================================
//   *  *   *  *  *  *  *  *  *  *  ERROR HANDLER   *  *  *  *  *  *  *  *  *  *  *  *  *
// ======================================================================================
void error_shutdown() {

if(ECHO_TO_SERIAL){
   Serial.print(F("Shutting Down: LowBattery @")); Serial.println(LowestBattery); Serial.flush();
}
  
  power_twi_enable();
  RTC_DS3231_turnOffBothAlarms(); //before we disable I2C
  noInterrupts ();
  bitSet(EIFR,INTF0);                               // clear flag for interrupt 0  see: https://gammon.com.au/interrupts
  bitSet(EIFR,INTF1);                               // clear flag for interrupt 1
  interrupts (); 

  pinMode(13, OUTPUT);                              // the built-in red led on the Arduino is on D13
  for (byte CNTR = 0; CNTR < 253; CNTR++) {         // FLASH red indicator LED to indicate error state
    PINB = B00100000;                               // writing a bit to the pin register TOGGLES D13 LED pullup resistor On/Off
    LowPower.powerDown(SLEEP_250MS, ADC_ON, BOD_OFF);
  }
  
  bitSet(ACSR,ACD);                                 // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
  ADCSRA = 0; SPCR = 0;                             // Disable ADC & SPI // only use PRR after disabling the peripheral clocks, otherwise the ADC is "frozen" in an active state drawing power
  power_all_disable();                              // Turn ALL the internal peripherals off - must come after disabling ADC

 // FLOAT digital pins so no current can leak after shutdown:
    for (int i = 2; i <=13; i++) { 
    pinMode(i, INPUT);  digitalWrite(i, LOW);  
    }
 if(!ECHO_TO_SERIAL){
    pinMode(0, INPUT);  digitalWrite(0, LOW);     // d1&0 set as inputs/low
    pinMode(1, INPUT);  digitalWrite(1, LOW);     // don't change USART pins in ECHO_TO_SERIAL debug mode
  }// #endif

  pinMode(A0, INPUT);  digitalWrite(A0, LOW);
  pinMode(A1, INPUT);  digitalWrite(A1, LOW);
  pinMode(A2, INPUT);  digitalWrite(A2, LOW);
  pinMode(A3, INPUT);  digitalWrite(A3, LOW);
  pinMode(A4, INPUT);  digitalWrite(A4, LOW);
  pinMode(A5, INPUT);  digitalWrite(A5, LOW);     //Note: A4 & A5 are still connected to I2C pullups on RTC module
  
  LowPower.powerDown(SLEEP_FOREVER,ADC_ON,BOD_OFF);  //ADC_ON is a bit confusing here - what it really means is 'leave the existing ADC state alone', and we have already disabled it with power_all_disable();
}

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
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); // coincell battery recovery
  
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
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); // coincell battery recovery
  
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

void RTC_DS3231_turnOffBothAlarms() {             // from http://forum.arduino.cc/index.php?topic=109062.0
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
  if (m > 2 && (y % 4 == 0)) // modulo checks (if is LeapYear) add extra day
    ++days;
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

#ifdef Si7051_Address  //compiler will include these functions up to the next #endif statement

void initializeSI7051() {
//---------------------------------------------------------------------------------------------
  if(ECHO_TO_SERIAL){
  Serial.println(F("INIT: SI7051 sensor..."));Serial.flush(); 
  }
  
  Wire.beginTransmission(Si7051_Address);
  Wire.write(0xE6);                         //settings regester
  Wire.write(0x0);                          //when bit 0 and bit 7 to zero, sets highest 14 bit resolution on sensor
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

  //first transaction targets the memory register(s) we want to read
  Wire.beginTransmission(Si7051_Address);
  Wire.write(0xF3);
  Wire.write(Si7051_Address);
  byteBuffer1 = Wire.endTransmission();
    if ( byteBuffer1 != 0) {
      if(ECHO_TO_SERIAL){                   //if echo is on, we are in debug mode, and errors force a halt.
      Serial.println(F("FAIL request data:si7051")); Serial.flush(); 
      error_shutdown();
      }
    byteBuffer1=0;
  } 
  //delay(10);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF); // some of my sensors need settling time after a register change.

  //second I2C transaction requests the data from those memory registers
  Wire.requestFrom(Si7051_Address, 2);       //this sensor stores its output in two memory registers
  byte msb = Wire.read();
  byte lsb = Wire.read();
  uint16_t val= msb << 8 | lsb;             // merge the two output register bytes into one integer number
  return val;                               //note:  to calculate TEMP_degC =(175.26*val) / 65536 - 46.85;
}

#endif // end of code for SI7051 sensor included via controlling #ifdef / #endif statements
// ========================================================================================


#if defined(readLDR_onD6) || defined(readNTC_onD7)
// ============================================================================================================
// ===========================================================================================================
// Based on Nick Gammon's frequency counter at https://www.gammon.com.au/forum/?id=11504
// Adapted by Edward Mallon for ratiometric reading of resistive sensors with ICU on D8. For a details see 
// https://thecavepearlproject.org/2019/03/25/using-arduinos-input-capture-unit-for-high-resolution-sensor-readings/
// NewSensorReading =(elapsedTimeSensor * referenceResistorValue) / elapsedTimeReff;
// with [104] caps this calculation fits within uint32_t variables
// HOWEVER w 105 capacitors must change read functions & cast this calculation to higher uint64_t bit depth
// NTC_NewReading=((uint64_t)elapsedTimeSensor * (uint64_t)referenceResistorValue) / elapsedTimeReff;
// FAST PORT COMMANDS used throughout these functions to increase timing accuracy

//-----------------------------------------------------------------------------------------
void ConditionCapacitorOnD8(){            // 2023-06-20: internal pullup resistor on D8 as ref- D6=LDR (not D9), D7=10kNTC
//-----------------------------------------------------------------------------------------
  // Charge & discharge [104] 0.1uF capacitor through 300ohm resistor on D8 (5xRC takes about 0.15 milliseconds each direction)
  // RC disharge time calculator: https://www.digikey.com/en/resources/conversion-calculators/conversion-calculator-time-constant
  // this cycles the capacitor to bring it to a 'standardised' residual level/condition
  // AND floats [Input & Low] all pins with resistor/sensors connected to that common capacitor 

  bitSet(ACSR,ACD);                       // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
  ADCSRA = 0; SPCR = 0;                   // Disable ADC & SPI
  power_all_disable();                    // must come after disabling ADC
  power_timer1_enable();                  // need Timer1 to clock the change on the D8 pin

  //MUST float ALL pins with resistor/sensors connected to the common capacitor
  //or you wont be able to read the others!
  bitClear(PORTB,0);bitClear(DDRB,0);     //D8 [300Ω] to LOW(0) & INPUT(0) 
  PORTD &= B00111111;DDRD &= B00111111;   //D6 [LDR] & D7[NTC] to LOW(0) & INPUT(0)
  
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
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); // + extra 2msec for osc start! //ADC_ON leaves the already sleeping ACD alone as is
  // NOTE: 15MS is overkill:  5T with 300ohm&105(1uF) is 1.5 msec, with 300Ω&104(100nF) 5RC is only 0.15ms 
  bitClear(DDRB,0);  // pinMode(8,INPUT); 
  // could insert the timer2 delays here from void Write_i2c_eeprom_array 
  // to make this dischage faster? but that works in sleep mode IDLE?

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

  bitSet(ACSR,ACD);                       // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
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
  bitSet(PORTD,6);              // D6 HIGH -> now charging the cap through 10k ref
  do{
  interrupts(); sleep_cpu(); noInterrupts();
  }while(!triggered);         // trapped here till TIMER1_CAPT_vect changes triggered to true

  //elapsedTime = timer1CounterValue; // capped in ISR (TIMER1_OVF_vect) at 65534
  interrupts ();                // can't use I2C bus or powerDown without interrupts...
  // sleep_disable() inside ISR (TIMER1_CAPT_vect)
  
    bitClear(DDRD,6); bitClear(PORTD,6); // D6 INPUT & LOW -> stops the capacitor charge
    bitSet(DDRB,0); //bitClear(PORTB,0); //D8 OUTPUT & (already) LOW to discharge the capacitor through 300Ω on D8
      LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); // sleep processor during the discharge
    bitClear(DDRB,0);                 // D8 INPUT 
    
//re-enable timers after Dpin read:
    power_timer1_disable();    // no longer need Timer1
    power_timer0_enable();     // MUST RE-ENABLE for any delay/serial statements,etc
    power_twi_enable();        // cant use I2C bus without interrupts enabled
    if(ECHO_TO_SERIAL){
      power_usart0_enable();   
      Serial.print(F(" D6raw:"));Serial.print(timer1CounterValue);;Serial.flush(); 
      }

return timer1CounterValue;
} // terminates ReadD6riseTime 
//-----------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------
uint16_t ReadD7riseTimeOnD8(){
//-----------------------------------------------------------------------------------------
  noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_1;interrupts(); //this function must run a max clock speed

  bitSet(ACSR,ACD);                       // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
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
    LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
    bitClear(DDRB,0);                     //  D8 INPUT 

//re-enable timers after Dpin read:
    power_timer1_disable();    // no longer need Timer1
    power_timer0_enable();     // MUST RE-ENABLE for any delay/serial statements,etc
    power_twi_enable();        // cant use I2C bus without interrupts enabled
    if(ECHO_TO_SERIAL){
      power_usart0_enable();   
      Serial.print(F(" D7raw:"));Serial.print(timer1CounterValue);Serial.flush(); 
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

  bitSet(ACSR,ACD);                       // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
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
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); 
  bitClear(DDRB,0);                     //  D8 INPUT 

  //re-enable timers after Dpin read:
    power_timer1_disable();    // no longer need Timer1
    power_timer0_enable();     // MUST RE-ENABLE for any delay/serial statements,etc
    power_twi_enable();        // cant use I2C bus without interrupts enabled
      if(ECHO_TO_SERIAL){
      power_usart0_enable();   
      Serial.print(F(" D8(35k ref):"));Serial.print(timer1CounterValue);Serial.flush(); 
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
#endif //end of #if defined(readLDR_onD6) || defined(readNTC_onD7)
// ============================================================================================================
// ============================================================================================================
