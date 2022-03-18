#include <Arduino.h>    //  Arduino Ref Required by PlatformIO
#include <Wire.h> //  I2C Library https://www.arduino.cc/en/reference/wire
#include <Streaming.h>  // C++ Style Output << https://github.com/janelia-arduino/Streaming
/* OLED */
#include <Adafruit_GFX.h> //  Graphics Library https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_SSD1306.h> //  OLED driver Library https://github.com/adafruit/Adafruit_SSD1306
#include <braciola5pt7b.h>  // Mono Space Font 

/* INA219 */
 #include <Adafruit_INA219.h> //  INA219 driver https://github.com/adafruit/Adafruit_INA219

// BEN this needs work, power down & EEPROM stuff
/* Cross Power-Down State Saving */    
//#include <EEPROMWearLevel.h> 
//#include <EEPROM.h>
//#include <FlashStorage_STM32.h>   //  Flash Storage / EEPROM Emulation Library (STM32F411 has no internal EEPROM) https://github.com/khoih-prog/FlashStorage_STM32
#include <extEEPROM.h>  //  External EEPROM Library https://github.com/PaoloP74/extEEPROM

extEEPROM myEEPROM(kbits_256, 1, 64, 0x50); // Initialize External EEPROM extEEPROM myEEPROM(eeprom_size_t devCap, byte nDev, unsigned int pgSize, byte busAddr);



#define EEPROM_LAYOUT_VERSION 0                                               // every time amount of indexes changes, ie we change the layout of the EEPROM map, we must increment this to let the library know to initialise EEPROM
#define AMOUNT_OF_INDEXES     3                                               // 3 variables will be saved : 1 uint8_t and 2 uint32_t (hopefully the library will account for this)
#define INDEX_disp            100
#define INDEX_mAh             200
#define INDEX_mWh             300
#define OPEN                  0
#define SAVE                  1

/* Pinout Table - STM32F411CE Blackpill V2/ 3*/ 
// I2C - SCL/SCK = PB6
// I2C - SDA = PB7
#define button1          PB15
#define modeButton_LED   PB13
#define powerButton_LED  PB12
#define power_off_detect PB1
#define longHoldDuration 400
#define dim_disp_after   1200000                                         // dim display after 1 200 000 ms = 20 min

// BLE HM-10 / AT-09 Software Serial
#define bleTX PA9
#define bleRX  PA10
#define bleState PB0   // 1HZ blink is unconnected, on is Connected
bool bleHeader = false;
//  OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 6                                                          // assigned to some unused pin - hardware reset not used (and not even available)
                                            
Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  // construct a display object named "OLED"

//  INA219 
Adafruit_INA219 ina219(0x40);                                                       // construct a power monitor object named "ina219"
Adafruit_INA219 ina219Batt(0x41);

#define ON  '1'
#define OFF '0'

#define voltage 0
#define current 1
#define wattage 2

#define max_allowed_current 3100      // INA219 with .1 Ohm resistore can measure up to 3.2A - to discourage abuse, do not display any values above this
#define max_allowed_voltage 25100

#define num_of_last_mode 5         // excluding the dark mode, which we don't wish to boot to (we will also not boot to BATT mode either)
#define A            0
#define V_A_W        1
#define V_A_W_Ah_Wh  2
#define V_A_W_MAX    3
#define SER          4
#define BATT         5
#define ScreenOFF    6   

volatile uint8_t buttonWasPressed   = 0;
//uint32_t         timeOfLastInterrupt;             // used for push button interrupt debouncing
//uint32_t         t1 = 0;                          // used for display refresh clocking
//uint32_t         t2 = 0;                          // used for serial print clocking : 1Hz
//uint32_t         t3 = 0;                          // used for easter egg (invert display when rapid clicking)
//uint32_t         t4 = 0;                          // used for DUT value injection clocking

int         timeOfLastInterrupt;             // used for push button interrupt debouncing
int         t1 = 0;                          // used for display refresh clocking
int         t2 = 0;                          // used for serial print clocking : 1Hz
int         t3 = 0;                          // used for easter egg (invert display when rapid clicking)
int         t4 = 0;                          // used for DUT value injection clocking


uint8_t  DUT_mode                    = 0;
uint8_t  currentDisplayMode          = 1;
uint8_t  mode_changes_in_last_5s     = 0;
uint8_t  dimMode                     = 0;
uint8_t  disp_inverted               = 0;
uint8_t  batt_low_nag_screen         = 0;
uint8_t  system_still_initialising   = 1;

uint16_t i = 0;
uint16_t j = 0;

int16_t battLevel = 5000;                                                                             // smoothed battery level for global use (must be initialised to >2800 otherwise will always jump to batt screen)
uint8_t battState = 5;


int16_t rawBusVoltage      = 0;                                                                       // int16_t getBusVoltage_raw()          : ±        32 767            mV
int16_t rawBusBattVoltage      = 0;  
int16_t rawCurrent         = 0;                                                                       // int16_t getCurrent_raw()             : ±        32 767     1/10 * mA

// data saved for [0.0 , 3.1] A n .1A increments   ::   for example - offset for 1.4A is cal[14] =      15, meaning when we should be reading exactly  1.400A, we read  2.415A instead
// we ALWAYS read ABOVE !!!
const PROGMEM uint8_t current_cal_data[]  = { 0,  1,  2,  2,  4,  4,  5,  7,  7,  8,  9,  11,  12,  14,  15,  17,  19,  21,  24,  26,  29,  31,  35,  39,  43,  43,  51,  53,  59,  63,  71 };
// data saved for [0.0 , 25.0] V in 1V increments   ::   for example - offset for 14V is cal[14]  =  50,    meaning when we should be reading exactly 14.000V, we read 13.950V instead
// we ALWAYS read BELOW !!!
const PROGMEM uint8_t voltage_cal_data[]  = { 0, 0, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 50, 50, 50, 50, 50, 50, 50, 60, 60, 60, 60, 60                                         };

int16_t maxBusVoltage      = 0;
int16_t maxCurrent         = 0;
int32_t maxPower           = 0;

int16_t filteredBusVoltage     = 0;
int16_t filteredBusBattVoltage = 0;
int16_t filteredCurrent        = 0;
int32_t filteredPower          = 0;                                                                   // int32_t = filteredCurrent * filteredBusVoltage : ±     2 147 483 647        uW

int16_t voltageAccumulator = 0;
int16_t voltageBattAccumulator = 0;
int16_t currentAccumulator = 0;

volatile uint32_t mWs = 0; 
volatile uint32_t mWh = 0;
volatile uint32_t mAh = 0;
volatile uint32_t mAs = 0;


uint8_t  negativeCurrent    = 0;
uint8_t  maxCurrentNegative = 0;
String   reading            = String(7);

// Put functions up top for PlatformIO //
/*
   Filtered values will quickly follow raw values in steps of 10, then settle on the average of the raw values over time
*/
void valueFilterR(int16_t* rawValue, int16_t* filteredValue, int16_t* accumulator, const int16_t maxInstantDeviation, const int16_t maxTotalDeviation) {

  if ( abs(*rawValue - *filteredValue) >= maxInstantDeviation ) *filteredValue = *rawValue;                       // precisely follow the raw value when it's racing up/down

  else
  {
    *accumulator += *filteredValue - *rawValue; // we keep track of all the deviations we incur by displaying
                                                // a filtered version (positive & negative deviations should cancel each other out)
                                                // !!!!!!!!!!!!! *accumulator++ will increment the address of
                                                //               the pointer not the value stored at its address !!!!!!!!!!!!!

    if (*accumulator >= maxTotalDeviation)
    {
      *accumulator = 0;
      *filteredValue = *filteredValue - 1; // we incured positive deviations : decrease displayed value to prevent this
    }

    if (*accumulator <= -maxTotalDeviation)
    {
      *accumulator = 0;
      *filteredValue = *filteredValue + 1; // we incured negative deviations : increase displayed value to prevent this
    }
  }
}


/*
 * Center justifies Strings in a gives space
 * 
 * Accepts a pointer to the String to be modified, and modifies it in place
 * 
 * set noMod to 1 to leave setCursor the way it already is
 */
void center_and_print(String* input, const uint8_t fixed_output_length, const uint8_t pos_x, const uint8_t pos_y, uint8_t dontTouchCoords) {

    uint8_t padding;
    
//    j = 0;
//    while (input->charAt(j) == ' ')                                       // remove preceeding white spaces : they fuck up our centering
//         {
//         input->remove(j , 1);                     
//         }
//    
    padding = ( fixed_output_length - input->length() );                  // compute how many white spaces to add left/right

    if ( !dontTouchCoords ) if (padding % 2 != 0)  OLED.setCursor(pos_x + 7, pos_y);              // expecting font size of 5px with 3x scaler : 15px/char => we need to shift right by half a char : 7px
                            else                   OLED.setCursor(pos_x , pos_y);

    padding /= 2;

    for (j=1; j<=padding; j++) *input = *input + F(" ");
    for (j=1; j<=padding; j++) *input = " " + *input;

    OLED << *input;
  
}



/*
   Returns the length of the number it's given
*/
uint8_t numberLength(int32_t input) {
  i = 0;

  if (input == 0) return 1;

  if (input < 0) input = -input;

  while (input > 0) {
                    i++;
                    input /= 10;
                    }

  return i;
}

/*
 * Converts:
 *          [0 , 99999] -> ab.cde
 * 
 * Returns 6 character string with inputs mentioned above (does output longer strings when fed larger numbers)
 *          
 *          current_voltage_wattage :: 0 - voltage
 *                                     1 - current
 *                                     2 - wattage
 */

String divideBy1000(int32_t input, const uint8_t negativeSign, const uint8_t outputLength, uint8_t current_voltage_wattage) {

  reading = "";

  if (outputLength == 6){
    switch (current_voltage_wattage)
    {
    case voltage:
    {
      if (input > max_allowed_voltage)
        return F(" O.L  ");
    }
    break;
    case current:
    {
      if (input > max_allowed_current)
        return F(" O.L  ");
    }
    break;
    }

    switch (numberLength(input))
    {
    case 1:
    { // 1       -> __.001

      if (negativeSign)
        reading.concat(F(" -.00"));
      else
        reading.concat(F("  .00"));
      reading.concat(input);
    }
    break;
    case 2:
    { // 12      -> __.012
      if (negativeSign)
        reading.concat(F(" -.0"));
      else
        reading.concat(F("  .0"));
      reading.concat(input);
    }
    break;
    case 3:
    { // 123     -> __.123
      if (negativeSign)
        reading.concat(F(" -."));
      else
        reading.concat(F("  ."));
      reading.concat(input);
    }
    break;
    case 4:
    { // 1 234   -> _1.234
      if (negativeSign)
        reading.concat(F("-"));
      else
        reading.concat(F(" "));
      reading.concat(input / 1000);
      reading.concat(".");
      for (j = 1; j <= 3 - numberLength(input % 1000); j++)
        reading.concat(F("0")); // 1 003 -> _1.003 not 1.3
      reading.concat(input % 1000);
    }
    break;
    default:
    { // 12 345  -> 12.345
      if (negativeSign)
        reading.concat(F("-"));
      reading.concat(input / 1000);
      reading.concat(F("."));
      for (j = 1; j <= 3 - numberLength(input % 1000); j++)
        reading.concat(F("0")); // 12 003 -> 12.003 not 12.3
      reading.concat(input % 1000);
    }
    break;
    }
  }

  if (outputLength == 4)
  {
    switch (current_voltage_wattage)
    {
    case voltage:
    {
      if (input > max_allowed_voltage)
        return F("O.L ");
    }
    break;
    case current:
    {
      if (input > max_allowed_current)
        return F("O.L ");
    }
    break;
    }

    switch (numberLength(input))
    {
    case 1:
    {
      if (negativeSign)
        reading = F("-");
      else
        reading = F(" ");

      if (input % 10 >= 5)
        reading += F(".01");
      else
        reading += F(".00");
    }
    break;
    case 2:
    {
      if (negativeSign)
        reading = F("-");
      else
        reading = F(" ");

      if (input % 10 >= 5)
        if (input / 10 % 10 == 9)
          reading += (String)F(".10"); // 98 -> _.10
        else
          reading += (String)F(".0") + String(input / 10 + 1); // 38 -> _.04
      else
        reading += (String)F(".0") + String(input / 10); // 32 -> _.03
    }
    break;
    case 3:
    {
      if (negativeSign)
        if (input % 10 >= 5)
          if ((input / 10 % 10 == 9) && (input / 100 % 10 == 9))
            reading += (String)F("-1.0"); // 548
          else
            reading += (String)F("-.") + String(input / 10 + 1); // -> .55
        else
          reading += (String)F("-.") + String(input / 10);

      else if (input % 10 >= 5)
        if ((input / 10 % 10 == 9) && (input / 100 % 10 == 9))
          reading += (String)F("1.00");
        else
          reading += (String)F(" .") + String(input / 10 + 1);
      else
        reading += (String)F(" .") + String(input / 10);
    }
    break;
    case 4:
    { // only 1 decimal : cut off 2 digits : we look at the last 2 together
      if (negativeSign)
        if (input % 100 >= 50)
          if (input / 100 % 10 == 9)
            reading = (String)F("-") + String(input / 1000 + 1) + F(".0");
          else
            reading = (String)F("-") + String(input / 1000) + F(".") + String(input / 100 % 10 + 1);
        else
          reading = (String)F("-") + String(input / 1000) + F(".") + String(input / 100 % 10);

      // 2 decimal places : we first look at the last digit
      else if (input % 10 >= 5)
        if (input / 10 % 10 == 9)
          if (input / 100 % 10 == 9)
            reading = String(input / 1000 + 1) + (String)F(".00"); // 3 997 -> 4.00
          else
            reading = String(input / 1000) + (String)F(".") + String(input / 10 % 100 + 1);
        else
          reading = String(input / 1000) + (String)F(".") + String(input / 100 % 10) + String(input / 10 % 10 + 1);
      else
        reading = String(input / 1000) + (String)F(".") + String(input / 100 % 10) + String(input / 10 % 10);
    }
    break;
    case 5:
    {
      if (input >= 99500)
        return F("O.L");

      // no decimals : we look at last 3 digits together
      if (negativeSign)
        if (input % 1000 >= 500)
          reading = (String)F("-") + String(input / 1000 + 1) + F(" ");
        else
          reading = (String)F("-") + String(input / 1000) + F(" ");

      // 1 decimal place : we look at last 2 digits together
      else if (input % 100 >= 50)
        if (input / 100 % 10 == 9)
          reading = String(input / 1000 + 1) + (String)F(".0");
        else
          reading = String(input / 1000) + (String)F(".") + String(input / 100 % 10 + 1);
      else
        reading = String(input / 1000) + (String)F(".") + String(input / 100 % 10);
    }
    break;
    default:
    {
      return F("O.L");
    }
    }
  }

  ///////////////////////////////while (reading.length() < 4) reading.concat(F("0"));             // 4.9 > 4.90
  
  return reading;

}


/*
   Converts:
           rawPower : / 1 000 000 and keeping 3 decimal points
                                  also rounds least significant digit accordingly
*/
String divideBy1000000(int32_t input, const uint8_t negativeSign, const uint8_t outputLength) {

  if ( (input % 1000) >= 500 )             return divideBy1000 (input / 1000 + 1 , negativeSign , outputLength, wattage);                     // 3 913 uW -> 4 000 uW ->  .004 W
  else                                     return divideBy1000 (input / 1000     , negativeSign , outputLength, wattage);

}

/*
 * Prints out large numbers by adding a separating space between each set of 3 digits AND center justifying on a 9 character field
 * 
 * Font Size = 2 [ 5 character space ]
 *          1 -->   __1__
 *         12 --> > _12_      (will have to be indented half a space manually to be centered)
 *        123 -->   _123_
 *      1 234 -->   1_234
 *      
 * Font Size = 1 [ 9 character space ]
 *     12 345 --> > _12 345_      (will have to be indented half a space manually to be center)
 *    123 456 -->   _123 456_
 *  1 234 567 -->   1 234 567
 *  9 999 999 -->   9 999 999
 * 10 000 000 -->   10 mil +
 * 
 */
 void format_and_print_large_number(uint32_t input, const uint8_t pos_X, const uint8_t pos_Y) {

      i = numberLength(input);
      j = 0;
      uint8_t constantWidth;


      //if (i > 7) center_and_print(*String(F("10 mil +")), constantWidth, 0,0, 1);

      if (i >= 5)
      {
        constantWidth = 9;

        OLED.setTextSize(1);

        if (i == 5)
          OLED.setCursor(pos_X + 2 + 3, pos_Y - 2); // +3 to adjust centering for small font in the case of 5 digit numbers _12_444_ is 8 chars on a 9 char space
        else
          OLED.setCursor(pos_X + 3, pos_Y - 2); // 2 digit long numbers are the only ones who can't be centered, the rest get placed exactly as instructed
      }
      else
      {
        constantWidth = 5;

        OLED.setTextSize(2);

        if (i == 2)
          OLED.setCursor(pos_X + 6, pos_Y); // Font size: 5, scaler: 2 => 1 char is 10px wide, half a char is 5px wide, indent 5 to center 4 char string in 5 char space
        else
          OLED.setCursor(pos_X, pos_Y); // 2 digit long numbers are the only ones who can't be centered, the rest get placed exactly as instructed
      }

      reading = "";

      while (i != 0)
      { // add separating space between each 3 digits

        reading = String(input % 10) + reading;
        input /= 10;
        i--;
        j++;

        if (j == 3 && i != 0)
        {
          j = 0;
          reading = " " + reading;
        }
      }

      center_and_print(&reading, constantWidth, 0, 0, 1);
 }


/*
 * I N T E R R U P T    S E R V I C E    R O U T I N E
 * 
 * Sets:
 *      buttonWasPressed  ::  on interrupt 
 */
void registerButtonPress() {

  if (millis() < 500)
    return;               // prevent false triggering @ bootup & every 50 days for half a second we won't register button presses
                          // necessarry when using internal PULLUP

  if (abs((int)millis() - timeOfLastInterrupt) > 350)
  {                       // debouncing if less than 350ms passed since we were FIRST time here, we do nothing
    if (dimMode == 0)
      buttonWasPressed = 1;         // when in dim mode, first button press deactivates it (by simply setting the timeOfLastInterrupt), and doesn't flag it as a normal button press
    timeOfLastInterrupt = millis(); // dimmer function will clear the dimMode flag
  }
  else
    return;
}


/*
 * Processes any interrupts that might have occured since last time it was called
 */
void humanInterfaceController () {
  
  uint16_t buttonPressDuration = 0;

  /* Handle Button Press if Present */
  if (buttonWasPressed)
  {
    while (digitalRead(button1) == LOW)
    { // this function is called only after the button pressed flag has been set; we check if button STILL pressed
      buttonPressDuration++;
      delay(1);

      if (buttonPressDuration > longHoldDuration)
      {
        switch (currentDisplayMode)
        {
        /* Enter Device Under Test Mode and seed random Values for V, A and W */
        case A:
        {
        } // with no break; this will go on to execute the next case (which is what we want)
        case V_A_W:
        {
          OLED.clearDisplay();
          OLED.setTextSize(2);
          OLED.setCursor(0, 36);
          if (!DUT_mode)
          {
            OLED << F("DUT_mode=1;");
            DUT_mode = 1;
          }
          else
          {
            OLED << F("DUT_mode=0;");
            DUT_mode = 0;
          }
          OLED.display();
        }
        break;

        /* Reset Accumulator Variables for mAh and mWh */
        case SER:
        {
          
        } // with no break; this will go on to execute the next case (which is what we want)
        case V_A_W_Ah_Wh:
        {
          OLED.clearDisplay();
          OLED.setTextSize(2);
          OLED.setCursor(20, 20);
          OLED << F("mAh = 0;");
          OLED.setCursor(20, 51);
          OLED << F("mWh = 0;");
          OLED.display();

          mAh = 0;
          mAs = 0;

          mWh = 0;
          mWs = 0;
        }
        break;

        /* Reset Maximum Variables for V A and W */
        case V_A_W_MAX:
        {
          OLED.clearDisplay();
          OLED.setTextSize(1);
          OLED.setCursor(20, 17);
          OLED << F("maxVoltage = 0;");
          OLED.setCursor(20, 51);
          OLED << F("maxCurrent = 0;");
          OLED.display();

          maxBusVoltage = 0;
          maxCurrent = 0;
          maxPower = 0;
        }
        break;
        case BATT:
        {
          OLED.clearDisplay();
          OLED.setTextSize(1);
          OLED.setCursor(20, 20);
          OLED << F("up time / use");

          format_and_print_large_number(millis() / 1000 / 60 + 1, 6, 35);
          OLED << F("min");
          format_and_print_large_number(ina219Batt.getCurrent_mA(), 6,51  );
          OLED << F("mA");
          OLED.display();
        }
        break;
        }
        delay(500); // have the text stay on the screen for at least half a second
        while (digitalRead(button1) == LOW)
          ; // have the text stay on the screen for as long as the button is pressed
      }
    }
    buttonWasPressed = 0; // this used to be above while, wrongly:
                          // the interrupt for button presses would fire during our long presses and set the flag to 1 again
                          // can't sit inside the while, because we may never go in it, and still need the flag cleared

    if (buttonPressDuration > longHoldDuration)
      return; // no more mode changes after long press
  }

  /* Handle Mode Change if Applicable */
  if (currentDisplayMode == num_of_last_mode+1) currentDisplayMode = 0;
  else                                          currentDisplayMode ++ ;


  /* Easter - Egg : Inverts Display after 10 mode changes in 5 seconds (start counting on first mode change) */
  if (mode_changes_in_last_5s == 0) {
                                    t3 = millis();
                                    mode_changes_in_last_5s++;
                                    }
       // cast to int (int) stm32                             
  if ( abs((int)millis() - t3) < 5000 ) {
                                   mode_changes_in_last_5s++;

                                   if (mode_changes_in_last_5s == 10) if (disp_inverted) disp_inverted = 0;
                                                                      else               disp_inverted = 1;
                                   
                                   OLED.invertDisplay(disp_inverted);
                                   }
  else                             mode_changes_in_last_5s = 0;
                                   

}

/*
   Sets:
        rawBusVoltage      : V convert to mV
        rawCurrent         : mA

        filteredBusVoltage : mV
        filteredCurrent    : mA
        filteredPower      : uW
*/
void aquireData() {

  if (!DUT_mode) {
                 /*** Raw Values are Aquired from INA219 ***/
                 rawBusVoltage = ina219.getBusVoltage_raw();
                 rawBusBattVoltage = ina219Batt.getBusVoltage_raw();
                 //rawBusVoltage    = ina219.getBusVoltage_V();
               
                rawCurrent    = ina219.getCurrent_raw();
                 //rawCurrent   = ina219.getCurrent_mA();
                 if (rawCurrent % 10 >= 5)   rawCurrent = rawCurrent / 10 + 1;                        // 1237 -> 124
                 else                        rawCurrent = rawCurrent / 10    ;                        // 1234 -? 123
                 if ( abs(rawCurrent) <= 1 ) rawCurrent = 0;
                 /******************************************/

     
                 /*** Separate Sign for Current Measurement ***/
                 if ( rawCurrent <= 0 ) {                                // Actual Negative Values will be Regarded as Positive, Positive values will be regarded as Negative (makes more sense when doing 3 wire measurements)
                                        rawCurrent = -rawCurrent;
                                        negativeCurrent = 0;
                                        }
                 else                   negativeCurrent = 1;
                 /*********************************************/

    
                 /*** Adjust Raw Values to Null Out their Error :: Voltage ALWAYS reads LOW :: Current ALWAYS reads HIGH ***/
                 if (rawBusVoltage/1 <= 25)                // was 1000                                                       // Make sure we read within bounds
                     rawBusVoltage = rawBusVoltage + pgm_read_byte_near(voltage_cal_data + rawBusVoltage / 1); // was 1000
                
                 if (rawCurrent/100     <= 30)                                                                       // Make sure we read within bounds
                     rawCurrent    = rawCurrent    - pgm_read_byte_near(current_cal_data + rawCurrent/100    );      
                 /*********************************************************************************************************/

    
                 /*** Filter Display Values to reduce jitter ***/
                 valueFilterR (&rawBusVoltage, &filteredBusVoltage, &voltageAccumulator, 10, 10);                    // we pass this function the addresses of the input parameters, since they are to be operated on directly
                 valueFilterR (&rawCurrent   , &filteredCurrent   , &currentAccumulator, 10, 10);
                 valueFilterR (&rawBusBattVoltage, &filteredBusBattVoltage, &voltageBattAccumulator, 100, 100);     
                 //BING
                 /**********************************************/
                 }
                 
  else           {
                 /*** Inject random values into the pipeline ***/
                 if ( abs((int)millis() - t4) > 2000 ) {
                                                  t4 = millis();
                   
                                                  rawBusVoltage   = random( 0, max_allowed_voltage + 5001);            // get O L to show up too
                                                  rawCurrent      = random( 0, max_allowed_current + 701 );
                                                  negativeCurrent = random( 0, 2);                                     // random(a, b) = [a, b) !!!
                                                  }
                 /**********************************************/


                 /*** No real filtering needed in this case ***/   
                 filteredBusVoltage = rawBusVoltage;
                 filteredBusBattVoltage = rawBusBattVoltage;
                 filteredCurrent    = rawCurrent;
                 /*********************************************/
                 }


  /*** Compute Power based on the on screen (filtered) values ***/
  filteredPower      = (int32_t) filteredCurrent * filteredBusVoltage;                                // !!!!!!!!!!!!! CAST TO 32b OTHERWISE MULTIPLICATION DONE IN 16b THEN EXTENDED !!!!!!!!!!!!!
  /**************************************************************/


  /*** Compute MAX Values for V, A and W ***/
  if ( rawBusVoltage > maxBusVoltage ) maxBusVoltage      = rawBusVoltage;
  if ( rawCurrent    > maxCurrent    ) {
                                       maxCurrent         = rawCurrent     ;
                                       maxCurrentNegative = negativeCurrent;
                                       }
                                                                                                        /**************** IMPORTANT ****************/
  if ( filteredPower > maxPower     )  maxPower           = (int32_t) maxCurrent * maxBusVoltage;       // maxPower is independent from maxVoltage and maxCurrent !!! For example, 2V at 2A is 4W, let's say that's the max
                                                                                                        // but 4V at .1A gives a maxVoltage of 4V
                                                                                                        // and .1V at 4A gives a maxCurrent of 4A
                                                                                                        // but maxPower still is 4W, not the now displayed 4A*4W
  /*****************************************/ 
}

/*
 * I N T E R R U P T    S E R V I C E    R O U T I N E
 * 
 * Sets:
 *            mAh
 *            mWh
 * Reads:
 *            filteredCurrent : mA value once per second
*/
void mAh_mWh_computer() {

  mAs += abs(filteredCurrent);
  
  if (filteredPower%1000 < 500) mWs += abs(filteredPower/1000    );
  else                          mWs += abs(filteredPower/1000 + 1);


  if (mAs >= 3600) {
                   mAh = mAh + mAs / 3600;                 // accrue mAs into mAh
                   
                   mAs = mAs % 3600;                       // recirculate remainder from division with 3600
                 //mAs = mAs - mAs / 3600;                    !!!!!!!!!!!!! DOES NOT WORK !!!!!!!!!!!!!
                 //Serial.println(mAs);
                   }

   
  if (mWs >= 3600) {
                   mWh = mWh + mWs / 3600;                 // accrue mAs into mAh
                   
                   mWs = mWs % 3600;                       // recirculate remainder from division with 3600
                 //mAs = mAs - mAs / 3600;                    !!!!!!!!!!!!! DOES NOT WORK !!!!!!!!!!!!!
                 //Serial.println(mWs);
                   }

}


void turn(uint8_t LED, char desiredState) {

  if (desiredState == ON) digitalWrite(LED , HIGH);
  else                    digitalWrite(LED , LOW );
  
}


/*
   Sets OLED brightness : 1 - 255
*/
void setBrightness(uint8_t brightness)
{
  // OLED.dim(false);
  //     OLED.ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
  //   ssd1306_command(0x8F);

  OLED.ssd1306_command(SSD1306_SETCONTRAST);
  OLED.ssd1306_command(brightness);
 
}

/*
   Displays:
        voltage in V  from avgBusVoltage
        current in mA from avgCurrent
        power   in W  from avgPower
*/
void displayController(uint8_t mode) {

  OLED.clearDisplay();

  switch (mode) {
                case A :                {
                                        OLED.setTextSize(2);

                                        OLED.setCursor(60, 15);
                                        OLED << F("A");

                                        OLED.setTextSize(3);

                                        reading = divideBy1000(filteredCurrent    , negativeCurrent , 6, current);
                                        center_and_print(&reading , 6 , 13,50 , 0);
                                        }break;
    
                case V_A_W :            {
                                        OLED.setTextSize(1);
                                
                                        OLED.setCursor(122, 11);
                                        OLED << F("V");
                                        OLED.setCursor(122, 34);
                                        OLED << F("A");
                                        OLED.setCursor(122, 57);
                                        OLED << F("W");
                                
                                        OLED.setTextSize(3);
                                
                                        OLED.setCursor(0, 15);
                                        OLED << divideBy1000(filteredBusVoltage , 0               , 6, voltage);
                                
                                        OLED.setCursor(0, 38);
                                        OLED << divideBy1000(filteredCurrent    , negativeCurrent , 6, current);
                                
                                        OLED.setCursor(0, 61);
                                        OLED << divideBy1000000(filteredPower   , 0               , 6);

//                                        if (negativeCurrent)  Serial << F("-");
//                                                              Serial << filteredCurrent << F("\t\t") << filteredBusVoltage << F("\t\t") << filteredPower << F("\n");
                                        }break;
                
                case V_A_W_Ah_Wh :      {
                                        OLED.setTextSize(1);
                                
                                        OLED.setCursor(52, 8);
                                        OLED << F("V");
                                        OLED.setCursor(52, 34);
                                        OLED << F("A");
                                        OLED.setCursor(52, 60);
                                        OLED << F("W");
                                        OLED.setCursor(88, 11);
                                        OLED << F("mAh");
                                        OLED.setCursor(88, 41);
                                        OLED << F("mWh");

                                
                                        OLED.setTextSize(2);
                                
                                        OLED.setCursor(0, 10);
                                        //filteredBusVoltage = 5023;                 
                                        OLED << divideBy1000(filteredBusVoltage , 0               , 4, voltage);
                                
                                        OLED.setCursor(0, 36);
                                        //filteredCurrent = -3200;
                                        OLED << divideBy1000(filteredCurrent    , negativeCurrent , 4, current);
                                
                                        OLED.setCursor(0, 62);
                                        OLED << divideBy1000000(filteredPower   , 0               , 4);


                                        //OLED.writeFastVLine(64,0 , 64 , WHITE); 


//                                        OLED.setCursor(68,25);
//                                        OLED << voltageAccumulator;
//                                        OLED.setCursor(68,55);
//                                        OLED << currentAccumulator;
                                        
                                        format_and_print_large_number(mAh , 68, 25);
                                       //format_and_print_large_number(analogRead(power_off_detect) , 68, 25);
//                                        format_and_print_large_number(analogRead(battery_level) , 68,55);
                                        format_and_print_large_number(mWh , 68, 55);
                                        
                                        }break;

                case V_A_W_MAX :        {
                                        OLED.setTextSize(1);
                                        
                                        OLED.setCursor(122, 26);
                                        OLED << F("V");
                                        OLED.setCursor(122, 42);
                                        OLED << F("A");
                                        OLED.setCursor(122, 58);
                                        OLED << F("W");

//                                        OLED.setCursor(12,10);
//                                        OLED << F("M A X");
//                                        OLED.setCursor(80,10);
//                                        OLED << F("C U R");
                                        OLED.setCursor(20,10);
                                        OLED << F("M A X  -  C U R");
                                        


                                        OLED.setTextSize(2);

                                        // Display MAX Values on LEFT
                                        OLED.setCursor(0, 28);
                                        OLED << divideBy1000(maxBusVoltage , 0                  , 4, voltage);
                                        OLED.setCursor(0, 44);
                                        OLED << divideBy1000(maxCurrent    , maxCurrentNegative , 4, current);
                                        OLED.setCursor(0, 60);
                                        OLED << divideBy1000000(maxPower   , 0                  , 4);

                                        // Display CURR Values on RIGHT
                                        OLED.setCursor(68, 28);
                                        OLED << divideBy1000(filteredBusVoltage , 0                  , 4, voltage);
                                        OLED.setCursor(68, 44);
                                        OLED << divideBy1000(filteredCurrent    , negativeCurrent    , 4, current);
                                        OLED.setCursor(68, 60);
                                        OLED << divideBy1000000(filteredPower   , 0                  , 4);

                                        }break;

                case BATT :             {
                                        // If we are just starting up, aquire data to get batt voltage
                                        
                                        if(t1 < 3000){

                                          aquireData();
                                        }
                                        
                                        //noInterrupts();                                              // spurreous values here - try with no interrupts
                                        //int16_t battReading = analogRead(battery_level);
                                        //int16_t battReading = ina219Batt.getBusVoltage_V() * 1000;
                                        int16_t battReading = filteredBusBattVoltage;
                                        //interrupts();

                                        // Draw battery shape
                                        // drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color)
                                        OLED.drawRoundRect(24 ,5 , 80,40 , 3 , WHITE);
                                        OLED.fillRect     (103,20 , 3 ,10     , WHITE);

                                        // Convert analog reading into mV
                                        // Serial.println("Battery!");
                                        // Serial.println(battReading);
                                        // battReading = map(battReading , 0,850 , 0,4200);
                                       
                                        // Only recalculate bars once change is more than 100 mV to avoid jitter
                                        if ( abs(battReading - battLevel) > 100 )  {
                                                                                  if             (battReading > 3750) battState = 3;                                  //          [3750 , 4200]       =       3 bars
                                                                                  else if        (battReading > 3500) battState = 2;                                  //          [3500 , 3750]       =       2 bars
                                                                                       else if   (battReading > 3200) battState = 1;                                  //          [3500 , 3200]       =       1 bar
                                                                                            else                      battState = 0;                                  //          [3200 , 2700]       =       0 bars

                                                                                  battLevel = battReading;
                                                                                  }

                                        // Draw battery bars / blink battery shell for 0 bars
                                        switch(battState) {
                                                             case 3 : {
                                                                      OLED.fillRoundRect(78,8 , 23,34 , 2 , WHITE);
                                                                      }
                                      
                                                             case 2 : {
                                                                      OLED.fillRoundRect(52,8 , 24,34 , 2 , WHITE);
                                                                      }
                                      
                                                             case 1 : {
                                                                      OLED.fillRoundRect(27,8 , 23,34 , 2 , WHITE);
                                                                      }break;
                                                                      
                                                             case 0 : {
                                                                      if (system_still_initialising) break;                                                                   // displaying battery at powerup, we don't want to fall on off blink cycle
                                                              
                                                                      if (battLevel < 2800)   switch( millis()/1000%4 ) {                                                     // blink and turn entire screen white for extremely low battery
                                                                                                                        case 0 : {
                                                                                                                                                                              // empty battery
                                                                                                                                 turn(modeButton_LED  , OFF);
                                                                                                                                 }break;
                                                                                                                        case 1 : {
                                                                                                                                 OLED.fillRect(0,0, 127,50 , BLACK);          // just writing
                                                                                                                                 
                                                                                                                                 turn(modeButton_LED  , ON);
                                                                                                                                 }break;
                                                                                                                        case 2 : {
                                                                                                                                 OLED.clearDisplay();                         // FULL WHITE
                                                                                                                                 disp_inverted = 1;
                                                                                                                                 OLED.invertDisplay(disp_inverted);

                                                                                                                                 turn(modeButton_LED  , OFF);
                                                                                                                                 }break;
                                                                                                                        case 3 : {
                                                                                                                                 OLED.clearDisplay();                         // FULL BLACK
                                                                                                                                 disp_inverted = 0;
                                                                                                                                 OLED.invertDisplay(disp_inverted);

                                                                                                                                 turn(modeButton_LED  , ON);
                                                                                                                                 }break;                 
                                                                                                                        }
                                                                      //else if ( millis()/1000%2 == 0 )                           OLED.fillRect(0,0, 127,50 , BLACK);          // blink battery every other second for low battery

                                                                                                                                            
                                                                      }break;
                                                          }
                                        
                                        // Print mV value below battery for added effect
                                        OLED.setTextSize(1);
                                        OLED.setCursor(41 , 58);
                                        OLED << divideBy1000(filteredBusBattVoltage , 0               , 6, voltage) << " V";
                                        //OLED << battReading/1000 << F(",") << battReading/100%10 << battReading/10%10 << battReading%10 << F(" mV");
                                        //OLED << filteredBusBattVoltage/1000 << F(",") << filteredBusBattVoltage/100%10 << filteredBusBattVoltage/10%10 << filteredBusBattVoltage%10 << F(" mV");
                                        //OLED << filteredBusBattVoltage << "V";
                                        }break;
                case SER :              {
                  // We send serial data to USB Serial if on Serial screen & send to BLE Serial if BLE Connected even if not on serial screen (See Below Switch Case)
                                        if ( abs((int)millis() - t2) > 1000 ) {                                                                                    // only print every 1 second
                                                                         Serial << divideBy1000   (filteredCurrent    , negativeCurrent , 6, current) << F(" , "); 
                                                                         Serial << divideBy1000   (filteredBusVoltage , 0               , 6, voltage) << F(" , ");
                                                                         Serial << divideBy1000000(filteredPower      , 0               , 6)          << F(" , ");
                                                                         Serial << mAh                                                    << F(" , ");
                                                                         Serial << mWh;
                                                                         Serial << F("\n");

                                                                         t2 = millis();
                                                                         }
                                        
                  
                                        OLED.setTextSize(1);
                                
                                        OLED.setCursor(52, 24);
                                        OLED << F("V");
                                        OLED.setCursor(52, 42);
                                        OLED << F("A");
                                        OLED.setCursor(52, 60);
                                        OLED << F("W");
                                        OLED.setCursor(88, 22);
                                        OLED << F("mAh");
                                        OLED.setCursor(88, 48);
                                        OLED << F("mWh");

                                        OLED.setCursor(4,10);
                                        OLED << F("S E R I A L  M O D E");
                                
                                        OLED.setTextSize(2);
                                
                                        OLED.setCursor(0, 26);           
                                        OLED << divideBy1000(filteredBusVoltage , 0               , 4, voltage);
                                
                                        OLED.setCursor(0, 44);
                                        OLED << divideBy1000(filteredCurrent    , negativeCurrent , 4, current);
                                
                                        OLED.setCursor(0, 62);
                                        OLED << divideBy1000000(filteredPower   , 0               , 4);


                                        //OLED.writeFastVLine(64,16 , 54 , WHITE); 

                                        
                                        format_and_print_large_number(mAh , 68, 36);
                                        format_and_print_large_number(mWh , 68, 62);
                                        
                                        }break;

                case ScreenOFF :              {
                                        turn(modeButton_LED  , OFF);
                                        OLED.display();
                                        OLED.clearDisplay();
                                        
                                        return;                     // we want the Mode Button LED off, ALWAYS, so we prevent auto-dimmer function below from firing
                                        }break;
                }

  OLED.display();

// Send data to BLE Serial if BLE Connected, bleState reads > 600.  Reads 1 to 4 when disconnected, ~900 when connected
if (analogRead(bleState) > 600)
{
  if (!bleHeader)  // Print header to BLE just once
  {

    Serial1 << F("\n") << F("Current") << F(",,,") << F("Voltage") << F(",,,") << F("Power") << F(",,,") << F("mAh") << F(",,,") << F("mWh") << F("\n");
    bleHeader= true;
  }
  
  if ( abs((int)millis() - t2) > 1000 ) {                                                                                    // only print every 1 second
      Serial1 << divideBy1000   (filteredCurrent    , negativeCurrent , 6, current) << F(" , "); 
      Serial1 << divideBy1000   (filteredBusVoltage , 0               , 6, voltage) << F(" , ");
      Serial1 << divideBy1000000(filteredPower      , 0               , 6)          << F(" , ");
      Serial1 << mAh                                                    << F(" , ");
      Serial1 << mWh;
      Serial1 << F("\n");

      t2 = millis();
      }
}



   if ( abs((int)millis() - timeOfLastInterrupt) > dim_disp_after ) {
                                                               turn(modeButton_LED  , OFF);                                       // turn off MODE LED (uses quite a bit of power)
                                                               setBrightness(1);                                                  // dim display after 1h of inactivity = 'dim_disp_after' ms
                                                               
                                                               dimMode = 1;                                                       // keep track of display dimming state
                                                               }
  else                                                         {
                                                               if (!system_still_initialising) turn(modeButton_LED  , ON);        // have all LEDs off while booting
                                                               setBrightness(255);
                                                               
                                                               dimMode = 0;
                                                               }

}





/*
 * SAVE:
 *      Stores values that are to survive reboots to the EEPROM:
 *          currentDisplayMode
 * OPEN:         
 *      Retrieves same values it saved and stores them in RAM for active use
 */
void EEPROM_controller(uint8_t action) {

  if (action == SAVE) {
    Serial.println("Save ");
                      if (myEEPROM.read(INDEX_disp) != currentDisplayMode)                                                                         // only consider saving if the value has changed
                            if (currentDisplayMode >= 0 && currentDisplayMode <= SER) {                                                            // only save if the present value makes sense (we also don't want to start of BATT or the BLACK SCREEN)
                                                                                      myEEPROM.update(INDEX_disp, currentDisplayMode);
                                                                                      turn(PC13 , OFF);  // built in led is opposite, active low
                                                                                      }

                     // EEPROM.get (INDEX_mAh, mAs);                                                                                                                                            // use mAs as temporary storage for EEPROM value for mAh
                     // EEPROM.get (INDEX_mWh, mWs);                                                                                                                                            // use mWs as temporary storage for EEPROM value for mWh
                      if (mAs != mAh)                                                                                            myEEPROM.write(INDEX_mAh, mAh);                                // mAh and mWh cannot and will not be conditioned
                      if (mWs != mWh)                                                                                            myEEPROM.write(INDEX_mWh, mWh);
                      
                      if (mAs != mAh)                                                                                             turn(PC13 , OFF);
                      
                      mAs = 0;                                                                                                                                                                  // minimise impact of a false power-down detection
                      mWs = 0;
                      return;
                      }

  if (action == OPEN) {
                      Serial.print("bef ");
                      Serial.print(currentDisplayMode);
                      currentDisplayMode = myEEPROM.read(INDEX_disp);                             // EEPROM.read retrieves 1 Byte
                    //  EEPROM.get (INDEX_mAh, mAh);                                              // EEPROM.get  retriebes the number of bytes required for the specific variable to be filled in
                    //  EEPROM.get (INDEX_mWh, mWh);
                      Serial.println("OPEN ");
                      Serial.println(myEEPROM.read(INDEX_disp));
                      if (currentDisplayMode > SER) currentDisplayMode = 2;                       // condition currentDisplayMode to make sure that any corrupted data read from the EEPROM won't brick the power monitor
                      
                      return;
                      } 
}



/*
 * I N T E R R U P T    S E R V I C E    R O U T I N E
 * 
 * Called every 10 ms to analyse wether power still present  --- not used for now, may add back if writing to SD card or something
 */
void powerLoss_detector() {
  if(analogRead(power_off_detect) <  800 ){
    pinMode(power_off_detect, OUTPUT);
    digitalWrite(power_off_detect, HIGH);  // For some reason Analog write didn't charge the cap beyond 500, on the scope it was only charging for an instant...
      //Serial.println("Charging...");
      //Serial.println(analogRead(power_off_detect));

  }
  if ( analogRead(power_off_detect) > 1000 ) {
                                            // Turn off all the lights - conserve as much power for saving to EEPROM as possible
                                            turn(powerButton_LED , OFF);
                                            turn(modeButton_LED  , OFF);
                                            //Serial.println("save...");
                                            // Save to EEPROM
                                            EEPROM_controller(SAVE);

                                            // Turn off display; not worth doing before actually saving, since this takes A LOT of time. Just makes sure the display goes out in an orderly fashion with no garbage on it.
                                            // Also, turning on interrupts inside an ISR I'm quite sure is very illegal, please no one tell my mom
                                            interrupts();
                                            OLED.clearDisplay();
                                            OLED.display();
                                            
                                            }else{
                                             // Serial.println("no save...");
                                            }
                                            
}

void sendCommand(const char * command) {
  Serial.print("Command send :");
  Serial.println(command);
  Serial1.println(command);
  //wait some time
  delay(100);

  char reply[100];
  int i = 0;
  while (Serial1.available()) {
    reply[i] = Serial1.read();
    i += 1;
  }
  //end the string
  reply[i] = '\0';
  Serial.print(reply);
  Serial.println("Reply end");                 
  delay(50);
}

void setup() {

  Serial.begin(9600UL);
  Serial1.begin(9600UL);

  sendCommand("AT");
  sendCommand("AT+ROLE0");
  sendCommand("AT+UUID0xFFE0");
  sendCommand("AT+CHAR0xFFE1");
  sendCommand("AT+NAMEPower-Monitor");

  Serial << F("\n") << F("Current") << F(",,,") << F("Voltage") << F(",,,") << F("Power") << F(",,,") << F("mAh") << F(",,,") << F("mWh") << F("\n");
  Serial1 << F("\n") << F("Current") << F(",,,") << F("Voltage") << F(",,,") << F("Power") << F(",,,") << F("mAh") << F(",,,") << F("mWh") << F("\n");
  pinMode(modeButton_LED , OUTPUT);
  pinMode(powerButton_LED, OUTPUT);
  pinMode(PC13, OUTPUT);
  pinMode(power_off_detect, INPUT);

  /****************** Initialize EEPROM *******************/
  
    byte i2cStat = myEEPROM.begin(myEEPROM.twiClock100kHz);
  if ( i2cStat != 0 ) {
    SerialUSB.println(F("I2C Problem"));
  }

  /****************** Attach Interrupt Pins *******************/
  pinMode(button1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button1), registerButtonPress, FALLING);       // will call funtion 'registerButtonPress' when there is a falling edge on pin2
  
  /************************************************************/

  /***************** Attach Timer1 Interrupt ******************/
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *MyTim = new HardwareTimer(Instance); 
  MyTim->setOverflow(1, HERTZ_FORMAT); // 1 Hz
  MyTim->attachInterrupt( mAh_mWh_computer); // mAh_mWh_computer will be called every 1 second
  MyTim->resume();
  /************************************************************/

  /***************** Attach Timer2 Interrupt ******************/
  TIM_TypeDef *Instance2 = TIM2;
  HardwareTimer *MyTim2 = new HardwareTimer(Instance2); 
  MyTim2->setOverflow(100, HERTZ_FORMAT); // 10 Hz
  MyTim2->attachInterrupt( powerLoss_detector); // 
  MyTim2->resume();
  /************************************************************/


  /******************** Initialise Display ********************/
 OLED.begin(SSD1306_SWITCHCAPVCC , 0x3C);                                              // initialise Display and set address to 3C   
  OLED.display();
  delay(1000);                                                                         // keep startup logo for 1 second on screen

  OLED.setFont(&braciola5pt7b);
  OLED.setTextColor(WHITE , BLACK);
  /************************************************************/

  /***************** Initialise Power Monitor *****************/
//  ina219.begin(0x40);        
// initialise INA219 module; its default address is Ox40
  ina219.begin();  
  ina219Batt.begin();
  /************************************************************/

  /******************** EEPROM Controller *********************/
  //EEPROMwl.begin(EEPROM_LAYOUT_VERSION, AMOUNT_OF_INDEXES);                             // initialise library
  
  EEPROM_controller(OPEN);                                                              // read memories from EEPROM
  /************************************************************/

  turn(modeButton_LED  , ON);

  t1 = millis();
    /****************** Show Battery for .5 s *******************/
  turn(powerButton_LED , ON);
  displayController(BATT);
  delay(500);
  /************************************************************/
  system_still_initialising = 0;
  // turn(PC13, OFF); // on board LED is opposite, active low
  // delay(1000);
   turn(PC13, ON);
}





void loop() {

  if (buttonWasPressed) humanInterfaceController();                                                                   // check if button was pressed or if mode was changed

  if ( abs((int)millis() - t1) >= 100)  {                                                                             // since t1 is recorded before performing time-consuming actions, this will happen every 100ms
                                   t1 = millis();

                                   aquireData();                                                                 // retrieve data fron INA219

                                   
                                   displayController(currentDisplayMode);                                        // display  data on   OLED

                                    //if (battLevel < 2800 && battLevel > 2400 && !batt_low_nag_screen) {  
                                   if (battLevel < 0 && battLevel > 0 && !batt_low_nag_screen) {                               // when connected to and powered by a computer, voltage will be < 1.5V - no point in going to nag screen
                                                                                                     currentDisplayMode = BATT;
                                                                                                     batt_low_nag_screen = 1;        // only switch to batt mode ONCE
                                                                                                     }
                                   if (battLevel > 3000)                                             batt_low_nag_screen = 0;
                                   }

}
