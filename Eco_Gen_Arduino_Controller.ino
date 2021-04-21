// Control System Code for DP5 Project "Innovation in Renewable Energy Education"  by Ari Biggart as part of a Master's in Engineering Design from the University of Bristol.
// V1.0 Published - DATE

#include <Wire.h>  //Library for I2C communication
#include <Adafruit_INA219.h> //Library for power sensing module
#include <Adafruit_INA260.h> //Library for other power sensing module
#include <SD.h> //Libary for SD card module
#include <SPI.h> //Library for SPI communication
#include <RTClib.h> //Library for RTC
#include <LiquidCrystal_I2C.h> // Library for LCD I2c modules

//Declare Variables
const int T_sampling_ms = 1000; //MPPT sampling time
const int CS_pin = 10; // Pin 10 on arduino uno
const int PV_pin = 5; // PV PWM Pin
const int WTG_pin = 6; // WTG PWM Pin
const int T_adjust = 64; //Slow down delays to compensate for increased clock speed
const int PWM_step[2] = {4 , 2}; // Size of step in P&O control system [PV , WTG] 
const int PWM_PV = 138; // Initial PV PWM duty cycle
const int PWM_WTG = 172; // Initial WTG PWM duty cycle

int PWM[2] = {PWM_PV , PWM_WTG}; //Starting WTG and PV control signals
int PWM_Prev[2] = {PWM_PV - 1 , PWM_WTG - 1}; //Starting WTG and PV "previous" control signals
int PWM_Next[2] = {PWM_PV , PWM_WTG}; //WTG and PV next control signals
float T_el = 0; //
float T_start = 0; //
int numReadings = 0; //MPPT samples 

// Set up Power Monitoring Variables
float V_t[4];   // Voltage reading from ina219_A module
float V[4];     // Voltage reading from ina219_A module
float I_t[4];   // Current reading from ina219_A module
float I[4];     // Current reading from ina219_A module
float P[4];     // smoothed power
float P_Prev[2] = {0 , 0}; // Power Recorded in previous iteration


Adafruit_INA219 ina219_A(0x40); // Set up power sense modules as classes
Adafruit_INA219 ina219_B(0x41);
Adafruit_INA219 ina219_C(0x44); // Set up power sense modules as classes
Adafruit_INA219 ina219_D(0x45);

// File sdcard_file; // Set up SD card file
RTC_DS3231 rtc; // Set up RTC object
LiquidCrystal_I2C lcd_A(0x27,16,2);  // initialise LCD_A
LiquidCrystal_I2C lcd_B(0x23,16,2);  // initialise LCD_B

//Wait for Serial Motor
void setup(void) 
{
  Serial.begin(57600);
  while (!Serial) {
      //wait for Serial Monitor to connect. Needed for native USB port boards only
      delay(1);
  } 
// Set timer0 clockspeed
  TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency for pins 5&6 to 62500.00 Hz   
//Initialise RTC module
  if (rtc.begin())
  {
    Serial.println(F(""));
    Serial.println(F("rtc card is ready to use."));
  } else
  {
    Serial.println(F("rtc  initialization failed"));
    return;
  }
  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, let's set the time!"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Sets the RTC to the date & time this sketch was compiled
    // This line sets the RTC with an explicit date & time, for example to set
  }
  // Initialise SD Card 
//  if (SD.begin())
//  {
//    Serial.println(F("SD card is ready to use."));
//  } else
//  {
//    Serial.println(F("SD card initialization failed"));
//    return;
//  }
//  sdcard_file = SD.open("test.txt", FILE_WRITE);
//  if (sdcard_file) { 
//    sdcard_file.print(F("   Time    ")); sdcard_file.print("      "); 
//    sdcard_file.print(F("V1   ")); sdcard_file.print("     "); sdcard_file.print(F("mA   ")); sdcard_file.print("     ");sdcard_file.print(F("V2   ")); sdcard_file.print("     "); sdcard_file.print(F("mA   ")); sdcard_file.print("     ");
//    sdcard_file.print(F("P1   ")); sdcard_file.print("     "); sdcard_file.print(F("P1_prev     ")); sdcard_file.print("     "); sdcard_file.print(F("D1   ")); sdcard_file.print("     ");
//    sdcard_file.print(F("V2   ")); sdcard_file.print("     "); sdcard_file.print(F("mA   ")); sdcard_file.print("     "); sdcard_file.print(F("V3   ")); sdcard_file.print("     "); sdcard_file.print(F("mA   ")); sdcard_file.print("     ");
//    sdcard_file.print(F("P2   ")); sdcard_file.print("     "); sdcard_file.print(F("P2_prev     ")); sdcard_file.print("     "); sdcard_file.print(F("D2   ")); sdcard_file.println("     ");
//    sdcard_file.close(); // close the file
//  }
  // Initialize the INA219.
  if (! ina219_A.begin()) {
    Serial.println(F("Failed to find INA219_A chip"));
    while (1) { delay(10*T_adjust); }
  }
  if (! ina219_B.begin()) {
    Serial.println(F("Failed to find INA219_B chip"));
    while (1) { delay(10*T_adjust); }
  }
  if (! ina219_C.begin()) {
    Serial.println(F("Failed to find INA219_C chip"));
    while (1) { delay(10*T_adjust); }
  }
  if (! ina219_D.begin()) {
    Serial.println(F("Failed to find INA219_D chip"));
  while (1) { delay(10*T_adjust); }
  }
  Serial.println("Measuring voltage and current with INA219 ...");
  // initialize LCD displays
  lcd_A.init(); // initialize the lcd 
  lcd_A.backlight();
  lcd_B.init(); // initialize the lcd 
  lcd_B.backlight();

  // Create initial PWM signals
    pinMode(WTG_pin, OUTPUT);  // define the WTG_pin as output pin
    pinMode(PV_pin, OUTPUT);  // define the PV_pin as output pin
    analogWrite(WTG_pin, PWM[1]);  // output for PV PWM signal 
    analogWrite(PV_pin, PWM[0]);  // output for PV PWM signal 
  //   
} 
void loop(void) 
{
 
 // Measure Power and Smooth Readings
 for (int i = 0; i < 4; i++) {
    V_t[i] = 0 ;  // Voltage reading from ina219_A module
    V[i] = 0;    // Voltage reading from ina219_A module
    I_t[i] = 0;  // Current reading from ina219_A module
    I[i] = 0;    // Current reading from ina219_A module
    P[i] = 0;    // smoothed power
  }
  
  numReadings = 0;
  T_start = millis()/T_adjust;
  T_el = T_start;
  
  while (T_el < T_start + T_sampling_ms) {
    V_t[0] = V_t[0] + ina219_A.getBusVoltage_V() ; // PV input  
    V_t[1] = V_t[1] + ina219_B.getBusVoltage_V(); // PV output
    V_t[2] = V_t[2] + ina219_C.getBusVoltage_V(); ; // WTG input
    V_t[3] = V_t[3] + ina219_D.getBusVoltage_V();; // WTG output 
    I_t[0] = I_t[0] + ina219_A.getCurrent_mA();  // PV input
    I_t[1] = I_t[1] + ina219_B.getCurrent_mA(); // PV output 
    I_t[2] = I_t[2] + ina219_C.getCurrent_mA(); // WTG input  
    I_t[3] = I_t[3] + ina219_D.getCurrent_mA(); // WTG output
    numReadings = numReadings + 1;
    T_el = millis()/T_adjust;
    delay(5*T_adjust);
  }
  
 // add the reading to the total:

 for (int i = 0; i < 4; i++) {
    V[i] = V_t[i]/numReadings;  // smoothed voltage
    I[i] = I_t[i]/numReadings;  // smoothed current
    P[i] = V[i] * I[i] ;    // smoothed power
  }

  // Display Readings
  DateTime now = rtc.now();
  Serial.print(now.year(), DEC); Serial.print(F("/")); Serial.print(now.month(), DEC); Serial.print('/');Serial.print(now.day(), DEC); ; Serial.print(F("     ")); 
  Serial.print(now.hour(), DEC); Serial.print(F(":")); Serial.print(now.minute(), DEC); Serial.print(':'); Serial.print(now.second(), DEC);; Serial.print(F("      ")); 
  Serial.print(V[0]);Serial.print(F("      ")); Serial.print(I[0]); Serial.print(F("      ")); Serial.print(V[1]);Serial.print(F("      ")); Serial.print(I[1]); Serial.print(F("      "));
  Serial.print(P[1]); Serial.print(F("      ")); Serial.print(P_Prev[0]); Serial.print(F("      ")); Serial.print(PWM[0]); Serial.print(F("      "));
  Serial.print(V[2]);Serial.print(F("      ")); Serial.print(I[2]); Serial.print(F("      ")); Serial.print(V[3]);Serial.print(F("      ")); Serial.print(I[3]); Serial.print(F("      "));
  Serial.print(P[3]); Serial.print(F("      ")); Serial.print(P_Prev[1]); Serial.print(F("      ")); Serial.print(PWM[1]); Serial.println(F("      ")); 
   
  // Display Readings on LCD
  lcd_A.setCursor(0,0);lcd_A.print(V[0]); lcd_A.setCursor(4,0); lcd_A.print(F("V")); // First line of PV 16x2 Display
  lcd_A.setCursor(6,0);lcd_A.print(I[0]); lcd_A.setCursor(10,0); lcd_A.print(F("mA"));
  lcd_A.setCursor(13,0);lcd_A.print(PWM[0]*100/255); lcd_A.setCursor(15,0); lcd_A.print(F("%"));
 
  lcd_A.setCursor(0,1);lcd_A.print(V[1]); lcd_A.setCursor(4,1); lcd_A.print(F("V")); //Second line of PV 16x2 Display
  lcd_A.setCursor(6,1);lcd_A.print(I[1]); lcd_A.setCursor(10,1); lcd_A.print(F("mA"));
  lcd_A.setCursor(13,1);lcd_A.print(P[1]/P[0]*100); lcd_A.setCursor(15,1); lcd_A.print(F("%"));

  lcd_B.setCursor(0,0);lcd_B.print(V[2]); lcd_B.setCursor(4,0); lcd_B.print(F("V")); //First line of WTG  16x2 Display
  lcd_B.setCursor(6,0);lcd_B.print(I[2]); lcd_B.setCursor(10,0); lcd_B.print(F("mA"));
  lcd_B.setCursor(13,0);lcd_B.print(PWM[1]*100/255); lcd_B.setCursor(15,0); lcd_B.print(F("%"));
  
  lcd_B.setCursor(0,1);lcd_B.print(V[3]); lcd_B.setCursor(4,1); lcd_B.print(F("V")); //Second line of WTG 16x2 Display
  lcd_B.setCursor(6,1);lcd_B.print(I[3]); lcd_B.setCursor(10,1); lcd_B.print(F("mA"));
  lcd_B.setCursor(13,1);lcd_B.print(P[3]/P[2]*100); lcd_B.setCursor(15,1); lcd_B.print(F("%"));

//  // Log Readings
//  sdcard_file = SD.open("test.txt", FILE_WRITE);
//  if (sdcard_file) {    
//    sdcard_file.print(now.year(), DEC); sdcard_file.print('/'); sdcard_file.print(now.month(), DEC); sdcard_file.print('/');sdcard_file.print(now.day(), DEC); ; sdcard_file.print("      "); 
//    sdcard_file.print(now.hour(), DEC); sdcard_file.print(':'); sdcard_file.print(now.minute(), DEC); sdcard_file.print(':'); sdcard_file.print(now.second(), DEC);; sdcard_file.print("      "); 
//    sdcard_file.print(V[0]); sdcard_file.print("      ");  sdcard_file.print(I[0]); sdcard_file.print("      "); sdcard_file.print(V[1]); sdcard_file.print("      "); sdcard_file.print(I[1]); sdcard_file.print("      "); 
//    sdcard_file.print(P[1]); sdcard_file.print("      ");  sdcard_file.print(P_Prev[0]);sdcard_file.print("      "); sdcard_file.print(PWM[0]); sdcard_file.print("      ");
//    sdcard_file.print(V[2]); sdcard_file.print("      ");  sdcard_file.print(I[2]); sdcard_file.print("      "); sdcard_file.print(V[3]); sdcard_file.print("      "); sdcard_file.print(I[3]); sdcard_file.print("      "); 
//    sdcard_file.print(P[3]); sdcard_file.print("      ");  sdcard_file.print(P_Prev[1]); sdcard_file.print("      "); sdcard_file.print(PWM[1]); sdcard_file.println("      ");
//    sdcard_file.close(); // close the file
//  }
//  // if the file didn't open, print an error:
//  else {
//    Serial.println("error opening test.txt");
//  }

  // Perturb and Observe Control System
  // Loop for WTG and PV
  for (int i = 0; i < 2; i = i + 1) {
    if(P[1+2*i] > 15) {  // If power is below 15mW the system is assumed to be idling the controller will not respond
      if (P[1+2*i] >= P_Prev[i]) { // Did the power increase?
        if (PWM[i] >= PWM_Prev[i]){ // Did the duty cycle increase?
          PWM_Next[i] = PWM[i] + PWM_step[i]; //increase PWM
        }
        else if (PWM[i] < PWM_Prev[i]){ // Did the duty cycle decrease?
          PWM_Next[i] = PWM[i] - PWM_step[i]; //decrease PWM
        }    
      }
      else if (P[1+2*i] < P_Prev[i]){ // Did the power decrease?
        if (PWM[i] >= PWM_Prev[i]){ // Did the duty cycle increase?
         PWM_Next[i] = PWM[i] - PWM_step[i]; //decrease PWM
        }
        else if (PWM[i] < PWM_Prev[i]){ // Did the duty cycle decrease?
         PWM_Next[i] = PWM[i] + PWM_step[i]; //increase PWM
        }    
       }
      }
     }

  // Update Variables with new values for next loop
   if(PWM_Next[0] != PWM[0]){
   analogWrite(PV_pin, PWM_Next[0]);  // output for PV PWM signal 
   }
   if(PWM_Next[1] != PWM[2]){
   analogWrite(WTG_pin, PWM_Next[1]);  // output for PV PWM signal 
   }
   for (int i = 0; i < 2; i = i + 1) { 
   P_Prev[i] = P[1+2*i];
   PWM_Prev[i] = PWM[i];
   PWM[i] = PWM_Next[i]; 
   }
}
