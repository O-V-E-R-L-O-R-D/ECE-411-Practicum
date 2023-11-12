// I2C library
#include <Wire.h>
// OLED display library
#include <Adafruit_SSD1306.h>
// RGB color sensor library
#include <Adafruit_TCS34725.h>
// Non-volatile memory
#include <EEPROM.h>

// mode/state of operation
#define calibrationMode       1
#define colorSensingMode      2
#define idleMode              3
#define sleepMode             4

// Threshold time for button press
// decuces misclick
#define debounceDelay 500

// Button pin
#define buttonPin 4 //Pin 6 on Raw ATmega // 4 on Arduino Uno

#define minRedLocation    0
#define maxRedLocation    6
// Sensor LED pin
#define senpin  A1  //Pin 24 on Raw ATmega // A1 on Arduino Uno
// OLED Parameters
#define SCREEN_WIDTH    128 // OLED display width, in pixels
#define SCREEN_HEIGHT    32 // OLED display height, in pixels
#define OLED_RESET       -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// RGB color sensor parameters
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_499MS, TCS34725_GAIN_4X);

// state machine for mode of operation
/*  1 = color calibration mode  (hold button for x amount of time to enter calibration mode)
                                (after calibration, goes into idle mode)
    2 = color sensing mode      (after sensing color, goes into idle mode)
    3 = idle mode               (waiting for button press to enter color sensing mode) 
                                (if idle for x amount of time go into sleep mode)
    4 = sleep mode              (low power mode until button press, then go into color sensoing mode)
*/
//Initialize to calibration mode
uint8_t mode_state;

// Used to store button states and remove debounce
bool buttonReading;
bool lastButtonReading = LOW;
uint32_t startPress = 0;
uint32_t endPress = 0;

// Save these variables between loops (global)
// Calibration values
struct calibrationCodes{
  uint16_t min_red, min_green, min_blue;
  uint16_t max_red, max_green, max_blue; 
};
calibrationCodes cal_val;

void setup()
{
  // Set Baud rate
  Serial.begin(9600);

  // Initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
  // Sensor LED as ounput to turn it off or on
  pinMode(senpin, OUTPUT);
  // Turn off RGB LED
  digitalWrite(senpin, LOW);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(("SSD1306 allocation failed"));
    while (1); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();
  
  // Initialize OLED display
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.display();

  if (tcs.begin()) {
    Serial.println("Color Sensor Working...");
    display.println("Color Sensor Working...");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    display.println("No TCS34725 found ... check your connections");
    while (1); // Don't proceed, loop forever
  }

  // IF first ever boot (EEPROM has no valid Data)
  if(( 0xFF == EEPROM.read(minRedLocation)) && (0xFF == EEPROM.read(maxRedLocation))){
    cal_val.min_red   = 255;
    cal_val.min_green = 255;
    cal_val.min_blue  = 255;
    cal_val.max_red   = 0;
    cal_val.max_green = 0;
    cal_val.max_blue  = 0;
    // Go to calibration mode
    mode_state = calibrationMode;
  }

  // Retrieve stored calibration values
  else{
    EEPROM.get(0, cal_val);
    // Go to idle mode
    mode_state = idleMode;
  }
  
}

void loop()
{
  // Raw RGB values
  uint16_t raw_red, raw_green, raw_blue, clear;
  // Real RGB values
  float red, green, blue, total; 

  // Color calibration mode
  if(mode_state == calibrationMode){
    digitalWrite(senpin, HIGH);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("\tCalibrating Mode...");
    display.display();
    delay(2000);

    // Tell user were entering black calibration
    display.display();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Calibrate for black");
    Serial.println("Calibrate for black");  //debug line
    display.print("target: #000000");
    display.display();
    
    // Wait until button is pressed to collect black data
    while(digitalRead(buttonPin) == LOW){} 
  
    // Get all black RGB values
    tcs.getRawData(&raw_red, &raw_green, &raw_blue, &clear);
    cal_val.min_red   = raw_red; 
    cal_val.min_green = raw_green;
    cal_val.min_blue  = raw_blue;
    delay(500);

    // Tell user were entering white calibration
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Calibrate for white");
    Serial.println("Calibrate for white");  //debug line
    display.print("target: #FFFFFF");
    display.display();
    delay(1000);

    // Wait unitl button is pressed to collect white data
    while(digitalRead(buttonPin) == LOW){}

    // Get all white RGB values
    tcs.getRawData(&raw_red, &raw_green, &raw_blue, &clear);
    cal_val.max_red   = raw_red; 
    cal_val.max_green = raw_green;
    cal_val.max_blue  = raw_blue;
    delay(500);

    // Turn off LED on RBG sensor
    digitalWrite(senpin, LOW);

    // Save new calibration data to memory
    EEPROM.put(0,cal_val);

    // End calibration and go to idle state
    mode_state = idleMode;
    display.clearDisplay();
    display.display();
  }

  // Color sensing mode
  else if(mode_state == colorSensingMode){
    // Turn on light for Color Sensor
    digitalWrite(senpin, HIGH);
  
    display.clearDisplay();
    display.setCursor(0, 0);
    delay(500);
    
    /*//Debug
    //////////////////////////////////////////////////////////// 
    Serial.print("Rmin: ");   Serial.print(min_red);
    Serial.print("\tGmin: "); Serial.print(min_green);
    Serial.print("\tBmin: "); Serial.print(min_blue);
    Serial.println();
    Serial.print("Rmax: ");   Serial.print(max_red);
    Serial.print("\tGmax: "); Serial.print(max_green);
    Serial.print("\tBmax: "); Serial.print(max_blue);
    Serial.println();
    //////////////////////////////////////////////////////////// 
    */

    // Sample Color sensor
    tcs.getRawData(&raw_red, &raw_green, &raw_blue, &clear);
    // Make adjustment based off of calibration
    // and conver raw value into RGB value
    red = constrain(map(raw_red, cal_val.min_red, cal_val.max_red, 0, 255),0,255);
    green = constrain(map(raw_green, cal_val.min_green, cal_val.max_green, 0, 255),0,255);
    blue = constrain(map(raw_blue, cal_val.min_blue, cal_val.max_blue, 0, 255),0,255);

    /*// Print RGB values to OLED Diplay
    display.print("R:");    display.print(int(red));   
    display.print(" G:");   display.println(int(green)); 
    display.print("B:");    display.print(int(blue));   
    */
    
    // Print Hex value to OLED Display
    display.print("Hex:"); 
    if(red < 16){display.print("0");}    //add leading zero
    display.print(int(red),HEX);
    if(green < 16){display.print("0");}  //add leading zero
    display.print(int(green),HEX);
    if(blue < 16){display.print("0");}  //add leading zero
    display.print(int(blue),HEX);
    display.display();
    
    /*// Debug
    Serial.print("R:");     Serial.print(int(red)); 
    Serial.print("\tG:");   Serial.print(int(green)); 
    Serial.print("\tB:");   Serial.print(int(blue));
    Serial.print("\n");
    */
    // Turn off LED on RBG sensor
    digitalWrite(senpin, LOW);

    // return to idle mode
    mode_state = idleMode;
  }
  
  // Idle mode
  else if(mode_state == idleMode){
    // Check if button has been pressed
    buttonReading = digitalRead(buttonPin);
    
    // Get time when button changes states
    if(buttonReading != lastButtonReading){
      //detect Rising edge of button press
      if(buttonReading == HIGH){
        startPress = millis();
        endPress = millis();
      }
      if(buttonReading == LOW){
        endPress = millis();
      }
    }
    // Rcognise button press if button was held down longer than debounceDelay 
    if((endPress - startPress) > debounceDelay){
      if((endPress - startPress) > 5000){
        mode_state = calibrationMode;
        // Debug
        Serial.println("go to colorSensingMode");
      }
      else{
        mode_state = colorSensingMode;
        // Debug
        Serial.println("go to calibrationMode");
      }
      // Reset button timmer
      startPress = 0;
      endPress = 0;
    }
    lastButtonReading = buttonReading;
  }

  //state = 4 (sleep mode)
  else{
  }
  
}
