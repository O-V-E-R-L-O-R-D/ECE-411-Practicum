// I2C library
#include <Wire.h>
// OLED display library
#include <Adafruit_SSD1306.h>
// RGB color sensor library
#include <Adafruit_TCS34725.h>
// Non-volatile memory
#include <EEPROM.h>

// Button pin
#define buttonPin 4 //Pin 6 on ATmega

#define minRedLocation    0
#define maxRedLocation    6
// Sensor LED pin
#define senpin A1   //Pin 24 on ATmega
// OLED Parameters
#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_HEIGHT 32      // OLED display height, in pixels
#define OLED_RESET     -1     // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C   // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
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
uint8_t mode_state = 1;   

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
    for(;;); // Don't proceed, loop forever
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
    while (1);
  }

  // IF first ever boot (EEPROM has no valid Data)
  if(( 0xFF == EEPROM.read(minRedLocation)) && (0xFF == EEPROM.read(maxRedLocation))){
    cal_val.min_red   = 255;
    cal_val.min_green = 255;
    cal_val.min_blue  = 255;
    cal_val.max_red   = 0;
    cal_val.max_green = 0;
    cal_val.max_blue  = 0;
  }

  // Retrieve stored calibration values
  else{
    EEPROM.get(0, cal_val);
  }



}

void loop()
{
  // Raw RGB values
  uint16_t raw_red, raw_green, raw_blue, clear;
  // Real RGB values
  float red, green, blue, total; 

  // Color calibration mode
  if(mode_state == 1){
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
    min_red   = raw_red; 
    min_green = raw_green;
    min_blue  = raw_blue;
    delay(500);
    
    /*
    // Seve calibration data to memory
    for(int i; i < num_val_stored * sizeof(uint16_t); i += sizeof(uint16_t)){
      EEPROM.update()

    }
    */

    // Tell user were entering white calibration
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Calibrate for white");
    Serial.println("Calibrate for white");  //debug line
    display.print("target: #FFFFFF");
    display.display();

    // Wait unitl button is pressed to collect white data
    while(digitalRead(buttonPin) == LOW){}

    // Get all white RGB values
    tcs.getRawData(&raw_red, &raw_green, &raw_blue, &clear);
    max_red   = raw_red; 
    max_green = raw_green;
    max_blue  = raw_blue;
    delay(500);

    // End calibration and go to idle state
    mode_state = 3;
    display.clearDisplay();
    display.display();
  }

  // Color sensing mode
  else if(mode_state == 2){
    // Turn on light for Color Sensor
    digitalWrite(senpin, HIGH);
    delay(100);
    
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

    display.clearDisplay();
    display.setCursor(0, 0);
    delay(50);
    // Sample Color sensor
    tcs.getRawData(&raw_red, &raw_green, &raw_blue, &clear);
    // Make adjustment based off of calibration
    // and conver raw value into RGB value
    red = constrain(map(raw_red, min_red, max_red, 0, 255),0,255);
    green = constrain(map(raw_green, min_green, max_green, 0, 255),0,255);
    blue = constrain(map(raw_blue, min_blue, max_blue, 0, 255),0,255);

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

    // return to idle mode
    mode_state = 3;
  }

  // Idle mode
  else if(mode_state == 3){
    /*// Degub
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Idle");
    display.display();
    */
    // Turn off LED on RBG sensor
    digitalWrite(senpin, LOW);
    if (digitalRead(buttonPin) == HIGH){
      mode_state = 2;
    }

  }
  //state = 4 (sleep mode)
  else{

  }
  
}
