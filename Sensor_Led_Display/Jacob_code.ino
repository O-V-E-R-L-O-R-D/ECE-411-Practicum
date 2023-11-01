// I2C library
#include <Wire.h>
// OLED display library
#include <Adafruit_SSD1306.h>
// RGB color sensor library
#include <Adafruit_TCS34725.h>
//pins for LED
#include  <SPI.h>
#define redpin 5
#define greenpin 9
#define bluepin 10
byte gammatable[256];
// Button pin
#define buttonPin 11
// Sensor LED pin
#define senpin 3
int button = 0; //active high
// OLED Parameters
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
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
uint8_t mode_state = 1;   //initialize to calibration mode

void setup()
{
  // Set Baud rate
  Serial.begin(9600);

  // LED pins out
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
  // RGB gammatable
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
    gammatable[i] = x;
    }
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  // Sensor LED as ounput to turn it off or on
  pinMode(senpin, OUTPUT);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

    // Clear the buffer
  display.clearDisplay();
  
  // initialize OLED display
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.display();

  if (tcs.begin()) {
    Serial.println("Color Sensor Working...");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}
// save these variables between loops (global)
uint16_t min_red, min_green, min_blue = 255;
uint16_t max_red, max_green, max_blue = 0;
void loop()
{
  uint16_t raw_red, raw_green, raw_blue, clear;
  float red, green, blue, total; 
  // color calibration mode
  if(mode_state == 1){
    digitalWrite(senpin, HIGH);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("\tCalibrating Mode...");
    display.display();
    delay(2000);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Calibrate for black");
    Serial.println("Calibrate for black");  //debug line
    display.print("target: #000000");
    display.display();
    while(digitalRead(buttonPin) == LOW){} // wait until button is pressed
    //delay(5000); //replace with while loop once I add button
    // get all white RGB values
    tcs.getRawData(&raw_red, &raw_green, &raw_blue, &clear);
    min_red   = raw_red; 
    min_green = raw_green;
    min_blue  = raw_blue;
    delay(500);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Calibrate for white");
    Serial.println("Calibrate for white");  //debug line
    display.print("target: #FFFFFF");
    display.display();
    while(digitalRead(buttonPin) == LOW){} // wait until button is pressed
    //delay(5000); //replace with while loop once I add button
    // get all white RGB values
    tcs.getRawData(&raw_red, &raw_green, &raw_blue, &clear);
    max_red   = raw_red; 
    max_green = raw_green;
    max_blue  = raw_blue;
    delay(500);
    mode_state = 3;
    display.clearDisplay();
    display.display();
  }

  // color sensing mode
  else if(mode_state == 2){
    digitalWrite(senpin, HIGH);
    delay(500);
    //bebug
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

    display.clearDisplay();
    display.setCursor(0, 0);
    delay(50);
    //Sample Color sensor
    tcs.getRawData(&raw_red, &raw_green, &raw_blue, &clear);
    //adjustment based off of calibration
    //and conver raw value into RGB value
    red = constrain(map(raw_red, min_red, max_red, 0, 255),0,255);
    green = constrain(map(raw_green, min_green, max_green, 0, 255),0,255);
    blue = constrain(map(raw_blue, min_blue, max_blue, 0, 255),0,255);

   // write to LED pins
   analogWrite(redpin, gammatable[(int)red]);
   analogWrite(greenpin, gammatable[(int)green]);
   analogWrite(bluepin, gammatable[(int)blue]);

    /*// print RGB values to OLED Diplay
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
    /*
    //debug
    Serial.print("R:");     Serial.print(int(red)); 
    Serial.print("\tG:");   Serial.print(int(green)); 
    Serial.print("\tB:");   Serial.print(int(blue));
    Serial.print("\n");
    */
    mode_state = 3;
  }
  // idle mode
  else if(mode_state == 3){
    /*
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Idle");
    display.display();
    */
    digitalWrite(senpin, LOW);
    if (digitalRead(buttonPin) == HIGH){
      mode_state = 2;
    }

  }
  //state = 4 (sleep mode)
  else{

  }
  
}