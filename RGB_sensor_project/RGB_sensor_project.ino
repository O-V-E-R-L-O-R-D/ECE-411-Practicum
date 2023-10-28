// I2C library
#include <Wire.h>
// OLED display library
#include <Adafruit_SSD1306.h>
// RGB color sensor library
#include <Adafruit_TCS34725.h>


// OLED Parameters
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// RGB color sensor parameters
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup()
{
  // Set Baud rate
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
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

void loop()
{
  float red, green, blue, clear;
  display.clearDisplay();
  display.setCursor(0, 0);
  delay(1000);
  tcs.getRGB(&red, &green, &blue);  // get RGB values from sensor
  // print RGB values to OLED Diplay
  display.print("R:");   display.print(int(red)); 
  display.print(" G:");  display.println(int(green)); 
  display.print("B:");  display.print(int(blue));
  display.display();

  /*//debug
  Serial.print("R:"); Serial.print(int(red)); 
  Serial.print("\tG:"); Serial.print(int(green)); 
  Serial.print("\tB:"); Serial.print(int(blue));
  Serial.print("\n");
  */
}
