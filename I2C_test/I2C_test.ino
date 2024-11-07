////////////////////////////////////////////////////////////// Encoder //////////////////////////////////////////////////////////////
#include "AS5600.h"

AS5600 as5600_0(&Wire);

////////////////////////////////////////////////////////////// Encoder //////////////////////////////////////////////////////////////

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

void setup() {
  Serial.begin(460800);
  Serial.println("开始初始化");

  // AS5600_setup();

  OLED_setup();

  Serial.println("初始化完成");

}

float angle0;

void loop() {
  // 读取AS5600传感器的角度
  angle0 = float(as5600_0.readAngle())/4096*2*M_PI;

  // 输出数据到串口
  Serial.print("Ang 1: ");
  Serial.print(angle0);
  Serial.println();

  testdrawstyles();

  delay(4);
}

void OLED_setup() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);
  // display.display() is NOT necessary after every single drawing command,
  // unless that's what you want...rather, you can batch up a bunch of
  // drawing operations and then update the screen all at once by calling
  // display.display(). These examples demonstrate both approaches...

  // testdrawchar();      // Draw characters of the default font

  testdrawstyles();    // Draw 'stylized' characters

  // testscrolltext();    // Draw scrolling text

  // testdrawbitmap();    // Draw a small bitmap image

  // Invert and restore display, pausing in-between
  // display.invertDisplay(true);
  // delay(1000);
  // display.invertDisplay(false);
  // delay(1000);

  // testanimate(logo_bmp, LOGO_WIDTH, LOGO_HEIGHT); // Animate bitmaps
}


void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Angle(rad): "));

  display.setTextSize(5);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(4,20); 
  display.print(angle0); //display.println(angle0, HEX);

  display.display();
}


////////////////////////////////////////////////////////////// Encoder //////////////////////////////////////////////////////////////
void AS5600_setup(){
  Wire.begin(21, 22);    // SDA=21, SCL=22
  if (!as5600_0.begin()) {
    Serial.println("No 1 AS5600 Found");
    while (1);
  }
}
////////////////////////////////////////////////////////////// Encoder //////////////////////////////////////////////////////////////