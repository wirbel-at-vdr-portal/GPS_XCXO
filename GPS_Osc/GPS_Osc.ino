/* Until we know anything better, we expect this to be the correct
 * Arduino IDE board:
 * "ESP32 Dev Module"
 */

#include <SPI.h>              // SPI(MOSI,SCK,SS) for ST7735 1.8 TFT display
#include <Adafruit_GFX.h>     // display
#include <Adafruit_ST7735.h>  // display



/* Now, let's declare the pins we used on the PCB.
 */
#define SS    5  // slave select, or, chip select
#define DC    16 // data/command (DC), labeled 'AO' on TFT
#define RST   17 // tft reset (-1 for Arduino RESET pin doesnt work..)
//      MOSI  23
//      MISO  19
//      SCK   18



Adafruit_ST7735 Display = Adafruit_ST7735(SS, DC, RST); // hardware SPI for the display.

void setup(void) {
  Serial.begin(115200);
  delay(1000);
  Serial.print(F("Hello! ST7735 TFT Test"));
  Display.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab

  Display.fillScreen(ST77XX_BLACK);
  testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST77XX_WHITE);
  delay(1000);
}

bool inverted;

void loop() {
  inverted = not inverted;
  Display.invertDisplay(inverted);
  delay(500);
}


void testdrawtext(char *text, uint16_t color) {
  Display.setCursor(0, 0);
  Display.setTextColor(color);
  Display.setTextWrap(true);
  Display.print(text);
}
