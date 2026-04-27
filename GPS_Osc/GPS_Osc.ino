#include <Arduino.h>
#include <stdint.h>           // uint16_t, uint32_t
#include <SPI.h>              // SPI(MOSI,SCK,SS) for ST7735 1.8 TFT display
#include <Adafruit_GFX.h>     // display
#include <Adafruit_ST7735.h>  // display



/* Until we know anything better, we expect this to be the correct
 * Arduino IDE board:
 * "ESP32 Dev Module"
 *
 * Now, let's declare the pins we used on the PCB.
 */
#define SS      5  // slave select, or, chip select
#define DC      16 // data/command (DC), labeled 'AO' on TFT
#define RST     17 // tft reset (-1 for Arduino RESET pin doesnt work..)
//      MOSI    23
//      MISO    19
//      SCK     18
#define PPS_IN  34 // 1pps signal from GPS
#define OSC_IN  35 // 10MHz CMOS signal from OCXO
#define PWM_OUT 32 // 16bit PWM


#define kHz(f) 1000ULL * f
#define MHz(f) 1000ULL * kHz(f)

const uint32_t frequency = MHz(10);


Adafruit_ST7735 Display = Adafruit_ST7735(SS, DC, RST); // hardware SPI for the display.
volatile uint32_t clocks;
volatile bool gate_open;
volatile bool restart;
volatile int seconds;
uint16_t pwm_val;

#define GATE_1SEC  1
#define GATE_2SEC  2
#define GATE_5SEC  5
#define GATE_10SEC 10



void setup(void) {
  Serial.begin(115200);
  delay(1000);
  Display.initR(INITR_BLACKTAB);    // Init ST7735S chip, black tab
  Display.fillScreen(ST77XX_BLACK);
  testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST77XX_WHITE);
  delay(1000);
  gate_open = restart = false;
  attachInterrupt(OSC_IN, On10MHz, RISING);
  attachInterrupt(PPS_IN, OnPPS, RISING);
  clocks = 0;
  seconds = GATE_1SEC;
  pwm_val = 65353UL >> 1;
}


void ARDUINO_ISR_ATTR OnPPS() {
  if (seconds > 0) {
     if (seconds == 1) {
        // 1sec duration, or last sec of several seconds
        gate_open = false;
        }
     seconds--;
     }
  else if (restart) {
     // not counting, waiting for restart of counter
     clocks = 0;
     gate_open = true;
     }
}

void ARDUINO_ISR_ATTR On10MHz() {
  if (gate_open) clocks++;
}




void loop() {
  static uint32_t last_offset;
  static int last_seconds;

  if (seconds != last_seconds) {
     last_seconds = seconds;
     Display.invertDisplay(last_seconds & 1 > 0);
     if (seconds == 0) {
        // timer finished.
        uint32_t offset = clocks - frequency;
        if (last_offset != offset) {
           // update display.
           }
        }
     }


  delay(100);
}










void testdrawtext(char *text, uint16_t color) {
  Display.setCursor(0, 0);
  Display.setTextColor(color);
  Display.setTextWrap(true);
  Display.print(text);
}
