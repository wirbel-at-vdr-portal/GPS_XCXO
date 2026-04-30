#include <Arduino.h>
#include <stdint.h>           // uint16_t, uint32_t
#include <SPI.h>              // SPI(MOSI,SCK,SS) for ST7735 1.8 TFT display
#include <Adafruit_GFX.h>     // display
#include <Adafruit_ST7735.h>  // display
#include "driver/pcnt.h"      // PCNT is a 16bit hardware counter on ESP32
#include "hal/pcnt_ll.h"

// generic macros
#define kHz(f) 1000ULL * (unsigned long long)f
#define MHz(f) 1000ULL * kHz(f)
#define ARRAY_SIZE(a)  ( sizeof(a)/sizeof(a[0]) )





/* In Arduino IDE, configure this board:
 * "ESP32 Arduino -> ESP32 Dev Module"
 *
 * Now, let's declare the pins we used on the PCB.
 *
 * NOTE: GPIO 34,35,36,39 are input only and doent have internal pullup
 *       resistors -> avoid those pins for the PCNT counter, as they
 *       throw a runtime error regarding the pullup config. 
 */
#define SS      5  // slave select, or, chip select
#define DC      16 // data/command (DC), labeled 'AO' on TFT
#define RST     17 // tft reset (-1 for Arduino RESET pin doesnt work..)
//      MOSI    23
//      MISO    19
//      SCK     18
#define PPS_IN  35 // 1Hz signal from GPS, pin 35 supports hardware PCNT counter.
#define OSC_IN   4 // 10MHz CMOS signal from OCXO
#define PWM_OUT 32 // 16bit PWM
#define FREQUENCY      MHz(10)
#define MAX_COUNTER   (int16_t) 30000 // 16bit signed: less or equal 32767


Adafruit_ST7735 Display = Adafruit_ST7735(SS, DC, RST); // hardware SPI for the display.

volatile uint32_t overflows = 0;     // increased in OnOverflow, reset in OnPPS
volatile uint32_t lastFrequency;     // set in OnPPS
volatile bool dataReady = false;     // set in OnPPS
volatile uint32_t RingBuffer[16];    // ring buffer, set in OnPPS
volatile int writeIndex = 0;         // set in OnPPS
volatile bool secondElapsed = false; // set in OnPPS
         int16_t count;              // set in OnPPS

volatile bool OverflowPending = false;
bool RingBufferInit = false;
int gate_time = 10;                  // time for smoothing, up to 16
uint16_t pwm_val;                    // PWM freq < (80*1000*1000)/2^Resolution; 1220Hz for 16bit.


volatile int ov_cnt = 0; 
// ISR: triggered every MAX_COUNTER counts by 
void IRAM_ATTR OnOverflow(void *arg) {
  ov_cnt++;
  overflows++;
  uint32_t status = 0;
  pcnt_get_event_status(PCNT_UNIT_0, &status); // acknowledge interrupt
}



// ISR: triggered every 1.0000000 Hz
void IRAM_ATTR OnPPS() {
  count = pcnt_ll_get_count(PCNT_LL_GET_HW(0), 0);
  pcnt_ll_clear_count(PCNT_LL_GET_HW(0), 0);
  lastFrequency = (overflows * MAX_COUNTER) + count;
  overflows = 0;
  RingBuffer[writeIndex] = lastFrequency;
  writeIndex = (writeIndex + 1) % 16;//gate_time;
  secondElapsed = true;
}

void setup(void) {
  Serial.begin(115200);
  delay(1000);

  Display.setRotation(1);
  Display.initR(INITR_BLACKTAB);    // Init ST7735S chip, black tab
  Display.fillScreen(ST77XX_BLACK);
  Display.setTextColor(ST77XX_WHITE);

  pinMode(OSC_IN, INPUT);
  pinMode(PPS_IN, INPUT);
  pinMode(PWM_OUT, OUTPUT);


  /* setup PCNT 16-bit hardware counter
   * NOTES:
   * 1. The counter value is signed (-32768..32767). Therefore,
   *    MAX_COUNTER <= 32767.
   * 2. The PCNT noise filter runs on 80MHz, one clk is 12.5nsec;
   *    for 10MHz the high period should be ~50nsec at 50 duty cycle.
   *    Setting the filter higher than 4 will definitly loose all
   *    counts, values less than 2 should be resonable.
   */
  pcnt_config_t pcnt_cfg = {
     .pulse_gpio_num = OSC_IN,                // input pin
     .ctrl_gpio_num  = -1,                    // 
     .lctrl_mode     = PCNT_MODE_KEEP,        // 
     .hctrl_mode     = PCNT_MODE_KEEP,        // 
     .pos_mode       = PCNT_COUNT_INC,        // count rising edges
     .neg_mode       = PCNT_COUNT_DIS,        // 
     .counter_h_lim  = MAX_COUNTER,           // counter end (overflow)
     .counter_l_lim  = 0,                     // counter begin
     .unit           = PCNT_UNIT_0,           // 
     .channel        = PCNT_CHANNEL_0,        // 
     };
  pcnt_unit_config(&pcnt_cfg);
  pcnt_set_filter_value(PCNT_UNIT_0, 1); // running on 80MHz. 1clk = 12.5nsec
  pcnt_filter_enable(PCNT_UNIT_0);
  pcnt_isr_service_install(0);
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
  pcnt_isr_handler_add(PCNT_UNIT_0, OnOverflow, NULL);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  // enable 1Hz ISR
  attachInterrupt(PPS_IN, OnPPS, RISING);
  pwm_val = 65536UL >> 1;
}


void loop() {
  //Serial.println(pps_cnt);
  //Serial.println(overflows);

  if (secondElapsed) {
     //Serial.print("lastFrequency = "); Serial.println(lastFrequency);
     Serial.print("OverflowPending = "); Serial.println(OverflowPending);
     Serial.print("writeIndex = "); Serial.println(writeIndex);

     secondElapsed = false;

     if (not RingBufferInit) {
        RingBufferInit = true;
        for(size_t i=1; i<ARRAY_SIZE(RingBuffer); i++)
           RingBuffer[i] = lastFrequency;
        }

     uint64_t pulses = 0;
     // ***** critical section begin: disable interrupts while copy value
     noInterrupts();
     for(int i=0; i<gate_time; i++) {
        int idx = (writeIndex-1 - i + 16) % 16;
        Serial.print("idx = "); Serial.println(idx);
        pulses += RingBuffer[idx];
        }
     interrupts();
     // ***** critical section end
     Serial.print("pulses = "); Serial.println(pulses);

     // we take up to (gate_time) secs samples and average
     double frequency = ((double) pulses) / gate_time;
     double offset = frequency - FREQUENCY;
     double offset_ppm = (frequency - FREQUENCY) / (FREQUENCY / 1000000.0);

     int new_gate_time;
     if      (abs(offset_ppm) >  2.0) new_gate_time = 1;
     else if (abs(offset_ppm) >= 1.5) new_gate_time = 2;
     else                             new_gate_time = 10;

     if (new_gate_time != gate_time) {
        gate_time = new_gate_time;
        for(int i=0; i<ARRAY_SIZE(RingBuffer); i++)
           RingBuffer[i] = lastFrequency;
        }

     if      (offset_ppm >  2.0) new_gate_time = 1;
     else if (offset_ppm >= 1.5) new_gate_time = 2;
     else                        new_gate_time = 10;

     int32_t next_pwm = pwm_val;
     if      (offset_ppm >  1.0) next_pwm -= (0xFFFF / 8); // about 25% of range.
     else if (offset_ppm < -1.0) next_pwm += (0xFFFF / 8); // about 25% of range.
     else if (offset_ppm >  0.5) next_pwm -= (0xFFFF / 16);
     else if (offset_ppm < -0.5) next_pwm += (0xFFFF / 16);
     else if (offset_ppm > 0)    next_pwm -= 1;
     else if (offset_ppm < 0)    next_pwm += 1;
     pwm_val = constrain(next_pwm, 0, 0xFFFF);

     
     Serial.print("Gate time: ");
     Serial.print(gate_time);
     Serial.println("s");

     Serial.print("Frequency: ");
     Serial.print(frequency, 2); // 0.1Hz Auflösung sichtbar
     Serial.println(" Hz            ");

     Serial.print("Offset: ");
     Serial.print(offset);
     Serial.println("Hz");
     



     // --- TFT AUSGABE (QUERFORMAT) ---
     Display.setRotation(1); // 1 = 160x128 (Landscape)
     Display.setTextWrap(false); // Verhindert Zeilenumbruch bei Überlänge

     // 1. Frequenz (Groß in der Mitte)
     Display.setTextSize(1);
     Display.setCursor(10, 10);
     Display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
     Display.println("ACTUAL FREQUENCY");

     Display.setTextSize(2);
     Display.setCursor(10, 25);
     Display.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    

     Display.print(frequency, 1); 
     Display.print("Hz  ");

     // 2. Offset
     Display.setTextSize(1);
     Display.setCursor(10, 60);
     Display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
     Display.println("OFFSET TO 10.0 MHz");

     Display.setTextSize(2);
     Display.setCursor(10, 75);
     if (abs(offset) < 1.0)
        Display.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
     else
        Display.setTextColor(ST77XX_RED, ST77XX_BLACK);
    
     if (offset >= 0) Display.print("+");
     Display.print(offset, 4); // Hier 4 Nachkommastellen für Präzision
     Display.print("Hz      ");

     // 3. Statuszeile unten
     Display.setTextSize(1);
     Display.setCursor(10, 110);
     Display.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
     if (gate_time < 10) Display.print(" ");
     Display.print(gate_time); Display.print("sec");
     Display.print(" | ");
     Display.print(offset_ppm,1); Display.print("ppm");
     Display.print(" | ");     
     Display.print(pwm_val); Display.print("            ");















/*
     Display.setTextSize(2); // 14px height, 10px width
     Display.setCursor(0, 5);
     Display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
     Display.println("Frequency:");

     Display.setCursor(0, 25); // Abstand für Größe 2
     Display.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
     // Bei Größe 2 passen ca. 10-12 Zeichen pro Zeile. 
     // "10000000.00" sind 11 Zeichen -> knapp!
     Display.print(frequency, 2); 
     Display.println(" Hz");

    // 2. Offset (Mittlerer Bereich)
    Display.setCursor(0, 65);
    Display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    Display.println("Offset:");

    Display.setCursor(0, 85);
    if (abs(offset) < 0.1) Display.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    else Display.setTextColor(ST77XX_RED, ST77XX_BLACK);

    if (offset >= 0) Display.print("+"); 
    Display.print(offset, 2);
    Display.println(" Hz  "); // Leerzeichen zum Löschen alter Reste

    // 3. Gate Time (Unten)
    Display.setCursor(0, 130);
    Display.setTextSize(1); // Gate Time lassen wir klein, damit oben mehr Platz ist
    Display.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    Display.print("Gate Time: ");
    Display.print(gate_time);
    Display.println("s  ");
*/

//
//      Display.fillScreen(ST77XX_BLACK);
//      Display.setCursor(0, 0);
//      Display.print("waiting "); Display.print(seconds); Display.println(" second(s).");
//      Display.setCursor(0, 10);
//      Display.print("offset = "); Display.print(offset); Display.println(" Hz");
//
     }
  delay(100);

}

