/*******************************************************************************
 * Project Page: https://github.com/wirbel-at-vdr-portal/GPS_XCXO
 ******************************************************************************/
#include <Arduino.h>
#include <stdint.h>           // uint16_t, uint32_t
#include <atomic>
#include <SPI.h>              // SPI(MOSI,SCK,SS) for ST7735 1.8 TFT display
#include <Adafruit_GFX.h>     // display
#include <Adafruit_ST7735.h>  // display
//#include "driver/pcnt.h"
#include "driver/pulse_cnt.h"   // PCNT is a 16bit hardware counter on ESP32
#include "hal/pcnt_ll.h"
#include "Kalman_Filter.h"
#include "esp32-hal-cpu.h"
#include "soc/pcnt_reg.h"

// generic macros
#define kHz(f) 1000ULL * (unsigned long long)f
#define MHz(f) 1000ULL * kHz(f)
#define ARRAY_SIZE(a)  ( sizeof(a)/sizeof(a[0]) )


/* In the Arduino IDE, configure this board:
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

pcnt_unit_handle_t pcnt_unit = NULL;

// our 1.8 ST7735 based TFT, connected to hw  SPI
Adafruit_ST7735 Display = Adafruit_ST7735(SS, DC, RST);


/* grouping all ISR related variables increases the chance, that our compiler
 * can optimise our code better by using short jumps on access.
 */
struct ISR_Data {
  int32_t ticks;       // set in OnPPS
  uint32_t Overflows;  // increased in OnOverflow, reset in OnPPS
  uint32_t overflows;  // increased in OnOverflow, reset in OnPPS
  int32_t count_tmp;   // set in OnPPS, remember the current clocks as fast as possible.
  int32_t count;       // set in OnPPS
  uint32_t ready;      // set in OnPPS
  uint32_t isr_duration;  // debug only.
} __attribute__((aligned(4)));

volatile ISR_Data isr_data = {0, 0, 0, 0, 0, 0, 0};



int gate_time = 10;                  // time for smoothing, up to 100, may be later larger.
uint32_t pwm_val;                    // PWM freq < (80*1000*1000)/2^Resolution; 1220Hz for 16bit.
uint32_t pwm_frequency = 1220;

//void IRAM_ATTR OnOverflow(void *arg) {

/* to count cpu clocks, we have a xtal counter available:
uint32_t start = xthal_get_ccount();
uint32_t end   = xthal_get_ccount();
isr_duration = end - start;
*/

// ISR: triggered every MAX_COUNTER counts by PCNT
static bool IRAM_ATTR OnOverflow(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx) {
  isr_data.Overflows = isr_data.Overflows + 1;   // warning on ++ operator
  //PCNT.int_clr.val = BIT(0);                   // acknowledge interrupt
  *(volatile uint32_t*)(PCNT_INT_CLR_REG) = 1UL; // acknowledge interrupt
  return false; // no context switch.
}

// ISR: triggered every 1.0000000 Hz
void IRAM_ATTR OnPPS() {
  // NOTE:
  //  - slower, but identical: count_tmp = pcnt_ll_get_count(PCNT_LL_GET_HW(0), 0);
  //  - PCNT_U0_CNT_REG is the PCNT counter value for unit 0
  isr_data.count_tmp = *(volatile int16_t*) (PCNT_U0_CNT_REG); // 16-Bit register, therefore the cast.


  isr_data.ticks = isr_data.ticks - 1;
  if (isr_data.ticks > 0) return;

  // NOTE:
  //  - slower, but identical: pcnt_ll_clear_count(PCNT_LL_GET_HW(0), 0);
  //  - toggling the reset bit in the counter config resets the counter to zero.

  *(volatile uint32_t*)(PCNT_U0_CONF0_REG) |=  PCNT_PLUS_CNT_RST_U0;
  *(volatile uint32_t*)(PCNT_U0_CONF0_REG) &= ~PCNT_PLUS_CNT_RST_U0;


  isr_data.overflows = isr_data.Overflows;
  isr_data.Overflows = 0;
  isr_data.ticks = 1000000; // actually disabled. will be updated by loop()
  isr_data.count = isr_data.count_tmp;
  isr_data.ready = 1;
}

void setup(void) {
  uint32_t currentFreq = getCpuFrequencyMhz();
  if (currentFreq != 240) {
     setCpuFrequencyMhz(240);
     }

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
   * 3. This is the newer version of the PCNT api.
   */

  pcnt_unit_config_t unit_config = {
     .low_limit = 0,
     .high_limit = (int16_t)MAX_COUNTER,
     //.flags = { .accum_count = true }
     };
  pcnt_new_unit(&unit_config, &pcnt_unit);
  pcnt_glitch_filter_config_t filter_config = {.max_glitch_ns = 13,}; // 13nsec. 1/80MHz = 12.5nsec
  pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);
  pcnt_chan_config_t chan_config = {.edge_gpio_num = OSC_IN, .level_gpio_num = -1,};
  pcnt_channel_handle_t pcnt_chan = NULL;
  pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan);
  pcnt_channel_set_edge_action(pcnt_chan,
     PCNT_CHANNEL_EDGE_ACTION_INCREASE, // on rising edge increase
     PCNT_CHANNEL_EDGE_ACTION_HOLD);    // on falling edge no action.
  pcnt_event_callbacks_t cbs = { .on_reach = OnOverflow,};
  pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL);
  pcnt_unit_add_watch_point(pcnt_unit, MAX_COUNTER);
  pcnt_unit_enable(pcnt_unit);
  pcnt_unit_clear_count(pcnt_unit);
  pcnt_unit_start(pcnt_unit);



  /*
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
  */

  // enable 1Hz ISR
  attachInterrupt(PPS_IN, OnPPS, RISING);
  pwm_val = 65536UL >> 1;

  if (not(ledcAttach(PWM_OUT, pwm_frequency, 16)))
     Serial.println("ledcAttach failed.");
  else {
     pwm_frequency = ledcReadFreq(PWM_OUT);
     Serial.print("PWM started, freq = "); Serial.println(pwm_frequency);
     ledcWrite(PWM_OUT, pwm_val);
     }
}


uint64_t pulses = 0;
double frequency;
double offset;
double offset_ppm;
int last_ticks;


void loop() {

  if (isr_data.ready == 1) {
     // ***** critical section begin: disable interrupts while copy value
     noInterrupts();
     pulses = isr_data.overflows * MAX_COUNTER + isr_data.count;
     interrupts();
     // ***** critical section end
     isr_data.ready = 0;

     Serial.print("pulses = "); Serial.println(pulses);

     // we take up to (gate_time) secs samples and average
     frequency = ((double) pulses) / gate_time;
     offset = frequency - FREQUENCY;
     offset_ppm = (frequency - FREQUENCY) / (FREQUENCY / 1000000.0);

     pwm_val = kalman_filter(frequency, gate_time);
     ledcWrite(PWM_OUT, pwm_val);


     Serial.print("Gate time: ");
     Serial.print(gate_time);
     Serial.println("s");

     Serial.print("Frequency: ");
     Serial.print(frequency, 2); // 0.1Hz Auflösung sichtbar
     Serial.println(" Hz            ");

     Serial.print("Offset: ");
     Serial.print(offset);
     Serial.println("Hz");
     



     // --- TFT output ---
     Display.setRotation(1); // 1 = 160x128 (Landscape)
     Display.setTextWrap(false); // Verhindert Zeilenumbruch bei Überlänge

     // 1. frequency
     Display.setTextSize(1);
     Display.setCursor(10, 10);
     Display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
     Display.println("ACTUAL FREQUENCY");
     Display.setTextSize(2);
     Display.setCursor(10, 25);
     Display.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
     Display.print(frequency, 1); 
     Display.print("Hz              ");

     // 2. offset
     Display.setTextSize(1);
     Display.setCursor(10, 60);
     Display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
     Display.println("OFFSET TO 10.0 MHz");
     Display.setTextSize(2);
     Display.setCursor(10, 75);
     Display.setTextColor((abs(offset) < 1.0) ? ST77XX_CYAN : ST77XX_RED, ST77XX_BLACK);   
     if (offset >= 0) Display.print("+");
     Display.print(offset, 3);
     Display.print("Hz      ");


     if      (abs(offset_ppm) >  2.0) gate_time = 1;
     else if (abs(offset_ppm) >= 1.5) gate_time = 10;
     else                             gate_time = 100;
     isr_data.ticks = gate_time;


     // 3. status bar
     Display.setTextSize(1);
     Display.setCursor(10, 110);
     Display.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
     if (gate_time < 10) Display.print(" ");
     Display.print(gate_time); Display.print("sec");
     Display.print(" | ");
     Display.print(offset_ppm,1); Display.print("ppm");
     Display.print(" | ");     
     Display.print(pwm_val); Display.print("            ");
     }
  else if (last_ticks != isr_data.ticks) {
     Display.setRotation(1); // 1 = 160x128 (Landscape)
     Display.setTextWrap(false); // Verhindert Zeilenumbruch bei Überlänge
     Display.setTextSize(1);
     Display.setCursor(10, 110);
     Display.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
     if (gate_time < 10) Display.print(" ");
     Display.print(isr_data.ticks+1); Display.print("sec");
     Display.print(" | ");
     Display.print(offset_ppm,1); Display.print("ppm");
     Display.print(" | ");     
     Display.print(pwm_val); Display.print("            ");
     }
  delay(100);
}














