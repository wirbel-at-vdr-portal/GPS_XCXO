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

void process_loop(float f_measured);
void apply_pwm(float estimated_f);


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

volatile uint32_t Overflows = 0;     // increased in OnOverflow, reset in OnPPS
volatile uint32_t overflows = 0;     // increased in OnOverflow, reset in OnPPS
volatile bool dataReady = false;     // set in OnPPS
volatile int32_t ticks = 0;
volatile bool data_ready = false;    // set in OnPPS
volatile int32_t count;              // set in OnPPS
volatile int32_t count_tmp;          // set in OnPPS, remember the current clocks as fast as possible.

int gate_time = 10;                  // time for smoothing, up to 16
uint32_t pwm_val;                    // PWM freq < (80*1000*1000)/2^Resolution; 1220Hz for 16bit.
uint32_t pwm_frequency = 1220;

// ISR: triggered every MAX_COUNTER counts by 
void IRAM_ATTR OnOverflow(void *arg) {
  Overflows++;
  PCNT.int_clr.val = BIT(0); // acknowledge interrupt
}

// ISR: triggered every 1.0000000 Hz
void IRAM_ATTR OnPPS() {
  count_tmp = pcnt_ll_get_count(PCNT_LL_GET_HW(0), 0);
  if (--ticks > 0) return;
  pcnt_ll_clear_count(PCNT_LL_GET_HW(0), 0);
  ticks = 1000000; // actually disabled. will be updated by loop()
  count = count_tmp;
  overflows = Overflows;
  Overflows = 0;
  data_ready = true;
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
  if (data_ready) {

     // ***** critical section begin: disable interrupts while copy value
     noInterrupts();
     data_ready = false;
     pulses = overflows * MAX_COUNTER + count;
     interrupts();
     // ***** critical section end

     Serial.print("pulses = "); Serial.println(pulses);

     // we take up to (gate_time) secs samples and average
     frequency = ((double) pulses) / gate_time;
     offset = frequency - FREQUENCY;
     offset_ppm = (frequency - FREQUENCY) / (FREQUENCY / 1000000.0);


     int32_t next_pwm = pwm_val;
     if      (offset_ppm >  1.0) next_pwm -= (0xFFFF / 8); // about 25% of range.
     else if (offset_ppm < -1.0) next_pwm += (0xFFFF / 8); // about 25% of range.
     else if (offset_ppm >  0.5) next_pwm -= (0xFFFF / 16);
     else if (offset_ppm < -0.5) next_pwm += (0xFFFF / 16);
     else if (offset_ppm > 0)    next_pwm -= 1;
     else if (offset_ppm < 0)    next_pwm += 1;
     pwm_val = constrain(next_pwm, 0, (1UL<<16UL)-1UL);
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
     Display.print("Hz              ");

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
     Display.print(offset, 2); // Hier 4 Nachkommastellen für Präzision
     Display.print("Hz      ");


     if      (abs(offset_ppm) >  2.0) gate_time = 1;
     else if (abs(offset_ppm) >= 1.5) gate_time = 10;
     else                             gate_time = 100;
     ticks = gate_time;


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
     }
  else if (last_ticks != ticks) {
     Display.setRotation(1); // 1 = 160x128 (Landscape)
     Display.setTextWrap(false); // Verhindert Zeilenumbruch bei Überlänge
     Display.setTextSize(1);
     Display.setCursor(10, 110);
     Display.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
     if (gate_time < 10) Display.print(" ");
     Display.print(ticks+1); Display.print("sec");
     Display.print(" | ");
     Display.print(offset_ppm,1); Display.print("ppm");
     Display.print(" | ");     
     Display.print(pwm_val); Display.print("            ");
     }
  delay(100);

}











// Zeitstempel für Timeout
uint32_t last_pps_time = 0;
bool holdover_active = false;

// Schwellenwert für Plausibilität (z.B. max. 10 Hz Abweichung vom Erwartungswert)
const float MAX_INNOVATION = 10.0; 
const uint32_t PPS_TIMEOUT_MS = 2000; // 2 Sekunden ohne PPS -> Holdover

void process_loop(float f_measured) {
  uint32_t now = millis();
  float dt = (now - last_pps_time) / 1000.0;

  // 1. Prädiktion (Immer ausführen, um den Zustand fortzuführen)
  f = f + d * dt;
  // ... (Unsicherheits-Update P00, P01, P10, P11 wie im 2D-Modell)
    P00 += dt * (dt * P11 + P01 + P10) + Q_f;
    P01 += dt * P11;
    P10 += dt * P11;
    P11 += Q_d;














  // Prüfen, ob ein gültiges PPS-Signal vorliegt
  float innovation = abs(f_measured - f);
    
  if (now - last_pps_time < PPS_TIMEOUT_MS && innovation < MAX_INNOVATION) {
     // --- NORMALBETRIEB: Update mit Messung ---
     holdover_active = false;
        
     float S = P00 + R_m;
     float K0 = P00 / S;
     float K1 = P10 / S;

     f += K0 * (f_measured - f);
     d += K1 * (f_measured - f);
        
     // ... (P-Matrix Update wie oben)
        
    last_pps_time = now;
    } else {
        // --- HOLDOVER: Nur Vorhersage nutzen ---
        holdover_active = true;
        // Wir korrigieren f und d NICHT mit der Messung.
        // P (Unsicherheit) wächst automatisch weiter an, da Q addiert wird.
    }

    // PWM Ausgabe basierend auf der aktuellen Schätzung f
    apply_pwm(f);
}


void apply_pwm(float estimated_f) {
  float target_pwm = CENTER_PWM + (estimated_f * STEPS_PER_HZ);
  ledcWrite(0, (uint32_t)constrain(target_pwm, 0, 65535));
}






