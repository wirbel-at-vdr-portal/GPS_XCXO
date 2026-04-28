/*

Ich werde morgen mit dem letzten Stand dieses Codes erneut anfragen.Das klingt nach einem guten Plan. Da wir die PWM bereits als Fehlerquelle ausgeschlossen haben, liegt das Problem definitiv in der Signalübergabe an die PCNT-Hardware innerhalb des neuen v3.0 Treibers.Für Ihre Tests morgen früh noch zwei abschließende Tipps:Hardware-Brücke prüfen: Stellen Sie sicher, dass das Kabel wirklich Kontakt hat. Manchmal sind die Breadboards oder Jumper-Kabel die Ursache.Frequenz-Erhöhung: Falls die 10 Hz immer noch nicht erkannt werden, setzen Sie PWM_FREQ im Code kurz auf 1000 (1 kHz). Manche digitalen Eingangsfilter des ESP32 sind bei extrem niedrigen Frequenzen manchmal eigenwillig.Ich bin bereit, wenn Sie morgen mit den neuen Ergebnissen weitermachen möchten. Wir werden den PCNT definitiv zum Zählen bringen!Einen schönen Feierabend und bis morgen!


*/



















/*
 * ESP32 PWM (16-bit) and PCNT Frequency Measurement
 * Final Software Fix for ESP32 Core v3.0+
 */

#include "driver/pulse_cnt.h"

// Configuration constants
const int PWM_OUT_PIN = 18;
const int PCNT_IN_PIN = 15;
const uint32_t PWM_FREQ = 10;      
const int PWM_RES_BITS = 16;       
const int16_t PCNT_H_LIMIT = 10;   

// Global handles and variables
pcnt_unit_handle_t pc_unit = NULL;
volatile uint32_t overflows = 0;       
volatile uint32_t isr_call_counter = 0; 
unsigned long last_report_time = 0;

/* 
 * Corrected ISR (OnClock)
 */
static bool IRAM_ATTR OnClock(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    overflows++;
    isr_call_counter++;
    return true; 
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("--- Starting PCNT Debugger v3.0+ ---");

    // 1. Force PWM Output
    ledcAttach(PWM_OUT_PIN, PWM_FREQ, PWM_RES_BITS);
    ledcWrite(PWM_OUT_PIN, 32768); 

    // 2. Unit Configuration
    pcnt_unit_config_t unit_config = {
        .low_limit = -1,
        .high_limit = PCNT_H_LIMIT,
        .flags = { .accum_count = true } 
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pc_unit));

    // 3. Channel Configuration
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = PCNT_IN_PIN,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pc_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pc_unit, &chan_config, &pc_chan));

    /* 
     * IMPORTANT: Set actions for BOTH edges explicitly.
     * Increase on rising, stay on falling.
     */
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pc_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    // 4. Register Watch Point before enabling
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pc_unit, PCNT_H_LIMIT));

    // 5. Explicit Event Callbacks
    pcnt_event_callbacks_t cbs = {
        .on_reach = OnClock,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pc_unit, &cbs, NULL));

    // 6. Enable and Start Sequence
    ESP_ERROR_CHECK(pcnt_unit_enable(pc_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pc_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pc_unit));

    Serial.println("PCNT Hardware active.");
}

void loop() {
    if (millis() - last_report_time >= 2000) {
        int pulse_count = 0;
        pcnt_unit_get_count(pc_unit, &pulse_count);
        
        uint32_t total = (overflows * PCNT_H_LIMIT) + (uint32_t)pulse_count;
        
        Serial.printf("Current: %d | Overflows: %u | ISR Calls: %u | Total: %u\n", 
                      pulse_count, overflows, isr_call_counter, total);
        
        last_report_time = millis();
    }
}
