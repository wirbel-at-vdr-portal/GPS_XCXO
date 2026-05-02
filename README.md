# About
Project goal is a ~~less than 100 bucks~~ small budget hobbyists GPS disciplined 10MHz XCXO with less than 0.5Hz freq offset at 10MHz (i want less) using a cheap GPS receiver with 1PPS output.

# Ingredients
- an cheap 10MHz OCXO with 0..4 volts V_control input, which has in it's voltage control range the center freq of 10.000000 MHz and CMOS or TTL output,
  costs at common market places ~15€. I use an CTI OSC5A2B02 OCXO based board.
- an ESP32 devkit C V4, ~12€ (mine was ~8€)
- Arduino IDE with a few libs, 0€, but some efforts in/plus time/research.
- a GPS module with an 1pps (pulse per second) digital signal output ~~, 12€. Mine has an SMT LED, which i use.~~
  I switched to an Sparkfun GPS-17285 module, based on Neo-M9N, together with a Sparkfun GPS-14986 antenna. A bit expensive, but more fun.
- a few electronic components
- an old 1.8 inch TFT based on St7735, i use the one with the large SD card slot, ~6€
- the PCB, design is here, ~20€ for 5 pieces.
- a 10..12volts DC power supply with ~1amp continously, - no costs. Everybody has such one laying around. Preferred with the 5.5mm 'DC connector', plus is inner.

# How it is planned to work
- the 1PPS signal from the GNSS / GPS module triggers an interrupt routine, which resets a counter.
- the 10MHz signal is counted by the ESP32 specific hardware counter PCNT. Every 30000 counts, this counter fires a second ISR.
- the counter overflows are counted. This is an important step, as no µC will fire 10*10^6 interrupts per second.
  With this solution, our ISR is called 333 times a second only.
- the if the gate time is over, stopped (again by the 1PPS), we calcalute the number of pulses: pulses = current_counter + overflows * 30000
- now, we can calculate the frequency: frequency = pulses / gate_time
- and the current frequency error in ppm: errorPPM = (measuredFreq - TARGET_FREQ) / (TARGET_FREQ / 1e6)
- as our OCXO is voltage controlled by 0..4volts, we need to generate a control voltage. We generate a PWM, again using hardware counters of the ESP32.
- the pwm signal is RC filtered, we need to put a bit effort in this filter. We want more than 100dB frequency suppression for the PWM frequency.
- and finally, we somehow need to map an error in ppm to and PWM value. This job serves a digital filter with a bit brain, the Kalman filter.

***NOTE NOTE NOTE:***

***This is work in progress. Everything still changes day by day. Success is optional, this is a fully fun based project.***

Thoughts:
1. ~~To be understood: Do we really need an real OCXO for this, or, might a cheap second ESP32, with xtra programming do the job of an OCXO?
   -> Tested && understood. Not really, even the short time stability is too bad.
2. ~~How should i implement PWM corrections? Basic idea, the larger the offset is, the larger the steps.
   -> we need a bit of statistic and use an Kalman filter approach.
