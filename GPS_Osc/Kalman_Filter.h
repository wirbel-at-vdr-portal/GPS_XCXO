/*******************************************************************************
 * Project Page: https://github.com/wirbel-at-vdr-portal/GPS_XCXO
 ******************************************************************************/
#include <Arduino.h>


/* NOTES:
 *------------------------------------------------------------------------------
 * 0. Before reading this source code, be shure to read about the
 *    'Kalman filter', for example:
 *       - https://en.wikipedia.org/wiki/Kalman_filter
 *    Only after that, continue.
 *
 *
 *    From the link above..
 *    [cite]
 *     "This Kalman filtering was first described and developed partially in
 *      technical papers by Swerling (1958), Kalman (1960) and Kalman and Bucy
 *      (1961).
 *      The Apollo computer used 2k of magnetic core RAM and 36k wire rope [...].
 *      The CPU was built from ICs [...]. Clock speed was under 100 kHz [...]. The
 *      fact that the MIT engineers were able to pack such good software (one of
 *      the very first applications of the Kalman filter) into such a tiny
 *      computer is truly remarkable.
 *         Interview with Jack Crenshaw, by Matthew Reed, TRS-80.org (2009) [1]"
 *    [/cite]
 *------------------------------------------------------------------------------
 *
 * 1. State coupling: Through P01 and P10, the filter "knows" that an error
 *    in the frequency could also be due to an incorrectly estimated drift.
 *
 * 2. Holdover intelligence: If GPS fails (ppsValid = false), the filter
 *    only calculates the "predict" part. Since x_drift was learned during
 *    good times, your OCXO continues to run much more precisely than with
 *    a simple fixed value.
 *
 * 3. Adaptation to dt: Since the Variance R becomes extremely small at
 *    measuring time dt=100, the filter almost perfectly replicates the
 *    measurement. At dt=1, however, it massively "smooths" across the matrix.
 *
 * 4. A small practical tip: Before you activate the system, let it run for
 *    about 10-20 minutes at dt=1 so that the matrix P can decrease from its
 *    high initial values ​​to stable operating values.
 *
 * 5. In Kalman filter theory, S stands for innovation covariance (also called
 *    system uncertainty). I use the variable name 'total_uncertainty'.
 *    It can be simplified as the total uncertainty present at the moment
 *    of measurement.
 *    It consists of two parts:
 *        P00 (estimation error): How uncertain is the filter about its own
 *                                state (x_pwm)?
 *        the variance 'R', or measurement noise: How much does the current
 *                          GPS measurement fluctuate?
 *    Why is S so important? The variable S forms the denominator in the
 *    calculation of the Kalman gain (K).
 *       K = P/S = (own uncertainty) / (own uncertainty + measurement noise)
 *
 *    This gives the logic of the filter:
 *    - If our measurement noise (R) is huge compared to your own uncertainty (P), S becomes very large.
 *      This results in a tiny K. The filter says, "The measurement is so noisy, I'll almost completely
 *      ignore it."
 *    - If your own uncertainty (P) is very large (e.g., right after starting) and the measurement is clean,
 *      K approaches 1. The filter says, "I have no idea where I am; I trust the measurement completely."
 *
 *    - In summary, S is the measure of the combined uncertainty, which the filter uses to decide how much
 *      weight to give to the new measurement compared to its previous prediction.
 *
 *    - Variance (P00, P11): The square of the standard deviation.
 *                           A high value means "I'm just guessing," a small value means "I'm very sure."
 *
 *    - Correlation (P01, P10): These values ​​link the two states.
 *                              They allow the filter to say, "If the frequency continues to deviate upwards,
 *                              I probably need to adjust my drift estimate."
 *
 *    - Innovation: The difference between what the filter predicted and what the GPS actually
 *                  measured (the "surprise effect").
 *
 *    - Observe the variable `state.x_drift`.
 *         Does it drift steadily towards a small value (e.g., 0.000123)?
 *           -> Yes. Your filter is learning the aging perfectly.
 *           -> No.  Does it jump around wildly? Then either your `Q_drift` is too high or
 *                   the `DEADZONE_PWM` is still too small for your current measurement
 *                   interval `dt`.
 *
 */




struct KalmanGPSDO {
  // --- States ---
  float x_pwm;      // Estimated optimal PWM value (0 - 65535)
  float x_drift;    // Estimated drift rate (PWM change per second)

  // --- Covariance Matrix P (System Uncertainty) ---
  // P represents the estimation error.
  // The diagonal elements (P00, P11) are the variances of our states.
  float P00;        // Estimation error variance for PWM value
  float P01;        // Correlation between PWM error and Drift error
  float P10;        // (Symmetric to P01)
  float P11;        // Estimation error variance for the Drift rate

  // --- Process Noise Q (OCXO Stability) ---
  // Q defines how much uncertainty we add per second.
  const float Q_pwm   = 0.0001f;   // Random walk of the OCXO frequency
  const float Q_drift = 0.000001f; // Instability of the aging/drift rate

  float max_ppm;    // Dynamic acceptance threshold for PPS measurements
};






KalmanGPSDO state = {
  .x_pwm   = 32768.0,         // middle of 16-bit PWM range.
  .x_drift = 0.0,             // no freq drift
  .P00     = 100.0,           // Estimation error variance for PWM value
  .P01     = 0.0, .P10 = 0.0, // Correlation between PWM error and Drift error. Symmetric.
  .P11     = 1.0,             // Estimation error variance for the Drift rate
  .max_ppm = 10.0f,           // update only, if we find samples less than 10ppm off
};


/* Our measure uncertainty is about 3 counts (0.3ppm). For one sec measurement time that
 * means 3Hz. If we increase the measurement time to 10Hz, we get 0.3Hz. And finally,
 * for 100sec 0.03Hz. This is the uncertainty in PWM bits:
 */
const float TARGET_FREQ = 10000000.0;
const float PWM_PRO_PPM = 65535.0 / 4.0; 
const float MEASUREMENT_ERROR_PWM = 0.3 * PWM_PRO_PPM; // 3 counts error at 1s; ~4915
const float DEADZONE_PPM = 0.003f;                     // Deadzone: If error is less than 0.003 ppm (0.03 Hz @ 10 MHz), stay quiet
const float DEADZONE_PWM = DEADZONE_PPM * PWM_PRO_PPM; // approx. 49 digits
const float MAX_DRIFT = 0.01f;                         // PWM change per second: ~864 digits/day




/* measuredFreq: our current measured frequency
 * dt          : the time difference in seconds between that last time updateKalman()
 *               was called and this time. It's used to predict how far the result
 *               is estimated to drift.
 *
 * return value: an uint32_t, as input for a 16bit hardware PWM, designed for any
 *               native 32bit µC. range: 2^16 / 2 == center (default);
 *                                       otherwise 0x0 .. (2^16-1)
 */
uint32_t kalman_filter(float measuredFreq, float dt) {
  // 0. check first.
  float errorPPM = (measuredFreq - TARGET_FREQ) / (TARGET_FREQ / 1e6);
       if (state.P00 < 1.0 ) state.max_ppm = 3.0;
  else if (state.P00 < 10.0) state.max_ppm = 5.0;
  else                       state.max_ppm = 10.0f;
  bool ppsValid = fabs(errorPPM) < state.max_ppm;


  // --- 1. PREDICT PHASE ---
  // Project the state ahead: x = F * x
  state.x_pwm += state.x_drift * dt;



  // Project the error covariance ahead: P = F*P*F' + Q
  // Or, let's say, predict the uncertainty matrix P.
  // This accounts for the fact that our certainty decreases over time
  // F is the transition matrix for time integration
  state.P00 += dt * (state.P01 + state.P10 + dt * state.P11) + state.Q_pwm;
  state.P01 += dt * state.P11;
  state.P10 += dt * state.P11;
  state.P11 += state.Q_drift;

  if (ppsValid) {
     // measurement
     float errorPPM = (measuredFreq - TARGET_FREQ) / (TARGET_FREQ / 1e6);
     float measurement = state.x_pwm - (errorPPM * PWM_PRO_PPM);

     // Variance R of measurement noise (decreases quadratic with time)
     float variance = pow(MEASUREMENT_ERROR_PWM / dt, 2);


     // --- 2. UPDATE PHASE (Correction) ---
     // Innovation Covariance S
     // (Total uncertainty of the current measurement, often called system uncertainty)
     float total_uncertainty = state.P00 + variance;

     // K = Kalman Gain (Weighting between estimate and measurement)
     // K0 for PWM correction, K1 for learning the Drift
     float K0 = state.P00 / total_uncertainty;
     float K1 = state.P10 / total_uncertainty;


     // Update State with measurement innovation
     float innovation = measurement - state.x_pwm;
     if (fabs(innovation) < DEADZONE_PWM)
        innovation = 0.0f; // too small offset to do some work.

     state.x_pwm   += K0 * innovation;
     state.x_drift += K1 * innovation;


     // CLAMPING: we don't want any unrealistic values for frequency drift.
     // And, dont wan't wind-up, if GPS is missing.
     state.x_drift = constrain(state.x_drift, -MAX_DRIFT, MAX_DRIFT);
     state.x_pwm = constrain(state.x_pwm, 0.0f, 65535.0f);

     if (state.x_pwm < 1.0f || state.x_pwm > 65534.0f) {
        // we are out of control. hard reset!
        state.x_pwm   = 32768.0;
        state.x_drift = 0.0;
        state.P00 = 100.0;
        state.P01 = state.P10 = 0.0;
        state.P11 = 1.0;
        state.max_ppm = 10.0f;
        }
     
     // Update Error Covariance Matrix: P = (I - K*H) * P
     // This reduces our uncertainty because we just received a valid measurement
     float P00_old = state.P00;
     float P01_old = state.P01;

     state.P00 -= K0 * P00_old;
     state.P01 -= K0 * P01_old;
     state.P10 -= K1 * P00_old;
     state.P11 -= K1 * P01_old;


     Serial.print("Freq:");         Serial.print(measuredFreq, 2);
     Serial.print(",PWM:");         Serial.print(state.x_pwm, 2);
     Serial.print(",Drift:");       Serial.print(state.x_drift, 6);
     Serial.print(",Innovation:");  Serial.print(innovation, 4);
     Serial.print(",Uncertainty:"); Serial.println(state.P00, 4);
     }

  // 4. output
  return (uint32_t) constrain(state.x_pwm, 0, 65535);
}
