// Set PWM output on exponential curve based on a linear potentiometer value
// This code designed around a linear potentiometer and a LED output on pin 9

#include <math.h>
#include <stdint.h>

// Set pin numbers:
static constexpr uint8_t potentiometerPin{ A0 };  // the number of the potentiometer pin

// In the Arduino world, ints are 16 bits
static constexpr uint16_t potLowerThreshold{ 12 };
static constexpr uint16_t potUpperThreshold{ 1012 };
static constexpr uint16_t potRange{ potUpperThreshold - potLowerThreshold };

// Output constants
static constexpr uint16_t pwmTop{ 16000 };
static constexpr uint16_t pwmMin{ 16 };
static constexpr float dynamicRange{ static_cast<float>(pwmTop) / static_cast<float>(pwmMin) };

/*
 * Exponential mapping
 * -------------------
 *
 * Map the input range [k, k + potRange] exponentially onto
 * [pwmMin, pwmMin * dr], where:
 *
 *     x = k             -> pwmMin
 *     x = k + potRange  -> pwmMin * dr
 *
 * The desired relationship is:
 *
 *     f(x) = pwmMin * dr^((x - k) / potRange)
 *
 * Expressing the power as an exponential gives:
 *
 *     f(x) = exp( ln(pwmMin) + ln(dr) * (x - k) / potRange )
 *
 * Expanding and collecting the terms dependent on x:
 *
 *     f(x) = exp( x * ln(dr) / potRange
 *                 - k * ln(dr) / potRange
 *                 + ln(pwmMin) )
 *
 * Define the constants:
 *
 *     a = ln(dr) / potRange
 *     b = a * k + ln(pwmMin)
 *
 * giving the compact form:
 *
 *     f(x) = exp(a * x + b)
 *
 * Thus the runtime calculation reduces to a single fused
 * multiply-add followed by the exponential:
 *
 *     exponent = fmaf(a, x, b);
 *     output   = expf(exponent);
 *
 * The constants a and b are calculated once; the only
 * floating-point work required for each new input is therefore
 * the affine transformation of x and the subsequent expf().
 */

// Pre-compute some more output constants
static constexpr float a{ logf(dynamicRange) / static_cast<float>(potRange) };
static constexpr float b{ a * static_cast<float>(potLowerThreshold) + logf(static_cast<float>(pwmMin)) };

// Time control
static constexpr uint16_t loopIntervalMs{ 10 };  // Limit to 100Hz (10ms interval)

// Maximum rate of change
static constexpr uint16_t maxStepChange{ 5 };  // Max ±5 steps per iteration -> 1000 / 5 * 10 = 2s fully off to fully on

// Saved variables
unsigned long previousMillis{ 0 };
uint16_t previousPosition{ 0 };

void setup() {
  // Set up PWM on pins D9

  // Set pin 9 (PORTB1) as an output
  DDRB |= (1 << DDB1);      // Correctly configure pin 9 as an output
  PORTB &= ~(1 << PORTB1);  // Correctly set pin 9 to low

  // Stop Timer1 and reset configuration
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;  // Reset Timer1 counter

  // Set non-inverting mode for output A (pin 9)
  TCCR1A |= (1 << COM1A1);  // Clear OC1A on compare match, set at BOTTOM

  // Set Fast PWM mode with ICR1 as TOP
  TCCR1A |= (1 << WGM11);  // Fast PWM, mode 14 (WGM13:WGM10 = 1110)
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << WGM13);

  // Set no prescaler (full speed, 16 MHz clock)
  TCCR1B |= (1 << CS10);  // CS12:0 = 001 -> No prescaler
  //TCCR1B |= (1 << CS11); // Set prescaler to /64

  // Set TOP value for the timer (ICR1)
  ICR1 = pwmTop;  // Timer1 TOP: counter runs from 0 to pwmTop, inclusive

  // Initalise PWM with LEDs off
  pwm_16(0);

  // Initialize digital pin LED_BUILTIN as an output.
  DDRB |= (1 << DDB5);  // Set PB5 (Pin 13) as output

  // Configure analog input pin and reference voltage
  pinMode(potentiometerPin, INPUT);
  analogReference(DEFAULT);

  uint16_t sensorValue = (uint16_t)analogRead(potentiometerPin);  // 0-1023

  for (uint16_t i = 0; i <= sensorValue; ++i) {
    pwm_16(mapInputToOutputUsingExponentialCurve(i));
    delay(1);
  }

  previousPosition = sensorValue;
  previousMillis = millis();
}

/// @brief This function maps the portentiometer value to a point on an exponentional curve.
///        If the input value is less than the lower deadzone value, the LEDs goes off.
///        If the input value is equal to the lower deadzone value, the PWM duty cycle will be (100/dynamic_range) percent.
///        If the input value more than the upper deadzone value, the LEDs are limits to 100% PWM
static uint16_t mapInputToOutputUsingExponentialCurve(uint16_t input) {
  // Ensure the input value is within the expected range
  if (input < potLowerThreshold) return 0;       // LEDs off
  if (input > potUpperThreshold) return pwmTop;  // LEDs 100%

  // Calculate the output using an exponential function
  const float exponent{ fmaf(a, static_cast<float>(input), b) };
  const uint16_t output{ static_cast<uint16_t>(expf(exponent)) };

  // Cap the output to the maximum allowed value
  return min(output, pwmTop);
}

static void pwm_16(uint16_t val) {
  if (val != OCR1A) {
    OCR1A = val;  // Duty cycle
  }
}

static void update() {
  uint16_t sensorValue = (uint16_t)analogRead(potentiometerPin);  // 0-1023

  // Limit the rate of change to ±5 steps per iteration
  int16_t delta = static_cast<int16_t>(sensorValue - previousPosition);
  if (delta > static_cast<int16_t>(maxStepChange)) {
    delta = maxStepChange;
  } else if (delta < -static_cast<int16_t>(maxStepChange)) {
    delta = -maxStepChange;
  }

  // Exit function if value hasn't changed.
  if (delta == 0) {
    return;
  }

  // Calculate new 'damped' position
  sensorValue = previousPosition + delta;

  // Save new position
  previousPosition = sensorValue;

  // Use direct register manipulation to control LED_BUILTIN
  if (sensorValue == potLowerThreshold || sensorValue == potUpperThreshold) {
    PORTB |= (1 << PB5);  // Turn on LED_BUILTIN (Pin 13)
  } else {
    PORTB &= ~(1 << PB5);  // Turn off LED_BUILTIN (Pin 13)
  }

  // Do the math...
  uint16_t output = mapInputToOutputUsingExponentialCurve(sensorValue);

  // Assign PWM duty cycle to pin
  pwm_16(output);
}

void loop() {

  // Ensure we only re-enter this loop every 10ms (100Hz)
  unsigned long now = millis();
  if (now - previousMillis < loopIntervalMs) {
    return;  // Exit if 10ms hasn't passed
  }

  previousMillis += loopIntervalMs;  // Save the last time we ran

  update();
}
