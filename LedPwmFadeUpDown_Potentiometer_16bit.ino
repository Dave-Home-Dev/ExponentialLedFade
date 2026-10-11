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
static constexpr uint16_t frequency{ 1000 };
static constexpr uint16_t pwmBits{ 16000000 / frequency };
static constexpr uint16_t pwmMin{ pwmBits / 1000 };
static constexpr uint16_t pwmTop{ pwmBits-1 }; // Count from 0 - pwmTop
static constexpr float dynamicRange{ static_cast<float>(pwmBits) / static_cast<float>(pwmMin) };

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
  TCCR1A |= (0x02 << COM1A0);  // Clear OC1A on compare match, set at BOTTOM

  // Set Fast PWM mode with ICR1 as TOP

  static constexpr uint8_t WGM {14};  // Fast PWM, mode 14 (WGM13:WGM10 = 1110)
  TCCR1A |= ((WGM & 0x03) << WGM10);
  TCCR1B |= ((WGM & 0x0C) << (WGM12 - 2));

  // Clock Select (Timer) 1
  // 0 -> No clock source
  // 1 -> /1
  // 2 -> /8
  // 3 -> /64
  // 4 -> /256
  // 5 -> /1024
  // 6 -> Ext clk source, clock on falling edge
  // 7 -> Ext clk source, clock on rising edge

  // Set no prescaler (full speed, 16 MHz clock)
  static constexpr uint8_t CS {1};
  TCCR1B |= CS << CS10; // Set prescaler

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
  if (input > potUpperThreshold) return pwmBits; // LEDs 100%

  // Calculate the output using an exponential function
  const float exponent{ fmaf(a, static_cast<float>(input), b) };
  const uint16_t output{ static_cast<uint16_t>(expf(exponent)) };

  return output;
}

/// @note  In AVR, it is not possibe to express a PWM duty cycle of (N-1)/N, where N is the number of bit periods.
///        If N = 100, ie ICR1 = 99, it is not possibe be possible to express a duty cycle of 99%.
///        Example: Configure PWM to have 100 periods by setting ICR1 = 99.
///            When OCR1A = 97 the duty cycle with be 97%
///            When OCR1A = 98 the duty cycle with be 98%
///            There will not be a 99% duty cycle!
///            When OCR1A = 99 the duty cycle with be 100%
static void pwm_16(uint16_t val) {
  // Cap the output to the maximum allowed OCR1A value for 100% PWM duty cycle.
  OCR1A = min(val, pwmTop);  // Duty cycle
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
