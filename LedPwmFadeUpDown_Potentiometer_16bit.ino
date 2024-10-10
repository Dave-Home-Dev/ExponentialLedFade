// Set PWM output on exponential curver based on potentiometer value
// This code designed around a linear potentiometer

#include <math.h>

// Set pin numbers:
const int potentiometerPin          {A0};   // the number of the potentiometer pin
const int ledPwmPin                 {9};    // the number of the LED pin

// In the Arduino world, ints are 16 bits
const unsigned int potentiometerMin {12};
const unsigned int potentiometerMax {1012};
const unsigned int inputMax         {potentiometerMax - potentiometerMin};
const float        dynamicRange     {100.0};
const unsigned int outputMax        {4000};

// Time control
const unsigned long loopIntervalMs  {10};  // Limit to 100Hz (10ms interval)
unsigned long previousMillis        {0};

// Maximum rate of change
const int maxStepChange             {5};  // Max ±5 steps per iteration -> 1000 / 5 * 20 = 4s fully off to fully on

// Saved variables
int previousPosition                {0};

void setup() {
  // Set up PWM on pins D9 and D10

  // Set pin 9 (PORTB1) as an output
  DDRB |= (1 << DDB1);      // Correctly configure pin 9 as an output
  PORTB &= ~(1 << PORTB1);  // Correctly set pin 9 to low

  // Stop Timer1 and reset configuration
  TCCR1A = 0; 
  TCCR1B = 0;
  TCNT1 = 0;  // Reset Timer1 counter

  // Set Fast PWM mode with ICR1 as TOP
  TCCR1A |= (1 << WGM11); // Fast PWM, mode 14 (WGM13:WGM10 = 1110)
  TCCR1B |= (1 << WGM13) | (1 << WGM12); 

  // Set no prescaler (full speed, 16 MHz clock)
  TCCR1B |= (1 << CS10);  // CS12:0 = 001 -> No prescaler

  // Set TOP value for the timer (ICR1)
  ICR1 = outputMax;  // Set the range to 0-outputMax

  // Set non-inverting mode for output A (pin 9)
  TCCR1A |= (1 << COM1A1); // Clear OC1A on compare match, set at BOTTOM

  // Set initial duty cycle to 0 (PWM off)
  OCR1A = 0; // Duty cycle = 0%
  
  // LEDs off
  pwm_16(0);
 
  // Initialize digital pin LED_BUILTIN as an output.
  DDRB |= (1 << DDB5);  // Set PB5 (Pin 13) as output

  // Configure analog input pin and reference voltage
  pinMode(potentiometerPin, INPUT);
  analogReference(DEFAULT);

  for (unsigned int i=0;i<=inputMax;++i)
  {
    pwm_16( mapInputToOutputUsingExponentialCurve(i));
    delay(1);
  }

  // Serial.begin(115200);

  previousPosition = inputMax;
  previousMillis = millis();
}

unsigned int mapInputToOutputUsingExponentialCurve(unsigned int input)
{
  // Ensure the input value is within the expected range
  input = min(input, inputMax);

  // Calculate the output using an exponential function
  const double lnDR { log(dynamicRange) }; // Precompute log(dynamicRange) for efficiency
  double exponent { (static_cast<double>(input) / inputMax) - 1.0 };
  unsigned int output { static_cast<unsigned int>(outputMax * exp(exponent * lnDR)) };

  // Cap the output to the maximum allowed value
  return min(output, outputMax);
}

void loop() {

  // Ensure we only re-enter this loop every 10ms (100Hz)
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < loopIntervalMs) {
    return;  // Exit if 10ms hasn't passed
  }

  previousMillis += loopIntervalMs;  // Save the last time we ran

  int sensorValue = analogRead(potentiometerPin); // 0-1023

  // Limit the rate of change to ±10 steps per iteration
  int delta = (sensorValue - previousPosition)/10;
  if (delta > maxStepChange) {
    delta = maxStepChange;
  } else if (delta < -maxStepChange) {
    delta = -maxStepChange;
  }

  // Serial.print(previousPosition);
  // Serial.print(" ");
  // Serial.print(sensorValue);
  // Serial.print(" ");
  // Serial.println(delta);

  // Exit function if value hasn't changed.
  if (delta == 0) {
    return;
  }

  // Calculate new 'damped' position
  sensorValue = previousPosition + delta;
  
  // Save new position
  previousPosition = sensorValue;

  // If sensorValue < 0, potentionmeter wiper is in deadzone
  if (sensorValue < potentiometerMin) {
    pwm_16(0); // LED OFF
    PORTB &= ~(1 << PB5);  // Turn off LED_BUILTIN (Pin 13)
    return;
  }

  if (sensorValue > potentiometerMax) {
    sensorValue = potentiometerMax;
  }

  // Deduct deadzone at start of potentiometer
  sensorValue -= potentiometerMin;

  // Use direct register manipulation to control LED_BUILTIN
  if (sensorValue == 0 || sensorValue == inputMax) {
    PORTB |= (1 << PB5);   // Turn on LED_BUILTIN (Pin 13)
  } else {
    PORTB &= ~(1 << PB5);  // Turn off LED_BUILTIN (Pin 13)
  }

  // pwm_16(outputMax); // LED ON
  unsigned int output = mapInputToOutputUsingExponentialCurve(sensorValue);
  // pwm_16(0); // LED OFF

  // Assign PWM duty cycle to pin
  pwm_16(output);
}

void pwm_16(unsigned int val) {
  OCR1A = val;
}
