#include <math.h>

// Set PWM output on exponential curver based on potentiometer value
// This code designed around a linear potentiometer

// Set pin numbers:
const PROGMEM int potentiometerPin = A0;   // the number of the potentiometer pin
const PROGMEM int ledPwmPin        = 9;    // the number of the LED pin

// In the Arduino world, ints are 16 bits
const PROGMEM unsigned int potentiometerMin = 10;
const PROGMEM unsigned int potentiometerMax = 1010;
const PROGMEM unsigned int inputMax         = potentiometerMax - potentiometerMin;
const PROGMEM unsigned int outputMax        = 1000;

// Define dynamic range to use between min and max potentiometer input.
const PROGMEM float dynamicRange            = 100.0;

// Constants to simplify PWM calculations
const PROGMEM float k1                      = log(dynamicRange) / inputMax; // Natural log
const PROGMEM float k2                      = (float)outputMax / dynamicRange;

// Smallest value of PWM that can be calculated above zero
const PROGMEM unsigned int outputMin        = (float)outputMax / dynamicRange + 0.5;

// Saved variables
int previousPosition = 0;

//#define DEBUG

void setup() {
  // Set up PWM on pins D9 and D10
  pinMode(ledPwmPin, OUTPUT);
  // Set Waveform Generation Mode 10 
  TCCR1A = (TCCR1A & B00111100) | B10000010; // Set WGM13+12 Phase & freq correct, non-inverting
  TCCR1B = (TCCR1B & B11100000) | B00010001; // Set WGM11+10, Clock Select = No Prescale
  ICR1   = outputMax - 1;
  pwm_16(0); // LEDs off

  // Initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Configure analog input pin and reference voltage
  pinMode(potentiometerPin, INPUT);
  analogReference(DEFAULT);

  // Settle time 100ms
  delay(100);

#ifdef DEBUG
  // Serial
  Serial.begin(115200);
#endif
}

void loop() {
#ifdef DEBUG
  // Read pot position
  int sensorValue = analogRead(potentiometerPin); // 0-1023
  unsigned int outputValue = map( sensorValue, 0, 1023, 0, outputMax);
  Serial.print("Input\t");
  Serial.print(sensorValue);
  Serial.print("\tOutput\t");
  Serial.println(outputValue);
  pwm_16( outputValue );
  return;
#endif

  int sensorValue = analogRead(potentiometerPin); // 0-1023

  // Exit function if value hasn't changed.
  if (sensorValue == previousPosition) {
    return;
  }

  // Save new position
  previousPosition = sensorValue;

  // Deduct deadzone at start of potentiometer
  sensorValue -= potentiometerMin;

  // If sensorValue < 0, potentionmeter wiper is in deadzone
  if (sensorValue < 0) {
    pwm_16(0); // LED OFF
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  // Remove deadzones and baseline to zero
  if (sensorValue > inputMax) {
    sensorValue = inputMax;
  }

  // The meat of the calculation
  // Calculate new output value, round to nearest int.
  // Beware of precision limited to 22 bits, causing
  // rounding value of 'e' in the exponential function
  unsigned int output = k2 * exp(k1 * (float)sensorValue) + 0.5;

  // Belt, braces
  if (output > outputMax) {
    output = outputMax;
  }

  if (output == outputMin || output == outputMax) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  // Assign PWM duty cycle to pin
  pwm_16(output);

  //Serial.print("Input:\t");
  //Serial.print(sensorValue);
  //Serial.print("\tOutput:\t");
  //Serial.println(output);
}

void pwm_16(unsigned int val) {
  OCR1A = val;
}
