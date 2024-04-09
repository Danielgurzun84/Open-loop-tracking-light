#define FIRMWARE_VERSION "v1.0.0"

#define UART_BAUDRATE 9600

#define MOTOR_L1_PIN 5
#define MOTOR_L2_PIN 6
#define MOTOR_R1_PIN 9
#define MOTOR_R2_PIN 10

#define LDR_L_PIN A0
#define LDR_R_PIN A1

#define ADC_RESOLUTION 10
#define ADC_OUTPUT_RANGE (1 << ADC_RESOLUTION)
#define ADC_FULL_SCALE_VOLTAGE 5.0

#define LDR_DIFF_THRESHOLD 100
#define LDR_LOCK_THRESHOLD 800

#define MOTOR_FORWARD_PWM_VALUE_RIGHT 130
#define MOTOR_FORWARD_PWM_VALUE_LEFT 160
#define MOTOR_REVERSE_PWM_VALUE -150
#define MOTOR_LEFT_TURN_PWM_VALUE_LEFT 100
#define MOTOR_LEFT_TURN_PWM_VALUE_RIGHT 240
#define MOTOR_RIGHT_TURN_PWM_VALUE_RIGHT 70
#define MOTOR_RIGHT_TURN_PWM_VALUE_LEFT 240
#define MOTOR_TURN_DURATION_MS 250

// Function to measure LDR Circuit Voltage
int MeasureLDRCircuitVoltage(int PinNumber) {
  int AnalogReadResult = analogRead(PinNumber);
  return AnalogReadResult;
}

// Function to Set Motor Control Parameters
void SetMotorControlParameters(int PWMValue, unsigned int HBridgeControlPinA, unsigned int HBridgeControlPinB) {
  if (PWMValue > 0) {
    analogWrite(HBridgeControlPinB, PWMValue);
    digitalWrite(HBridgeControlPinA, LOW);
  } else if (PWMValue < 0) {
    analogWrite(HBridgeControlPinA, -PWMValue);
    digitalWrite(HBridgeControlPinB, LOW);
  } else {
    // Stop the motor if PWM is 0
    analogWrite(HBridgeControlPinA, 0);
    analogWrite(HBridgeControlPinB, 0);
  }
}

// Function to Update Motor Speed
void UpdateMotorSpeed(int LeftMotorPWMValue, int RightMotorPWMValue, unsigned long DurationMilliseconds) {
  // Set Left Motor Control Parameters
  SetMotorControlParameters(LeftMotorPWMValue, MOTOR_L1_PIN, MOTOR_L2_PIN);

  // Set Right Motor Control Parameters
  SetMotorControlParameters(RightMotorPWMValue, MOTOR_R1_PIN, MOTOR_R2_PIN);

  // Add blocking delay (in ms)
  delay(DurationMilliseconds);
}

// Setup function runs once when board is powered up or reset
void setup() {
  // Initialise UART
  Serial.begin(UART_BAUDRATE);

  // Initialise Motor Driver
  pinMode(MOTOR_L1_PIN, OUTPUT);
  pinMode(MOTOR_L2_PIN, OUTPUT);
  pinMode(MOTOR_R1_PIN, OUTPUT);
  pinMode(MOTOR_R2_PIN, OUTPUT);


}

// Loop function runs over and over again forever
void loop() {
  int LeftLDRValue = MeasureLDRCircuitVoltage(LDR_L_PIN);
  int RightLDRValue = MeasureLDRCircuitVoltage(LDR_R_PIN);

  unsigned int LDRDiffMagnitude = abs(LeftLDRValue - RightLDRValue);

  int LeftMotorPWMValue = 0;
  int RightMotorPWMValue = 0;
  unsigned long TurnDuration = 0;



  if (LDRDiffMagnitude > LDR_DIFF_THRESHOLD) {
    if (LeftLDRValue > RightLDRValue) {
      LeftMotorPWMValue = MOTOR_RIGHT_TURN_PWM_VALUE_LEFT;
      RightMotorPWMValue = MOTOR_RIGHT_TURN_PWM_VALUE_RIGHT;
      TurnDuration = MOTOR_TURN_DURATION_MS;
    } else {
      LeftMotorPWMValue = MOTOR_LEFT_TURN_PWM_VALUE_LEFT;
      RightMotorPWMValue = MOTOR_LEFT_TURN_PWM_VALUE_RIGHT;
      TurnDuration = MOTOR_TURN_DURATION_MS;
    }

    UpdateMotorSpeed(LeftMotorPWMValue, RightMotorPWMValue, TurnDuration);
  } else {
    if (min(LeftLDRValue, RightLDRValue) > LDR_LOCK_THRESHOLD) {
      // Stop the motors if light intensity is high
      LeftMotorPWMValue = 0;
      RightMotorPWMValue = 0;
    } else {
      // Move forward if light intensity is low
      LeftMotorPWMValue = MOTOR_FORWARD_PWM_VALUE_LEFT;
      RightMotorPWMValue = MOTOR_FORWARD_PWM_VALUE_RIGHT;
    }

    UpdateMotorSpeed(LeftMotorPWMValue, RightMotorPWMValue, 0);
  }
}