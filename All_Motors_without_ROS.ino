#include <AccelStepper.h>

// Define pump pins
#define PUMP1_PIN1 23
#define PUMP1_PIN2 22
#define PUMP2_PIN1 19
#define PUMP2_PIN2 18
#define PUMP3_PIN1 5
#define PUMP3_PIN2 17

// Define Johnson motor pins
#define MOTOR1_PIN1 4 // IN1
#define MOTOR1_PIN2 16 // IN2

// Define encoder connections
#define ENCODER_PIN_A 25
#define ENCODER_PIN_B 26

volatile long encoderCount = 0;
const int stepsPerRevolution = 25; // Adjust based on your encoder's specifications
const int gearRatio = 1620; // Gear ratio
const long stepsPerOutputRevolution = stepsPerRevolution * gearRatio; // Total steps per output revolution

// Create an instance of the AccelStepper class for the stepper motor
AccelStepper stepper(AccelStepper::DRIVER, 2, 15);

void setup() {
  Serial.begin(115200); // Set the baud rate to 115200

  // Set pump pins as outputs
  pinMode(PUMP1_PIN1, OUTPUT);
  pinMode(PUMP1_PIN2, OUTPUT);
  pinMode(PUMP2_PIN1, OUTPUT);
  pinMode(PUMP2_PIN2, OUTPUT);
  pinMode(PUMP3_PIN1, OUTPUT);
  pinMode(PUMP3_PIN2, OUTPUT);

  // Set Johnson motor control pins as outputs
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);

  // Set encoder pins as inputs
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);

  // Attach interrupt to the encoder pin
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);

  // Enable the stepper driver
  pinMode(2, OUTPUT);
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH); // Set initial direction

  stepper.setMaxSpeed(20); // Adjust as needed
  stepper.setAcceleration(10); // Adjust as needed

  Serial.println("Setup complete. Commands:");
  Serial.println("'1' - Turn on Pump 1");
  Serial.println("'2' - Turn off Pump 1");
  Serial.println("'3' - Turn on Pump 2");
  Serial.println("'4' - Turn off Pump 2");
  Serial.println("'5' - Turn on Pump 3");
  Serial.println("'6' - Turn off Pump 3");
  Serial.println("'7' - Rotate Johnson Motor +120 degrees");
  Serial.println("'8' - Rotate Johnson Motor -120 degrees");
  Serial.println("'9' - Control Stepper Motor +90 degrees");
  Serial.println("'0' - Control Stepper Motor -90 degrees");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case '1':
        digitalWrite(PUMP1_PIN1, HIGH);
        digitalWrite(PUMP1_PIN2, LOW);
        Serial.println("Pump 1 turned ON");
        break;
      case '2':
        digitalWrite(PUMP1_PIN1, LOW);
        digitalWrite(PUMP1_PIN2, LOW);
        Serial.println("Pump 1 turned OFF");
        break;
      case '3':
        digitalWrite(PUMP2_PIN1, HIGH);
        digitalWrite(PUMP2_PIN2, LOW);
        Serial.println("Pump 2 turned ON");
        break;
      case '4':
        digitalWrite(PUMP2_PIN1, LOW);
        digitalWrite(PUMP2_PIN2, LOW);
        Serial.println("Pump 2 turned OFF");
        break;
      case '5':
        digitalWrite(PUMP3_PIN1, HIGH);
        digitalWrite(PUMP3_PIN2, LOW);
        Serial.println("Pump 3 turned ON");
        break;
      case '6':
        digitalWrite(PUMP3_PIN1, LOW);
        digitalWrite(PUMP3_PIN2, LOW);
        Serial.println("Pump 3 turned OFF");
        break;
      case '7':
        rotateJohnsonMotor(120); // Rotate Johnson motor +120 degrees
        break;
      case '8':
        rotateJohnsonMotor(-120); // Rotate Johnson motor -120 degrees
        break;
      case '9':
        controlStepperMotor(90); // Control stepper motor +90 degrees
        break;
      case '0':
        controlStepperMotor(-90); // Control stepper motor -90 degrees
        break;
      default:
        Serial.println("Invalid command.");
        break;
    }
  }
}

// Function to rotate Johnson motor by a specific angle
void rotateJohnsonMotor(int angle) {
  long steps = (stepsPerOutputRevolution / 360) * abs(angle);
  encoderCount = 0;

  if (angle > 0) {
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
  } else {
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
  }

  unsigned long startTime = millis();
  while (abs(encoderCount) < steps) {
    // Add a small delay to control speed
    delay(10); // Adjust this delay to control the motor speed
  }

  // Stop the motor
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  Serial.print("Rotation complete in ");
  Serial.print(millis() - startTime);
  Serial.println(" ms");
}

// Function to control stepper motor by a specific angle
void controlStepperMotor(int angle) {
  long steps = (200 / 360.0) * abs(angle); // Assuming 200 steps per revolution and 1.8 degrees per step
  stepper.moveTo(stepper.currentPosition() + steps);
  stepper.runToPosition();
}

// Interrupt Service Routine for the encoder
void encoderISR() {
  int stateA = digitalRead(ENCODER_PIN_A);
  int stateB = digitalRead(ENCODER_PIN_B);

  if (stateA == stateB) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}
