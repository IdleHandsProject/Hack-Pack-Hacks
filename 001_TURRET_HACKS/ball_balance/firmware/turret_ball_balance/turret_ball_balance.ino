#include <Servo.h>

// Pins
const int triggerPin = 2;
const int echoPin = 12;
const int servoPin = 11;

// Desired distance
const float desiredDistance = 12.0; // cm

// PID parameters
const float Kp = 1.2; // Proportional gain
const float Ki = 0.0; // Integral gain
const float Kd = 20.0; // Derivative gain

// PID variables
float previousError = 0.0;
float integral = 0.0;
int servoPosition = 105;
// Servo and sensor
Servo beamServo;

void setup() {
  Serial.begin(115200); // Set baud rate to 115200
  
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  beamServo.attach(servoPin);
  beamServo.write(105); // Start with the beam in a neutral position (middle of 70 and 140)
}

void loop() {
  float distance = measureDistance();

  if (distance > 35.0) {
    // Move servo to the middle position if distance is greater than 30 cm
    beamServo.write(105);
  } else {
    // PID calculations
    float error = desiredDistance - distance;
    integral += error;
    integral = constrain(integral, -10, 10);
    float derivative = error - previousError;
    
    // Calculate PID output
    float output = Kp * error + Ki * integral + Kd * derivative;
    
    // Map PID output to servo position range
    // PID output range is -20 to 20, map this to servo range 70 to 140
    servoPosition = map(output, 25, -25, 80, 120);
    servoPosition = constrain(servoPosition, 80, 120);
    
    beamServo.write(servoPosition);
    
    // Update previous error
    previousError = error;
  }
  
  // Output for debugging
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm, Error: ");
  Serial.print(desiredDistance - distance);
  Serial.print(", Servo Position: ");
  Serial.println(beamServo.read());
  
  delay(50); // Short delay for stability
}

float measureDistance() {
  // Send pulse to trigger
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Read the echo pulse
  long duration = pulseIn(echoPin, HIGH);
  
  // Calculate distance in cm
  float distance = (duration / 2.0) * 0.0344;
  
  // Ensure the distance is within the sensor range
  if (distance < 2.0) distance = 2.0;
  if (distance > 400.0) distance = 400.0;

  return distance;
}