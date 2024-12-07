/* This example will read all channels from the AS7341 and print out reported values */

#include <Adafruit_AS7341.h>
#include <Stepper.h>
#include <Servo.h>




Adafruit_AS7341 as7341;

// Stepper motor parameters
const int stepCount = 200;
const int stepsPerRevolution = 2048;



int xPins[4] = {6, 8, 7, 9};  // pins for x-motor coils
int yPins[4] = {2, 4, 3, 5};    // pins for y-motor coils


//Servo
const int SERVO_PIN  = 13;
Servo servo;
int angle = 30; // the current angle of servo motor

bool sensorClose = false; // Goes True when the servo brings the sensor close

void setup() {
  Serial.begin(115200);

  // Wait for communication with the host computer serial monitor
  while (!Serial) {
    delay(1);
  }

  if (!as7341.begin()) {
    Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }

  as7341.setATIME(100);
  as7341.setASTEP(500);
  as7341.setGain(AS7341_GAIN_256X);
}

void loop() {
  uint16_t readings[12];
  as7341.setLEDCurrent(20);  // 4mA
  as7341.enableLED(true);
  delay(10);
  if (!as7341.readAllChannels()) {
    Serial.println("Error reading all channels!");
    return;
  }

  //as7341.enableLED(false);
  int red1 = as7341.getChannel(AS7341_CHANNEL_630nm_F7);
  int red2 = as7341.getChannel(AS7341_CHANNEL_680nm_F8);

  int green1 = as7341.getChannel(AS7341_CHANNEL_515nm_F4);
  int green2 = as7341.getChannel(AS7341_CHANNEL_555nm_F5);

  int blue1 = as7341.getChannel(AS7341_CHANNEL_445nm_F2);
  int blue2 = as7341.getChannel(AS7341_CHANNEL_480nm_F3);

  // Serial.print(red1);
  // Serial.print(',');
  // Serial.println(red2);

  float red = (red1 * 0.5 + red2 * 0.5);        // 630nm and 680nm weighted for red
  float green = (green1 * 0.5 + green2 * 0.5);  // 515nm and 555nm weighted for green
  float blue = (blue1 * 0.5 + blue2 * 0.5);     // 445nm and 480nm weighted for blue

  // Apply different weightings to each channel for better color accuracy
  // Adjust these weights based on the wavelength and the desired output


  // float red = (readings[9] * 0.5+ readings[8] * 0.5);    // 630nm and 680nm weighted for red
  // float green = (readings[3] * 0.5 + readings[6] * 0.5);  // 515nm and 555nm weighted for green
  // float blue = (readings[1] * 0.5 + readings[2] * 0.5);   // 445nm and 480nm weighted for blue

  // Map sensor readings (0-65535) to RGB values (0-255)
  int r = map(red, 0, 65535, 0, 255);
  int g = map(green, 0, 65535, 0, 255);
  int b = map(blue, 0, 65535, 0, 255);

  // int r = map(readings[9], 0, 65535, 0, 255);
  // int g = map(readings[6], 0, 65535, 0, 255);
  // int b = map(readings[2], 0, 65535, 0, 255);



  // Output the RGB values in the format RRR,GGG,BBB
  Serial.print(r);
  Serial.print(",");
  Serial.print(g);
  Serial.print(",");
  Serial.println(b);

  delay(50);  // Delay for readability in Processing
}

void homeYAxis() {
  yStepper.step(-3000);  //lowers the pen holder to it's lowest position.
}
