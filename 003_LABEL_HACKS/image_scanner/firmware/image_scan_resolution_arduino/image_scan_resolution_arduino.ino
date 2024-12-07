#include <Adafruit_AS7341.h>
#include <Stepper.h>

Adafruit_AS7341 as7341;

const int stepsPerRevolution = 200;                // Adjust this for your stepper motor
Stepper stepperY(stepsPerRevolution, 2, 4, 3, 5);  // Stepper for X-axis (right/left)
Stepper stepperX(stepsPerRevolution, 6, 8, 7, 9);  // Stepper for Y-axis (up/down)

const int pixelSize = 10;

int xPins[4] = { 6, 8, 7, 9 };  // pins for x-motor coils
int yPins[4] = { 2, 4, 3, 5 };  // pins for y-motor coils

void setup() {
  Serial.begin(115200);

  if (!as7341.begin()) {
    Serial.println("Failed to initialize AS7341!");
    while (1)
      ;
  }
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);
  stepperY.setSpeed(10);  // set first stepper speed (these should stay the same)
  stepperX.setSpeed(12);  // set second stepper speed (^ weird stuff happens when you push it too fast)
  releaseMotors();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();  // Read the incoming command

    switch (command) {
      case 's':  // Take a color reading
        sendColorReading();
        break;
      case 'u':                    // Move up
        stepperY.step(pixelSize);  // Move one step up
                                   //Serial.write('r'); // Take a new reading after moving
        Serial.println("d");       // Signal move is done
        break;
      case 'd':                     // Move down
        stepperY.step(-pixelSize);  // Move one step down
                                    //Serial.write('r'); // Take a new reading after moving
        Serial.println("d");        // Signal move is done
        break;
      case 'r':                     // Move right
        stepperX.step(-pixelSize);  // Move one step right
                                    //Serial.write('r'); // Take a new reading after moving
        Serial.println("d");        // Signal move is done
        break;
      case 'l':                    // Move left
        stepperX.step(pixelSize);  // Move one step right
                                   //Serial.write('r'); // Take a new reading after moving
        Serial.println("d");       // Signal move is done
        break;
      case 'h':  // HOME
        homeYAxis();
        //Serial.write('r'); // Take a new reading after moving
        Serial.println("d");  // Signal move is done
        break;
    }
  }
}

void sendColorReading() {
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

  Serial.print(red1);
  Serial.print(',');
  Serial.println(red2);

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
  as7341.enableLED(false);
}

void homeYAxis() {
  stepperY.step(-1000);  //lowers the pen holder to it's lowest position.
  stepperY.step(20);     //lowers the pen holder to it's first position.
}

void releaseMotors() {
  for (int i = 0; i < 4; i++) {  //deactivates all the motor coils
    digitalWrite(xPins[i], 0);   //just picks each motor pin and send 0 voltage
    digitalWrite(yPins[i], 0);
  }
}