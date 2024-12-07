import processing.serial.*; 

Serial myPort; // The serial port
int imagex = 100; // Define image width in pixels
int imagey = 100; // Define image height in pixels

int currentX = 0;  // Current X pixel
int currentY = 0;  // Current Y pixel
color[][] imagePixels;  // Array to store pixel colors
boolean scanningUp = true; // Flag to track scan direction

void setup() {
  size(imagex, imagey);  // Set the size of the window based on resolution
  imagePixels = new color[imagex][imagey]; // Initialize array to hold pixel data
  
  // Initialize serial connection
  // Replace with correct port your Arduino is connected to
  String portName = Serial.list()[0];  
  myPort = new Serial(this, portName, 115200);
  
  // Set background to black initially
  background(0);
  
  // Start scanning by requesting the first color reading from the Arduino
  myPort.write('r'); // Send 'r' to take the first color reading
}

void draw() {
  // Draw the scanned pixels as they are read
  for (int x = 0; x < imagex; x++) {
    for (int y = 0; y < imagey; y++) {
      fill(imagePixels[x][y]);
      rect(x, height - 1 - y, 1, 1); // Draw individual pixels
    }
  }
}

void serialEvent(Serial myPort) {
  // When Arduino sends a color reading in RRR,GGG,BBB format, parse it
  String data = myPort.readStringUntil('\n');
  if (data != null) {
    data = trim(data);  // Remove any extra whitespace or line breaks
    String[] rgbValues = split(data, ',');
    
    if (rgbValues.length == 3) {
      // Parse RGB values from Arduino
      int r = int(rgbValues[0]);
      int g = int(rgbValues[1]);
      int b = int(rgbValues[2]);
      
      // Store the color in the pixel array
      imagePixels[currentX][currentY] = color(r, g, b);
      
      // Determine the next move
      if (scanningUp) {
        if (currentY < imagey - 1) {
          currentY++;  // Move up
          myPort.write('u');
        } else {
          scanningUp = false;  // Switch direction when top is reached
          currentX++;  // Move right
          if (currentX < imagex) {
            myPort.write('s'); // Move right
            myPort.write('r'); // Take color reading after moving right
          }
        }
      } else {
        if (currentY > 0) {
          currentY--;  // Move down
          myPort.write('d');
        } else {
          scanningUp = true;  // Switch direction when bottom is reached
          currentX++;  // Move right
          if (currentX < imagex) {
            myPort.write('s'); // Move right
            myPort.write('r'); // Take color reading after moving right
          }
        }
      }
    }
  }
}
