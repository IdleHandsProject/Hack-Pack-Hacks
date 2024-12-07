import processing.serial.*; 

Serial myPort; // The serial port
String serialInput = ""; 
int x = 0, y = 0, r = 0, g = 0, b = 0; // Initialize RGB values

void setup() {
  size(400, 400);
  
  // List all available serial ports for debugging (optional)
  // printArray(Serial.list());
  
  // Replace with the correct port your Arduino is connected to (you may need to adjust this)
  myPort = new Serial(this, Serial.list()[0], 115200);  
  myPort.bufferUntil('\n');
  while (myPort.available() > 0) {
    String data = myPort.readStringUntil('\n');
    if (data != null && data.trim().equals("d")) {
      break;  // Exit the loop when 'd' is received
    }
  }
  delay(2000);  // Wait for 2 seconds
  myPort.write("s\r\n");  // Send 's' with newline and carriage return
  textSize(16); // Set text size for RGB value display
  textAlign(CENTER, CENTER); // Align text in the center
}

void draw() {
  background(255); // White background
  fill(r, g, b); // Set fill color using RGB values from Arduino
  rect(100, 100, 200, 200); // Draw a square in the middle of the screen
  
  // Display RGB values below the square
  fill(0); // Set text color to black
  String rgbText = nf(r, 3) + "," + nf(g, 3) + "," + nf(b, 3); // Format RGB as RRR,GGG,BBB
  text(rgbText, width / 2, 350); // Display text below the square
}

void serialEvent(Serial myPort) {
  serialInput = myPort.readStringUntil('\n');
  serialInput = trim(serialInput); // Remove any extra whitespace or line breaks
  
  // Parse the RGB values from the string in the format RRR,GGG,BBB
  String[] rgbValues = split(serialInput, ',');
  if (rgbValues.length == 5) {
    x = int(rgbValues[0]);
    y = int(rgbValues[1]);
    r = int(rgbValues[2]);
    g = int(rgbValues[3]);
    b = int(rgbValues[4]);
  }
}
