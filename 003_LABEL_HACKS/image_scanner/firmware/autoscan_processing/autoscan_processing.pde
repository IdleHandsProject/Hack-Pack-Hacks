import processing.serial.*;

Serial myPort; // The serial port
String serialInput = "";
int imagex = 100; // Define image width in pixels
int imagey = 100; // Define image height in pixels
int[][][] imagePixels = new int[imagex][imagey][3];  // 3D array for RGB values
int xp = 0, yp = 0, r = 0, g = 0, b = 0; // Initialize RGB values

void setup() {
  size(500, 500); // Start with a default size
  //surface.setSize(imagex, imagey); // Dynamically set the window size after it's created
  //imagePixels = new color[imagex][imagey]; // Initialize array to hold pixel data

  // Initialize each pixel to white (255, 255, 255 for RGB)
  for (int x = 0; x < imagex; x++) {
    for (int y = 0; y < imagey; y++) {
      imagePixels[x][y][0] = 255;  // Red component
      imagePixels[x][y][1] = 255;  // Green component
      imagePixels[x][y][2] = 255;  // Blue component
    }
  }

  for (int x = 0; x < imagex; x++) {
    for (int y = 0; y < imagey; y++) {
      fill(imagePixels[x][y][0], imagePixels[x][y][1], imagePixels[x][y][2]);

      rect(x*5, y*5, 5, 5); // Draw individual pixels
    }
  }

  // Initialize serial connection
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
  // Set background to black initially
  background(0);
  while (myPort.available() > 0) {
    String data = myPort.readStringUntil('\n');
    if (data != null && data.trim().equals("d")) {
      break;  // Exit the loop when 'd' is received
    }
  }
  delay(2000);  // Wait for 2 seconds
  myPort.write("s\r\n");  // Send 's' with newline and carriage return
}

void draw() {
  // Draw the scanned pixels as they are read
  for (int x = 0; x < imagex; x++) {
    for (int y = 0; y < imagey; y++) {
      fill(imagePixels[x][y][0], imagePixels[x][y][1], imagePixels[x][y][2]);
      //fill(255,255,255);
      rect(x*5, y*5, 5, 5); // Draw individual pixels
    }
  }
}

void serialEvent(Serial myPort) {
  serialInput = myPort.readStringUntil('\n');
  serialInput = trim(serialInput); // Remove any extra whitespace or line breaks
  println("Received: " + serialInput); // Print received data for debugging

  // Parse the received data in the format "X, Y, RRR, GGG, BBB"
  String[] rgbValues = split(serialInput, ',');
  if (rgbValues.length == 5) {
    xp = int(rgbValues[0])-1;
    yp = int(rgbValues[1]);
    r = int(rgbValues[2]);
    g = int(rgbValues[3]);
    b = int(rgbValues[4]);

    // Store the color at the specified x, y position
    // To set colors:
    imagePixels[xp][yp][0] = r;
    imagePixels[xp][yp][1] = g;
    imagePixels[xp][yp][2] = b;

    //imagePixels[xp][yp] = color(r, g, b);

    // Send 's' to continue scanning
    //myPort.write('s');
    //println("Sent 's' to request next scan.");
  }
}
