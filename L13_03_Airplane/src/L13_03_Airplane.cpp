/* 
 * Project L13_02_MPU6050
 * Author: Brigham
 * Date: 3/25/2025
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "math.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "IoTClassroom_CNM.h"

// Constants for OLED display
const int OLED_RESET = -1;
Adafruit_SSD1306 display(OLED_RESET);

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// MPU6050 I2C address
const int MPU_ADDR = (0x68);

// Accelerometer scaling for ±2g (16384 LSB per g)
const float ACCEL_SCALE = 16384.0;

//Pi=
const double PI = 3.14159265358979323846;

// Accelerometer data variables
byte accel_x_h, accel_x_l; // bytes for x-axis
int16_t accel_x; // Variable to store the x-acceleration
byte accel_y_h, accel_y_l; // bytes for y-axis
int16_t accel_y; // Variable to store the y-acceleration
byte accel_z_h, accel_z_l; // bytes for z-axis
int16_t accel_z; // Variable to store the z-acceleration

// Time tracking
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000;

int getRotationFromAccel(float accel_x_g, float accel_y_g, float accel_z_g);

// Setup function
void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 10000);

  // Initialize OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display(); // Show splashscreen
  delay(2000);
  display.clearDisplay(); // Clears the screen and buffer
  
  // Initialize I2C and wake up the MPU6050 sensor
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power Management register
  Wire.write(0x00); // Wake MPU-6050
  Wire.endTransmission(true);
}

// Main loop function
void loop() {
  // Begin transmission...............................
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Register address...............
  Wire.endTransmission(false);
  // End transmission.................................

  // Request 6 bytes (2 bytes per axis)
  Wire.requestFrom(MPU_ADDR, 6, true);

  // Data for X, Y, and Z
  accel_x_h = Wire.read(); 
  accel_x_l = Wire.read(); 
  accel_x = accel_x_h << 8 | accel_x_l;

  accel_y_h = Wire.read(); 
  accel_y_l = Wire.read(); 
  accel_y = accel_y_h << 8 | accel_y_l;

  accel_z_h = Wire.read();
  accel_z_l = Wire.read(); 
  accel_z = accel_z_h << 8 | accel_z_l;

  // Convert raw data to G's
  float accel_x_g = accel_x / ACCEL_SCALE;
  float accel_y_g = accel_y / ACCEL_SCALE;
  float accel_z_g = accel_z / ACCEL_SCALE;

  // Update the display with time, date, and accelerometer data
  if (millis() - lastPrintTime >= printInterval) {
    // Display Time and Date (using Particle Cloud time)
    Time.zone(-5); // Adjust for your timezone
    String timeString = String(Time.hour()) + ":" + String(Time.minute()) + ":" + String(Time.second());
    String dateString = String(Time.month()) + "/" + String(Time.day()) + "/" + String(Time.year());

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Time: " + timeString);
    display.println("Date: " + dateString);

    // Rotate the screen based on accelerometer data
    int rotation = getRotationFromAccel(accel_x_g, accel_y_g, accel_z_g);
    display.setRotation(rotation);

    display.display(); // Update the display

    // YAW REQUIRES A MAG AND CANNOT BE CALCULATED FROM ACCELEROMETER ALONE //

    Serial.printf("X-axis: %.1f g\nY-axis: %.1f g\nZ-axis: %.1f g\n", accel_x_g, accel_y_g, accel_z_g);

    //accel_x_g, accel_y_g, accel_z_g are already calculated
    float pitch = atan2(accel_y_g, accel_z_g) * (180.0 / PI);  // Convert roll to degrees
    float roll = atan2(-accel_x_g, sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * (180.0 / PI);  // Convert pitch to degrees

    // Print the pitch (Y) and roll (X)
    Serial.printf("Pitch: %.1f degrees\n", pitch);
    Serial.printf("Roll: %.1f degrees\n", roll);
    


    lastPrintTime = millis(); // Update time for next print
  }
}

// Function to determine rotation angle based on accelerometer data
int getRotationFromAccel(float accel_x_g, float accel_y_g, float accel_z_g) {
  int rotation = 0;

  // If accelerometer on X is more positive, rotate to 0 degrees
  if (fabs(accel_x_g) > fabs(accel_y_g) && fabs(accel_x_g) > fabs(accel_z_g)) {
    rotation = (accel_x_g > 0) ? 0 : 2; // 0 for right-side up, 2 for upside down
  }
  // If accelerometer on Y is more positive, rotate to 90 or 270 degrees (flipped logic)
  else if (fabs(accel_y_g) > fabs(accel_x_g) && fabs(accel_y_g) > fabs(accel_z_g)) {
    rotation = (accel_y_g < 0) ? 1 : 3; // Flip the comparison for Y-axis
  }

  return rotation;
}
