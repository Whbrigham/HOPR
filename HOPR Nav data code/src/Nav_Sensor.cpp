/*
 * Project HOPR 
 * Description: Using the Adafruit_GPS Module, BME280, and MPU6050 and Libraries
 * Author: Brigham
 * Date: 4-16-2025
 */
#define _USE_MATH_DEFINES
#include "Particle.h"
#include "math.h"
#include "Adafruit_BME280.h"
#include <Adafruit_GPS.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "IoTClassroom_CNM.h"
#include <MQTT.h>

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

//BME280 address  (Temp, Humidity & Baro Pressure)
Adafruit_BME280 bme; 
int hexaddress= (0x77);

// MPU6050 I2C address
const int MPU_ADDR = (0x68);

//GPS address, not needed b/c directly addressed
//(0x10)

// Accelerometer scaling for ±2g (16384 LSB per g)
const float ACCEL_SCALE = 16384.0;

//Pi=(used math.h, commented this out)
//constexpr double PI = 3.14159265358979323846;

// Accelerometer data variables
byte accel_x_h, accel_x_l; // bytes for x-axis
int16_t accel_x; // Variable to store the x-acceleration
byte accel_y_h, accel_y_l; // bytes for y-axis
int16_t accel_y; // Variable to store the y-acceleration
byte accel_z_h, accel_z_l; // bytes for z-axis
int16_t accel_z; // Variable to store the z-acceleration

// Define Constants

char status;
const char degree =248; //ASCII code for degrees page 275 of lesson

const int OLED_RESET=-1;
Adafruit_SSD1306 display(OLED_RESET);

Adafruit_GPS GPS(&Wire);

// Time tracking
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 5000;

const int TIMEZONE = -6;
const unsigned int UPDATE = 5000;



// Declare Variables 
float lat, lon, alt;
int sat;
unsigned int lastGPS;
int getRotationFromAccel(float accel_x_g, float accel_y_g, float accel_z_g);

void getGPS(float *latitude, float *longitude, float *altitude, int *satellites);
SYSTEM_MODE(SEMI_AUTOMATIC);

byte server[] = { 192, 168, 12, 197 }; //  Mosquitto broker's IP
MQTT client(server, 1883, 0); // 0 = no callback function (for now)

//Setup
void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,5000);

  // Initialize BME280 sensor
  status = bme.begin();
    if (status == false) {
  Serial.printf("BME280 at address 0x%02X failed", hexaddress);
 }

  //Initialize GPS
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPS.println(PMTK_Q_RELEASE);

  // Initialize I2C and wake up the MPU6050 sensor
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power Management register
  Wire.write(0x00); // Wake MPU-6050
  Wire.endTransmission(true);

  //Initialize OLED 0x3C (if using it)
 display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

 TCPClient testClient;
 if (testClient.connect(server, 1883)) {
   Serial.println("✅ Photon can reach MQTT broker on port 1883");
   testClient.stop();
 } else {
   Serial.println("❌ Photon cannot reach MQTT broker (port blocked or broker issue)");
 }
 

   if (client.connect(System.deviceID())) {
    Serial.println("MQTT connected!");
  } else {
    Serial.println("MQTT connection failed.");
  }
}

void loop() {
  // Begin transmission...............................
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Register address...............
  Wire.endTransmission(false);
  // End transmission.................................
  // Get data from GSP unit (best if you do this continuously)

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
  
    // Update the display with time, date, and accelerometer data (if using OLED)
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

      //-----------------------------------------------------------------------------//
  
      // YAW REQUIRES A MAG AND CANNOT BE CALCULATED FROM ACCELEROMETER ALONE //

      ///insert compass here//
      ///compass 0x13//

      ///equation for Z//

      ///heading = atan2(y, x) * 180 / M_PI;//

      //------------------------------------------------------------------------------//
  
      Serial.printf("X-axis: %.1f g\nY-axis: %.1f g\nZ-axis: %.1f g\n", accel_x_g, accel_y_g, accel_z_g);
  
      //accel_x_g, accel_y_g, accel_z_g are already calculated
      float roll = atan2(-accel_y_g, accel_z_g) * (180.0 / M_PI);  // Convert roll to degrees
      float pitch = atan2(-accel_x_g, sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * (180.0 / M_PI);  // Convert pitch to degrees
  
      // Print the pitch (Y) and roll (X)
      Serial.printf("Pitch: %.1f degrees\n", pitch);
      Serial.printf("Roll: %.1f degrees\n", roll);
      
      if (bme.begin()) {
        int tempC = bme.readTemperature();
        float pressPA = bme.readPressure();
        float humidity = bme.readHumidity();
        float baro_Hg = pressPA / 100.0 * 0.02953;
        float F = tempC * 9.0 / 5.0 + 32;
        // Display data on Serial Monitor
        Serial.printf("\n--------------------------------------------------------\n");
        Serial.printf("[Temp: %0.f F]\t[Pressure: %.2f Hg]\n  [Humidity: %0.0f %%]\n",F,baro_Hg,humidity);
        Serial.println("");Serial.println("");Serial.println("");Serial.println("");
        Serial.printf("\n--------------------------------------------------------\n");
        // Display data on OLED (if using it)
        display.printf("%0.fF\t%0.0f%%\n",F,humidity);
        display.printf("%.2f\n",baro_Hg);
        display.setRotation(2);
        display.display();
        display.setTextSize(6);
        display.setTextColor(WHITE);
        display.setCursor(1,0);
        display.clearDisplay();
      }
  
  
      lastPrintTime = millis(); // Update time for next print
    }
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }   
  }

  if (millis() - lastGPS > UPDATE) {
    lastGPS = millis(); // reset the timer
    getGPS(&lat,&lon,&alt,&sat);
    float feet = alt * 3.28084;
    Serial.printf("\n=================================================================\n");
    Serial.printf("Lat: %0.6f, Lon: %0.6f, Alt: %0.1f, Satellites: %i\n",lat, lon, feet, sat);
    Serial.printf("=================================================================\n\n");

    if (client.isConnected()) {
      String payload = String::format("{\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"accel_x\":%.2f,\"accel_y\":%.2f,\"accel_z\":%.2f}", 
                                      lat, lon, alt, accel_x / ACCEL_SCALE, accel_y / ACCEL_SCALE, accel_z / ACCEL_SCALE);
      client.publish("hopr/gps", payload);
      Serial.println("Published to MQTT: " + payload);
    } else {
      Serial.println("MQTT not connected.");
    }
        // Build and send JSON to Node-RED via Particle Webhook (if you want a webhook)
       // if (GPS.fix) {
         // char payload[256];
         // snprintf(payload, sizeof(payload),
        //    "{\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.2f,\"accel_x\":%.2f,\"accel_y\":%.2f,\"accel_z\":%.2f}",
           // lat, lon, alt,
          //  accel_x / ACCEL_SCALE,
            //accel_y / ACCEL_SCALE,
           // accel_z / ACCEL_SCALE
          //);
         // Particle.publish("hopr_data", payload, PRIVATE);
        //}
    
  }
}

void getGPS(float *latitude, float *longitude, float *altitude, int *satellites){
  int theHour;

  theHour = GPS.hour + TIMEZONE;
  if(theHour < 0) {
    theHour = theHour + 24;
  }
  
  Serial.printf("Time: %02i:%02i:%02i:%03i\n",theHour, GPS.minute, GPS.seconds, GPS.milliseconds);
  Serial.printf("Dates: %02i-%02i-20%02i\n", GPS.month, GPS.day, GPS.year);
  Serial.printf("Fix: %i, Quality: %i",(int)GPS.fix,(int)GPS.fixquality);
    if (GPS.fix) {
      *latitude = GPS.latitudeDegrees;
      *longitude = GPS.longitudeDegrees; 
      *altitude = GPS.altitude;
      *satellites = (int)GPS.satellites;
      // Serial.printf("Lat: %0.6f, Lon: %0.6f, Alt: %0.6f\n",*latitude, *longitude, *altitude);
      // Serial.printf("Speed (m/s): %0.2f\n",GPS.speed/1.944);
      // Serial.printf("Angle: %0.2f\n",GPS.angle);
      // Serial.printf("Satellites: %i\n",*satellites);
    }
}
  // Function to determine rotation angle based on accelerometer data (if using oled)
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
