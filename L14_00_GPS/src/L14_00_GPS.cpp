/*
 * Project L14_00_GPS
 * Description: Using the Adafruit_GPS Module and Library
 * Author: Brigham
 * Date: 4-14-2025
 */

#include "Particle.h"
#include <Adafruit_GPS.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

const int OLED_RESET=-1;
Adafruit_SSD1306 display(OLED_RESET);


const int OLED_RESET=-1;
Adafruit_SSD1306 display(OLED_RESET);

Adafruit_GPS GPS(&Wire);

// Define Constants
const int TIMEZONE = -6;
const unsigned int UPDATE = 30000;

// Declare Variables 
float lat, lon, alt;
int sat;
unsigned int lastGPS;

void getGPS(float *latitude, float *longitude, float *altitude, int *satellites);
SYSTEM_MODE(SEMI_AUTOMATIC);

void setup() {

  Serial.begin(9600);
  waitFor(Serial.isConnected,5000);

  //Initialize GPS
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPS.println(PMTK_Q_RELEASE);
}

void loop() {

  // Get data from GSP unit (best if you do this continuously)
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