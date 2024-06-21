// include libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// create objects
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// define pins
int buzzerPin = 7;
int ledPin = 4;

// define threshold and debounce time
int threshold = 12;
int debounceTime = 1000;

// define variables for debounce
bool earthquakeDetected = false;
unsigned long earthquakeTime = 0;

void setup() {
  // initialize serial communication
  Serial.begin(9600);
  
  // initialize pins
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  // check if the accelerometer is connected
  if(!accel.begin())
  {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    while(1);
  }
}

void loop() {
  // read accelerometer data
  sensors_event_t event; 
  accel.getEvent(&event);

  // calculate magnitude of acceleration vector
  float magnitude = sqrt(event.acceleration.x * event.acceleration.x +
                         event.acceleration.y * event.acceleration.y +
                         event.acceleration.z * event.acceleration.z);

  // print magnitude to serial monitor
  Serial.print("Magnitude: ");
  Serial.println(magnitude);

  // check if magnitude is above threshold
  if (magnitude > threshold && !earthquakeDetected) {
    // trigger the buzzer and LED bulb
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(ledPin, HIGH);
    Serial.println("Earthquake detected!");
    
    // set debounce flag and time
    earthquakeDetected = true;
    earthquakeTime = millis();
  }
  
  // check if debounce time has elapsed
  if (earthquakeDetected && millis() - earthquakeTime > debounceTime) {
    // turn off the buzzer and LED bulb
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
    Serial.println("Earthquake over.");
    
    // clear debounce flag
    earthquakeDetected = false;
  }

  // wait for a short time before reading accelerometer data again
  delay(100);
}
