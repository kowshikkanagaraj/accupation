#define TRIGGER_PIN 2 // Pin connected to the trigger pin of the ultrasonic sensor
#define ECHO_PIN 3    // Pin connected to the echo pin of the ultrasonic sensor
#define BUZZER_PIN 4  // Pin connected to the buzzer

#define MAX_DISTANCE 200 // Maximum detection distance in centimeters
#define ALARM_DISTANCE 50 // Distance threshold for triggering the alarm in centimeters

#define MAX_HEART_RATE 100 // Maximum heart rate threshold
#define MIN_SPO2 90 // Minimum SpO2 threshold
#define MAX_TEMPERATURE 37.5 // Maximum temperature threshold in Celsius
#define MAX_GAS_RESISTANCE 100 // Maximum gas resistance threshold

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

// Define the analog pin where LM35 is connected
const int lm35Pin = A0;

// Define the analog pin where MQ-7 sensor is connected
const int mq7Pin = A1;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0); // wait until user presses a key
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void loop() {
  // Read LM35 temperature
  int lm35Value = analogRead(lm35Pin);
  float voltageLM35 = lm35Value * (5000.0 / 1024.0);
  float temperatureC = voltageLM35 / 10.0;

  // Read MQ-7 gas sensor
  int mq7Value = analogRead(mq7Pin);
  float voltageMQ7 = mq7Value * (5000.0 / 1024.0);
  float gasResistance = (5.0 - voltageMQ7) / voltageMQ7 * 10.0;

  // Output LM35 temperature and MQ-7 gas resistance
  Serial.print("LM35 Temperature: ");
  Serial.print(temperatureC);
  Serial.println("  C");
  Serial.print("MQ-7 Gas Resistance: ");
  Serial.println(gasResistance);

  // Heart rate and SpO2 monitoring
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  // Read the first 100 samples and determine the signal range
  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false) // do we have new data?
      particleSensor.check(); // Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // We're finished with this sample so move to next sample
  }

  // Calculate heart rate and SpO2 after the first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Output the heart rate and SpO2 values
  Serial.print(F("Heart Rate: "));
  Serial.print(heartRate);
  Serial.print(F(", SpO2: "));
  Serial.println(spo2);

  // Read ultrasonic sensor
  long duration, distance;
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  // Print distance to serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Check if the distance is within the alarm threshold
  if (distance <= ALARM_DISTANCE && distance > 0) {
    // If an object is detected within the alarm distance, trigger the buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("ALERT: Object detected! Take necessary action.");
  } else {
    // If no object is detected or it's outside the alarm distance, turn off the buzzer
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Check if any of the sensor readings exceed the threshold levels
  if (heartRate > MAX_HEART_RATE || spo2 < MIN_SPO2 || temperatureC > MAX_TEMPERATURE || gasResistance > MAX_GAS_RESISTANCE) {
    // If any threshold is exceeded, trigger the buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("ALERT: Sensor readings exceed threshold levels! Take necessary action.");
  } else {
    // If all sensor readings are within the threshold levels, turn off the buzzer
    digitalWrite(BUZZER_PIN, LOW);
  }
}
