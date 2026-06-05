#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Pins
const int EMG_PIN = A0;
const int LED_GOOD = 8;
const int LED_BAD = 9;
const int BUZZER_PIN = 10;

// EMG variables
float emgRaw = 0;
float emgCentered = 0;
float emgRectified = 0;
float emgEnvelope = 0;
float emgBaseline = 0;

// IMU variables
float pitchAngle = 0;
float pitchBaseline = 0;

// Thresholds
float emgActivationThreshold = 60;
float postureThreshold = 12.0;

// Smoothing
const float emgAlpha = 0.08;

// Calibration
const int calibrationSamples = 300;

// Timing
unsigned long lastPrintTime = 0;
const int printInterval = 200;

void setup() {
  Serial.begin(115200);

  pinMode(LED_GOOD, OUTPUT);
  pinMode(LED_BAD, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_GOOD, LOW);
  digitalWrite(LED_BAD, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  if (!mpu.begin()) {
    Serial.println("Could not find MPU6050. Check wiring.");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("System starting...");
  delay(1000);

  calibrateSensors();

  Serial.println("Calibration complete.");
  Serial.println("Starting posture detection...");
}

void loop() {
  readEMG();
  readIMU();
  classifyPosture();

  if (millis() - lastPrintTime > printInterval) {
    printData();
    lastPrintTime = millis();
  }

  delay(10);
}

void calibrateSensors() {
  Serial.println("Calibrating...");
  Serial.println("Sit in GOOD posture and keep muscles relaxed.");

  float emgSum = 0;
  float pitchSum = 0;

  for (int i = 0; i < calibrationSamples; i++) {
    int rawEMG = analogRead(EMG_PIN);
    emgSum += rawEMG;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float pitch = atan2(a.acceleration.x,
                        sqrt(a.acceleration.y * a.acceleration.y +
                             a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

    pitchSum += pitch;

    delay(10);
  }

  emgBaseline = emgSum / calibrationSamples;
  pitchBaseline = pitchSum / calibrationSamples;

  Serial.print("EMG baseline: ");
  Serial.println(emgBaseline);

  Serial.print("Pitch baseline: ");
  Serial.println(pitchBaseline);
}

void readEMG() {
  emgRaw = analogRead(EMG_PIN);
  emgCentered = emgRaw - emgBaseline;
  emgRectified = abs(emgCentered);
  emgEnvelope = (emgAlpha * emgRectified) + ((1 - emgAlpha) * emgEnvelope);
}

void readIMU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  pitchAngle = atan2(a.acceleration.x,
                     sqrt(a.acceleration.y * a.acceleration.y +
                          a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
}

void classifyPosture() {
  float postureDeviation = abs(pitchAngle - pitchBaseline);

  bool postureBad = postureDeviation > postureThreshold;
  bool muscleActivated = emgEnvelope > emgActivationThreshold;

  if (!postureBad && muscleActivated) {
    digitalWrite(LED_GOOD, HIGH);
    digitalWrite(LED_BAD, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }
  else if (!postureBad && !muscleActivated) {
    digitalWrite(LED_GOOD, HIGH);
    digitalWrite(LED_BAD, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }
  else if (postureBad && muscleActivated) {
    digitalWrite(LED_GOOD, LOW);
    digitalWrite(LED_BAD, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
  }
  else {
    digitalWrite(LED_GOOD, LOW);
    digitalWrite(LED_BAD, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
  }
}

void printData() {
  float postureDeviation = abs(pitchAngle - pitchBaseline);

  Serial.print("Raw EMG: ");
  Serial.print(emgRaw);

  Serial.print(" | EMG Envelope: ");
  Serial.print(emgEnvelope);

  Serial.print(" | Pitch: ");
  Serial.print(pitchAngle);

  Serial.print(" | Deviation: ");
  Serial.print(postureDeviation);

  Serial.print(" | EMG Active: ");
  if (emgEnvelope > emgActivationThreshold) {
    Serial.print("YES");
  } else {
    Serial.print("NO");
  }

  Serial.print(" | Posture: ");
  if (postureDeviation <= postureThreshold) {
    Serial.println("GOOD");
  } else {
    Serial.println("BAD");
  }
}
