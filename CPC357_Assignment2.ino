#include <LeverageConnect.h> // Leverege Connect library
#include <WiFi.h>
#include <MQTT.h>
#include <Arduino.h>

// Sensor and actuator pin definitions
const int RAIN_SENSOR_PIN = 2;
const int HUMIDITY_SENSOR_PIN = 3;
const int SERVO_MOTOR_PIN = 9;
const int DRYER_MOTOR_PIN = 10;

// Global variables
bool clothesHanging = false; // Indicates clothes hanging status
bool dryerOn = false; // Indicates dryer motor/fan status

// Sensor reading functions
int readRainSensor() {
  return digitalRead(RAIN_SENSOR_PIN);
}

int readHumiditySensor() {
  // Replace with your specific humidity sensor read function
  return analogRead(HUMIDITY_SENSOR_PIN);
}

// Actuator control functions
void hangClothes(bool hang) {
  if (hang) {
    // Rotate servo to desired angle to hang clothes
    Servo myservo;
    myservo.attach(SERVO_MOTOR_PIN);
    myservo.write(90);  // Adjust angle based on your servo and hanging mechanism
    clothesHanging = true;
  } else {
    // Rotate servo to desired angle to retract clothes
    Servo myservo;
    myservo.attach(SERVO_MOTOR_PIN);
    myservo.write(0); // Adjust angle based on your servo and hanging mechanism
    clothesHanging = false;
  }
}

void turnOnDryer(bool on) {
  if (on) {
    digitalWrite(DRYER_MOTOR_PIN, HIGH);
    dryerOn = true;
  } else {
    digitalWrite(DRYER_MOTOR_PIN, LOW);
    dryerOn = false;
  }
}

void setup() {
  // Initialize Leverege Connect and MQTT client
  LeverageConnect client(username, password, projectId);
  MQTTClient mqttClient(512);

 // Connect to Wi-Fi and Leverege Connect
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  client.connect();

  // Connect to MQTT broker
  mqttClient.begin(broker_url, 8883, net);

  Serial.begin(9600);
  pinMode(RAIN_SENSOR_PIN, INPUT);
  pinMode(HUMIDITY_SENSOR_PIN, INPUT);
  pinMode(SERVO_MOTOR_PIN, OUTPUT);
  pinMode(DRYER_MOTOR_PIN, OUTPUT);
}

void loop() {
  // Read sensor values
  int rain = readRainSensor();
  int humidity = readHumiditySensor();

  // Clothes hanging control
  if (clothesHanging) {
    // Retract clothes if raining
    if (rain) {
      Serial.println("Rain detected! Retracting clothes.");
      hangClothes(false);
    }
  } else {
    // Hang clothes if high humidity and no rain
    if (!rain && humidity > 70) {
      Serial.println("High humidity and no rain. Hanging clothes.");
      hangClothes(true);
    }
  }

  // Dryer control
  if (clothesHanging && !dryerOn && humidity > 80) {
    Serial.println("High humidity with clothes hanging. Turning on dryer.");
    turnOnDryer(true);
  } else if (!clothesHanging || humidity < 60) {
    Serial.println("Low humidity or clothes not hanging. Turning off dryer.");
    turnOnDryer(false);
  }

  client.loop();
  mqttClient.loop();

  // Publish sensor and actuator data
  mqttClient.publish("sensor/rain", String(rain));
  mqttClient.publish("sensor/humidity", String(humidity));
  mqttClient.publish("actuator/clothes", String(clothesHanging));
  mqttClient.publish("actuator/dryer", String(dryerOn));

  // Delay for sensor readings and stability
  delay(1000);
}