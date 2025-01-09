Ich habe dieses Script was auf einem Arduino läuft:
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <PDM.h>
#include <Wire.h>
#include <VL53L0X.h>

ros::NodeHandle nh;
std_msgs::String sound_msg;
ros::Publisher sound_publisher("alarm_trigger", &sound_msg);

VL53L0X tofSensor;

short sampleBuffer[256];
volatile int samplesRead = 0;

void setup() {
  nh.initNode();
  nh.advertise(sound_publisher);

  PDM.onReceive(onPDMdata);

  // Initialize PDM microphone
  if (!PDM.begin(1, 16000)) {
    while (1);
  }

  // Initialize ToF sensor
  Wire.begin();
  tofSensor.setTimeout(500); // Set a timeout for ToF sensor
  if (!tofSensor.init()) {
    Serial.println("Failed to initialize VL53L0X (ToF)! Check wiring.");
    while (1); // Stop the program if ToF sensor initialization fails
  }

  // Start continuous measurements for ToF sensor
  tofSensor.startContinuous();
}

void loop() {
  // Handle sound data
  if (samplesRead) {
    int soundLevel = 0;
    for (int i = 0; i < samplesRead; i++) {
      soundLevel += abs(sampleBuffer[i]);
    }
    soundLevel /= samplesRead;

    if (soundLevel > 500) {
      sound_msg.data = "Loud sound detected";
      sound_publisher.publish(&sound_msg);
    } else if (soundLevel > 250) {
      sound_msg.data = "Moderate sound detected";
    } else {
      sound_msg.data = "Quiet sound detected";
    }

    nh.spinOnce();
    samplesRead = 0;
  }

  // Handle ToF sensor data
  uint16_t distanceToF = tofSensor.readRangeContinuousMillimeters();
  if (tofSensor.timeoutOccurred()) {
    Serial.println("ToF Sensor Timeout!");
  } else {
    Serial.print("ToF Sensor: Distance: ");
    Serial.print(distanceToF);
    Serial.println(" mm");

    // Check for movement (e.g., threshold change in distance)
    static uint16_t lastDistance = 0;
    if (abs(distanceToF - lastDistance) > 50) { // Movement threshold of 50 mm
      sound_msg.data = "Movement detected by ToF sensor";
      sound_publisher.publish(&sound_msg);
    }
    lastDistance = distanceToF;
  }

  nh.spinOnce();
  delay(1000);
}

void onPDMdata() {
  // Query the number of bytes available
  int bytesAvailable = PDM.available();
  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);
  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}