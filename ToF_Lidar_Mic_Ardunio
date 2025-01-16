#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <PDM.h>
#include <Wire.h>
#include <VL53L0X.h>

ros::NodeHandle nh;

std_msgs::String trigger_msg;
ros::Publisher trigger_publisher("alarm_trigger", &trigger_msg);

std_msgs::String lidar_msg;
ros::Publisher lidar_publisher("user_position", &lidar_msg);

// Function prototype for lidarCallback
void lidarCallback(const sensor_msgs::LaserScan& scan);

ros::Subscriber<sensor_msgs::LaserScan> lidar_subscriber("/scan", lidarCallback);

VL53L0X tofSensor;

short sampleBuffer[256];
volatile int samplesRead = 0;
int tofThreshold = 50; // ToF threshold in mm

// LIDAR variables
bool lidarMovementDetected = false;
float lidarThreshold = 0.5; // Minimum movement threshold in meters
float maxLidarDistance = 10.0; // Maximum distance to consider for movement detection in meters
float previousScan[360]; // Store previous LIDAR scan data
bool firstScan = true;

// Callback definition
void lidarCallback(const sensor_msgs::LaserScan& scan) {
  lidarMovementDetected = false;

  // Analyze the LIDAR scan data
  for (int i = 0; i < scan.ranges_length; i++) {  // Use ranges_length
    if (scan.ranges[i] < scan.range_max && scan.ranges[i] > scan.range_min && scan.ranges[i] <= maxLidarDistance) {
      if (!firstScan && abs(scan.ranges[i] - previousScan[i]) > lidarThreshold) {
        lidarMovementDetected = true;

        char buffer[50];
        sprintf(buffer, "Movement detected in %.2f m", scan.ranges[i]);
        lidar_msg.data = buffer;
        lidar_publisher.publish(&lidar_msg);

        trigger_msg.data = "Alarm Trigger: LIDAR movement detected";
        trigger_publisher.publish(&trigger_msg);

        break;
      }
      previousScan[i] = scan.ranges[i];
    }
  }

  firstScan = false;

  nh.spinOnce();
}

// Setup function
void setup() {
  nh.initNode();
  nh.advertise(trigger_publisher);
  nh.advertise(lidar_publisher);
  nh.subscribe(lidar_subscriber);

  PDM.onReceive(onPDMdata);

  // Initialize PDM microphone
  if (!PDM.begin(1, 16000)) {
    while (1);
  }

  // Initialize ToF sensor
  Wire.begin();
  tofSensor.setTimeout(500); // Set a timeout for ToF sensor
  if (!tofSensor.init()) {
    while (1); // Stop the program if ToF sensor initialization fails
  }

  // Start continuous measurements for ToF sensor
  tofSensor.startContinuous();
}

// Loop function
void loop() {
  // Handle sound data
  if (samplesRead) {
    int soundLevel = 0;
    for (int i = 0; i < samplesRead; i++) {
      soundLevel += abs(sampleBuffer[i]);
    }
    soundLevel /= samplesRead;

    if (soundLevel > 500) {
      trigger_msg.data = "Alarm Trigger: Sound detected";
      trigger_publisher.publish(&trigger_msg);
    } else if (soundLevel > 250) {
      trigger_msg.data = "Moderate sound detected";
    } else {
      trigger_msg.data = "Quiet sound detected";
    }

    nh.spinOnce();
    samplesRead = 0;
  }

  // Handle ToF sensor data
  uint16_t distanceToF = tofSensor.readRangeContinuousMillimeters();
  if (!tofSensor.timeoutOccurred()) {
    // Check for movement (e.g., threshold change in distance)
    static uint16_t lastDistance = 0;
    if (abs(distanceToF - lastDistance) > tofThreshold) { // Movement threshold of 50 mm
      trigger_msg.data = "Alarm Trigger: Movement detected by ToF sensor";
      trigger_publisher.publish(&trigger_msg);
    }
    lastDistance = distanceToF;
  }

  nh.spinOnce();
  delay(1000);
}

// PDM data callback
void onPDMdata() {
  // Query the number of bytes available
  int bytesAvailable = PDM.available();
  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);
  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
