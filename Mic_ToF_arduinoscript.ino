#include <ros.h>
#include <std_msgs/String.h>
#include <PDM.h>
#include <Wire.h>
#include <VL53L0X.h>

ros::NodeHandle nh;
std_msgs::String sound_msg;
ros::Publisher sound_publisher("alarm_trigger", &sound_msg);

VL53L0X tofSensor;


short sampleBuffer[256];
volatile int samplesRead = 0;
/*Before the setup() there are two types of variables initialized. 
One is a short variable and the other is a volatile int variable. 
We use the short type variable to store 16-bit data-types as the sampleBuffer[256]. 
The other, volatile, is a keyword known as a variable qualifier. 
It is usually used before the datatype of a variable, 
in order to modify the way in which the compiler and subsequent program treat the variable. 
In this case, it directs the compiler to load the variable samplesRead from RAM and not from a storage register.*/


void setup() {
  nh.initNode();
  nh.advertise(sound_publisher);

  PDM.onReceive(onPDMdata);

//Initialize sensor, one channel at 16kHz
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
    //sound_publisher.publish(&sound_msg);
    nh.spinOnce();
    samplesRead = 0;
  }
    uint16_t distanceToF = tofSensor.readRangeContinuousMillimeters();
  if (tofSensor.timeoutOccurred()) {
    Serial.println("ToF Sensor Timeout!");
  } else {
    Serial.print("ToF Sensor: Distance: ");
    Serial.print(distanceToF);
    Serial.println(" mm");
  }

  delay(1000);
}


void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();
  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);
  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}