#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <driver/adc.h>
#include <vector>
#include <utility>
#include <string>
#include <algorithm>
#include "sensor_node.h"
#include "esp_system.h"

#define MAX_FAILED_ATTEMPTS 10
#define CALIBRATE 0
#define NO_OF_SAMPLES 65
#define SENSOR_RING 2
#define LOW_PASS 0.45

#define GPIO32 ADC1_CHANNEL_4
#define GPIO33 ADC1_CHANNEL_5
#define GPIO34 ADC1_CHANNEL_6
#define GPIO35 ADC1_CHANNEL_7

typedef struct {
  std::string frame;
  adc1_channel_t pin;
  double A;
  double B;
  double K;
} sensor_t;

// {"sensor_4_back", GPIO35, 0.0000315383707, 0.002978491566, 1.5}, best sensor so far

std::vector<sensor_t> sensors = {
#if SENSOR_RING == 2
  {"sensor_2_front", GPIO34, 0.00003095630509, 0.003538019272, 1.6},
  {"sensor_2_left", GPIO33, 0.00003276791572, 0.00284869739, 0.9},
  {"sensor_2_back", GPIO32, 0.00003344496413, 0.002923084849, 0.9},
  {"sensor_2_right", GPIO35, 0.00003138671844, 0.004107259821, 1.5},
#elif SENSOR_RING == 4
  {"sensor_4_front", GPIO33, 0.00003043101771, 0.002900329139, 1.5},
  {"sensor_4_left", GPIO32, 0.00003110156825, 0.002753638173, 1.5},
  {"sensor_4_back", GPIO35, 0.0000315383707, 0.002978491566, 1.5},
  {"sensor_4_right", GPIO34, 0.00003096265037, 0.002803218224, 1.5},
#elif SENSOR_RING == 6
  {"sensor_6_front_right", GPIO34, 0.00002897696896, 0.003815309638, 1.5},
  {"sensor_6_front_left", GPIO35, 0.0000282914602, 0.002773656281, 2.0},
#endif
};

ros::NodeHandle nh;

sensor_msgs::Range range_msg;
ros::Publisher range_publisher("sensor_data", &range_msg);
int failed_attempts = 0;

void rosserial_setup()
{
  // Initialize ROS
  nh.initNode();
  nh.advertise(range_publisher);

  // Initialize static message data
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.1;
  range_msg.max_range = 0.8;

  // Initialize ADC
  adc1_config_width(ADC_WIDTH_BIT_9);
  for(auto i = sensors.begin(); i < sensors.end(); i++) {
    adc1_config_channel_atten(i->pin,ADC_ATTEN_DB_11);
  }
}

void rosserial_publish()
{
  if (nh.connected()) {
    failed_attempts = 0;
  } else if (++failed_attempts > MAX_FAILED_ATTEMPTS) {
    esp_restart();
  }

  for(auto i = sensors.begin(); i < sensors.end(); i++) {
    // Fill dynamic message data
    // Multisampling with median
    int raw_samples[NO_OF_SAMPLES];
    for (int j = 0; j < NO_OF_SAMPLES; j++) {
      raw_samples[j] = adc1_get_raw(i->pin);
    }
    std::sort(raw_samples, raw_samples+NO_OF_SAMPLES);
    auto adc_value = raw_samples[NO_OF_SAMPLES/2];
    #if CALIBRATE == 1
    // Use raw adc value on callibration process
    range_msg.range = adc_value;
    #else
    range_msg.range = (1 / (i->A * adc_value + i->B) - i->K) / 100;
    // Low Pass Filter trying to reduce noise in sensor values
    // Sensor tends to get inaccurate on higher distances
    if (range_msg.range > LOW_PASS) {
      continue;
    }
    #endif

    range_msg.header.frame_id = i->frame.c_str();
    range_msg.header.stamp = nh.now();
    
    // Publish message
    range_publisher.publish(&range_msg);
  }

  nh.spinOnce();
}
