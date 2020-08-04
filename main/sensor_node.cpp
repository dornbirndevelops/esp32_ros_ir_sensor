#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <driver/adc.h>
#include <vector>
#include <utility>
#include <string>
#include <algorithm>
#include "sensor_node.h"

#define CALIBRATE 0
#define NO_OF_SAMPLES 65
#define RING 6

typedef struct {
  std::string frame;
  adc1_channel_t pin;
  float A;
  float B;
  float K;
} sensor_t;

/*
ADC1 channel 0 is GPIO36
ADC1 channel 1 is GPIO37
ADC1 channel 2 is GPIO38
ADC1 channel 3 is GPIO39
ADC1 channel 4 is GPIO32
ADC1 channel 5 is GPIO33
ADC1 channel 6 is GPIO34
ADC1 channel 7 is GPIO35
*/

// {"ring_4_back", ADC1_CHANNEL_7, 0.0000315383707f, 0.002978491566f, 1.5f}, best sensor so far

std::vector<sensor_t> sensors = {
#if RING == 2
  {"ring_2_front", ADC1_CHANNEL_6, 0.00003096556491f, 0.004266254642f, 2.1f},
  {"ring_2_left", ADC1_CHANNEL_5, 0.00003276791572f, 0.00284869739f, 0.9f},
  {"ring_2_back", ADC1_CHANNEL_4, 0.00003344496413f, 0.002923084849f, 0.9f},
  {"ring_2_right", ADC1_CHANNEL_7, 0.00003138671844f, 0.004107259821f, 1.5f},
#elif RING == 4
  {"ring_4_front", ADC1_CHANNEL_5, 0.00003043101771f, 0.002900329139f, 1.5f},
  {"ring_4_left", ADC1_CHANNEL_4, 0.00003110156825f, 0.002753638173f, 1.5f},
  {"ring_4_back", ADC1_CHANNEL_7, 0.0000315383707f, 0.002978491566f, 1.5f},
  {"ring_4_right", ADC1_CHANNEL_6, 0.00003096265037f, 0.002803218224f, 1.5f},
#elif RING == 6
  {"ring_6_front_right", ADC1_CHANNEL_6, 0.00002897696896f, 0.003815309638f, 1.5f},
  {"ring_6_front_left", ADC1_CHANNEL_7, 0.0000282914602f, 0.002773656281f, 2.0f},
#endif
};

ros::NodeHandle nh;

sensor_msgs::Range range_msg;
ros::Publisher range_publisher("ir_data", &range_msg);

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
  adc1_config_width(ADC_WIDTH_BIT_12);
  for(auto i = sensors.begin(); i < sensors.end(); i++) {
    adc1_config_channel_atten(i->pin,ADC_ATTEN_DB_11);
  }
}

void rosserial_publish()
{
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
    range_msg.range = adc_value;
    #else
    range_msg.range = (1 / (i->A * adc_value + i->B) - i->K) / 100;
    #endif
    range_msg.header.frame_id = i->frame.c_str();
    range_msg.header.stamp = nh.now();
    
    // Publish message
    range_publisher.publish(&range_msg);
  }

  nh.spinOnce();
}
