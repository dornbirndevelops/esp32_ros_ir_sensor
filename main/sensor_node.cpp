#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <driver/adc.h>
#include <vector>
#include <utility>
#include <string>
#include <algorithm>
#include "sensor_node.h"

#define NO_OF_SAMPLES 65
#define A 0.00003047956374f
#define B 0.004288992178f
#define K 1.5f

ros::NodeHandle nh;

sensor_msgs::Range range_msg;
ros::Publisher range_publisher("ir_data", &range_msg);

std::vector<std::pair<std::string, adc1_channel_t> > sensors = {
  std::make_pair("bar", ADC1_CHANNEL_6)
};

float read_range(adc1_channel_t channel)
{
  // Multisampling with median
  int raw_samples[NO_OF_SAMPLES];
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    raw_samples[i] = adc1_get_raw(channel);
  }
  std::sort(raw_samples, raw_samples+NO_OF_SAMPLES);
  auto adc_value = raw_samples[NO_OF_SAMPLES/2];

  return (1 / (A * adc_value + B) - K) / 100;
}

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
    adc1_config_channel_atten(i->second,ADC_ATTEN_DB_11);
  }
}

void rosserial_publish()
{
  for(auto i = sensors.begin(); i < sensors.end(); i++) {
    // Fill dynamic message data
    range_msg.header.frame_id = i->first.c_str();
    range_msg.range = read_range(i->second);
    range_msg.header.stamp = nh.now();
    
    // Publish message
    range_publisher.publish(&range_msg);
  }

  nh.spinOnce();
}
