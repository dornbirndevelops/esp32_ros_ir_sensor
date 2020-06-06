#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

#include <driver/adc.h>

#include <vector>
#include <utility>
#include <string>

#define NO_OF_SAMPLES 64

ros::NodeHandle nh;

sensor_msgs::Range range_msg;
ros::Publisher range_publisher("ir_data", &range_msg);

std::vector<std::pair<std::string, adc1_channel_t> > sensors = {
  std::make_pair("foo", ADC1_CHANNEL_6),
  std::make_pair("bar", ADC1_CHANNEL_6)
};

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float read_range(adc1_channel_t channel)
{
  uint32_t adc_reading = 0;
  //Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    adc_reading += adc1_get_raw(channel);
  }
  adc_reading /= NO_OF_SAMPLES;

  return 29.988 * pow(map(adc_reading, 0, 4095, 0, 5000)/1000.0, -1.173);
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
