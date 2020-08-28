// rtos includes
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>

// ros includes
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

// util includes
#include <driver/adc.h>
#include <vector>
#include <utility>
#include <string>
#include <queue>
#include <algorithm>

// header includes
#include "sensor_node.h"

// configuration defines
#define CALIBRATE 0 // 1 use raw sensor values, 0 use distances
#define SENSOR_RING 6 // which sensors to use
#define MAX_FAILED_ATTEMPTS 10 // try to publish x times before restarting the esp
#define NO_OF_SAMPLES 20 // get this amount of samples per sensor
#define LOW_PASS 0.65 // distance value defining the maximum reliable value
#define ADC_WIDTH ADC_WIDTH_BIT_9 // adc resolution value
#define ADC_ATTEN_DB ADC_ATTEN_DB_11 // adc attenuation value

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
  {"sensor_2_front", ADC1_GPIO34_CHANNEL, 0.02892085115, 0.335067863, 0.0},
  {"sensor_2_left", ADC1_GPIO33_CHANNEL, 0.02835561933, 0.2885939545, 0.0},
  {"sensor_2_back", ADC1_GPIO32_CHANNEL, 0.02911063456, 0.3416997073, 0.0},
  {"sensor_2_right", ADC1_GPIO35_CHANNEL, 0.0284007823, 0.2294180786, 0.0},
#elif SENSOR_RING == 4
  {"sensor_4_front", ADC1_GPIO33_CHANNEL, 0.02248221728, 0.3395412617, 0.03},
  {"sensor_4_left", ADC1_GPIO32_CHANNEL, 0.01978953937, 0.5868486286, 0.04},
  {"sensor_4_back", ADC1_GPIO35_CHANNEL, 0.02424616864, 0.456042385, 0.02},
  {"sensor_4_right", ADC1_GPIO34_CHANNEL, 0.02203498917, 0.4580941459, 0.03},
#elif SENSOR_RING == 6
  {"sensor_6_front_left", ADC1_GPIO35_CHANNEL, 0.02182198866, 0.4452075733, 0.02},
  {"sensor_6_front_right", ADC1_GPIO34_CHANNEL, 0.01897069853, 0.5805050759, 0.04},
#endif
};

ros::NodeHandle nh;

sensor_msgs::Range range_msg;
ros::Publisher range_publisher("sensor_data", &range_msg);

SemaphoreHandle_t semaphore = NULL;

void sensor_node_read_task(void* param)
{
  auto sensor = (sensor_t*)param;
  
  while (1) {
    // Fill dynamic message data
    // Multisampling with median
    int raw_samples[NO_OF_SAMPLES];
    for (int j = 0; j < NO_OF_SAMPLES; j++) {
      raw_samples[j] = adc1_get_raw(sensor->pin);
      vTaskDelay(1);
    }
    std::sort(raw_samples, raw_samples+NO_OF_SAMPLES);
    auto adc_value = raw_samples[NO_OF_SAMPLES/2];
    
    #if CALIBRATE == 1
    // Use raw adc value on callibration process
    auto distance = adc_value;
    #else
    auto distance = 1 / (sensor->A * adc_value + sensor->B) - sensor->K;
    
    #ifdef LOW_PASS
    // Low Pass Filter trying to reduce noise in sensor values
    // Sensor tends to get inaccurate on higher distances
    if (distance > LOW_PASS) {
      continue;
    }
    #endif
    #endif

    if (semaphore != NULL) {
      if (xSemaphoreTake(semaphore, 10) == pdTRUE) {
        // critical section start

        // prepare message with data
        range_msg.range = distance;
        range_msg.header.frame_id = sensor->frame.c_str();
        range_msg.header.stamp = nh.now();
        
        // publish message
        range_publisher.publish(&range_msg);

        // critical section end
        xSemaphoreGive(semaphore);
      }
    }
  }
}

void sensor_node_start()
{
  vSemaphoreCreateBinary(semaphore);

  // Initialize ROS
  nh.initNode();
  nh.advertise(range_publisher);

  // Initialize static message data
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.1;
  range_msg.max_range = 0.8;

  // Initialize ADC
  adc1_config_width(ADC_WIDTH);
  for(auto it = sensors.begin(); it < sensors.end(); it++) {
    adc1_config_channel_atten(it->pin, ADC_ATTEN_DB);
  }

  // Start an observation task for every sensor
  for(auto it = sensors.begin(); it < sensors.end(); it++) {
    xTaskCreate(sensor_node_read_task, it->frame.c_str(), 2048, &(*it), 5, nullptr);
  }

  // keep checking for connection, if it disconnects, restart
  unsigned int failed_attempts = 0;
  while (1) {
    if (nh.connected()) {
      failed_attempts = 0;
    } else if (++failed_attempts > MAX_FAILED_ATTEMPTS) {
      // lost connection, restart device
      esp_restart();
    }
    nh.spinOnce();
    vTaskDelay(20);
  }

}
