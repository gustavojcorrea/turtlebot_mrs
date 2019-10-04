#include <Q2HX711.h>
//#define USE_USBCON
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

//const int LED_PIN = 6;
const int RELAY_PIN = 12;
const int hx711_data_pin = 9; //Brown wire
const int hx711_clock_pin = 8; //White wire

int cur_val = 0;
const int num_values = 75;
float measurements[num_values];
bool first_run_done = false;
int run_number = 0;
const int num_runs = 5;

float offset = 0.0;

Q2HX711 hx711(hx711_data_pin, hx711_clock_pin);

std_msgs::Float32 load_msg;
ros::Publisher pub("robot_load", &load_msg);

void toggle_relay(const std_msgs::Bool& toggle_msg)
{
  digitalWrite(RELAY_PIN, toggle_msg.data ? HIGH : LOW);
}

ros::Subscriber<std_msgs::Bool> sub("toggle_relay", &toggle_relay);

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
//  pinMode(LED_PIN, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

//  Serial.begin(9600);
}

void loop() {
  const float reading = -1 * (hx711.read()/100.0) - offset;

  if (cur_val == num_values && !first_run_done) {
    first_run_done = true;
    offset = get_average(num_values, measurements);
  }
  measurements[cur_val % num_values] = reading;
  cur_val++;
  
  
  if (cur_val > num_values + 1 && run_number >= num_runs) {
    load_msg.data = get_average(num_values, measurements);
    pub.publish(&load_msg);
    run_number = 0;
//    Serial.println(reading);
  }
  run_number++;
  nh.spinOnce();
  delay(5);
}

float get_average(int size, float* array) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += array[i];
  }
  return sum / size;
}

