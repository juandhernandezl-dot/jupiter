#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// WIFI_SSID, WIFI_PASSWORD, AGENT_IP y AGENT_PORT
// se definen en platformio.ini bajo build_flags — no editar aquí.

#define RCCHECK(fn)                         \
  {                                         \
    rcl_ret_t rc = fn;                      \
    if (rc != RCL_RET_OK) error_loop();     \
  }

#define RCSOFTCHECK(fn)                     \
  {                                         \
    rcl_ret_t rc = fn;                      \
    (void)rc;                               \
  }

rcl_publisher_t publisher;
rcl_timer_t     timer;
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
std_msgs__msg__Int32 msg;

void error_loop()
{
  while (true) { delay(100); }
}

void timer_callback(rcl_timer_t * timer, int64_t /*last_call_time*/)
{
  if (timer != NULL) {
    msg.data++;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  // Credenciales e IP vienen de platformio.ini → build_flags
  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);

  set_microros_wifi_transports(
    (char*)WIFI_SSID,
    (char*)WIFI_PASSWORD,
    agent_ip,
    AGENT_PORT
  );

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "zumo_s3_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "zumo_heartbeat"
  ));

  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(500),
    timer_callback
  ));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}