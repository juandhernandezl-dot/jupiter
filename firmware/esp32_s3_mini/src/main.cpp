#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// WIFI_SSID, WIFI_PASSWORD, AGENT_IP y AGENT_PORT
// se definen en platformio.ini bajo build_flags — no editar aquí.

// ── Macros ───────────────────────────────────────────────────
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

// ── Objetos micro-ROS ────────────────────────────────────────
rcl_publisher_t publisher;
rcl_timer_t     timer;
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
std_msgs__msg__Int32 msg;

// ── Error loop ───────────────────────────────────────────────
void error_loop()
{
  while (true) { vTaskDelay(pdMS_TO_TICKS(100)); }
}

// ── Timer callback ───────────────────────────────────────────
void timer_callback(rcl_timer_t * timer, int64_t /*last_call_time*/)
{
  if (timer != NULL) {
    msg.data++;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

// ════════════════════════════════════════════════════════════
//  FreeRTOS Task — corre micro-ROS en Core 0
//  El Core 1 queda libre para otras tareas (sensores, motores)
// ════════════════════════════════════════════════════════════
void microros_task(void * /*param*/)
{
  // ── Conexión WiFi + transporte UDP ───────────────────────
  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);

  set_microros_wifi_transports(
    (char*)WIFI_SSID,
    (char*)WIFI_PASSWORD,
    agent_ip,
    AGENT_PORT
  );

  // Espera a que el agente esté listo
  vTaskDelay(pdMS_TO_TICKS(2000));

  // ── Inicialización micro-ROS ─────────────────────────────
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

  // ── Loop del executor ────────────────────────────────────
  while (true) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Nunca llega aquí, pero buena práctica
  vTaskDelete(NULL);
}

// ════════════════════════════════════════════════════════════
//  Setup y Loop de Arduino — solo crean el task y ceden control
// ════════════════════════════════════════════════════════════
void setup()
{
  Serial.begin(115200);

  // Crear el task de micro-ROS en Core 0
  // Stack de 8 KB es suficiente para micro-ROS + WiFi
  xTaskCreatePinnedToCore(
    microros_task,    // función del task
    "microros_task",  // nombre (debug)
    8192,             // stack en bytes
    NULL,             // parámetro
    5,                // prioridad (5 = alta, por encima de tareas de usuario)
    NULL,             // handle (no necesario aquí)
    0                 // Core 0
  );
}

void loop()
{
  // El scheduler de FreeRTOS toma el control.
  // Este loop queda vacío — agregar aquí tareas de baja prioridad
  // o crear tasks adicionales en setup() para sensores, motores, etc.
  vTaskDelay(pdMS_TO_TICKS(1000));
}