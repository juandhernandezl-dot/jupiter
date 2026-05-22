/**
 * main.cpp — Zumo ESP32 DevKit v1
 * ══════════════════════════════════════════════════════════════
 * Combina tres funcionalidades principales:
 *   1. Publicador /zumo_imu  (sensor_msgs/Imu) con quaterniones
 *      reales desde el DMP del MPU6050 vía Simple_MPU6050.
 *   2. Suscriptor /cmd_vel   (geometry_msgs/Twist) que controla
 *      los motores mediante el driver TB6612FNG.
 *   3. Transporte micro-ROS sobre WiFi UDP.
 *
 * Arquitectura FreeRTOS (dual-core ESP32):
 *   Core 0 → microros_task  : WiFi + agente + executor ROS 2
 *   Core 1 → motors_task    : aplica cmd_vel a motores a 50 Hz
 *   (el DMP del MPU6050 dispara su propio callback por FIFO)
 * ══════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include "Simple_MPU6050.h"
#include <micro_ros_platformio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

// ════════════════════════════════════════════════════════════
//  Pines I2C
// ════════════════════════════════════════════════════════════
#define I2C_SDA 22
#define I2C_SCL 21

// ════════════════════════════════════════════════════════════
//  Pines TB6612FNG
//  Motor A = oruga DERECHA  |  Motor B = oruga IZQUIERDA
// ════════════════════════════════════════════════════════════
#define PWMA_PIN   25
#define PWMB_PIN   23
#define MOTORA_IN1 27
#define MOTORA_IN2 26
#define MOTORB_IN1 14
#define MOTORB_IN2 13

#define LEDC_FREQ  5000
#define LEDC_RES   8

// ════════════════════════════════════════════════════════════
//  Geometría del robot Zumo con orugas Pololu 22T
//
//  TRACK_SEPARATION: distancia entre los centros de las dos
//  orugas. Del URDF:
//    left_track_joint.y  =  0.04466 m
//    right_track_joint.y = -0.04488 m
//    total               =  0.08954 m
//
//  Si el giro en el robot real es más lento de lo esperado
//  → aumentar este valor.
//  Si es más rápido → disminuirlo.
//
//  MAX_LINEAR_VEL: velocidad lineal real a duty cycle 100%.
//  Medir en físico: marcar 1 m, cronometrar con cmd_vel=0.5
//  y ajustar hasta que la odometría sea correcta.
// ════════════════════════════════════════════════════════════
#define TRACK_SEPARATION 0.08954f   // metros — coincide con el URDF
#define MAX_LINEAR_VEL   0.4f       // m/s  — límite real del Zumo a 6V

// ════════════════════════════════════════════════════════════
//  Factor de escala de giro (steering_efficiency del firmware)
//
//  Controla cuánto contribuye angular_z al diferencial:
//    1.0 → ambas orugas en dirección opuesta a velocidad completa
//          (giro en punto fijo agresivo — igual que Gazebo)
//    0.5 → giro más suave, la oruga contraria no llega a invertir
//
//  Debe coincidir con steering_efficiency del URDF de Gazebo
//  para que la simulación y el robot real se comporten igual.
// ════════════════════════════════════════════════════════════
#define STEERING_EFFICIENCY 1.0f

// ════════════════════════════════════════════════════════════
//  Umbral de zona muerta PWM
//
//  Los motores del Zumo (micro metal gearmotors) no arrancan
//  por debajo de cierto duty cycle (~15-20% a 6V).
//  Comandos por debajo de este umbral se tratan como cero
//  para evitar que el motor zumbe sin moverse.
//
//  Ajustar midiendo el mínimo duty con el que cada motor
//  realmente gira sin carga.
// ════════════════════════════════════════════════════════════
#define MIN_PWM_THRESHOLD 0.10f   // 10% — por debajo → freno activo

// Escalas MPU6050
static constexpr float ACCEL_SCALE = 9.80665f / 16384.0f;
static constexpr float GYRO_SCALE  = (3.14159265f / 180.0f) / 131.0f;

#define RCSOFTCHECK(fn) { (void)(fn); }

// ════════════════════════════════════════════════════════════
//  Objetos micro-ROS
// ════════════════════════════════════════════════════════════
static rcl_publisher_t             pub_imu;
static rcl_subscription_t          sub_cmd_vel;
static rcl_timer_t                 timer;
static rclc_executor_t             executor;
static rclc_support_t              support;
static rcl_allocator_t             allocator;
static rcl_node_t                  node;
static sensor_msgs__msg__Imu       imu_msg;
static geometry_msgs__msg__Twist   cmd_vel_msg;

static Simple_MPU6050 mpu;

static SemaphoreHandle_t imu_mutex;
static struct {
  float qw, qx, qy, qz;
  float ax, ay, az;
  float gx, gy, gz;
} imu_data;

static SemaphoreHandle_t cmd_mutex;
static float g_linear_x  = 0.0f;
static float g_angular_z = 0.0f;


// ════════════════════════════════════════════════════════════
//  CONTROL DE MOTORES — TB6612FNG
// ════════════════════════════════════════════════════════════

void motors_init()
{
  pinMode(MOTORA_IN1, OUTPUT);
  pinMode(MOTORA_IN2, OUTPUT);
  pinMode(MOTORB_IN1, OUTPUT);
  pinMode(MOTORB_IN2, OUTPUT);

  ledcAttach(PWMA_PIN, LEDC_FREQ, LEDC_RES);
  ledcAttach(PWMB_PIN, LEDC_FREQ, LEDC_RES);

  ledcWrite(PWMA_PIN, 0);
  ledcWrite(PWMB_PIN, 0);
  digitalWrite(MOTORA_IN1, LOW);
  digitalWrite(MOTORA_IN2, LOW);
  digitalWrite(MOTORB_IN1, LOW);
  digitalWrite(MOTORB_IN2, LOW);
}

/**
 * set_motor_a(speed) — Oruga DERECHA
 * speed: -1.0 (reversa) a +1.0 (adelante)
 *
 * NOTA sobre polaridad:
 *   Si la oruga derecha gira al revés de lo esperado,
 *   intercambiar MOTORA_IN1 y MOTORA_IN2 en el hardware,
 *   o invertir la señal aquí multiplicando speed por -1.
 */
void set_motor_a(float speed)
{
  // Zona muerta: comandos muy pequeños no mueven el motor
  if (fabsf(speed) < MIN_PWM_THRESHOLD) {
    digitalWrite(MOTORA_IN1, LOW);
    digitalWrite(MOTORA_IN2, LOW);
    ledcWrite(PWMA_PIN, 0);
    return;
  }

  int pwm = (int)(fabsf(speed) * 255.0f);
  pwm = constrain(pwm, 0, 255);

  if (speed > 0.0f) {
    digitalWrite(MOTORA_IN1, HIGH);
    digitalWrite(MOTORA_IN2, LOW);
  } else {
    digitalWrite(MOTORA_IN1, LOW);
    digitalWrite(MOTORA_IN2, HIGH);
  }
  ledcWrite(PWMA_PIN, pwm);
}

/**
 * set_motor_b(speed) — Oruga IZQUIERDA
 * speed: -1.0 (reversa) a +1.0 (adelante)
 *
 * NOTA sobre polaridad:
 *   Si la oruga izquierda gira al revés de lo esperado,
 *   intercambiar MOTORB_IN1 y MOTORB_IN2 en el hardware,
 *   o invertir la señal aquí multiplicando speed por -1.
 */
void set_motor_b(float speed)
{
  if (fabsf(speed) < MIN_PWM_THRESHOLD) {
    digitalWrite(MOTORB_IN1, LOW);
    digitalWrite(MOTORB_IN2, LOW);
    ledcWrite(PWMB_PIN, 0);
    return;
  }

  int pwm = (int)(fabsf(speed) * 255.0f);
  pwm = constrain(pwm, 0, 255);

  if (speed > 0.0f) {
    digitalWrite(MOTORB_IN1, HIGH);
    digitalWrite(MOTORB_IN2, LOW);
  } else {
    digitalWrite(MOTORB_IN1, LOW);
    digitalWrite(MOTORB_IN2, HIGH);
  }
  ledcWrite(PWMB_PIN, pwm);
}

/**
 * apply_cmd_vel(linear_x, angular_z)
 * ════════════════════════════════════════════════════════════
 * Cinemática diferencial con STEERING_EFFICIENCY:
 *
 *   v_right = linear_x + angular_z * (TRACK_SEPARATION/2) * STEERING_EFFICIENCY
 *   v_left  = linear_x - angular_z * (TRACK_SEPARATION/2) * STEERING_EFFICIENCY
 *
 * Con STEERING_EFFICIENCY = 1.0 y linear_x = 0:
 *   angular_z > 0 (girar izquierda):
 *     v_right = +valor → oruga derecha AVANZA
 *     v_left  = -valor → oruga izquierda RETROCEDE  ← giro cooperativo
 *
 *   angular_z < 0 (girar derecha):
 *     v_right = -valor → oruga derecha RETROCEDE
 *     v_left  = +valor → oruga izquierda AVANZA     ← giro cooperativo
 *
 * El resultado es idéntico al steering_efficiency=1.0 del
 * SimpleTrackedVehiclePlugin en Gazebo.
 * ════════════════════════════════════════════════════════════
 */
void apply_cmd_vel(float linear_x, float angular_z)
{
  float half_sep = (TRACK_SEPARATION / 2.0f) * STEERING_EFFICIENCY;

  float v_right = linear_x + (angular_z * half_sep);
  float v_left  = linear_x - (angular_z * half_sep);

  // Normalizar a [-1, 1] dividiendo por MAX_LINEAR_VEL
  float speed_r = v_right / MAX_LINEAR_VEL;
  float speed_l = v_left  / MAX_LINEAR_VEL;

  // Clamp: si una oruga satura, recortar ambas proporcionalmente
  // para mantener la razón de giro (anti-windup cinemático)
  float max_speed = max(fabsf(speed_r), fabsf(speed_l));
  if (max_speed > 1.0f) {
    speed_r /= max_speed;
    speed_l /= max_speed;
  }

  set_motor_a(speed_r);   // Oruga derecha
  set_motor_b(speed_l);   // Oruga izquierda
}


// ════════════════════════════════════════════════════════════
//  CALLBACK DMP
// ════════════════════════════════════════════════════════════
void imu_dmp_callback(int16_t *gyro, int16_t *accel, int32_t *quat)
{
  Quaternion q;
  mpu.GetQuaternion(&q, quat);

  if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    imu_data.qw = q.w;
    imu_data.qx = q.x;
    imu_data.qy = q.y;
    imu_data.qz = q.z;

    imu_data.ax = accel[0] * ACCEL_SCALE;
    imu_data.ay = accel[1] * ACCEL_SCALE;
    imu_data.az = accel[2] * ACCEL_SCALE;

    imu_data.gx = gyro[0] * GYRO_SCALE;
    imu_data.gy = gyro[1] * GYRO_SCALE;
    imu_data.gz = gyro[2] * GYRO_SCALE;

    xSemaphoreGive(imu_mutex);
  }
}


// ════════════════════════════════════════════════════════════
//  CALLBACK cmd_vel
// ════════════════════════════════════════════════════════════
void cmd_vel_callback(const void * msg_in)
{
  const geometry_msgs__msg__Twist * msg =
    (const geometry_msgs__msg__Twist *)msg_in;

  if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    g_linear_x  = (float)msg->linear.x;
    g_angular_z = (float)msg->angular.z;
    xSemaphoreGive(cmd_mutex);
  }
}


// ════════════════════════════════════════════════════════════
//  CALLBACK TIMER — 50 Hz
// ════════════════════════════════════════════════════════════
void timer_callback(rcl_timer_t * timer, int64_t /*last_call_time*/)
{
  if (timer == NULL) return;

  if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    imu_msg.orientation.w = imu_data.qw;
    imu_msg.orientation.x = imu_data.qx;
    imu_msg.orientation.y = imu_data.qy;
    imu_msg.orientation.z = imu_data.qz;

    imu_msg.linear_acceleration.x = imu_data.ax;
    imu_msg.linear_acceleration.y = imu_data.ay;
    imu_msg.linear_acceleration.z = imu_data.az;

    imu_msg.angular_velocity.x = imu_data.gx;
    imu_msg.angular_velocity.y = imu_data.gy;
    imu_msg.angular_velocity.z = imu_data.gz;

    xSemaphoreGive(imu_mutex);
  }

  RCSOFTCHECK(rcl_publish(&pub_imu, &imu_msg, NULL));
}


// ════════════════════════════════════════════════════════════
//  TASK: MOTORES — Core 1, 50 Hz
// ════════════════════════════════════════════════════════════
void motors_task(void * /*param*/)
{
  const TickType_t period      = pdMS_TO_TICKS(20);
  const TickType_t timeout_tks = pdMS_TO_TICKS(500);
  TickType_t last_wake         = xTaskGetTickCount();
  TickType_t last_cmd_time     = xTaskGetTickCount();

  while (true) {
    float lin = 0.0f, ang = 0.0f;

    if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      lin = g_linear_x;
      ang = g_angular_z;
      xSemaphoreGive(cmd_mutex);
    }

    if (fabsf(lin) > 0.001f || fabsf(ang) > 0.001f) {
      last_cmd_time = xTaskGetTickCount();
      apply_cmd_vel(lin, ang);
    } else if ((xTaskGetTickCount() - last_cmd_time) > timeout_tks) {
      apply_cmd_vel(0.0f, 0.0f);
    }

    vTaskDelayUntil(&last_wake, period);
  }
}


// ════════════════════════════════════════════════════════════
//  TASK: IMU DMP — Core 1, 100 Hz
// ════════════════════════════════════════════════════════════
void imu_task(void * /*param*/)
{
  const TickType_t period = pdMS_TO_TICKS(10);
  TickType_t last_wake    = xTaskGetTickCount();

  while (true) {
    mpu.dmp_read_fifo(false);
    vTaskDelayUntil(&last_wake, period);
  }
}


// ════════════════════════════════════════════════════════════
//  Inicialización micro-ROS
// ════════════════════════════════════════════════════════════
bool microros_init()
{
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("[ERROR] rclc_support_init");
    return false;
  }

  if (rclc_node_init_default(&node, "zumo_node", "", &support) != RCL_RET_OK) {
    Serial.println("[ERROR] rclc_node_init");
    return false;
  }

  if (rclc_publisher_init_default(
        &pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "zumo_imu") != RCL_RET_OK) {
    Serial.println("[ERROR] publisher IMU");
    return false;
  }

  if (rclc_subscription_init_default(
        &sub_cmd_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) {
    Serial.println("[ERROR] subscriber cmd_vel");
    return false;
  }

  if (rclc_timer_init_default(&timer, &support,
        RCL_MS_TO_NS(20), timer_callback) != RCL_RET_OK) {
    Serial.println("[ERROR] timer");
    return false;
  }

  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) {
    Serial.println("[ERROR] executor_init");
    return false;
  }

  if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) {
    Serial.println("[ERROR] add_timer");
    return false;
  }

  if (rclc_executor_add_subscription(&executor, &sub_cmd_vel,
        &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("[ERROR] add_subscription");
    return false;
  }

  imu_msg.orientation_covariance[0] = 0.01f;
  imu_msg.orientation_covariance[4] = 0.01f;
  imu_msg.orientation_covariance[8] = 0.01f;

  return true;
}


// ════════════════════════════════════════════════════════════
//  TASK: micro-ROS — Core 0
// ════════════════════════════════════════════════════════════
void microros_task(void * /*param*/)
{
  Serial.println("[INFO] Conectando al WiFi...");

  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);
  set_microros_wifi_transports(
    (char*)WIFI_SSID, (char*)WIFI_PASSWORD, agent_ip, AGENT_PORT);

  Serial.printf("[INFO] SSID: %s | Agente: %s:%d\n",
                WIFI_SSID, AGENT_IP, AGENT_PORT);

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    Serial.println("[INFO] Intentando conectar al agente...");

    if (microros_init()) {
      Serial.println("[OK] micro-ROS listo. /zumo_imu pub | /cmd_vel sub");
      break;
    }

    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
    Serial.println("[WARN] Reintentando en 3 s...");
  }

  while (true) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  vTaskDelete(NULL);
}


// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========== ZUMO ESP32 BOOT ==========");

  motors_init();
  Serial.println("[OK] TB6612FNG inicializado.");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  mpu.begin();
  mpu.Set_DMP_Output_Rate_Hz(10);
  mpu.CalibrateMPU();
  mpu.load_DMP_Image();
  mpu.on_FIFO(imu_dmp_callback);

  Serial.println("[OK] MPU6050 DMP inicializado y calibrado.");

  imu_mutex = xSemaphoreCreateMutex();
  cmd_mutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(imu_task,      "imu_task",      4096, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(motors_task,   "motors_task",   2048, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(microros_task, "microros_task", 8192, NULL, 5, NULL, 0);

  Serial.println("[INFO] Tasks creados. FreeRTOS scheduler activo.");
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}