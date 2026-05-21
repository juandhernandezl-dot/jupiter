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
 *
 * Diferencia respecto a lectura raw:
 *   - Raw:  accel/gyro en LSB → hay que convertir y no hay orientación
 *   - DMP:  el MPU6050 fusiona internamente accel+gyro y entrega
 *           quaterniones calibrados directamente → más preciso,
 *           menos drift, orientación real disponible.
 * ══════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include "Simple_MPU6050.h"        // Librería DMP de FerJeffQ
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
//  Wire.begin(SDA, SCL) → Wire.begin(22, 21)
// ════════════════════════════════════════════════════════════
#define I2C_SDA 22
#define I2C_SCL 21

// ════════════════════════════════════════════════════════════
//  Pines TB6612FNG
//  Motor A = rueda derecha  |  Motor B = rueda izquierda
// ════════════════════════════════════════════════════════════
#define PWMA_PIN   25   // PWM motor A (derecho)
#define PWMB_PIN   23   // PWM motor B (izquierdo)
#define MOTORA_IN1 27   // Dirección motor A — pin 1
#define MOTORA_IN2 26   // Dirección motor A — pin 2
#define MOTORB_IN1 14   // Dirección motor B — pin 1
#define MOTORB_IN2 13   // Dirección motor B — pin 2

// Configuración PWM (API nueva arduinoespressif32 >= 3.x)
#define LEDC_FREQ  5000  // Frecuencia PWM en Hz
#define LEDC_RES   8     // Resolución en bits → rango 0-255

// Geometría del robot — ajustar según medición real
#define WHEEL_SEPARATION 0.10f   // Distancia entre ruedas en metros
#define MAX_LINEAR_VEL   0.5f    // Velocidad lineal máxima en m/s

// ════════════════════════════════════════════════════════════
//  Escalas MPU6050 para aceleración y giroscopio raw
//  (el DMP entrega quaterniones, pero también leemos raw
//   para llenar linear_acceleration y angular_velocity del msg)
//  ±2 g   → 16384 LSB/g   → convertir a m/s²
//  ±250°/s → 131 LSB/°/s  → convertir a rad/s
// ════════════════════════════════════════════════════════════
static constexpr float ACCEL_SCALE = 9.80665f / 16384.0f;
static constexpr float GYRO_SCALE  = (3.14159265f / 180.0f) / 131.0f;

// ════════════════════════════════════════════════════════════
//  Macro micro-ROS — versión suave (no detiene el sistema)
// ════════════════════════════════════════════════════════════
#define RCSOFTCHECK(fn) { (void)(fn); }

// ════════════════════════════════════════════════════════════
//  Objetos micro-ROS
// ════════════════════════════════════════════════════════════
static rcl_publisher_t             pub_imu;      // Publica /zumo_imu
static rcl_subscription_t          sub_cmd_vel;  // Suscribe /cmd_vel
static rcl_timer_t                 timer;        // Timer 50 Hz para publicar IMU
static rclc_executor_t             executor;     // Ejecuta callbacks ROS 2
static rclc_support_t              support;      // Soporte del nodo
static rcl_allocator_t             allocator;    // Allocator de memoria
static rcl_node_t                  node;         // Nodo "zumo_node"
static sensor_msgs__msg__Imu       imu_msg;      // Mensaje IMU a publicar
static geometry_msgs__msg__Twist   cmd_vel_msg;  // Mensaje cmd_vel recibido

// ════════════════════════════════════════════════════════════
//  Objeto MPU6050 con DMP (Digital Motion Processor)
//  El DMP fusiona accel + gyro internamente y entrega
//  quaterniones directamente sin necesidad de filtros externos.
// ════════════════════════════════════════════════════════════
static Simple_MPU6050 mpu;

// ════════════════════════════════════════════════════════════
//  Mutex + buffer compartido para datos del IMU
//  El callback del DMP (Core 1) escribe aquí.
//  El timer de micro-ROS (Core 0) lee aquí para publicar.
// ════════════════════════════════════════════════════════════
static SemaphoreHandle_t imu_mutex;

static struct {
  // Quaternión de orientación (del DMP)
  float qw, qx, qy, qz;
  // Aceleración lineal en m/s²
  float ax, ay, az;
  // Velocidad angular en rad/s
  float gx, gy, gz;
} imu_data;

// ════════════════════════════════════════════════════════════
//  Mutex + variables compartidas para cmd_vel
//  El callback de micro-ROS escribe aquí.
//  El motors_task lee aquí a 50 Hz.
// ════════════════════════════════════════════════════════════
static SemaphoreHandle_t cmd_mutex;
static float g_linear_x  = 0.0f;  // Velocidad lineal deseada (m/s)
static float g_angular_z = 0.0f;  // Velocidad angular deseada (rad/s)


// ════════════════════════════════════════════════════════════
//  CONTROL DE MOTORES — TB6612FNG
// ════════════════════════════════════════════════════════════

/**
 * motors_init()
 * Configura los pines de dirección y PWM del TB6612FNG.
 * Usa la API nueva de LEDC (arduinoespressif32 >= 3.x):
 *   ledcAttach(pin, freq, bits) — sin canales explícitos.
 */
void motors_init()
{
  // Configurar pines de dirección como salidas digitales
  pinMode(MOTORA_IN1, OUTPUT);
  pinMode(MOTORA_IN2, OUTPUT);
  pinMode(MOTORB_IN1, OUTPUT);
  pinMode(MOTORB_IN2, OUTPUT);

  // Adjuntar PWM a los pines (API nueva — sin ledcSetup/ledcAttachPin)
  ledcAttach(PWMA_PIN, LEDC_FREQ, LEDC_RES);
  ledcAttach(PWMB_PIN, LEDC_FREQ, LEDC_RES);

  // Estado inicial: motores parados
  ledcWrite(PWMA_PIN, 0);
  ledcWrite(PWMB_PIN, 0);
  digitalWrite(MOTORA_IN1, LOW);
  digitalWrite(MOTORA_IN2, LOW);
  digitalWrite(MOTORB_IN1, LOW);
  digitalWrite(MOTORB_IN2, LOW);
}

/**
 * set_motor_a(speed)
 * Controla el motor A (rueda derecha).
 * speed: -1.0 (reversa máxima) a 1.0 (adelante máximo)
 *   > 0  → IN1=HIGH, IN2=LOW  → adelante
 *   < 0  → IN1=LOW,  IN2=HIGH → reversa
 *   ≈ 0  → IN1=LOW,  IN2=LOW  → freno activo
 */
void set_motor_a(float speed)
{
  // Convertir velocidad normalizada a valor PWM (0-255)
  int pwm = (int)(fabsf(speed) * 255.0f);
  pwm = constrain(pwm, 0, 255);  // Limitar al rango válido

  if (speed > 0.01f) {
    digitalWrite(MOTORA_IN1, HIGH);
    digitalWrite(MOTORA_IN2, LOW);
  } else if (speed < -0.01f) {
    digitalWrite(MOTORA_IN1, LOW);
    digitalWrite(MOTORA_IN2, HIGH);
  } else {
    // Zona muerta: freno activo
    digitalWrite(MOTORA_IN1, LOW);
    digitalWrite(MOTORA_IN2, LOW);
    pwm = 0;
  }
  ledcWrite(PWMA_PIN, pwm);
}

/**
 * set_motor_b(speed)
 * Controla el motor B (rueda izquierda). Misma lógica que A.
 */
void set_motor_b(float speed)
{
  int pwm = (int)(fabsf(speed) * 255.0f);
  pwm = constrain(pwm, 0, 255);

  if (speed > 0.01f) {
    digitalWrite(MOTORB_IN1, HIGH);
    digitalWrite(MOTORB_IN2, LOW);
  } else if (speed < -0.01f) {
    digitalWrite(MOTORB_IN1, LOW);
    digitalWrite(MOTORB_IN2, HIGH);
  } else {
    digitalWrite(MOTORB_IN1, LOW);
    digitalWrite(MOTORB_IN2, LOW);
    pwm = 0;
  }
  ledcWrite(PWMB_PIN, pwm);
}

/**
 * apply_cmd_vel(linear_x, angular_z)
 * Modelo cinemático diferencial:
 *   v_derecha  = linear_x + angular_z * (d/2)
 *   v_izquierda = linear_x - angular_z * (d/2)
 * donde d = WHEEL_SEPARATION (distancia entre ruedas).
 * Normaliza a [-1, 1] dividiendo por MAX_LINEAR_VEL.
 */
void apply_cmd_vel(float linear_x, float angular_z)
{
  float v_right = linear_x + (angular_z * WHEEL_SEPARATION / 2.0f);
  float v_left  = linear_x - (angular_z * WHEEL_SEPARATION / 2.0f);

  // Normalizar: 1.0 = velocidad máxima
  float speed_r = v_right / MAX_LINEAR_VEL;
  float speed_l = v_left  / MAX_LINEAR_VEL;

  // Clamp a [-1, 1] para no saturar el PWM
  speed_r = constrain(speed_r, -1.0f, 1.0f);
  speed_l = constrain(speed_l, -1.0f, 1.0f);

  set_motor_a(speed_r);   // Rueda derecha
  set_motor_b(speed_l);   // Rueda izquierda
}


// ════════════════════════════════════════════════════════════
//  CALLBACK DMP — llamado automáticamente cuando el FIFO
//  del MPU6050 tiene datos listos.
//
//  Parámetros (igual que FerJeffQ):
//    gyro  → velocidad angular raw (int16_t[3])
//    accel → aceleración raw (int16_t[3])
//    quat  → quaternión raw del DMP (int32_t[4])
// ════════════════════════════════════════════════════════════
void imu_dmp_callback(int16_t *gyro, int16_t *accel, int32_t *quat)
{
  // Objeto Quaternion de la librería Simple_MPU6050
  // Contiene w, x, y, z normalizados a float
  Quaternion q;

  // GetQuaternion convierte los int32_t raw del DMP
  // a float normalizados en el objeto q
  mpu.GetQuaternion(&q, quat);

  // Escribir en el buffer compartido (protegido por mutex)
  // Timeout de 2 ms para no bloquear el callback del DMP
  if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {

    // Quaternión de orientación — directo del DMP, sin drift
    imu_data.qw = q.w;
    imu_data.qx = q.x;
    imu_data.qy = q.y;
    imu_data.qz = q.z;

    // Aceleración lineal: convertir LSB → m/s²
    imu_data.ax = accel[0] * ACCEL_SCALE;
    imu_data.ay = accel[1] * ACCEL_SCALE;
    imu_data.az = accel[2] * ACCEL_SCALE;

    // Velocidad angular: convertir LSB → rad/s
    imu_data.gx = gyro[0] * GYRO_SCALE;
    imu_data.gy = gyro[1] * GYRO_SCALE;
    imu_data.gz = gyro[2] * GYRO_SCALE;

    xSemaphoreGive(imu_mutex);
  }
}


// ════════════════════════════════════════════════════════════
//  CALLBACK cmd_vel — llamado por el executor de micro-ROS
//  cuando llega un mensaje en /cmd_vel.
//  Guarda los valores para que motors_task los aplique.
// ════════════════════════════════════════════════════════════
void cmd_vel_callback(const void * msg_in)
{
  const geometry_msgs__msg__Twist * msg =
    (const geometry_msgs__msg__Twist *)msg_in;

  if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    g_linear_x  = (float)msg->linear.x;   // m/s
    g_angular_z = (float)msg->angular.z;  // rad/s
    xSemaphoreGive(cmd_mutex);
  }
}


// ════════════════════════════════════════════════════════════
//  CALLBACK TIMER — 50 Hz, llamado por el executor.
//  Lee el buffer compartido del IMU y publica /zumo_imu.
// ════════════════════════════════════════════════════════════
void timer_callback(rcl_timer_t * timer, int64_t /*last_call_time*/)
{
  if (timer == NULL) return;

  // Leer buffer del IMU con mutex
  if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {

    // Orientación: quaternión del DMP
    // (antes publicábamos orientation_covariance[0]=-1 porque
    //  no teníamos orientación; ahora sí la tenemos del DMP)
    imu_msg.orientation.w = imu_data.qw;
    imu_msg.orientation.x = imu_data.qx;
    imu_msg.orientation.y = imu_data.qy;
    imu_msg.orientation.z = imu_data.qz;

    // Aceleración lineal en m/s²
    imu_msg.linear_acceleration.x = imu_data.ax;
    imu_msg.linear_acceleration.y = imu_data.ay;
    imu_msg.linear_acceleration.z = imu_data.az;

    // Velocidad angular en rad/s
    imu_msg.angular_velocity.x = imu_data.gx;
    imu_msg.angular_velocity.y = imu_data.gy;
    imu_msg.angular_velocity.z = imu_data.gz;

    xSemaphoreGive(imu_mutex);
  }

  // Publicar el mensaje en el tópico /zumo_imu
  RCSOFTCHECK(rcl_publish(&pub_imu, &imu_msg, NULL));
}


// ════════════════════════════════════════════════════════════
//  TASK: MOTORES — Core 1, prioridad 5, 50 Hz
//  Lee cmd_vel del buffer compartido y lo aplica a los motores.
//  Watchdog: si no hay comando en 500 ms → para los motores.
// ════════════════════════════════════════════════════════════
void motors_task(void * /*param*/)
{
  const TickType_t period      = pdMS_TO_TICKS(20);   // 50 Hz
  const TickType_t timeout_tks = pdMS_TO_TICKS(500);  // 500 ms watchdog
  TickType_t last_wake         = xTaskGetTickCount();
  TickType_t last_cmd_time     = xTaskGetTickCount();

  while (true) {
    float lin = 0.0f, ang = 0.0f;

    // Leer cmd_vel del buffer compartido
    if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      lin = g_linear_x;
      ang = g_angular_z;
      xSemaphoreGive(cmd_mutex);
    }

    if (fabsf(lin) > 0.001f || fabsf(ang) > 0.001f) {
      // Hay comando activo → aplicar y resetear watchdog
      last_cmd_time = xTaskGetTickCount();
      apply_cmd_vel(lin, ang);
    } else if ((xTaskGetTickCount() - last_cmd_time) > timeout_tks) {
      // Timeout: sin comando por 500 ms → parar por seguridad
      apply_cmd_vel(0.0f, 0.0f);
    }

    // Esperar hasta el siguiente ciclo de 20 ms (sin drift acumulado)
    vTaskDelayUntil(&last_wake, period);
  }
}


// ════════════════════════════════════════════════════════════
//  TASK: IMU DMP — Core 1, prioridad 6, polling FIFO
//  Llama a mpu.dmp_read_fifo() periódicamente.
//  Cuando hay datos en el FIFO, dispara imu_dmp_callback().
// ════════════════════════════════════════════════════════════
void imu_task(void * /*param*/)
{
  // Leer FIFO cada 10 ms (100 Hz) — igual que FerJeffQ
  const TickType_t period = pdMS_TO_TICKS(10);
  TickType_t last_wake    = xTaskGetTickCount();

  while (true) {
    // false = no bloquear esperando datos
    // Si hay datos en FIFO → llama a imu_dmp_callback()
    mpu.dmp_read_fifo(false);
    vTaskDelayUntil(&last_wake, period);
  }
}


// ════════════════════════════════════════════════════════════
//  Inicialización micro-ROS con reintentos
//  Retorna true si todos los pasos tuvieron éxito.
//  Si falla alguno, retorna false para reintentar desde
//  microros_task sin reiniciar el ESP32.
// ════════════════════════════════════════════════════════════
bool microros_init()
{
  allocator = rcl_get_default_allocator();

  // Inicializar soporte (conecta con el agente)
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("[ERROR] rclc_support_init");
    return false;
  }

  // Crear nodo "zumo_node" en namespace raíz
  if (rclc_node_init_default(&node, "zumo_node", "", &support) != RCL_RET_OK) {
    Serial.println("[ERROR] rclc_node_init");
    return false;
  }

  // Publisher: /zumo_imu con sensor_msgs/Imu
  if (rclc_publisher_init_default(
        &pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "zumo_imu") != RCL_RET_OK) {
    Serial.println("[ERROR] publisher IMU");
    return false;
  }

  // Subscriber: /cmd_vel con geometry_msgs/Twist
  if (rclc_subscription_init_default(
        &sub_cmd_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) {
    Serial.println("[ERROR] subscriber cmd_vel");
    return false;
  }

  // Timer: cada 20 ms → 50 Hz de publicación del IMU
  if (rclc_timer_init_default(&timer, &support,
        RCL_MS_TO_NS(20), timer_callback) != RCL_RET_OK) {
    Serial.println("[ERROR] timer");
    return false;
  }

  // Executor: 2 handles (1 timer + 1 subscriber)
  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) {
    Serial.println("[ERROR] executor_init");
    return false;
  }

  // Registrar timer en el executor
  if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) {
    Serial.println("[ERROR] add_timer");
    return false;
  }

  // Registrar subscriber en el executor con callback ON_NEW_DATA
  // ON_NEW_DATA = solo llama al callback si hay mensaje nuevo
  if (rclc_executor_add_subscription(&executor, &sub_cmd_vel,
        &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("[ERROR] add_subscription");
    return false;
  }

  // Covarianza de orientación: todos 0 = estimación disponible del DMP
  // (antes poníamos [0]=-1 porque no había orientación real)
  imu_msg.orientation_covariance[0]  = 0.01f;
  imu_msg.orientation_covariance[4]  = 0.01f;
  imu_msg.orientation_covariance[8]  = 0.01f;

  return true;
}


// ════════════════════════════════════════════════════════════
//  TASK: micro-ROS — Core 0, prioridad 5
//  Conecta al WiFi, intenta conectar al agente con reintentos,
//  y una vez conectado ejecuta el executor en loop.
// ════════════════════════════════════════════════════════════
void microros_task(void * /*param*/)
{
  Serial.println("[INFO] Conectando al WiFi...");

  // Configurar transporte WiFi UDP
  // WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT
  // vienen definidos en platformio.ini → build_flags
  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);
  set_microros_wifi_transports(
    (char*)WIFI_SSID, (char*)WIFI_PASSWORD, agent_ip, AGENT_PORT);

  Serial.printf("[INFO] SSID: %s | Agente: %s:%d\n",
                WIFI_SSID, AGENT_IP, AGENT_PORT);

  // Bucle de reintentos: espera 3 s y reintenta si el agente
  // no está disponible, sin reiniciar el ESP32
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    Serial.println("[INFO] Intentando conectar al agente...");

    if (microros_init()) {
      Serial.println("[OK] micro-ROS listo. /zumo_imu pub | /cmd_vel sub");
      break;
    }

    // Limpiar estado antes del próximo intento
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
    Serial.println("[WARN] Reintentando en 3 s...");
  }

  // Loop principal del executor:
  // spin_some procesa callbacks pendientes (timer + cmd_vel)
  // sin bloquear indefinidamente
  while (true) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  vTaskDelete(NULL);
}


// ════════════════════════════════════════════════════════════
//  SETUP — ejecuta una sola vez al arrancar
// ════════════════════════════════════════════════════════════
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========== ZUMO ESP32 BOOT ==========");

  // ── Motores ──────────────────────────────────────────────
  motors_init();
  Serial.println("[OK] TB6612FNG inicializado.");

  // ── I2C ──────────────────────────────────────────────────
  // Wire.begin(SDA, SCL) — igual que el sketch de Arduino
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // 100 kHz — más estable con el GY-521

  // ── MPU6050 con DMP ──────────────────────────────────────
  // begin() inicializa I2C y el chip
  mpu.begin();

  // Tasa de salida del DMP: cada 10 ms → 100 Hz
  // (igual que FerJeffQ: mpu.Set_DMP_Output_Rate_Hz(10))
  mpu.Set_DMP_Output_Rate_Hz(10);

  // CalibrateMPU() calcula los offsets de accel y gyro
  // automáticamente. El robot debe estar quieto y nivelado.
  mpu.CalibrateMPU();

  // load_DMP_Image() carga el firmware del DMP en el MPU6050
  // y finaliza la configuración. Habilita el FIFO.
  mpu.load_DMP_Image();

  // Registrar callback que se llama cuando el FIFO tiene datos
  // → llama a imu_dmp_callback(gyro, accel, quat)
  mpu.on_FIFO(imu_dmp_callback);

  Serial.println("[OK] MPU6050 DMP inicializado y calibrado.");

  // ── Mutexes ──────────────────────────────────────────────
  imu_mutex = xSemaphoreCreateMutex();  // Protege imu_data
  cmd_mutex = xSemaphoreCreateMutex();  // Protege g_linear_x, g_angular_z

  // ── Crear tasks FreeRTOS ─────────────────────────────────
  // Core 1: imu_task  (prio 6, mayor → lectura DMP determinista)
  // Core 1: motors_task (prio 5)
  // Core 0: microros_task (prio 5, WiFi + executor ROS 2)
  xTaskCreatePinnedToCore(imu_task,      "imu_task",      4096, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(motors_task,   "motors_task",   2048, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(microros_task, "microros_task", 8192, NULL, 5, NULL, 0);

  // Stack de imu_task aumentado a 4096 porque Simple_MPU6050
  // usa más memoria de stack que la librería MPU6050 básica

  Serial.println("[INFO] Tasks creados. FreeRTOS scheduler activo.");
}

// ════════════════════════════════════════════════════════════
//  LOOP — cedido al scheduler de FreeRTOS
//  No hace nada útil; todo el trabajo está en los tasks.
// ════════════════════════════════════════════════════════════
void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}