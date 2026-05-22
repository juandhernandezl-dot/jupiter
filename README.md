# 🤖 Zumo Robot Simulation (ROS 2 + Gazebo)

Simulación de un robot tipo **minisumo con orugas (track drive)** en **ROS 2 Humble + Gazebo Classic**, incluyendo:

- Control diferencial (`cmd_vel`)
- Visualización en RViz
- Sensor IMU simulado
- Visualización de datos con `rqt`

---

## 📦 Estructura del paquete

```
zumo_1/
├── launch/
│   ├── gazebo.launch.py
│   └── display.launch.py
├── urdf/
│   └── ROBOT_ZUMO_URDF.SLDASM.urdf
├── meshes/
├── worlds/
├── package.xml
└── setup.py
```

---

## 🚀 Instalación y ejecución

### 1. Construir el workspace

```bash
cd ~/tobar_ws
rm -rf build install log
colcon build --packages-select zumo_1
source install/setup.bash
```

---

### 2. Lanzar simulación en Gazebo

```bash
ros2 launch zumo_1 gazebo.launch.py
```

---

### 3. Visualizar solo el modelo (sin Gazebo)

```bash
ros2 launch zumo_1 display.launch.py
```

---

## 🎮 Control del robot

El robot se controla usando el plugin `diff_drive` mediante el tópico:

```
/cmd_vel
```

### Usando GUI:

```bash
ros2 run rqt_robot_steering rqt_robot_steering
```

---

### Movimiento manual por terminal

Avanzar:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
```

Girar:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}" -r 10
```

---

## 📡 Sensor IMU

El robot incluye un sensor IMU simulado en Gazebo:

- Link: `imu_link`
- Plugin: `gazebo_ros_imu_sensor`
- Tópico:

```
/imu_data
```

---

### Ver datos del IMU

```bash
ros2 topic echo /imu_data
```

---

### Frecuencia del IMU

```bash
ros2 topic hz /imu_data
```

---

## 📊 Visualización con rqt (IMU)

Ejecutar:

```bash
rqt
```

Luego:

- Plugins → Visualization → Plot

Agregar los siguientes tópicos:

```
/imu_plugin/out/linear_acceleration/x
/imu_plugin/out/linear_acceleration/y
/imu_plugin/out/linear_acceleration/z
```

---

## 📸 Resultados

### 🟢 Simulación en Gazebo + IMU en rqt

![IMU Plot](./docs/imu_plot.png)

---

## ⚙️ Notas técnicas importantes

### ✔️ Orugas (tracks)

- Se manejan como **visual-only**
- No tienen colisión → evita bloqueo del movimiento
- Las ruedas son las que generan la dinámica real

---

### ✔️ Control diferencial

- Plugin: `gazebo_ros_diff_drive`
- Basado en:
  - `left_wheel_joint`
  - `right_wheel_joint`

---

### ✔️ IMU

- Referenciado a `base_link`
- Ruido gaussiano incluido
- Publicación continua

---

### ✔️ Frame principal

```
base_link
```

---

## 🧪 Debug útil

Ver nodos:

```bash
ros2 node list
```

Ver tópicos:

```bash
ros2 topic list
```

---

## 📌 Autores

- Juan Diego Hernández Loaiza
- Juan Sebastián Valencia Pastrana
- Universidad Autónoma de Occidente  

---

## 🚧 Trabajo futuro

- Integración con IMU real
- Fusión sensorial (EKF / robot_localization)
- Control avanzado (PID / MPC)
- Comunicación CAN con ESP32
