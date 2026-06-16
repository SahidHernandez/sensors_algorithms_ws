# Sentinel Sensor & Algorithm Workspace

Repositorio central del sistema de percepción y navegación del robot **Sentinel** (plataforma Unitree Go2 EDU, equipo Sentinel NXL — RoboCup Rescue). Contiene la suite de drivers de sensores, algoritmos de visión artificial y la consola de monitoreo operacional.

---

## 🏗 Arquitectura del Sistema

El sistema se basa en una arquitectura modular de ROS 2 Humble dividida en cuatro capas:

1. **Hardware Abstraction Layer (HAL):** Drivers de cámara (RealSense D455, Fisheye, USB, Térmica TC001 Max) vía `v4l2_camera` y drivers específicos. CycloneDDS como middleware DDS con configuración de `MaxMessageSize` y fragmentación UDP para interfaces WiFi+Ethernet.
2. **Percepción & Algoritmos:** Detección de anillos de Landolt, análisis de estabilidad de movimiento, decodificación QR con filtro de nitidez y detección YOLO 3D con landmarks.
3. **Mapeo & Localización:** SLAM RGB-D con RTAB-Map (RealSense D455), Point-LIO para LiDAR, cadena TF `map → odom → base_link`.
4. **UI/Console:** Interfaz gráfica en **PySide6** con grid de cámaras 2×2, resultados de algoritmos, panel de diagnóstico y log de eventos. Bridge ROS2↔Qt vía `QThread` con throttle a 15 FPS.

---

## 🛠 Requisitos

### Sistema

- **OS:** Ubuntu 22.04 (Jetson) / Ubuntu 24.04 (Workstation)
- **ROS 2 Humble**
- **Python 3.10+**
- **CycloneDDS** (recomendado sobre FastDDS para este stack)

### Python

```bash
pip install pyzbar opencv-python PySide6 --break-system-packages
```

### Sistema (apt)

```bash
sudo apt install libzbar0 ros-humble-cv-bridge ros-humble-v4l2-camera \
                 ros-humble-realsense2-camera ros-humble-rtabmap-ros
```

### Hardware soportado

| Dispositivo | Driver / Paquete | Topic principal |
|---|---|---|
| RealSense D455 | `realsense2_camera` | `/camera/color/image_raw` |
| Cámara Fisheye USB | `v4l2_camera` | `/fisheye/image_raw` |
| Cámara USB frontal | `v4l2_camera` | `/usb_camera/image_raw` |
| Cámara Térmica TC001 Max | `thermal_camera` (custom) | `/thermal_camera/image_raw` |
| Magnetómetro I2C | custom node @ 100 Hz | `/magnetometer/heading` |

---

## 📦 Estructura del Workspace

```text
.
├── src/
│   ├── camera_bringup/               # Launch files maestros y configs YAML
│   │   ├── launch/
│   │   │   └── full_sensors_algorithms.launch.py
│   │   └── config/
│   │       └── *.yaml
│   ├── sentinel_sensor_console/      # Interfaz PySide6 (UI)
│   │   ├── widgets/
│   │   │   ├── camera_view.py        # Grid 2×2 de streams
│   │   │   ├── algorithm_results.py  # Tarjetas por algoritmo + modales
│   │   │   ├── status_panel.py       # Diagnóstico, brújula, quick actions
│   │   │   └── event_log.py          # Tabla de eventos INFO/WARN/ERROR
│   │   ├── ros_qt_bridge.py          # RosSignals + SentinelConsoleNode + RosThread
│   │   └── main.py                   # SentinelConsoleApp (QMainWindow)
│   ├── yolo_playground/              # Detección YOLO 3D y guardado de landmarks
│   ├── capra_landolt_ros/            # Detección de anillos de Landolt
│   ├── motion_stability_detector/    # Análisis de estabilidad de movimiento
│   ├── qr_detector/                  # Decodificador QR con filtro de nitidez
│   │   ├── qr_detector/
│   │   │   └── qr_detector_node.py
│   │   └── config/
│   │       └── qr_detector_params.yaml
│   └── thermal_camera/               # Driver TC001 Max con HUD de temperatura
└── README.md
```

---

## 🚀 Uso

### Build

```bash
cd ~/sensors_algorithms_ws
colcon build --symlink-install
source install/setup.bash
```

### Lanzar todo el stack

```bash
ros2 launch camera_bringup full_sensors_algorithms.launch.py
```

### Lanzar solo la consola UI

```bash
ros2 run sentinel_sensor_console main
```

### Lanzar un nodo individual (ejemplo QR)

```bash
ros2 run qr_detector qr_detector_node --ros-args \
  --params-file src/qr_detector/config/qr_detector_params.yaml
```

---

## 📡 Topics Principales

### Imágenes (QoS: BEST\_EFFORT, depth 1)

| Topic | Tipo | Fuente |
|---|---|---|
| `/usb_camera/image_raw` | `sensor_msgs/Image` | Cámara USB frontal |
| `/fisheye/image_raw` | `sensor_msgs/Image` | Cámara Fisheye |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RealSense D455 |
| `/thermal_camera/image_raw` | `sensor_msgs/Image` | TC001 Max |
| `/qr_detector/debug_image` | `sensor_msgs/Image` | QR con anotaciones |
| `/motion_detector/debug_image` | `sensor_msgs/Image` | Motion con anotaciones |
| `/yolo/dbg_image` | `sensor_msgs/Image` | YOLO con bounding boxes |
| `/image` | `sensor_msgs/Image` | Landolt stream |

### Datos / Estado (QoS: RELIABLE, depth 10)

| Topic | Tipo | Descripción |
|---|---|---|
| `/qr_detector/capture_text` | `std_msgs/String` | Texto QR capturado |
| `/qr_detector/sharpness` | `std_msgs/Float32` | Nitidez del frame |
| `/qr_detector/detected` | `std_msgs/Bool` | QR detectado en frame actual |
| `/captured/orientation` | `std_msgs/String` | Orientación Landolt |
| `/motion_detector/is_stable` | `std_msgs/Bool` | Escena estable/en movimiento |
| `/motion_detector/motion_area` | `std_msgs/Float32` | Área de movimiento (%) |
| `/yolo/detections_3d` | `yolo_msgs/DetectionArray` | Detecciones con posición 3D |
| `/magnetometer/heading` | `std_msgs/Float32` | Heading en grados (0–360) |
| `/thermal_camera/temperature_info` | `std_msgs/String` | Métricas de temperatura |

### Comandos UI → ROS2

| Topic | Tipo | Acción |
|---|---|---|
| `/landolt/capture_command` | `std_msgs/Bool` | Solicita captura manual |
| `/system/debug_mode` | `std_msgs/Bool` | Activa modo debug global |
| `/system/restart_cameras` | `std_msgs/Empty` | Reinicia bringup de cámaras |

---

## ⚙️ Configuración DDS (CycloneDDS)

Para streams de imagen de alta resolución es necesario ajustar los buffers UDP:

```bash
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.wmem_max=26214400
```

Archivo de configuración recomendado (`cyclonedds.xml`):

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" priority="default" multicast="default"/>
        <NetworkInterface name="wlan0" priority="default" multicast="default"/>
      </Interfaces>
    </General>
    <Internal>
      <MaxMessageSize>65500B</MaxMessageSize>
      <FragmentSize>4000B</FragmentSize>
    </Internal>
  </Domain>
</CycloneDDS>
```

```bash
export CYCLONEDDS_URI=file:///ruta/a/cyclonedds.xml
```

---

## 🐛 Problemas Conocidos

| Síntoma | Causa | Fix |
|---|---|---|
| `rcl_shutdown already called` en shutdown | `rclpy.shutdown()` llamado cuando el contexto ya fue apagado por SIGINT | Envolver con `if rclpy.ok(): rclpy.shutdown()` |
| `publisher's context is invalid` en logs de cierre | Mismo origen que el anterior | Mismo fix |
| RealSense tarda >5 s en cerrar y recibe SIGTERM | Tiempo de cierre del driver > timeout de launch | Aumentar `sigterm_timeout` en el launch file o ignorar el warning |
| Fisheye sin soporte MJPG en `v4l2_camera` Humble | El paquete oficial no compila el codec MJPG | Usar `format_string: "YUYV"` o compilar `v4l2_camera` desde fuente |

---

## 📋 Notas de Desarrollo

- El bridge ROS2↔PySide6 usa un `Context` dedicado por hilo para evitar conflictos con el contexto global de rclpy.
- Las imágenes se throttlean a **15 FPS** en `ros_qt_bridge.py` antes de emitir a la UI para no saturar el hilo principal de Qt.
- El proceso de launch se inicia con `os.setsid` para poder matar todo el grupo de procesos con una sola señal `SIGINT` vía `os.killpg`.
- `colcon build --symlink-install` permite editar los scripts Python sin rebuilding.
