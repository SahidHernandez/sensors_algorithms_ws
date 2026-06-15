# fisheye_camera

Paquete ROS2 Humble para bringup de cámara USB fisheye vía `v4l2_camera`,
con calibración y nodo de rectificación `image_proc`.

## Estructura

```
fisheye_camera/
├── camera_info/
│   └── fisheye_camera.yaml       # Calibración (placeholder — reemplazar)
├── config/
│   └── camera_params.yaml        # Parámetros del nodo v4l2
├── launch/
│   └── fisheye_camera.launch.py  # Launch principal
├── CMakeLists.txt
└── package.xml
```

## Build

```bash
cd ~/your_ws
colcon build --packages-select fisheye_camera --symlink-install
source install/setup.bash
```

## Uso

```bash
# Lanzar con valores por defecto
ros2 launch fisheye_camera fisheye_camera.launch.py

# Cambiar dispositivo o resolución desde CLI
ros2 launch fisheye_camera fisheye_camera.launch.py \
  video_device:=/dev/video2 \
  image_width:=1280 \
  image_height:=720
```

## Tópicos publicados

| Tópico                    | Tipo                          | Descripción                     |
|---------------------------|-------------------------------|---------------------------------|
| `/fisheye/image_raw`      | `sensor_msgs/Image`           | Imagen cruda de la cámara       |
| `/fisheye/camera_info`    | `sensor_msgs/CameraInfo`      | Parámetros de calibración       |
| `/fisheye/image_rect`     | `sensor_msgs/Image`           | Imagen rectificada (image_proc) |

## Calibración real

```bash
# Con un tablero de ajedrez 8x6, cuadros de 25mm
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args -r image:=/fisheye/image_raw -r camera:=/fisheye
```

Al terminar, copia el YAML generado a `camera_info/fisheye_camera.yaml`
y reconstruye el paquete.

> **Nota:** Si la lente tiene FOV > 120°, el modelo `plumb_bob` no es suficiente.
> Considera usar el modelo `equidistant` (Kannala-Brandt) con un paquete
> especializado como `fisheye_undistort` o `image_proc` con OpenCV fisheye.
