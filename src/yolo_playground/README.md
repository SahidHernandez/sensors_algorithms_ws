# yolo_playground

Paquete auxiliar para trabajar con `yolo_ros` dentro de este workspace.

## Estructura

- `launch/`: launch files propios.
- `config/`: configuraciones YAML para tus despliegues.
- `models/`: modelos `.pt` o trackers locales.
- `yolo_playground/`: nodos Python personalizados.

## Uso

1. Coloca tus modelos en `src/yolo_playground/models/`.
2. Edita `src/yolo_playground/config/yolo_params.yaml`.
3. Compila el workspace.
4. Lanza YOLO con:

```bash
ros2 launch yolo_playground yolo_playground.launch.py
```

## Notas

- El repositorio upstream quedó en `src/yolo_ros`.
- Si usas un modelo local, en el YAML puedes poner por ejemplo `model: models/mi_modelo.pt`.
- `yolo_ros` usa `uv` para preparar sus dependencias Python en tiempo de lanzamiento.
