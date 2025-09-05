# hikvision_ros2_driver

Este proyecto proporciona un conjunto de nodos ROS2 para la integración y captura de video desde cámaras Hikvision, facilitando la adquisición y procesamiento de imágenes en sistemas robóticos.

## Estructura del proyecto

- `src/`: Contiene los nodos principales del driver, incluyendo:
  - `capture_node.cpp`
  - `driver_node.cpp`
  - `intrinsic_capture_node.cpp`
  - `stereo_capture_node.cpp`
- `launch/`: Incluye archivos de lanzamiento para iniciar los nodos de manera sencilla:
  - `capture.launch.py`
  - `driver.launch.py`

## Descripción de los nodos

- **capture_node**: Sincroniza la captura de imágenes de dos cámaras (escucha en `camera1/image_raw` y `camera2/image_raw`). Al invocar el servicio `/capture_images`, guarda imágenes de ambas cámaras en disco de forma sincronizada.

- **driver_node**: Se conecta a una cámara Hikvision usando el SDK, publica el stream de video en un tópico ROS2 (`sensor_msgs/msg/Image`). No requiere servicios para funcionar, solo publica imágenes en tiempo real.

- **intrinsic_capture_node**: Escucha el stream de una cámara y, al invocar el servicio `/start_intrinsic_capture`, comienza a guardar imágenes a intervalos regulares para calibración intrínseca.

- **stereo_capture_node**: Similar al anterior, pero para dos cámaras (estéreo). Al invocar `/start_stereo_capture` comienza a guardar pares de imágenes a intervalos regulares para calibración estéreo; se detiene con `/stop_stereo_capture`.

## Requisitos previos

- ROS2 instalado y configurado.
- Dependencias de Hikvision ubicadas en `third_party/hikvision/`.

## Ejecución de nodos

Puedes ejecutar los nodos directamente usando ROS2 o mediante los archivos de lanzamiento para una configuración más sencilla. Ten en cuenta que los nodos de captura requieren la invocación de servicios para realizar la captura de imágenes.

### Ejecución directa

Para ejecutar un nodo específico, utiliza el siguiente comando:

```bash
ros2 run hikvision_ros2_driver <nombre_del_nodo>
```

Por ejemplo, para ejecutar el nodo de captura:

```bash
ros2 run hikvision_ros2_driver capture_node
```

#### Invocación de servicios

Una vez iniciado el nodo de captura, debes llamar al servicio `/capture_images` para realizar la captura:

```bash
ros2 service call /capture_images std_srvs/srv/Trigger
```

Para el nodo estéreo, debes iniciar y detener la captura usando los siguientes servicios:

Iniciar captura estéreo:
```bash
ros2 service call /start_stereo_capture std_srvs/srv/Trigger
```

Detener captura estéreo:
```bash
ros2 service call /stop_stereo_capture std_srvs/srv/Trigger
```

### Ejecución con archivos de lanzamiento

Para iniciar los nodos con una configuración predefinida, usa los archivos de lanzamiento:

```bash
ros2 launch hikvision_ros2_driver capture.launch.py
```

o

```bash
ros2 launch hikvision_ros2_driver driver.launch.py
```

Estos archivos configuran automáticamente los parámetros necesarios y permiten iniciar varios nodos simultáneamente.

Recuerda que, tras lanzar los nodos, debes invocar los servicios mencionados para activar la captura de imágenes.

## Notas adicionales

- Revisa el archivo `package.xml` y `CMakeLists.txt` para detalles sobre dependencias y configuración.
- Los logs generados se almacenan en la carpeta `log/`.
