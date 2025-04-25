# Robotica Móvil

## Pasos para utilizar GitHub:

```bash
cd workspace/src/
git clone https://github.com/RobotJuan/Robotica-Movil
cd ..
colcon build --symlink-install
source install/local_setup.bash
```

## Pasos para actualizar el GitHub:

```bash
cd workspace/src/Robotica-Movil
git pull origin main
cd ../..
colcon build --symlink-install
source install/local_setup.bash
```

## Pasos para agregar un archivo:

1. Agregar `archivo.py` en la carpeta `nodes`.  
   Asegurarse de que la línea 1 sea explícitamente:  
   ```python
   #!/usr/bin/env python3
   #
   ```
2. Modificar `CMakeLists.txt` para incluir su archivo en la línea 19.

## Pasos para agregar un launch:

Agregar `launch.xml` en la carpeta `launch`.

## Cómo correr un nodo:

```bash
$ ros2 run robotica_movil archivo.py
```

## Cómo lanzar un archivo launch:

```bash
$ ros2 launch robotica_movil launch.xml
```
