#!/bin/bash

# Ejecutar los comandos en paralelo
source install/local_setup.bash

ros2 run tf_transform TF2Read &
pid1=$!
ros2 run tf_transform OdomRead &
pid2=$!
ros2 launch navigation_tb3 mapping.launch.py &
pid3=$!
rviz2 -d src/navigation_tb3/config/mapping.rviz &
pid4=$!

# Esperar a que todos los comandos finalicen
wait $pid1 $pid2 $pid3 $pid4
echo "Todos los nodos han terminado."
