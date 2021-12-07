# CPR-proyecto
## Proyecto de Control y Programación de Robots utilizando ROS 



Turtlebot3 para ROS-melodic:
````
sudo apt-get install ros-melodic-turtlebot3
````
Paquetes que usaremos:
````
git clone https://github.com/ROBOTIS-GIT/turtlebot3
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations

````
Es necesario cargar el modelo:
````
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
````
Ejemplo de mapa con el robot:
````
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
````

Proyectos en ROS configuración diferencial:

https://github.com/rinoyp/path_tracker_ros  utiliza ROS-noetic...   
https://github.com/merose/diff_drive


Fuentes:

https://github.com/AtsushiSakai/PythonRobotics
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/pure_pursuit/pure_pursuit.py
