# CPR-proyecto
## Proyecto de Control y Programación de Robots utilizando ROS                               
Robot móvil con configuración diferencial, algoritmo pure pursuit                


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


--PLANIFICACIÓN--       
En orden de prioridad

* Probar con el control simplificado, es decir, no utilizar el control de bajo nivel y utilizar la odometría directamente. 
  Odometría devuelve las posiciones (x,y,z) orientaciones (r,p,y) velocidades lineal y angular en x,y,z. 
  Utilizar el PID para la prueba, le pasaremos como referencia la posición y saca como salida las velocidades del robot. El objetivo de esta parte
  es ver que las velocidades se envían correctamente al robot, y así poder visualizar en el Gazebo el movimiento del robot.
  Problema: ¿Como pasar la referencia? ¿En qué fromato? (a través de Goal)
  
* Configurar el repositorio de manera que podamos todos hacer pull y push y trabajar cómodamente.
  
  Estos dos se pueden resolver en paralelo:
* Una vez consigamos enviar las velocidades al robot a partir de la posición deseada, ver como se implementaría un planificador de trayectorias.
  Problema: Planificador de trayectorias
  
* Cambiar el control PID por el control pure-pursuit. El bloque funcional es el mismo, es decir, las entradas y salidas son las mismas, solo
  es sustituir un bloque por otro.
  Problema: Algoritmo Pure-Pursuit
 
* Descartamos el control de bajo nivel por simplicidad y para centrarnos en que funcione el sistema global. Una vez todo funcione ver cómo
  implementar el control de bajo nivel o ver si merece la pena "por completidad del sistema" en lugar de utilizar una función de alto nivel de Gazebo.
  
  
  -- Proyecto finalizado--
  
 Extras:
 * Evitación de obstáculos. LIDAR
 * GPS. Filtro de Kalman




Fuentes:

https://github.com/AtsushiSakai/PythonRobotics
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/pure_pursuit/pure_pursuit.py
