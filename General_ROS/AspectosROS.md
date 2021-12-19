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
git clone https://github.com/ros-planning/navigation

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

Planificador:
https://husarion.com/tutorials/ros-tutorials/7-path-planning

En general, el único problema que hay que resolver es configurar bien los launch, es decir, que los nodos estén suscritos y que publiquen a donde es necesario, y
que se manden y reciban los mensajes en el formato adecuado. Mirar rostopic. Especial atencion a estas cosas. Partiendo de código ya hecho, nuestro objetivo solo es unir todos los elementos y modificarlo correctamente, manteniendo coherencia entre los programas. Creo que deberíamos anotar todos los cambios que hagamos (que serán pocos pero muy importantes), por si empieza a fallar y tenemos que buscar el error, que sino será imposible, y también para que podamos trabajar todos. Por ejemplo, podríamos tener un readme para cada parte donde se registran los cambios. 



## Comunicación planificador con el controlador
El planificador publica un tópico con mensajes de tipo /nav_msgs/Path

Enlaces para ayudar resolver:

formato PoseStamped:
https://docs.freedomrobotics.ai/docs/send-waypoint-and-path-commands

frame_id: "map" "base_link"
http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF


3 ENLACES MUY INTERESANTES
## Waypoints
https://github.com/asinghani/pathfinder_ros
## Planificador
https://github.com/robotics-upo/Heuristic_path_planners
## Pure-pursuit
https://github.com/linklab-uva/pure_pursuit_controller

### ROS2 (galactic)
https://github.com/ros-planning/navigation2/tree/main/nav2_regulated_pure_pursuit_controller     
https://github.com/ros-planning/navigation2/tree/main/nav2_theta_star_planner


Fuentes:

https://github.com/AtsushiSakai/PythonRobotics
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/pure_pursuit/pure_pursuit.py
https://github.com/FernandoDorado/A-STAR-search-algorithm-implemented-in-ROS
https://husarion.com/tutorials/ros-tutorials/7-path-planning

http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS


Artículo interesante:
https://titanwolf.org/Network/Articles/Article?AID=df72bb4a-e259-4e1e-84a3-f36449de0959

ROS nodos C++:
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

NODO de ROS para mandar una secuencia de waypoints, ¿?,quizá funciona junto al planificador?, si no puede servir para probar el pure pursuit dando
la secuencia de puntos a mano, y ya luego si se puede, hacer el planificador:
https://github.com/rfzeg/navi_goals

actionlib (usado por el package navi_goals):
http://docs.ros.org/en/jade/api/actionlib/html/classactionlib_1_1SimpleActionClient.html#ae6a2e6904495e7c20c59e96af0d86801

--Posible candidato para el interpolador:
https://github.com/gkouros/path-smoothing-ros
