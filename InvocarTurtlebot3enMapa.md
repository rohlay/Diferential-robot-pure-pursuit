Pasos para cargar el turtlebot3 en un mapa vacío y poder teleoperarlo:
-->Instalación de paquetes:
````
sudo apt-get install ros-melodic-dynamixel-sdk
sudo apt-get install ros-melodic-turtlebot3-msgs
sudo apt-get install ros-melodic-turtlebot3
````

-->Elección del modelo por defecto:
````
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
````
-->Instalación de paquete de simulación: (el directorio catkin_ws puede tener otro nombre segun el workspace que hayas creado)
````
cd ~/catkin_ws/src/
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
````

-->Cargar mapa TurtleBot3 World:
````
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
````

--> Para teleoperarlo, en otro terminal:
````
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
````
