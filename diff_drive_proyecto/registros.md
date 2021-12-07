


## Juntando el turtlebot3 con diff_drive:
- La carpeta turtlebot3_simulations tiene que estar dentro de la carpeta diff_drive

- Se copió desde la capteta turtlebot3 > turtlebot3_description > urdf > turtlebot3_burguer_urdf.xacro
  y se pegó en diff_drive > urdf > diff_drive_description.xacro
  Esto consigue que los /joint_states del robot sean los mismos que los del control
  
  
- Se copió el launch de turtlebot3_simulations > turtlebot3_gazebo > launch > turtlebot3_empty_world.launch
    al launch de diff_drive > demo.launch
    Esto lo modificaremos si queremos cambiar de mapa
