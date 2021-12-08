


### Juntando el turtlebot3 con diff_drive:
- La carpeta turtlebot3_simulations tiene que estar dentro de la carpeta diff_drive

- Se copió el código desde la capteta turtlebot3 > turtlebot3_description > urdf > turtlebot3_burguer_urdf.xacro
  y se pegó en diff_drive > urdf > diff_drive_description.xacro
  Esto consigue que los /joint_states del robot sean los mismos que los del control
  
  
- Se copió el código del launch de turtlebot3_simulations > turtlebot3_gazebo > launch > turtlebot3_empty_world.launch
    al launch de diff_drive > demo.launch
    Esto lo modificaremos si queremos cambiar de mapa
    
- Se comentó el remapping de /robot/vel_cmd a /vel_cmd, así el tópico se publica al robot.

### Prueba con el robot con un controlador PID, mandando la referencia a través de Rviz

- Se ha modificado el demo.launch para eliminar de momento la parte de control de bajo nivel del sistema, para posteriormente probar exitosamente el seguimiento del objetivo mediante 2dnavgoal con el control ya existente en el nodo diff_drive_goto_goal

- Se ha guardado en el branch 'main' la configuración de rviz para probar esto y que se visualice, ademas he añadido que esta configuración se cargue directamente con el demo.launch 

### Se ha incluido el planificador basado en el nodo move_base

- Falta retocar cositas en cuanto a las conexiones concretas de cmd_vel
- Detalles de como darle el "goal" al nodo diff_drive_goto_goal sin "romper" el planificador
- Parece coherente darle los waypoints mediante move_base/TrajectoryPlannerROS/global_path, pero no funciona.
- Esto último se ha dejado asi pero para volver a tenerlo funcional aunque sea con el punto final como goal habria que descomentar la linea del nodo diff_drive_goto_goal
