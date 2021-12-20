Carpeta para realizar el interpolador, y unir el planificador con el planificador PID del nodo diff-drive-go-to-goal

-Paquete interpolador: https://github.com/gkouros/path-smoothing-ros

- Planificador + PID (turtleBot) -> hecho
- Interpolador + ir pasando WP al control -> nosotros
- Control pure pursuit -> ellos

Planificador -> interpolador -> ir pasando WP al control de uno en uno -> pure pursuit (publica /cmd_vel)


Diff_drive_go_to_goal:
	-"distance_to_goal" establecer un umbral a partir del cual darle el siguiente WP
	-Línea 47 "PoseStamped" way point que se pasa al control
	-En "on_goal" separar WP que vienen todos juntos y ver si se puede poner la condición de 	 la "d" ahí también

Planner:
	-El primer punto que llega del planner es el punto destino (último)
	-Línea 295 del planner es la línea correcta de esa parte
	-Publica todo del tirón, hay que ver cómo pasarlo de 1 en 1
