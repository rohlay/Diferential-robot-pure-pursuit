--PLANIFICACIÓN--       
En orden de prioridad

* Probar con el control simplificado, es decir, no utilizar el control de bajo nivel y utilizar la odometría directamente. 
  Odometría devuelve las posiciones (x,y,z) orientaciones (r,p,y) velocidades lineal y angular en x,y,z. 
  Utilizar el PID para la prueba, le pasaremos como referencia la posición y saca como salida las velocidades del robot. El objetivo de esta parte
  es ver que las velocidades se envían correctamente al robot, y así poder visualizar en Gazebo el movimiento del robot.
  Objetivo: ¿Como pasar la referencia? ¿En qué formato? (a través de Goal)
  
  (JAVI)-- Es necesario mediante Rviz hacer un estimate pose justo encima del TurtleBot3 para que guarde su pose inicial, posteriormente, con 2dNavGoal se comprueba que el robot se dirige correctamente al waypoint dado-- Ahora lo que veo importante es ver qué realiza el 2dnavgoal, en el sentido de que tipo de mensajes de ROS manda y donde los manda, para ver si sólo manda el pose final, o manda una serie de waypoints por medio.
  Supongo que si se mete el nodo que haga de planificador y publique algo similar a los mensajes de 2dNavGoal, el robot seguiría dicha trayectoria.
  
* Configurar el repositorio de manera que podamos todos hacer pull y push para trabajar cómodamente.
  
  Estos dos se pueden resolver en paralelo:
* Una vez consigamos enviar las velocidades al robot a partir de la posición deseada, ver como se implementaría un planificador de trayectorias.
  Objetivo: Planificador de trayectorias
  
* Cambiar el control PID por el control pure-pursuit. El bloque funcional es el mismo, es decir, las entradas y salidas son las mismas, solo
  es sustituir un bloque por otro. Podemos partir de purepursuit.py o buscar otro, el problema es hacer las modificaciones bien para que 
  se mantenga la consistencia/estructura como bloque funcional (respecto al anterior que sí funciona). 
  Objetivo: Algoritmo Pure-Pursuit
 
* Descartamos el control de bajo nivel por simplicidad para centrarnos en que funcione el sistema global. Una vez todo funcione ver cómo
  implementar el control de bajo nivel o ver si merece la pena "por completidud del sistema" en lugar de utilizar una función de alto nivel de ROS. 
  ¿Qué opina Ramiro?
  
  
  -- Proyecto finalizado--
  
 Extras:
 * Evitación de obstáculos. LIDAR
 * GPS. Filtro de Kalman
