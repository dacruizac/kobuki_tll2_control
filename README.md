# kobuki_tll2_control

En este paquete se implementan nodos con los cuales se hace un control del robot kobuki para que siga una trayectoria predeterminada haciendo uso de controladores proporcionales.

* Utilizando el archivo <code> taller_launch_world.launch</code>, usted puede abrir gazebo utilizando el mundo diseñado para este taller.

* Utilizando el archivo <code> spawn_tll2_kobuki.launch</code>, usted agrega el robot kobuki al mundo virtual de gazebo, e inicia el nodo con el cual se va a realizar el control, que se encuentra en el archivo <code> tll2_control_node.py </code>.

* Existen dos formas de realizar el recorrido, primero girando el robot y luego moviendose en linea recta de un punto a otro o realizando un control de sobre la velocidad lineal y angular al mismo tiempo. Por defecto el metodo predeterminado es el primero

* Si desea utilizar la segunda forma de realizar el recorrido puede publicar en el topico <code>tll2/control_method</code> el valor 1. Si desea cambiar al primer metodo publique un 1 en el mismo topico.

* Para iniciar el seguimiento publique en el topico <code>tll2/begin_control</code> el valor true.

* Despues de iniciado el proceso el nodo no se detendra a menos que usted mate el nodo, o que ya se haya terminado el recorrido