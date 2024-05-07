Albert Ardiaca Tent - ardiacaalbert01
Marc Turmo Sabater

El projecte turtle per ser executat s'ha de fer en un entorn funcional de ros.
Previament a l'execució, s'ha d'executar la comanda . install/setup_bash en les terminals on s'executaran els nodes. 
Posteriorment s'ha d'executar el colcon build - colcon build --packages-select projecte_turtle

S'han de obrir tres terminals. 
  1. Obrir turtlesim - ros2 run turtlesim turtlesim_node 
  2. Executar controller_tortuga - ros2 run projecte_turtle controller_tortuga
  3. Executar Paisatge_paint - ros2 run projecte_turtle paisatge_paint

