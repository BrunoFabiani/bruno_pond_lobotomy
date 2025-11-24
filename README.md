# nicola_repo

Pra rodar os algoritmos você tem que coloca esta pasta dentro do src do repositorio do culling_games e buildar novamente -->

colcon build --packages-select culling_controller

e então setar o ambiente pra rodar -->

source install/setup.bash

Com isso feito é pra Parte 1 é necessário rodar o comando ros2 run culling_controller path_follower, depois de buildar novamente, em outro terminal com o jogo rodando. Pra parte 2 é necessário os mesmos passo mas no final rodar o comando ros2 run culling_controller path_finder. 
