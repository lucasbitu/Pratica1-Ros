#!/bin/bash

# MENSAGEM INICIAL
echo "==========================================================="
echo "🤖 Iniciando todos os nós para a Prática 2 - LOCALIZAÇÃO..."
echo "==========================================================="
echo "Compilando o workspace e abrindo todos os terminais."

# Navega para o diretório do workspace
cd ~/workspace

# Garante que o ambiente ROS principal está carregado
source /opt/ros/humble/setup.bash

# Compila o workspace
echo ">>> Compilando o workspace com 'colcon build'..."
colcon build

# Carrega o ambiente do workspace recém-compilado
source install/setup.bash

# Comando de setup a ser executado em cada novo terminal
SETUP_CMD="source ~/workspace/install/setup.bash"

# Abre os 5 terminais básicos da simulação 
gnome-terminal --title="ROBOT DESCRIPTION" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO ROBOT DESCRIPTION ---'; ros2 launch robotics_class robot_description.launch.py; exec bash" &
sleep 2

gnome-terminal --title="SIMULATION WORLD" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO SIMULATION ---'; ros2 launch robotics_class simulation_world.launch.py; exec bash" &
sleep 2

gnome-terminal --title="EKF" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO EKF ---'; ros2 launch robotics_class ekf.launch.py; exec bash" &
sleep 2

gnome-terminal --title="RVIZ" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO RVIZ ---'; ros2 launch robotics_class rviz.launch.py; exec bash" &
sleep 2

gnome-terminal --title="TELEOP KEYBOARD" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO TELEOP (Controle o robô aqui) ---'; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=jetauto/cmd_vel; exec bash" &
sleep 5 # Pausa maior para garantir que a simulação e o robô estejam prontos

# Abre o terminal para o nó de LOCALIZAÇÃO (AMCL + Map Server)
# Este comando substitui os nós de SLAM e Nav2 da Parte A
# Ele passa o caminho do mapa como um argumento, conforme o roteiro 
MAP_PATH="/home/lucasbitu/workspace/src/robotics_class/maps/class_map.yaml"
gnome-terminal --title="LOCALIZATION (AMCL)" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO LOCALIZAÇÃO ---'; ros2 launch robotics_class localization.launch.py map:=$MAP_PATH; exec bash" &

# Abre o terminal para o nó de NAVEGAÇÃO
gnome-terminal --title="NAVIGATION" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO NAVEGAÇÃO ---'; ros2 launch robotics_class navigation.launch.py; exec bash" &

echo "✅ Todos os terminais foram iniciados com sucesso!"