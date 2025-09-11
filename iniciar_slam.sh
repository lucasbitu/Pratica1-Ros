#!/bin/bash

# MENSAGEM INICIAL
echo "======================================================"
echo "🤖 Iniciando todos os nós para a Prática 2 - SLAM..."
echo "======================================================"
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

# Abre um novo terminal para cada comando ROS
# O título (--title) ajuda a identificar cada janela

gnome-terminal --title="ROBOT DESCRIPTION" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO ROBOT DESCRIPTION ---'; ros2 launch robotics_class robot_description.launch.py; exec bash" &
sleep 2 # Pausa para os nós iniciarem

gnome-terminal --title="SIMULATION WORLD" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO SIMULATION ---'; ros2 launch robotics_class simulation_world.launch.py; exec bash" &
sleep 2

gnome-terminal --title="EKF" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO EKF ---'; ros2 launch robotics_class ekf.launch.py; exec bash" &
sleep 2

gnome-terminal --title="RVIZ" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO RVIZ ---'; ros2 launch robotics_class rviz.launch.py; exec bash" &
sleep 2

gnome-terminal --title="TELEOP KEYBOARD" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO TELEOP (Controle o robô aqui) ---'; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=jetauto/cmd_vel; exec bash" &
sleep 5 # Pausa maior para garantir que a simulação e o robô estejam prontos

gnome-terminal --title="NAV2" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO NAVIGATION2 ---'; ros2 launch nav2_bringup navigation_launch.py; exec bash" &
sleep 2

gnome-terminal --title="SLAM TOOLBOX" -- bash -c "$SETUP_CMD; echo '--- LANÇANDO SLAM ---'; ros2 launch robotics_class slam.launch.py; exec bash" &

echo "✅ Todos os terminais foram iniciados com sucesso!"