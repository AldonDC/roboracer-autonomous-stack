#!/bin/bash
# ==============================================================================
# 🏎️ ROBO RACER — PREMIUM WORLD SELECTOR
# ==============================================================================
# Autor: Antigravity AI
# Sistema: ROS 2 Jazzy + Gazebo Harmonic
# ==============================================================================

# Colores Pro
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m' # No Color

source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
source install/setup.bash

clear
echo -e "${CYAN}${BOLD}"
echo "  ____       _             ____                          "
echo " |  _ \ ___ | |__   ___   |  _ \ __ _  ___ ___ _ __ ___  "
echo " | |_) / _ \| '_ \ / _ \  | |_) / _\` |/ __/ _ \ '__/ __| "
echo " |  _ < (_) | |_) | (_) | |  _ < (_| | (_|  __/ |  \__ \ "
echo " |_| \_\___/|_.__/ \___/  |_| \_\__,_|\___\___|_|  |___/ "
echo -e "                                         ${YELLOW}[ENGINE v13.0]${NC}"
echo -e "\n${BOLD}Seleccione el escenario de entrenamiento:${NC}"
echo "--------------------------------------------------------"

WORLDS_DIR="src/roboracer/roboracer_gazebo/worlds"
OPTIONS=( $(ls $WORLDS_DIR | grep .sdf) "SALIR" )

select opt in "${OPTIONS[@]}"
do
    case $opt in
        *.sdf)
            echo -e "\n${GREEN}🚀 Iniciando COMPETICIÓN PROFESIONAL con:${NC} ${BOLD}$opt${NC}"
            echo -e "${CYAN}Cargando: Gazebo + RViz + Odom TF + Track Viz + Telemetry + Vision...${NC}\n"
            ros2 launch roboracer_racing competition.launch.py world:=$opt
            break
            ;;
        "SALIR")
            echo -e "${RED}Abortado.${NC}"
            exit 0
            ;;
        *) 
            echo -e "${RED}Opción inválida $REPLY${NC}"
            ;;
    esac
done
