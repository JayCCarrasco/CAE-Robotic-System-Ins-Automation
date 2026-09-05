#!/bin/bash

WORLD="$HOME/CAE-Robotic-System-Ins-Automation/src_public/balancing_robot/balancing_robot_description/sdf/balancing_robot_test.sdf"

echo "========================================"
echo "BALANCING ROBOT - DATASET V2"
echo "========================================"

# --------------------------------------------------
# 1. Lanzar Gazebo
# --------------------------------------------------

echo "[1/6] Lanzando Gazebo..."

ign gazebo "$WORLD" &
GAZEBO_PID=$!

echo "Esperando 25 segundos a que Gazebo y el robot carguen..."
sleep 25


# --------------------------------------------------
# 2. Lanzar bridge Ignition <-> ROS2
# --------------------------------------------------

echo "[2/6] Lanzando bridge Ignition <-> ROS2..."

ros2 run ros_gz_bridge parameter_bridge \
"/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU" \
"/model/balancing_robot/joint/right_wheel_joint/cmd_force@std_msgs/msg/Float64@ignition.msgs.Double" \
"/model/balancing_robot/joint/left_wheel_joint/cmd_force@std_msgs/msg/Float64@ignition.msgs.Double" \
"/world/balancing_robot_world/model/balancing_robot/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model" &

BRIDGE_PID=$!


# --------------------------------------------------
# 3. Lanzar PID
# --------------------------------------------------

echo "[3/6] Lanzando balancing_robot_pid..."

source install/setup.bash
ros2 run balancing_robot_control balancing_robot_pid &
PID_NODE_PID=$!


# --------------------------------------------------
# 4. Poner Gazebo en PLAY
# --------------------------------------------------

echo "[4/6] Poniendo simulación en PLAY..."

sleep 2

ign service -s /world/balancing_robot_world/control \
    --reqtype ignition.msgs.WorldControl \
    --reptype ignition.msgs.Boolean \
    --timeout 5000 \
    --req 'pause: false'

echo "Simulación en PLAY."


# --------------------------------------------------
# 5. Lanzar DataLogger
# --------------------------------------------------

echo "[5/6] Lanzando DataLogger..."

ros2 run balancing_robot_control data_logger &
LOGGER_PID=$!

echo "Esperando 5 segundos antes de la primera perturbación..."
sleep 5


# --------------------------------------------------
# 6. Perturbaciones
# --------------------------------------------------

echo ""
echo "[6/6] Ejecutando 30 perturbaciones..."
echo ""

CONSECUTIVE=0
SIGN=1

for i in $(seq 1 30)
do

    # Magnitud aleatoria entre 300 y 1500 N
    MAGNITUDE=$((300 + RANDOM % 1201))

    # --------------------------------------------------
    # Generar signo evitando más de 2 consecutivos
    # --------------------------------------------------

    if [ "$i" -eq 1 ]; then

        # Primera perturbación: signo aleatorio
        if (( RANDOM % 2 == 0 )); then
            SIGN=1
        else
            SIGN=-1
        fi

        CONSECUTIVE=1

    else

        if [ "$CONSECUTIVE" -ge 2 ]; then

            # Obligamos a cambiar de sentido
            SIGN=$((-SIGN))
            CONSECUTIVE=1

        else

            # Aleatorio
            if (( RANDOM % 2 == 0 )); then
                NEW_SIGN=1
            else
                NEW_SIGN=-1
            fi

            if [ "$NEW_SIGN" -eq "$SIGN" ]; then
                CONSECUTIVE=$((CONSECUTIVE + 1))
            else
                SIGN=$NEW_SIGN
                CONSECUTIVE=1
            fi

        fi

    fi

    FORCE=$((MAGNITUDE * SIGN))

    echo "Perturbación $i/30 -> ${FORCE} N"

    # --------------------------------------------------
    # Pulso de 10 ms
    # --------------------------------------------------

    START_NS=$(date +%s%N)

    while true
    do

        ign topic \
            -t /world/balancing_robot_world/wrench \
            -m ignition.msgs.EntityWrench \
            -p "entity: {name: \"chassis\", type: LINK}, wrench: {force: {x: ${FORCE}.0, y: 0.0, z: 0.0}}"

        NOW_NS=$(date +%s%N)
        ELAPSED_NS=$((NOW_NS - START_NS))

        if [ "$ELAPSED_NS" -ge 10000000 ]; then
            break
        fi

    done

    # Esperar 5 segundos antes de la siguiente perturbación
    if [ "$i" -lt 30 ]; then
        sleep 5
    fi

done


# --------------------------------------------------
# Recuperación final
# --------------------------------------------------

echo ""
echo "30 perturbaciones completadas."
echo "Esperando 5 segundos para registrar recuperación..."

sleep 5


# --------------------------------------------------
# Cierre
# --------------------------------------------------

echo ""
echo "Deteniendo DataLogger..."
kill "$LOGGER_PID"

sleep 2

echo "Deteniendo PID..."
kill "$PID_NODE_PID"

echo "Deteniendo bridge..."
kill "$BRIDGE_PID"

echo "Deteniendo Gazebo..."
kill "$GAZEBO_PID"

echo ""
echo "========================================"
echo "EXPERIMENTO TERMINADO"
echo "========================================"