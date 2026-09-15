#!/usr/bin/env python3

import json
import math

import numpy as np
import torch
import torch.nn as nn

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


# ============================================================
# RED NEURONAL
# ============================================================

class BalancingNetwork(nn.Module):

    def __init__(self):
        super().__init__()

        self.network = nn.Sequential(
            nn.Linear(3, 32),
            nn.ReLU(),

            nn.Linear(32, 32),
            nn.ReLU(),

            nn.Linear(32, 1)
        )

    def forward(self, x):
        return self.network(x)


# ============================================================
# CONTROLADOR
# ============================================================

class BalancingRobotNN(Node):

    def __init__(self):

        super().__init__("balancing_robot_nn")

        # ----------------------------------------------------
        # Parámetros
        # ----------------------------------------------------

        self.declare_parameter(
            "control_frequency",
            200.0
        )

        self.declare_parameter(
            "model_file",
            "/home/juancarlos/CAE-Robotic-System-Ins-Automation/src_public/balancing_robot/balancing_robot_ai/models/balancing_network_v1.pth"
        )

        self.declare_parameter(
            "normalization_file",
            "/home/juancarlos/CAE-Robotic-System-Ins-Automation/src_public/balancing_robot/balancing_robot_ai/models/normalization_v1.json"
        )

        self.control_frequency_ = self.get_parameter(
            "control_frequency"
        ).value

        self.model_file_ = self.get_parameter(
            "model_file"
        ).value

        self.normalization_file_ = self.get_parameter(
            "normalization_file"
        ).value

        self.dt_ = 1.0 / self.control_frequency_

        self.Fmax_ = 20.0

        # ----------------------------------------------------
        # Estado
        # ----------------------------------------------------

        self.theta_ = 0.0
        self.theta_dot_ = 0.0
        self.x_dot_ = 0.0

        self.imu_received_ = False
        self.joint_received_ = False

        # ----------------------------------------------------
        # Cargar normalización
        # ----------------------------------------------------

        self.load_normalization()

        # ----------------------------------------------------
        # Cargar red
        # ----------------------------------------------------

        self.device_ = torch.device("cpu")

        self.model_ = BalancingNetwork()

        state_dict = torch.load(
            self.model_file_,
            map_location=self.device_
        )

        self.model_.load_state_dict(state_dict)

        self.model_.eval()

        self.get_logger().info(
            "Neural network loaded successfully"
        )

        self.get_logger().info(
            f"Model: {self.model_file_}"
        )

        self.get_logger().info(
            f"Normalization: {self.normalization_file_}"
        )

        # ----------------------------------------------------
        # Subscribers
        # ----------------------------------------------------

        self.imu_sub_ = self.create_subscription(
            Imu,
            "/imu",
            self.imu_callback,
            10
        )

        self.joint_sub_ = self.create_subscription(
            JointState,
            "/world/balancing_robot_world/model/"
            "balancing_robot/joint_state",
            self.joint_callback,
            10
        )

        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------

        self.left_wheel_pub_ = self.create_publisher(
            Float64,
            "/model/balancing_robot/joint/"
            "left_wheel_joint/cmd_force",
            10
        )

        self.right_wheel_pub_ = self.create_publisher(
            Float64,
            "/model/balancing_robot/joint/"
            "right_wheel_joint/cmd_force",
            10
        )

        self.effort_pub_ = self.create_publisher(
            Float64,
            "/balancing_robot/effort",
            10
        )

        # ----------------------------------------------------
        # Timer
        # ----------------------------------------------------

        self.timer_ = self.create_timer(
            self.dt_,
            self.control_loop
        )

        self.get_logger().info(
            "Balancing robot NN node initialized"
        )

    # ========================================================
    # NORMALIZACIÓN
    # ========================================================

    def load_normalization(self):

        with open(
            self.normalization_file_,
            "r"
        ) as f:

            normalization = json.load(f)

        self.X_mean_ = np.array(
            normalization["X_mean"],
            dtype=np.float32
        )

        self.X_std_ = np.array(
            normalization["X_std"],
            dtype=np.float32
        )

        self.y_mean_ = float(
            normalization["y_mean"][0]
        )

        self.y_std_ = float(
            normalization["y_std"][0]
        )

        self.get_logger().info(
            f"X_mean = {self.X_mean_}"
        )

        self.get_logger().info(
            f"X_std  = {self.X_std_}"
        )

        self.get_logger().info(
            f"y_mean = {self.y_mean_}"
        )

        self.get_logger().info(
            f"y_std  = {self.y_std_}"
        )

    # ========================================================
    # IMU
    # ========================================================

    def imu_callback(self, msg):

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        self.theta_ = math.atan2(
            2.0 * (
                qw * qy +
                qx * qz
            ),
            1.0 - 2.0 * (
                qy * qy +
                qx * qx
            )
        )

        self.theta_dot_ = msg.angular_velocity.y

        self.imu_received_ = True

    # ========================================================
    # JOINT STATES
    # ========================================================

    def joint_callback(self, msg):

        omega_left = 0.0
        omega_right = 0.0

        for i, name in enumerate(msg.name):

            if name == "left_wheel_joint":
                omega_left = msg.velocity[i]

            elif name == "right_wheel_joint":
                omega_right = msg.velocity[i]

        # Exactamente igual que en el PID
        r = 0.05

        self.x_dot_ = (
            r *
            (omega_left + omega_right) /
            2.0
        )

        self.joint_received_ = True

    # ========================================================
    # INFERENCIA
    # ========================================================

    def predict_effort(self):

        # ----------------------------------------------------
        # Entradas
        # ----------------------------------------------------

        X = np.array(
            [
                self.theta_,
                self.theta_dot_,
                self.x_dot_
            ],
            dtype=np.float32
        )

        # ----------------------------------------------------
        # Normalización
        # ----------------------------------------------------

        X_normalized = (
            X - self.X_mean_
        ) / self.X_std_

        # ----------------------------------------------------
        # Tensor
        # ----------------------------------------------------

        X_tensor = torch.tensor(
            X_normalized,
            dtype=torch.float32,
            device=self.device_
        ).unsqueeze(0)

        # ----------------------------------------------------
        # Inferencia
        # ----------------------------------------------------

        with torch.no_grad():

            y_normalized = self.model_(
                X_tensor
            )

        # ----------------------------------------------------
        # Desnormalización
        # ----------------------------------------------------

        effort = (
            y_normalized.item() *
            self.y_std_
            +
            self.y_mean_
        )

        return effort

    # ========================================================
    # CONTROL LOOP
    # ========================================================

    def control_loop(self):

        # ----------------------------------------------------
        # Seguridad:
        # no controlar hasta recibir sensores
        # ----------------------------------------------------

        if not self.imu_received_:
            return

        if not self.joint_received_:
            return

        # ----------------------------------------------------
        # Inferencia
        # ----------------------------------------------------

        effort = self.predict_effort()

        # ----------------------------------------------------
        # Saturación
        # ----------------------------------------------------

        effort = max(
            -self.Fmax_,
            min(self.Fmax_, effort)
        )

        # ----------------------------------------------------
        # Publicar
        # ----------------------------------------------------

        msg = Float64()
        msg.data = effort

        self.left_wheel_pub_.publish(msg)
        self.right_wheel_pub_.publish(msg)
        self.effort_pub_.publish(msg)


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = BalancingRobotNN()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()