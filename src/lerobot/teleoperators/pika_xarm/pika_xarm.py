#!/usr/bin/env python

import logging
import os
import sys
import time
import math
import numpy as np
from typing import Any
from threading import Thread, Event, Lock
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from xarm.wrapper import XArmAPI

from lerobot.teleoperators import Teleoperator
from .config_pika_xarm import PikaxArmConfig
from .pika_device import PikaDevice


class Transformations:
    @staticmethod
    def quaternion_to_rotation_matrix(q):
        """
        将四元数转换为旋转矩阵
        
        注: 四元素顺序为xyzw
        """
        norm = np.linalg.norm(q)
        if norm < 1e-6:
            raise ValueError('零四元数无法归一化')

        x, y, z, w = q / norm  # 归一化
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z

        R = np.array([
            [1 - 2 * (yy + zz),     2 * (xy - wz),      2 * (xz + wy)],
            [    2 * (xy + wz), 1 - 2 * (xx + zz),      2 * (yz - wx)],
            [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)]
        ])
        return R

    @staticmethod
    def rpy_to_rotation_matrix(roll, pitch, yaw):
        """RPY角到旋转矩阵的转换"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R = np.array([
            [cp*cy,  -cr*sy + sr*sp*cy,    sr*sy + cr*sp*cy],
            [cp*sy,   cr*cy + sr*sp*sy,   -sr*cy + cr*sp*sy],
            [ -sp,        sr*cp,               cr*cp],
        ])

        return R
    
    @staticmethod
    def rotation_matrix_to_rpy(R, yaw_zero=True):
        """
        旋转矩阵到RPY角的转换
        
        yaw_zero: 万向节锁情况下, True就把yaw置0, False就把roll置0
        返回: roll, pitch, yaw
        """
        epsilon = 1e-6
        if abs(R[2, 0]) > 1 - epsilon: # 万向节锁(pitch=±90°)
            pitch = np.arcsin(-R[2, 0])
            roll_yaw = np.arctan2(-R[0, 1], R[1, 1])
            if yaw_zero:
                # 保留roll, 把yaw置0
                roll, yaw = roll_yaw, 0
            else:
                # 保留yaw, 把roll置0
                roll, yaw = 0, roll_yaw
        else:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arcsin(-R[2, 0])
            yaw = np.arctan2(R[1, 0], R[0, 0])

        return roll, pitch, yaw
    
    @staticmethod
    def rotation_matrix_to_rxryrz(R):
        """
        旋转矩阵到轴角的转换 (rx, ry, rz = aixs * angle)
        返回: rx, ry, rz
        """
        R = np.asarray(R)
        if R.shape[-2:] != (3, 3):
            raise ValueError("Input must be (..., 3, 3)")
        
        # 计算旋转角度 theta
        trace = np.trace(R)
        cos_theta = (trace - 1) / 2.0
        cos_theta = np.clip(cos_theta, -1.0, 1.0)  # 防止数值误差导致 arccos 越界
        theta = np.arccos(cos_theta)
        eps = 1e-8

        # 情况 1: 无旋转 (theta ≈ 0)
        if theta < eps:
            axis = np.array([1.0, 0.0, 0.0])
            return axis * 0.0

        # 情况 2: 旋转角度接近 pi (180 度)
        if np.pi - theta < eps:
            # 此时 sin(theta) ≈ 0，不能用反对称公式
            # 从 R 对角线提取轴：R = I + 2 * (uu^T - I) => uu^T = (R + I)/2
            # 所以 u_i^2 = (R_ii + 1)/2
            diag = np.diag(R)
            axis = np.sqrt(np.maximum(diag + 1, 0))  # 取非负根
            
            # 确定符号：利用非对角元素，例如 R[0,1] = 2*u0*u1
            if axis[0] > eps:
                if R[0, 1] < 0:
                    axis[1] *= -1
                if R[0, 2] < 0:
                    axis[2] *= -1
            elif axis[1] > eps:
                if R[1, 2] < 0:
                    axis[2] *= -1
            # 注意：可能存在符号歧义，但旋转效果相同
            
            axis = axis / np.linalg.norm(axis)
            return axis * theta

        # 情况 3: 一般情况 (0 < theta < pi)
        sin_theta = np.sin(theta)
        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ]) / (2 * sin_theta)

        axis = axis / np.linalg.norm(axis)  # 确保单位长度（数值误差可能破坏）
        return axis * theta


class PikaxArm(Teleoperator, Thread):
    
    config_class = PikaxArmConfig
    name = "pika_teleop"

    def __init__(self, config: PikaxArmConfig):
        
        super().__init__(config)
        Thread.__init__(self) # Do NOT REMOVE!
        self.stop_event = Event()
        self.config = config
        self.frequency = config.frequency
        self._is_connected = False
        self._is_calibrated = True
        self._data_lock = Lock()
        self._ctrl_flag = False
        self._need_initial = False

        self.pika_device = PikaDevice(1, pika_sense_port=self.config.port)
        self.pika_sense = self.pika_device.pika_sense

        # from pika.sense import Sense as PikaSense
        # self.pika_sense = PikaSense(port=self.config.port)
        # if not self.pika_sense.connect():
        #     raise ConnectionError(f"{self} oika sense connect failure")
        # tracker = self.pika_sense.get_vive_tracker()
        # if not tracker:
        #     print('Vive Tracker初始化失败')
        #     self.pika_sense.disconnect()
        #     exit(1)
        # print('Vive Tracker初始化成功')
        # time.sleep(2)

        # devices = self.pika_sense.get_tracker_devices()
        # if not devices:
        #     print('未检测到Vive Tracker设备')
        #     self.pika_sense.disconnect()
        #     exit(1)
        # print('检测到Vive Tracker设备: {}'.format(devices))

        # self.target_device = None
        # for device in devices:
        #     if device.startswith('WM'):
        #         self.target_device = device
        #         break
        # else:
        #     self.target_device = devices[0]
        # print('开始跟踪设备: {}\n'.format(self.target_device))

        self.arm = XArmAPI(self.config.robot_ip, is_radian=True)

        self._robot_target_pose = None
        self._gripper_target_pos = None

    @property
    def action_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (7,),
                "names": {"pose.x": 0, "pose.y": 1, "pose.z": 2, "pose.rx": 3, "pose.ry": 4, "pose.rz": 5, "gripper.pos": 6},
            }
        else:
            return {
                "dtype": "float32",
                "shape": (6,),
                "names": {"pose.x": 0, "pose.y": 1, "pose.z": 2, "pose.rx": 3, "pose.ry": 4, "pose.rz": 5},
            }

    @property
    def feedback_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (7,),
                "names": {"pose.x": 0, "pose.y": 1, "pose.z": 2, "pose.rx": 3, "pose.ry": 4, "pose.rz": 5, "gripper.pos": 6},
            }
        else:
            return {
                "dtype": "float32",
                "shape": (6,),
                "names": {"pose.x": 0, "pose.y": 1, "pose.z": 2, "pose.rx": 3, "pose.ry": 4, "pose.rz": 5},
            }

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    def calibrate(self) -> None:
        # CHECK!!
        pass

    def configure(self) -> None:
        pass

    def connect(self, calibrate: bool = False) -> None:
        self.start()

    def disconnect(self):
        if not self._is_connected:
            DeviceNotConnectedError(f"{self} is not connected.")

        self.stop_event.set()
        self._is_connected = False
        self.join()
    
    @staticmethod
    def xyzq_to_rotation_matrix(x, y, z, q):
        T = np.eye(4)
        T[:3, :3] = Transformations.quaternion_to_rotation_matrix(q)
        T[:3, 3] = [x, y, z]
        return T

    @staticmethod
    def xyzrpy_to_rotation_matrix(x, y, z, roll, pitch, yaw):
        """构造4x4齐次变换矩阵"""
        T = np.eye(4)
        T[:3, :3] = Transformations.rpy_to_rotation_matrix(roll, pitch, yaw)
        T[:3, 3] = [x, y, z]
        return T

    @staticmethod
    def rotation_matrix_to_xyzrpy(rotation_matrix):
        """从4x4齐次变换矩阵到xyzrpy的转换"""
        x, y, z = rotation_matrix[0, 3], rotation_matrix[1, 3], rotation_matrix[2, 3]
        roll, pitch, yaw = Transformations.rotation_matrix_to_rpy(rotation_matrix)
        return [x, y, z, roll, pitch, yaw]
    
    @staticmethod
    def rotation_matrix_to_xyzrxryrz(rotation_matrix):
        """从4x4齐次变换矩阵到xyzrxryrz的转换"""
        x, y, z = rotation_matrix[0, 3], rotation_matrix[1, 3], rotation_matrix[2, 3]
        rx, ry, rz = Transformations.rotation_matrix_to_rxryrz(rotation_matrix[:3,:3])
        return [x, y, z, rx, ry, rz]

    def pika_pose_to_robot_matrix(self, x, y, z, q, pika_to_robot_matrix):
        # pika位置对应的变换矩阵
        pika_matrix = self.xyzq_to_rotation_matrix(x, y, z, q)
        # pika位置转换到机械臂坐标系后对应的变换矩阵
        robot_matrix = np.dot(pika_matrix, pika_to_robot_matrix)
        return robot_matrix
    
    def pika_robot_matrix_to_robot_pose(self, pika_begin_robot_matrix, pika_end_robot_matrix, robot_base_matrix, is_axis_angle=False):
        # 机械臂目标位置对应的变换矩阵
        robot_martix = np.dot(robot_base_matrix, np.dot(np.linalg.inv(pika_begin_robot_matrix), pika_end_robot_matrix))
        if is_axis_angle:
            return self.rotation_matrix_to_xyzrxryrz(robot_martix)
        else:
            return self.rotation_matrix_to_xyzrpy(robot_martix)
    
    def set_ctrl_status(self, status):
        if status:
            if not self._ctrl_flag:
                print('开始遥操作')
                self._ctrl_flag = True
                self._need_initial = True
        else:
            self._ctrl_flag = False
            self._need_initial = False
            print('停止遥操作')

    def run(self):
        self._is_connected = True
        init_state = self.pika_sense.get_command_state()
        curr_state = init_state

        last_gripper_distance = 0

        self._ctrl_flag = False # 是否开启遥操作
        self._need_initial = False

        sleep_time = 1 / self.config.frequency

        self.arm.set_linear_spd_limit_factor(2.0)

        pika_to_robot_eef = [0, 0, 0, math.pi, -math.pi / 2, 0] # rpy
        # pika_to_robot_eef = [0, 0, 0, math.pi, 0, 0]

        # pika坐标系到机械臂坐标系的变换关系对应的变换矩阵
        pika_to_robot_matrix = self.xyzrpy_to_rotation_matrix(*pika_to_robot_eef)
        # 机械臂初始位置对应的变换矩阵
        robot_base_matrix = None
        # pika初始位置转换到机械臂坐标系后对应的变换矩阵
        pika_begin_robot_matrix = None
        # pika目标位置转换到机械臂坐标系后对应的变换矩阵
        pika_end_robot_matrix = None

        scale_xyz = self.config.scale_xyz

        while not self.stop_event.is_set():
            time.sleep(sleep_time)

            state = self.pika_sense.get_command_state()
            if state != curr_state:
                curr_state = state
                if not self._ctrl_flag and curr_state != init_state:
                    self._ctrl_flag = True
                    self._need_initial = True
                    # self.robot_init()
                    print('开始遥操作')
                    time.sleep(1)
                elif self._ctrl_flag and curr_state == init_state:
                    self._ctrl_flag = False
                    print('停止遥操作')
                    continue
            
            if self._ctrl_flag and (not self.arm.connected or self.arm.error_code != 0 or self.arm.state >= 4):
                print('机械臂原因, 遥操作自动停止')
                init_state = state
                curr_state = state
                self._ctrl_flag = False
                continue
            
            if not self._ctrl_flag:
                continue

            if self.config.use_gripper:
                distance  = min(max(self.pika_sense.get_gripper_distance(), 0), 100)

                if abs(last_gripper_distance - distance) > 2:
                    last_gripper_distance = distance
                    with self._data_lock:
                        self._gripper_target_pos = last_gripper_distance
                    # self.set_gripper_position(distance)

            pose = self.pika_sense.get_pose(self.pika_device.pika_tracker_device)
            if not pose:
                continue
            x, y, z = pose.position[0] * 1000 * scale_xyz, pose.position[1] * 1000 * scale_xyz, pose.position[2] * 1000 * scale_xyz

            if self._need_initial:
                self._need_initial = False
                # _, robot_pos = self.arm.get_position()
                _, robot_pos = self.arm.get_position(is_radian=True)
                robot_base_pose = robot_pos
                print('[初始] 机械臂位置: {}'.format(robot_pos))

                # 机械臂初始位置对应的变换矩阵
                robot_base_matrix = self.xyzrpy_to_rotation_matrix(*robot_pos)

                # pika初始位置转换到机械臂坐标系后对应的变换矩阵
                pika_begin_robot_matrix = self.pika_pose_to_robot_matrix(x, y, z, pose.rotation, pika_to_robot_matrix)
                pika_end_robot_matrix = pika_begin_robot_matrix
            else:
                # pika目标位置转换到机械臂坐标系后对应的变换矩阵
                pika_end_robot_matrix = self.pika_pose_to_robot_matrix(x, y, z, pose.rotation, pika_to_robot_matrix)

            robot_target_pose = self.pika_robot_matrix_to_robot_pose(pika_begin_robot_matrix, pika_end_robot_matrix, robot_base_matrix, is_axis_angle=True)
            # self.set_robot_position(robot_target_pose)
            with self._data_lock:
                self._robot_target_pose = robot_target_pose

    # delta action
    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "PikaTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

        with self._data_lock:
            if self._robot_target_pose is not None:
                robot_target_pose = self._robot_target_pose.copy()
            else:
                robot_target_pose = None
            if self._gripper_target_pos is not None:
                gripper_target_pos = (100 - self._gripper_target_pos) / (100 - 0)
            else:
                gripper_target_pos = 0.0

        if robot_target_pose is None:
            _, robot_target_pose = self.arm.get_position_aa(is_radian=True)
        # print(self._robot_target_pose, robot_target_pose)

        # output is delta change of the robot pose
        action_dict = {
            "pose.x": robot_target_pose[0],
            "pose.y": robot_target_pose[1],
            "pose.z": robot_target_pose[2],
            "pose.rx": robot_target_pose[3],
            "pose.ry": robot_target_pose[4],
            "pose.rz": robot_target_pose[5],
        }

        if self.config.rx_continuous and action_dict["pose.rx"] < 0:
            action_dict["pose.rx"] += math.pi * 2

        if self.config.use_gripper:
            action_dict.update({"gripper.pos": gripper_target_pos})

        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError