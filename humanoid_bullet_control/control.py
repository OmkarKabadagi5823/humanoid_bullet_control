from human_voxelpose_model import HumanPoseModel
import pybullet as pyb
from scipy.spatial.transform import Rotation
from math import pi

import pprint


class HumanoidControl():
    def __init__(self, bullet_client_id):
        self.bid = bullet_client_id
        self.__init_params()

    def __init_params(self):
        self.humanoid_id = None
        self.hpm = HumanPoseModel()
        self.joint_set = None
        self.offset_rot = Rotation.from_euler('xyz', [pi/2, 0, 0])
        self.multidof_joint_rot = {}
        self.revolute_joint_angle = {}

    def set_initial_joints(self, joint_set):
        self.joint_set = joint_set
        self.hpm.update(self.joint_set)

    def load_model(self, model_file):
        self.humanoid_id = pyb.loadURDF(
            model_file,
            basePosition=self.joint_set[0],
            baseOrientation=(self.hpm.rot['w|r']*self.offset_rot).as_quat(),
            globalScaling=0.2,
            flags=pyb.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self.bid
        )

        self.constraint_id = pyb.createConstraint(
            self.humanoid_id, 
            -1, -1, -1, 
            pyb.JOINT_FIXED, 
            [0, 0, 0], 
            [0, 0, 0], 
            [0, 0, 1],
            physicsClientId=self.bid
        )

        pyb.changeConstraint(
            self.constraint_id, 
            self.joint_set[0],
            jointChildFrameOrientation=(self.hpm.rot['w|r']*self.offset_rot).as_quat(),
            maxForce=500,
            physicsClientId=self.bid
        )

    def get_model_info(self):
        for joint_idx in range(pyb.getNumJoints(self.humanoid_id, physicsClientId=self.bid)):
            joint_info = pyb.getJointInfo(self.humanoid_id, joint_idx, physicsClientId=self.bid)
            joint_info_list = list(joint_info[0:5])
            joint_info_list.extend(list(joint_info[12:14]))

            if (joint_info[2] == pyb.JOINT_FIXED):
                joint_info_list[2] = 'FIXED'
            elif (joint_info[2] == pyb.JOINT_REVOLUTE):
                joint_info_list[2] = 'REVOLUTE'
            elif (joint_info[2] == pyb.JOINT_SPHERICAL):
                joint_info_list[2] = 'SPHERICAL'

            pprint.pprint(joint_info_list)

    def update(self, joint_set):
        self.joint_set = joint_set
        self.__update()

    def __update_joint_pos_maps(self):
        self.multidof_joint_rot = {
            1: self.__transform_axes(self.hpm.rot['r|0']),
            2: self.__transform_axes(self.hpm.rot['0|8']),
            3: self.__transform_axes(self.hpm.rot['0|9']),
            6: self.__transform_axes(self.hpm.rot['0|12']),
            9: self.__transform_axes(self.hpm.rot['r|1']),
            11: self.__transform_axes(self.hpm.rot['r|4'])
        }

        self.revolute_joint_angle = {
            4: self.hpm.rot['9|10'],
            7: self.hpm.rot['12|13'],
            10: self.hpm.rot['1|2'],
            13: self.hpm.rot['4|5']
        }

    def __update_multidof(self):
        for (joint_idx, rot) in self.multidof_joint_rot.items():
            pyb.setJointMotorControlMultiDof(
                self.humanoid_id,
                joint_idx,
                pyb.POSITION_CONTROL,
                targetPosition=rot.as_quat(),
                targetVelocity=[0, 0, 0],
                force=[500, 500, 500],
                physicsClientId=self.bid
            )

    def __update_singledof(self):
        for (joint_idx, angle) in self.revolute_joint_angle.items():
            pyb.setJointMotorControl2(
                self.humanoid_id,
                joint_idx,
                pyb.POSITION_CONTROL,
                targetPosition=angle,
                force=500,
                physicsClientId=self.bid
            )

    def __update(self):
        self.hpm.update(self.joint_set)

        pyb.changeConstraint(
            self.constraint_id, 
            self.joint_set[0],
            jointChildFrameOrientation=(self.hpm.rot['w|r']*self.offset_rot).as_quat(),
            maxForce=500,
            physicsClientId=self.bid
        )

        self.__update_joint_pos_maps()

        self.__update_multidof()
        self.__update_singledof()

    @staticmethod
    def __transform_axes(rot):
        euler_XYZ = rot.as_euler('XYZ')
        euler_XYZ[[1, 2]] = euler_XYZ[[2, 1]]
        euler_XYZ[2] = -euler_XYZ[2]
        
        return Rotation.from_euler('XZY', [euler_XYZ[0], euler_XYZ[2], euler_XYZ[1]])
