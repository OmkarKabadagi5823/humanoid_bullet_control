from human_voxelpose_model import HumanPoseModel
import pybullet as pyb
from scipy.spatial.transform import Rotation
from math import pi
import yaml

import pprint

class HumanoidControl():
    def __init__(self, bullet_client_id, scaling=0.2):
        self._bid = bullet_client_id
        self._scale = scaling
        self.__init_params()

    def __init_params(self):
        self._humanoid_id = None
        self._hpm = HumanPoseModel()
        self._joint_position_set = None
        self._offset_rot = Rotation.from_euler('xyz', [pi/2, 0, 0])

    def load_config(self, config_file):
        with open(config_file, 'r') as stream:
            self._config = yaml.load(stream, Loader=yaml.SafeLoader)

        self._joint_control_params = self._config['bullet_sim_hw_interface']['controllers_list']

    def load_model(self, model_file, initial_position_set):
        self._joint_position_set = initial_position_set
        self._hpm.update(self._joint_position_set)

        self._humanoid_id = pyb.loadURDF(
            model_file,
            basePosition=self._joint_position_set[0],
            baseOrientation=(self._hpm.rot['w|r']*self._offset_rot).as_quat(),
            globalScaling=self._scale,
            flags=pyb.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self._bid
        )

        self._constraint_id = pyb.createConstraint(
            self._humanoid_id, 
            -1, -1, -1, 
            pyb.JOINT_FIXED, 
            [0, 0, 0], 
            [0, 0, 0], 
            [0, 0, 1],
            physicsClientId=self._bid
        )

        pyb.changeConstraint(
            self._constraint_id, 
            self._joint_position_set[0],
            jointChildFrameOrientation=(self._hpm.rot['w|r']*self._offset_rot).as_quat(),
            maxForce=500,
            physicsClientId=self._bid
        )

        self.__setup_controllers()

    def get_model_info(self):
        for joint_idx in range(pyb.getNumJoints(self._humanoid_id, physicsClientId=self._bid)):
            joint_info = pyb.getJointInfo(self._humanoid_id, joint_idx, physicsClientId=self._bid)
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
        self._joint_position_set = joint_set
        self.__update()

    def __setup_controllers(self):
        self._multidof_joint_names = []
        self._multidof_joint_indices = []
        self._multidof_forces = []
        self._multidof_position_gains = []
        self._multidof_velocity_gains = []
        self._multidof_max_velocities = []

        self._singledof_joint_names = []
        self._singledof_joint_indices = []
        self._singledof_forces = []
        self._singledof_position_gains = []
        self._singledof_velocity_gains = []
        self._singledof_max_velocities = []

        for joint_idx in range(pyb.getNumJoints(self._humanoid_id, physicsClientId=self._bid)):
            joint_info = pyb.getJointInfo(self._humanoid_id, joint_idx, physicsClientId=self._bid)
            joint_name = joint_info[1].decode('ascii')
            
            # skip ankle as it is not supported
            if 'ankle' in joint_name:
                continue

            if joint_info[2] == pyb.JOINT_SPHERICAL:
                self._multidof_joint_names.append(joint_name)
                self._multidof_joint_indices.append(joint_info[0])
                self._multidof_forces.append(self._joint_control_params[joint_name]['force'])
                self._multidof_position_gains.append(self._joint_control_params[joint_name]['position_gain'])
                self._multidof_velocity_gains.append(self._joint_control_params[joint_name]['velocity_gain'])
                self._multidof_max_velocities.append(self._joint_control_params[joint_name]['max_velocity'])
            elif joint_info[2] == pyb.JOINT_REVOLUTE:
                self._singledof_joint_names.append(joint_name)
                self._singledof_joint_indices.append(joint_info[0])
                self._singledof_forces.append(self._joint_control_params[joint_name]['force'])
                self._singledof_position_gains.append(self._joint_control_params[joint_name]['position_gain'])
                self._singledof_velocity_gains.append(self._joint_control_params[joint_name]['velocity_gain'])
                self._singledof_max_velocities.append(self._joint_control_params[joint_name]['max_velocity'])

    def __update_joint_pos_maps(self):
        self._multidof_joint_rots = [
            self.__transform_axes(self._hpm.rot['r|0']).as_quat(),     # 1
            self.__transform_axes(self._hpm.rot['0|8']).as_quat(),     # 2
            self.__transform_axes(self._hpm.rot['0|9']).as_quat(),     # 3
            self.__transform_axes(self._hpm.rot['0|12']).as_quat(),    # 6
            self.__transform_axes(self._hpm.rot['r|1']).as_quat(),     # 9
            self.__transform_axes(self._hpm.rot['r|4']).as_quat()      # 12
        ]

        self._singledof_joint_rots = [
            self._hpm.rot['9|10'],   # 4
            self._hpm.rot['12|13'],  # 7
            self._hpm.rot['1|2'],    # 10
            self._hpm.rot['4|5']     # 13
        ]

    def __update_multidof(self):
        pyb.setJointMotorControlMultiDofArray(
            self._humanoid_id,
            self._multidof_joint_indices,
            pyb.POSITION_CONTROL,
            targetPositions=self._multidof_joint_rots,
            forces=self._multidof_forces,
            positionGains=self._multidof_position_gains,
            velocityGains=self._multidof_velocity_gains,
            physicsClientId=self._bid
        )

    def __update_singledof(self):
        pyb.setJointMotorControlArray(
            self._humanoid_id,
            self._singledof_joint_indices,
            pyb.POSITION_CONTROL,
            targetPositions=self._singledof_joint_rots,
            forces=self._singledof_forces,
            positionGains=self._singledof_position_gains,
            velocityGains=self._singledof_velocity_gains,
            physicsClientId=self._bid
        )

    def __update(self):
        self._hpm.update(self._joint_position_set)

        pyb.changeConstraint(
            self._constraint_id, 
            self._joint_position_set[0],
            jointChildFrameOrientation=(self._hpm.rot['w|r']*self._offset_rot).as_quat(),
            maxForce=500,
            physicsClientId=self._bid
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
