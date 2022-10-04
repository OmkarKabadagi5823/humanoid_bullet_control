from humanoid_bullet_control import HumanoidControl

import pybullet as pyb
import pybullet_data
import numpy as np
import time
import argparse
    
def main():
    args = parse_args()

    # connect to pybullet GUI server
    gui_client_id = pyb.connect(pyb.GUI)
    
    pyb.setGravity(0, 0, -10)

    # additional path imports for model files
    pyb.setAdditionalSearchPath(
        pybullet_data.getDataPath(), physicsClientId=gui_client_id
    )
    
    # load ground plane
    ground_id = pyb.loadURDF(
        'plane.urdf', [0, 0, 0], useFixedBase=True, physicsClientId=gui_client_id
    )
    
    # a joint set is required to initialize the model
    initial_position_set = next(get_joint_position_sets(args.data))
    
    humanoid_control = HumanoidControl(gui_client_id)
    humanoid_control.load_config(args.config)
    humanoid_control.load_model('humanoid/humanoid.urdf', initial_position_set)

    for joint_position_set in get_joint_position_sets(args.data):
        # update the model with the new joint set
        humanoid_control.update(joint_position_set)
        pyb.stepSimulation()
        time.sleep(1./25.)

def parse_args():
    parser = argparse.ArgumentParser()
    
    parser.add_argument(
        '--config',
        metavar='config_filepath',
        type=str,
        required=True,
        help='Path to the config file'
    )
    parser.add_argument(
        '--data',
        metavar='data_file',
        type=str,
        required=True,
        help='Path to the data file (.npy file)'
    )

    return parser.parse_args()

def get_joint_position_sets(data_file):
    frames = np.load(data_file)

    for joint_position_set in frames:
        axis_min = np.min(joint_position_set, axis=0)
        offset = np.array([0, 0, axis_min[2]])
        yield (joint_position_set - offset) / 1000



if __name__ == '__main__':
    main()