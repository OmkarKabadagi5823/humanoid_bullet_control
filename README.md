# Humanoid Bullet Control
## Description
`humanoid_bullet_control` is a package for [pybullet](https://github.com/bulletphysics/bullet3) that makes it easy to control the [humanoid](https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_data/humanoid/humanoid.urdf) urdf model provided with pybullet using human poses. Currently, it supports pose format, output by [voxelpose](https://github.com/microsoft/voxelpose-pytorch) model.

## Getting started
This package depends on [human_voxelpose_model](https://github.com/OmkarKabadagi5823/human_voxelpose_model) which reads the poses output by voxelpose and estimate joint angles. It is not available on pip repositories yet, so you need to install it manually.
### Installation
```bash
# human_voxelpose_model installation
cd ~/Downloads
git clone https://github.com/OmkarKabadagi5823/human_voxelpose_model.git
cd human_voxelpose_model/
python -m build
cd dist/
pip install human_voxelpose_model-<version>-py3-none-any.whl

# humanoid_bullet_control installation
cd ~/Downloads
git clone https://github.com/OmkarKabadagi5823/humanoid_bullet_control.git
cd humanoid_bullet_control/
python -m build
cd dist/
pip install humanoid_bullet_control-<version>-py3-none-any.whl
```

### Example
```bash
python examples/humanoid_motion.py --config config/humanoid_controllers.yaml --data data/hammer.npy
```