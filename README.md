# Installation Guide

This is the installation guideline for cuRobo with nvblox.

There are three ways to install:
1. Docker (recommanded)
2. Conda
3. Standard compile

Setting the environment variable to your device or docker container with `export TORCH_CUDA_ARCH_LIST=8.9`, where `8.9` should be replaced by your GPU’s [compute capability](https://developer.nvidia.com/cuda-gpus#collapseOne) or running the following command to check:
``` shell
nvidia-smi --query-gpu=compute_cap --format=csv,noheader | head -n 1
```

# Docker 
``` shell
docker pull ghcr.io/endermands/curobo:latest
docker run --name curobo --runtime=nvidia --gpus all -itd --network=host --ipc=host -v $HOME/.Xauthority:/root/.Xauthority -e DISPLAY -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -e RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION ghcr.io/endermands/curobo:latest
docker exec -it curobo zsh
./scripts/post_install.zsh
```

# Standard compile
## CUDA

``` shell
sudo apt update
sudo apt install -y git-lfs ninja-build libgoogle-glog-dev libgtest-dev libsqlite3-dev curl tcl libbenchmark-dev libeigen3-dev
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
sudo sh cuda_11.8.0_520.61.05_linux.run -a
rm cuda_11.8.0_520.61.05_linux.run
```

Add following lines to `~/.zshrc`
``` shell
export PATH=/usr/local/cuda-11.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH
```

## cuRobo

``` shell
python3 -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
git clone --depth 1 --recursive https://github.com/NVlabs/curobo.git
cd curobo
python3 -m pip install -e . --no-build-isolation
```

## nvblox

Notice that there are different versions of `nvblox_torch`:
1. [NVlabs/nvblox_torch](https://github.com/NVlabs/nvblox_torch.git): This is the old version. Reference by cuRobo
2. [nvidia-isaac/nvblox](https://github.com/nvidia-isaac/nvblox): This is the new version. Using `pip` install may meet torch version conflict

---

Check pyTorch compiler:
``` shell
python3 -c "import torch; print(torch._C._GLIBCXX_USE_CXX11_ABI)"
```
If `True`, go on. If `False`, refer to [cuRobo](https://curobo.org/get_started/1_install_instructions.html#installing-nvblox-for-precxx11-abi-and-isaac-sim).

``` shell
git clone --depth 1 --recursive https://github.com/valtsblukis/nvblox.git
cd nvblox/nvblox
mkdir build && cd build
cmake -DPRE_CXX11_ABI_LINKABLE=ON -DBUILD_TESTING=OFF -GNinja -DCMAKE_INSTALL_PREFIX="/opt/nvblox" ..
ninja
sudo ninja install
ninja clean
```

## nvblox_torch

``` shell
git clone --depth 1 --recursive https://github.com/NVlabs/nvblox_torch.git
cd nvblox_torch
mkdir -p src/nvblox_torch/bin
cd src/nvblox_torch/cpp
mkdir -p build
vim CMakeLists.txt
```

Modify and add `nvblox` path if setting `-DCMAKE_INSTALL_PREFIX="/opt/nvblox"`:
``` cmake
find_package(nvblox REQUIRED PATH="/opt/nvblox")
include_directories(/opt/nvblox/include)
target_link_directories(py_nvblox PRIVATE /opt/nvblox/lib)
```

Now go on building
``` shell
cd build
cmake -DCMAKE_PREFIX_PATH="$(python3 -c 'import torch.utils; print(torch.utils.cmake_prefix_path)')" -DCMAKE_CUDA_COMPILER=$(which nvcc) -GNinja ..
ninja
cd ../../../../ && cp -r src/nvblox_torch/cpp/build/*.so src/nvblox_torch/bin/
cd src/nvblox_torch/cpp/build && ninja clean && cd ../../../../
python3 -m pip install -e . --prefix=$HOME/.local
python3 -m pip install opencv-python pyrealsense2 transforms3d
```

# Conda

Using RoboStack ROS conda environment along with torch and cuRobo.

## Installing conda

``` shell
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -u -p ~/miniconda3
~/miniconda3/bin/conda init zsh
```

## Installing mamba

``` shell
conda install mamba -c conda-forge -y
```

Add mamba hook to `~/.zshrc` and relogin to shell.
``` shell
echo 'eval "$(~/miniconda3/bin/mamba shell hook --shell zsh)"' >> ~/.zshrc
```

## ROS
Create RoboStack `humble` environment. There are different channels depending on the version of ROS that you wish to install.

``` shell
mamba create -n curobo -y 
mamba activate curobo
conda config --env --add channels conda-forge
conda config --env --remove channels defaults
conda config --env --add channels robostack-humble
mamba install ros-humble-desktop -y
mamba deactivate
mamba activate curobo
mamba install colcon-common-extensions catkin_tools rosdep -y
```

## cuRobo

``` shell
python3 -m pip install torch==2.4.0 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
git clone --depth 1 --recursive https://github.com/NVlabs/curobo.git
cd curobo
sed -i 's/requires = \["setuptools>=45", "setuptools_scm>=6.2", "wheel", "torch"\]/requires = ["setuptools>=61,<69", "setuptools_scm>=6.2,<6.9", "wheel", "torch==2.4.0"]/' curobo/pyproject.toml
python3 -m pip install -e . --no-build-isolation
```

## nvblox_torch
``` shell
sudo apt install -y libglib2.0-0 libgl1
pip install https://github.com/nvidia-isaac/nvblox/releases/download/v0.0.8/nvblox_torch-0.0.8rc5+cu11ubuntu22-863-py3-none-linux_x86_64.whl
pip install opencv-python pyrealsense2 transforms3d
```

# Verify installation

Verify curobo installation by running `python scripts/curobo_motion.py`.

# Usage
Modify `src/motion_planner/motion_planner/motion_planner.py` to your own robot.
``` shell
ros2 run motion_planner motion_planner
```

# Known Issues

1. When running `colcon build`, stderr `ERROR setuptools_scm._file_finders.git listing git files failed - pretending there aren't any`.
    
    This warning is related to `setuptools_scm` not being able to find git version information for the curobo package. The warning doesn't affect functionality.

2. The ROS package name should not be the same as any packages in mamba environment.

3. There are some problems about setuptools version conflict (TODO).
