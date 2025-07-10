#!/bin/zsh --login

set -e

echo "Running post-install script..."

TORCH_CUDA_ARCH_LIST=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader | head -n 1)
if [[ -z "$TORCH_CUDA_ARCH_LIST" ]]; then
    echo "No NVIDIA GPU detected. Exiting."
    exit 1
fi
echo "Detected NVIDIA GPU with compute capability: $TORCH_CUDA_ARCH_LIST"
if ! grep -q "export TORCH_CUDA_ARCH_LIST" ~/.zshrc; then
    echo "export TORCH_CUDA_ARCH_LIST=$TORCH_CUDA_ARCH_LIST" >> ~/.zshrc
else
    sed -i "s/export TORCH_CUDA_ARCH_LIST=.*/export TORCH_CUDA_ARCH_LIST=$TORCH_CUDA_ARCH_LIST/" ~/.zshrc
fi

# echo "Installing CUDA 11.8..."
# if ! command -v nvcc &> /dev/null; then
#     wget -c https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
#     sudo sh cuda_11.8.0_520.61.05_linux.run --silent --toolkit
#     echo 'export PATH=/usr/local/cuda-11.8/bin:$PATH' >> ~/.zshrc
#     echo 'export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH' >> ~/.zshrc
#     if ! grep -q "export TORCH_CUDA_ARCH_LIST" ~/.zshrc; then
#         echo "export TORCH_CUDA_ARCH_LIST=$TORCH_CUDA_ARCH_LIST" >> ~/.zshrc
#     else
#         sed -i "s/export TORCH_CUDA_ARCH_LIST=.*/export TORCH_CUDA_ARCH_LIST=$TORCH_CUDA_ARCH_LIST/" ~/.zshrc
#     fi
#     rm cuda_11.8.0_520.61.05_linux.run
# else
#     echo "CUDA 11.8 is already installed."
# fi

# echo "Installing miniconda..."
# if ! command -v conda &> /dev/null; then
#     wget -c https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
#     bash Miniconda3-latest-Linux-x86_64.sh -b -u -p ~/miniconda3
#     rm Miniconda3-latest-Linux-x86_64.sh
# else
#     echo "Miniconda is already installed."
# fi

# echo "Installing mamba..."
# if ! command -v mamba &> /dev/null; then
#     ~/miniconda3/bin/conda install -n base -c conda-forge mamba -y
# else
#     echo "Mamba is already installed."
# fi
# if ! grep -q "mamba shell hook" ~/.zshrc; then
#     echo 'eval "$(~/miniconda3/bin/mamba shell hook --shell zsh)"' >> ~/.zshrc
#     source ~/.zshrc
# fi

# echo "Creating mamba environment..."
# if ! mamba env list | grep -q "curobo"; then
#     mamba create -n curobo
#     mamba activate curobo
#     conda config --env --add channels conda-forge
#     conda config --env --remove channels defaults
#     conda config --env --add channels robostack-humble
#     mamba install ros-humble-desktop -y
#     mamba deactivate
#     mamba activate curobo
#     mamba install colcon-common-extensions catkin_tools rosdep -y

echo "Installing torch"
mamba run -n curobo pip install torch==2.4.0 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
cd curobo
mamba run -n curobo pip install -e . --no-build-isolation
cd ~/code

echo "Installing nvblox_torch"
mamba run -n curobo pip install https://github.com/nvidia-isaac/nvblox/releases/download/v0.0.8/nvblox_torch-0.0.8rc5+cu11ubuntu22-863-py3-none-linux_x86_64.whl
mamba run -n curobo pip install opencv-python pyrealsense2 transforms3d

echo "Building ROS workspace..."
if [ -d "src" ]; then
    colcon build
else
    echo "No src directory found. Skipping ROS workspace build."
fi
