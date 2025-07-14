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

echo "Installing torch"
mamba run -n curobo pip install torch==2.4.0 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
cd curobo
mamba run -n curobo pip install -e . --no-build-isolation
cd ~/code

echo "Installing nvblox_torch"
mamba run -n curobo pip install https://github.com/nvidia-isaac/nvblox/releases/download/v0.0.8/nvblox_torch-0.0.8rc5+cu11ubuntu22-863-py3-none-linux_x86_64.whl
mamba run -n curobo pip install opencv-python pyrealsense2 transforms3d

echo "\nPost-install script completed successfully."
echo "Please relogin to your terminal to apply changes.\n"
