FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ubuntu

RUN apt update && \
    apt install -y git-lfs cmake sudo zsh curl wget tree vim && \
    apt install -y libglib2.0-0 libgl1 libeigen3-dev && \
    rm -rf /var/lib/apt/lists/*

# setup user
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID --force $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir -p /etc/sudoers.d \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod -aG sudo ubuntu
USER $USERNAME

# zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/g' /home/${USERNAME}/.zshrc
SHELL ["/bin/zsh", "-c"]

# CUDA
RUN echo "export PATH=/usr/local/cuda-11.8/bin:$PATH" >> /home/${USERNAME}/.zshrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH" >> /home/${USERNAME}/.zshrc

WORKDIR /home/${USERNAME}/code

# Install miniconda and mamba
RUN wget -c https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    bash Miniconda3-latest-Linux-x86_64.sh -b -u -p ~/miniconda3 && \
    rm Miniconda3-latest-Linux-x86_64.sh && \
    /home/${USERNAME}/miniconda3/bin/conda init zsh && \
    /home/${USERNAME}/miniconda3/bin/conda install -n base -c conda-forge mamba -y && \
    echo 'eval "$(~/miniconda3/bin/mamba shell hook --shell zsh)"' >> /home/${USERNAME}/.zshrc

# cuRobo
RUN git clone --depth 1 --recursive https://github.com/NVlabs/curobo.git && \
    sed -i 's/requires = \["setuptools>=45", "setuptools_scm>=6.2", "wheel", "torch"\]/requires = ["setuptools>=61,<69", "setuptools_scm>=6.2,<6.9", "wheel", "torch==2.4.0"]/' curobo/pyproject.toml && \
    echo ": 1700000000:0;ros2 run motion_planner motion_planner" >> /home/$USERNAME/.zsh_history && \
    echo ": 1700000001:0;colcon build --packages-select motion_planner" >> /home/$USERNAME/.zsh_history && \
    echo ": 1700000002:0;./scripts/post_install.zsh" >> /home/$USERNAME/.zsh_history

# ROS workspace
COPY . /home/${USERNAME}/code

ENTRYPOINT [ "/bin/zsh" ]
