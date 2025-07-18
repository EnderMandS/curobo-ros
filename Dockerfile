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
    git clone --depth 1 --recursive https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone --depth 1 --recursive https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    git clone --depth 1 --recursive https://github.com/conda-incubator/conda-zsh-completion ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/conda-zsh-completion && \
    sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting conda-zsh-completion)/g' /home/${USERNAME}/.zshrc && \
    sed -i '/^plugins=(/a\autoload -U compinit && compinit' /home/${USERNAME}/.zshrc
SHELL ["/bin/zsh", "-c"]

# CUDA
RUN echo "export PATH=/usr/local/cuda-11.8/bin:$PATH" >> /home/${USERNAME}/.zshrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH" >> /home/${USERNAME}/.zshrc

WORKDIR /home/${USERNAME}/code

# Install miniconda and mamba
RUN wget -c https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    bash Miniconda3-latest-Linux-x86_64.sh -b -u -p ~/miniconda3 && \
    rm Miniconda3-latest-Linux-x86_64.sh && \
    /home/${USERNAME}/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    /home/${USERNAME}/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r && \
    /home/${USERNAME}/miniconda3/bin/conda init zsh && \
    /home/${USERNAME}/miniconda3/bin/conda install -n base -c conda-forge mamba -y

# Install ROS 2 Humble
RUN /home/${USERNAME}/miniconda3/bin/mamba create -n curobo && \
    /home/${USERNAME}/miniconda3/bin/conda config --env --add channels conda-forge && \
    /home/${USERNAME}/miniconda3/bin/conda config --env --remove channels defaults && \
    /home/${USERNAME}/miniconda3/bin/conda config --env --add channels robostack-humble && \
    /home/${USERNAME}/miniconda3/bin/mamba install -n curobo ros-humble-desktop -y && \
    /home/${USERNAME}/miniconda3/bin/mamba install -n curobo colcon-common-extensions catkin_tools rosdep -y

# cuRobo
RUN git clone --depth 1 --recursive https://github.com/NVlabs/curobo.git && \
    sed -i 's/requires = \["setuptools>=45", "setuptools_scm>=6.2", "wheel", "torch"\]/requires = ["setuptools>=70", "setuptools_scm>=6.2", "wheel", "torch==2.4.0"]/' curobo/pyproject.toml && \
    sed -i '/setuptools_scm>=6.2/i\  setuptools>=70' curobo/setup.cfg && \
    sed -i 's/torch>=1.10/torch==2.4.0/' curobo/setup.cfg && \
    touch curobo/COLCON_IGNORE

# Build ROS workspace    
COPY --chown=${USER_UID}:${USER_GID} . /home/${USERNAME}/code
RUN /home/${USERNAME}/miniconda3/bin/mamba run -n curobo colcon build

RUN echo 'eval "$(~/miniconda3/bin/mamba shell hook --shell zsh)"' >> /home/${USERNAME}/.zshrc && \
    echo "mamba activate curobo" >> /home/${USERNAME}/.zshrc && \
    echo "source ~/code/install/setup.zsh" >> ~/.zshrc && \
    echo ": 1700000000:0;ros2 launch motion_planner motion_planner.launch.py" >> /home/$USERNAME/.zsh_history && \
    echo ": 1700000001:0;colcon build" >> /home/$USERNAME/.zsh_history && \
    echo ": 1700000002:0;./scripts/post_install.zsh" >> /home/$USERNAME/.zsh_history

VOLUME /home/${USERNAME}/code
ENTRYPOINT [ "/bin/zsh" ]
