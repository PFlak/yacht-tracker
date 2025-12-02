# 1. Obraz bazowy dla Raspberry Pi (ARM64v8)
# Używamy oficjalnego obrazu OSRF, ale z tagiem 'arm64v8', 
# który jest odpowiedni dla 64-bitowych systemów na RPi.
# Zmieniamy na 'humble-ros-base' zamiast 'desktop-full', 
# aby zredukować rozmiar, jeśli nie jest potrzebny pełny pulpit. 
# Jeśli potrzebujesz wszystkich pakietów 'desktop-full' (jak rviz), 
# zmień tag, ale pamiętaj, że będzie znacznie większy.
FROM ros:humble-ros-core

ARG ROS_PACKAGE=ros_core
ARG ROS_VERSION=humble

ENV ROS_DISTRO=${ROS_VERSION} \
    ROS_ROOT=/opt/ros/${ROS_VERSION} \
    ROS_PYTHON_VERSION=3 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash

# Zmienne użytkownika (zgodne z Twoimi)
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# 2. Tworzenie użytkownika i sudo
RUN apt-get update \
    && apt-get install -y sudo \
    ca-certificates \
    curl \
    gnupg \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# 3. Instalacja narzędzi deweloperskich i zależności systemowych (część 1)
# Dodano też nano
RUN apt-get update \
    && apt-get install -y \
    nano \
    jstest-gtk \
    usbutils \
    xxd \
    net-tools \
    iputils-ping \
    fzf \
    python3-pip \
    wget \
    libgeographic-dev \
    # Dodatkowe pakiety do budowania
    git cmake build-essential \
    libopencv-dev libglm-dev libsdl2-dev libfreetype6-dev doxygen \
    libx11-dev x11-apps \
    && rm -rf /var/lib/apt/lists/*

# 4. Instalacja pakietów ROS 2
# WAŻNE: Wersja 'humble-ros-base' może nie zawierać wszystkich zależności 
# dla tych pakietów, więc upewnij się, że masz je zainstalowane lub 
# zmień obraz bazowy na 'humble-desktop-full-arm64v8' (jeśli jest dostępny).
RUN apt-get update \
    && apt-get install -y \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

USER root

# 5. Klonowanie aliasów i kopiowanie środowiska
RUN git clone https://github.com/kimushun1101/ros2-aliases.git /home/$USERNAME/ros2-aliases 
COPY .devcontainer/ros2aliases.env /home/$USERNAME/ros2-aliases/.env

# --- SEKCJA NVIDIA USUNIĘTA/ZMIENIONA ---
# Usunięto instalację pakietów libgl* i ENV NVIDIA_*, 
# ponieważ są specyficzne dla kart NVIDIA.

# 6. Konfiguracja przestrzeni roboczej
RUN mkdir -p /home/$USERNAME/ros2_ws
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/ros2_ws # Zmieniono grupę na $USERNAME dla spójności

# 6.5. Instalacja Mapviz i pluginów ze źródeł
# Używamy użytkownika 'dev' do klonowania i budowania
# USER $USERNAME
# WORKDIR /home/$USERNAME/ros2_ws/src

# RUN git config --global credential.helper store
# RUN git config --global core.askPass true
# RUN git config --global http.sslVerify false

# # Klonowanie Mapviz i jego zależności
# RUN git clone -c core.askPass=true https://github.com/swri-robotics/mapviz.git \
#     && git clone -c core.askPass=true https://github.com/swri-robotics/mapviz_plugins.git \
#     && git clone -c core.askPass=true https://github.com/swri-robotics/tile_map.git \
#     && git clone -c core.askPass=true https://github.com/swri-robotics/multires_image.git

# Powrót do katalogu głównego workspace i budowanie
# WORKDIR /home/$USERNAME/ros2_ws
# Używamy narzędzia 'colcon' do budowania sklonowanych pakietów
# Argumenty:
# --merge-install - instaluje w jednym folderze 'install' (opcjonalne)
# --packages-skip-by-dep - pomija pakiety, dla których nie ma zależności (pomaga przy brakujących zależnościach)
# RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    # && colcon build --symlink-install --packages-select mapviz mapviz_plugins tile_map multires_image
    
# Powrót na roota, aby kontynuować instalację pip i uprawnienia
# USER root
# WORKDIR /home/$USERNAME/ros2_ws/

# 7. Instalacja zależności Pythona
# Używamy pip3 z sudo, aby zainstalować globalnie
RUN sudo pip3 install osqp
RUN sudo pip3 install casadi
RUN sudo pip3 install "fastapi[standard]"
RUN sudo pip3 install requests

# 8. Konfiguracja środowiska w .bashrc
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2-aliases/ros2_aliases.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc \
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

# 9. Ustawienia końcowe
USER $USERNAME

WORKDIR /home/$USERNAME/ros2_ws/

COPY .devcontainer/entrypoint.sh /entrypoint.sh

# Dostosowanie uprawnień dla entrypoint
USER root
RUN chmod +x /entrypoint.sh
USER $USERNAME

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]