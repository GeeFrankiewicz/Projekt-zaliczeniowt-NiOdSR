FROM osrf/ros:humble-desktop-full

# Instalacja zależności systemowych i sterowników UR
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-ur-robot-driver \
    ros-humble-ur-description \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Instalacja bibliotek Python do wizji
RUN pip3 install "numpy<2.0" opencv-python opencv-contrib-python transforms3d

# Ustawienie środowiska
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble

# Tworzenie przestrzeni roboczej
WORKDIR /ros2_ws

# Skrypt entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]

# Kopiujemy skrypt startowy do kontenera i nadajemy mu uprawnienia
COPY startup.sh /startup.sh
RUN chmod +x /startup.sh

# Ustawiamy ten skrypt jako domyślną komendę przy starcie
CMD ["/startup.sh"]