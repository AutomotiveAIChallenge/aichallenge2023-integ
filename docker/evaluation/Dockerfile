FROM ghcr.io/automotiveaichallenge/aichallenge2023-sim/autoware-universe-cuda:v1

# Advance Preparation (Create working files and download various files from Github)
RUN rm -rf /autoware/install/tier4_*_launch
RUN mkdir /ws
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
RUN sudo apt install -y git-lfs
RUN git lfs clone https://github.com/AutomotiveAIChallenge/aichallenge2023-sim /ws/aichallenge2023-sim

# Copy into Container
COPY mapfile /ws/mapfile
COPY aichallenge_submit.tar.gz /ws
COPY main.bash /ws

# Organize Files for Execution (Copy user files and map data to the executable folder)
RUN cp -r /ws/aichallenge2023-sim/docker/aichallenge /aichallenge
RUN chmod 757 /aichallenge
RUN rm -rf /aichallenge/aichallenge_ws/src/aichallenge_submit
RUN tar zxf /ws/aichallenge_submit.tar.gz -C /aichallenge/aichallenge_ws/src
RUN cp -r /ws/mapfile /aichallenge

# Build
RUN apt-get update
RUN bash -c ' \
  source /autoware/install/setup.bash; \
  cd /aichallenge/aichallenge_ws; \
  rosdep update; \
  rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO; \
  colcon build'

ENTRYPOINT []
CMD ["bash", "/ws/main.bash"]