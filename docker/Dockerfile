# 设置基础镜像
FROM zhuhu/cuda102-ros-ubuntu18:0.1

ARG DEBIAN_FRONTEND=noninteractive

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# 安装Python库和深度学习工具
RUN conda install -y -c pytorch pytorch==1.7.1 torchvision==0.8.2 torchaudio==0.7.2 cudatoolkit=10.2 && \
    conda install -y -c conda-forge addict rospkg pycocotools && \
    pip install alfred-py scipy==1.7.0 rich platformdirs==3.5.1 typing-extensions==4.7.1 

# 创建ROS catkin工作空间
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN bash -c "source /opt/ros/melodic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release"

# 设置ROS环境变量
RUN echo -e "\nsource /opt/ros/melodic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# 构建ROS软件包
RUN pwd && source /opt/ros/melodic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release

# 复制Docker入口脚本
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh

# 设置容器的入口点和默认命令
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# 设置工作目录
WORKDIR /root/catkin_ws/src