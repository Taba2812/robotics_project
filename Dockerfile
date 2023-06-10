FROM ros:noetic-robot
RUN apt-get update

ARG DEBIAN_FRONTEND=noninteractive apt-get install keyboard-configuration
RUN echo 'keyboard-configuration keyboard-configuration/layout select IT' | debconf-set-selections

# Installing Catkin-Tools
RUN apt-get -y install python3-catkin-tools


# Installing OpenCV
WORKDIR /root/OpenCV

RUN apt install -y cmake g++ wget unzip
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
RUN unzip opencv.zip
RUN mv opencv-4.x opencv
RUN echo $(ls)
RUN echo $(ls)
RUN mkdir -p build

WORKDIR /root/OpenCV/build

RUN cmake ../opencv
RUN make -j4

ENV OpenCV_DIR=/root/OpenCV/build


# Installing PCL
WORKDIR /root/code

RUN apt-get update
RUN apt-get install -y libpcl-dev --fix-missing
RUN apt-get install -y ros-noetic-pcl-ros
RUN apt-get install -y ros-noetic-pcl-conversions
RUN apt-get install -y ros-noetic-cv-bridge

# Final Setup
ENV CMAKE_PREFIX_PATH=/opt/ros/noetic
ENV DISPLAY=:1

#CMD ["cd", catkin_ws]

