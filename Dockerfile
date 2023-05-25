FROM ros:latest

WORKDIR /code

# Installing OpenCV
RUN sudo apt install -y cmake g++ wget unzip
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
RUN unzip opencv.zip
RUN mv opencv-4.x opencv
RUN mkdir -p build && cd build
RUN cmake ../opencv
RUN make -j4

# Installing PCL
RUN sudo apt install libpcl-dev


CMD ["cd", catkin_ws]

