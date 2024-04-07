FROM osrf/ros:noetic-desktop-full

RUN \
  apt-get -qq update && \
  apt-get -qq upgrade --yes && \
  apt-get -qq install vim git curl --yes

RUN curl -sL https://deb.nodesource.com/setup_18.x | bash -
RUN apt-get -qq update
RUN apt-get -qq install nodejs --yes
RUN apt install -y python3-pip

WORKDIR /home/Web
RUN git clone https://github.com/bomena/sensor_monitoring.git
WORKDIR /home/Web/sensor_monitoring
RUN chmod +x run.sh
RUN npm install
RUN apt-get install ros-noetic-rosbridge-suite --yes
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN apt update
WORKDIR /home/Web/sensor_monitoring/src/function
RUN chmod +x run.sh
RUN chmod +x record.sh
WORKDIR /home/Web/sensor_monitoring
