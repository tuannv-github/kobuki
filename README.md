# kobuki

```
git clone https://github.com/tuannv-github/kobuki.git
git submodule update --init --recursive
colcon build --symlink-install
```

```
sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src -r -yjoyn
```

```
git clone --recursive https://github.com/CollaborativeRoboticsLab/kobuki.git
```

```
colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_CXX_FLAGS="-Wno-error=vla"
```

```
ros2 launch kobuki kobuki.launch.py
```

```
ros2 run joy joy_node --ros-args -p dev:="/dev/input/js0" -r __node:=joy_ctrl -p deadzone:=0.05 -p autorepeat_rate:=20.0 --log-level info
```

```
sudo apt update
sudo apt install -y build-essential meson ninja-build git libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libx264-dev
sudo apt install -y \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  gstreamer1.0-x \
  gstreamer1.0-alsa \
  gstreamer1.0-gl \
  gstreamer1.0-gtk3 \
  gstreamer1.0-qt5 \
  gstreamer1.0-pulseaudio
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 ! flvmux streamable=true ! rtmpsink location="rtmp://10.1.101.210:/live/stream/kobuki" sync=false
```