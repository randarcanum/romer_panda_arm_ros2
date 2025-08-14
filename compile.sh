source /opt/ros/humble/setup.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH='/home/mrslabpanda/libfranka/build' -DSlicer_DIR:PATH='/home/mrslabpanda/Slicer-SuperBuild-Debug/Slicer-build' -Wno-dev

