source /opt/ros/humble/setup.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH='/home/mrslabpanda/libfranka/build' # -DSlicer_DIR:PATH='/home/mrslabpanda/Slicer-SuperBuild-Debug/Slicer-build' -Wno-dev

# https://www.pure.ed.ac.uk/ws/portalfiles/portal/313473567/Choosing_Stiffness_POLLAYIL_DOA01112022_AFV.pdf
# https://github.com/ffall007/franka_analytical_ik