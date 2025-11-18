run 3 camera nodes:

ros2 run v4l2_camera v4l2_camera_node   --ros-args   -p video_device:=/dev/video2   -p image_size:="[1920,1200]"   -p framerate:=60.0   -r __ns:=/camera0
ros2 run v4l2_camera v4l2_camera_node   --ros-args   -p video_device:=/dev/video3   -p image_size:="[1920,1200]"   -p framerate:=60.0   -r __ns:=/camera1

ros2 run v4l2_camera v4l2_camera_node   --ros-args   -p video_device:=/dev/video4   -p image_size:="[1920,1200]"   -p framerate:=60.0   -r __ns:=/camera2


check camera fps:

ros2 topic hz /image_raw

