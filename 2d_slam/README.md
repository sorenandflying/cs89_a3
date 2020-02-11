# cs89_a3

To run code:

* Set use_sim_time to true

* Run roscore

* Start bagfile with options rosbag play --pause --clock slambag.bag

* rosrun 2d_slam slam2d.py

* Play bagfile and wait for it to finish

* ctrl c to quit slam2d.py

* slam2d_optimized.g2o and slam2d_unoptimized.g2o will be saved to src folder
