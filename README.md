# ImageIMU_to_rosbag

ROS package to generate a rosbag from a collection of images and imu . Images and imus are ordered alphabetically. The timestamp for each image is assigned according to the specified frequency. 

The bag will publish the images to topic `/cam0/image_raw` and `/cam1/image_raw` , the imu to topic `/imu0`.

Tested in ROS indigo and ubuntu 14.04.

## Installation

In your ROS_PACKAGE_PATH (check your environment variable ROS_PACKAGE_PATH):

    git clone https:https://github.com/QingfengLi-hit/ImageIMU_to_rosbag.git
    
    cd ImageIMU_to_rosbag
    mkdir build
    cd build
    cmake ..
    make

## Usage:

   rosrun ImageIMU_to_bag ImageIMU_to_bag <path to directory including image file and imu file>   <path to bag>  <the numbers of camera>
  
 - `path to directory including image file and imu file`: Path to the folder with the images(.png) and imu(.csv) 
 - `path to bag`: Path to save the bag (including the filename e.g. directory/filename.bag)
 - `the numbers of camera`: 1 is monoclular + imu,  2 is stereo camera + imu.
 
## DATASET:
The example uses a system with two cameras and one IMU:

#+-- dataset-dir
#    +-- cam0
#    │   +-- 1385030208726607500.png
#    │   +--      ...
#    │   \-- 1385030212176607500.png
#    +-- cam1
#    │   +-- 1385030208726607500.png
#    │   +--      ...
#    │   \-- 1385030212176607500.png
#    \-- imu0.csv


My dataset come from oppoR11 phone can be download:
链接: https://pan.baidu.com/s/1quZ48pBZb5vjT3BueMja7w 密码: 8syv
