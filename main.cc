#include<iostream>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/Time.h>
#include<std_msgs/Header.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <boost/filesystem.hpp>
#include <fstream>

using namespace std;
void open_file(std::string path,std::vector < std::vector < std::string > > &image_names ,int numCameras);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ImageIMU_to_rosbag");

    if(argc!=4)
    {
        cerr << "Usage: rosrun ImageIMU_to_bag ImageIMU_to_bag <path to  directory including image file and imu file>   <path to bag>  <the numbers of camera>" << endl;
        return 0;
    }
    ros::start();

    //read the numbers of camera
    std::string N_camera(argv[3]);
    const int  numCameras =int (std::stoi(N_camera));

    // Output bag
    rosbag::Bag bag_out(argv[2],rosbag::bagmode::Write);

    // the folder path
    std::string path(argv[1]);
    std::vector < std::vector < std::string > > image_names(numCameras);
    open_file(path,image_names,numCameras);
    std::ifstream imu_file(path + "/imu0/data.csv");

   //get start address of cam-image
    std::vector < std::vector < std::string > ::iterator> cam_iterators(numCameras);
    for (size_t i = 0; i < numCameras; ++i)
             cam_iterators.at(i) = image_names.at(i).begin();

  string line;
  double start =0;
  while(true)
    {
           // check if at the end
           for (size_t i = 0; i < numCameras; ++i)
           {
              if (cam_iterators[i] == image_names[i].end())
                {
                   std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
                   cv::waitKey();
                   return 0;
               }
           }

           /// add images
           double t;
           for (size_t i = 0; i < numCameras; ++i)
           {
                 cv::Mat image = cv::imread(  path + "/cam" + std::to_string(i)  +"/" +*cam_iterators.at(i),  cv::IMREAD_GRAYSCALE);
                 std::string nanoseconds = cam_iterators.at(i)->substr( cam_iterators.at(i)->size() - 13, 9);
                 std::string seconds = cam_iterators.at(i)->substr( 0, cam_iterators.at(i)->size() - 13);
                 t= double(std::stoi(seconds) )+ double(std::stoi(nanoseconds))/10e8;
                 if (start == 0)
                     start = t;

                 // get all IMU measurements till then
                 double t_imu = start;
                 do
                {
                       if (!std::getline(imu_file, line))
                       {
                         std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
                         cv::waitKey();
                         return 0;
                       }

                       std::stringstream stream(line);
                       std::string s;
                       std::getline(stream, s, ',');
                       std::string nanoseconds = s.substr(s.size() - 9, 9);
                       std::string seconds = s.substr(0, s.size() - 9);
                       t_imu= double(std::stoi(seconds) )+ double(std::stoi(nanoseconds))/10e8;

                       double gyro[3];
                       for (int j = 0; j < 3; ++j)
                       {
                         std::getline(stream, s, ',');
                         gyro[j] = std::stof(s);
                       }

                       double acc[3];
                       for (int j = 0; j < 3; ++j)
                       {
                         std::getline(stream, s, ',');
                         acc[j] = std::stof(s);
                       }

                     //save imu to rosbag
                     {
                         size_t seq = 0;
                         sensor_msgs::Imu imu_msg;
                         imu_msg.header.stamp = ros::Time(t_imu);
                         imu_msg.header.seq = seq;
                         imu_msg.header.frame_id = "imu";
                         imu_msg.angular_velocity.x = gyro[0];
                         imu_msg.angular_velocity.y = gyro[1];
                         imu_msg.angular_velocity.z = gyro[2];
                         imu_msg.linear_acceleration.x = acc[0];
                         imu_msg.linear_acceleration.y = acc[1];
                         imu_msg.linear_acceleration.z = acc[2];
                          bag_out.write("/imu0",ros::Time(t_imu),imu_msg);
                     }
                 }
             while (t_imu <= t);

             // save image to rosbag
             {
                 uint seq = 0;
                 cv_bridge::CvImage ros_image;
                 ros_image.image = image;
                 ros_image.encoding = "mono8";
                 sensor_msgs::ImagePtr ros_image_msg;
                 ros_image_msg = ros_image.toImageMsg();
                 ros_image_msg->header.seq = seq;
                 ros_image_msg->header.stamp = ros::Time(t);
                 ros_image_msg->header.frame_id = "frame";
                if(i==0)
                bag_out.write("/cam0/image_raw",ros::Time(t),ros_image_msg);
                else if(i==1)
                bag_out.write("/cam1/image_raw",ros::Time(t),ros_image_msg);
                else
                    cout<<"error the numbers of camera"<<endl;
             }
             cam_iterators[i]++;
           }
       }

    bag_out.close();

    ros::shutdown();

    return 0;
}

//open imu and image file

void open_file(std::string path,std::vector < std::vector < std::string > > &image_names, int numCameras)
{
    // open the IMU file
   {
        std::string line;
        std::ifstream imu_file(path + "/imu0/data.csv");
        if (!imu_file.good())
        {
          cout<< "no imu file found at " << path+"/imu0/data.csv"<<endl;
          return ;
        }
        int number_of_lines = 0;
        while (std::getline(imu_file, line))
          ++number_of_lines;
         cout<< "No. IMU measurements: " << number_of_lines-1<<endl;
        if (number_of_lines - 1 <= 0)
        {
          cout<< "no imu messages present in " << path+"/imu0/data.csv"<<endl;
          return ;
        }
  }
  //open image file
    {
        int num_camera_images = 0;
        //std::vector < std::vector < std::string >> image_names(numCameras);
        for (int i = 0; i < numCameras; ++i) {
          num_camera_images = 0;
          std::string folder(path + "/cam" + std::to_string(i) );

          for (auto it = boost::filesystem::directory_iterator(folder);  it != boost::filesystem::directory_iterator(); it++)
          {
                if (!boost::filesystem::is_directory(it->path()))
                {  //we eliminate directories
                  num_camera_images++;
                  image_names.at(i).push_back(it->path().filename().string());
                }
                else
                  continue;
          }

          if (num_camera_images == 0)
          {
            cout<< "no images at " << folder<<endl;
            return ;
          }

          cout<< "No. cam " << i << " images: " << num_camera_images<<endl;
          // the filenames are not going to be sorted. So do this here
          std::sort(image_names.at(i).begin(), image_names.at(i).end());
          image_names.at(i).erase(image_names.at(i).begin() , image_names.at(i).begin() + 3);//there are three error image_name
        }
    }
}
