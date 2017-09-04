// RTIMULib ROS Node
// Copyright (c) 2015, Romain Reignier
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the <organization> nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <RTIMULib.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <math.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtimulib_node");
    ROS_INFO("Imu driver is now running");
    ros::NodeHandle n;
    ros::NodeHandle private_n("~");

    std::string topic_name;
    if(!private_n.getParam("topic_name", topic_name))
    {
        ROS_WARN("No topic_name provided - default: imu/data");
        topic_name = "imu/data";
    }

    std::string calibration_file_path;
    if(!private_n.getParam("calibration_file_path", calibration_file_path))
    {
        ROS_ERROR("The calibration_file_path parameter must be set to use a calibration file.");
    }

    std::string calibration_file_name;
    if(!private_n.getParam("calibration_file_name", calibration_file_name))
    {
        ROS_WARN("No calibration_file_name provided - default: RTIMULib.ini");
        calibration_file_name = "RTIMULib";
    }

    std::string frame_id;
    if(!private_n.getParam("frame_id", frame_id))
    {
        ROS_WARN("No frame_id provided - default: imu_link");
        frame_id = "imu_link";
    }

    double update_rate;
    if(!private_n.getParam("update_rate", update_rate))
    {
        ROS_WARN("No update_rate provided - default: 20 Hz");
        update_rate = 20;
    }
    double angular_velocity_std_dev_ = 0.05 * (M_PI / 180.0);
    double linear_acceleration_std_dev_ = (400 / 1000000.0) * 9.807;
    double pitch_roll_std_dev_ = 1.0 * (M_PI / 180.0);
    double yaw_std_dev_ = 0.5 * (M_PI / 180.0);
    double angular_velocity_covariance = angular_velocity_std_dev_ * angular_velocity_std_dev_;
    double linear_acceleration_covariance = linear_acceleration_std_dev_ * linear_acceleration_std_dev_;
    double pitch_roll_covariance = pitch_roll_std_dev_ * pitch_roll_std_dev_;
    double yaw_covariance = yaw_std_dev_ * yaw_std_dev_;

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>(topic_name.c_str(), 1);

    // Load the RTIMULib.ini config file
    RTIMUSettings *settings = new RTIMUSettings(calibration_file_path.c_str(), calibration_file_name.c_str()); 

    RTIMU *imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
    {
        ROS_ERROR("No Imu found");
        return -1;
    }

    // Initialise the imu object
    imu->IMUInit();

    // Set the Fusion coefficient
    imu->setSlerpPower(0.02);
    // Enable the sensors
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    ros::Rate loop_rate(update_rate);
    while (ros::ok())
    {
        sensor_msgs::Imu imu_msg;

        if (imu->IMURead())
        {
            RTIMU_DATA imu_data = imu->getIMUData();
            tf::Quaternion ned_to_enu = tf::createQuaternionFromRPY(M_PI, 0.0, 0.0);
            tf::Quaternion q1 = tf::Quaternion(imu_data.fusionQPose.x(), imu_data.fusionQPose.y(),
                                                  imu_data.fusionQPose.z(), imu_data.fusionQPose.scalar());
            tf::Quaternion q2 = ned_to_enu * q1;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = frame_id;
            imu_msg.orientation.x = q2.getX();
            imu_msg.orientation.y = q2.getY();
            imu_msg.orientation.z = q2.getZ();
            imu_msg.orientation.w = q2.getW();
            imu_msg.angular_velocity.x = imu_data.gyro.x();
            imu_msg.angular_velocity.y = imu_data.gyro.y();
            imu_msg.angular_velocity.z = imu_data.gyro.z();
            imu_msg.linear_acceleration.x = imu_data.accel.x();
            imu_msg.linear_acceleration.y = imu_data.accel.y();
            imu_msg.linear_acceleration.z = imu_data.accel.z();
            imu_msg.linear_acceleration_covariance[0] = linear_acceleration_covariance;
	    imu_msg.linear_acceleration_covariance[4] = linear_acceleration_covariance;
	    imu_msg.linear_acceleration_covariance[8] = linear_acceleration_covariance;

	    imu_msg.angular_velocity_covariance[0] = angular_velocity_covariance;
	    imu_msg.angular_velocity_covariance[4] = angular_velocity_covariance;
	    imu_msg.angular_velocity_covariance[8] = angular_velocity_covariance;
    
	    imu_msg.orientation_covariance[0] = pitch_roll_covariance;
	    imu_msg.orientation_covariance[4] = pitch_roll_covariance;
	    imu_msg.orientation_covariance[8] = yaw_covariance;


            imu_pub.publish(imu_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
