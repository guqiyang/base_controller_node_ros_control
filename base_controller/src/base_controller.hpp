#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <imu_sensor_controller/imu_sensor_controller.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <ros/ros.h>
#include <iostream>

class MyRobot: public hardware_interface::RobotHW
{
public:
	MyRobot()
	{
		for(int i=0;i<2;i++)
		{
			cmd[i]=0;
			cmd_[i]=0;
			cmd__[i]=0;
			pos[i]=0;
			pos_fake[i]=0;
			vel[i]=0;
			vel_fake[i]=0;
			eff[i]=0;
			eff_fake[i]=0;
		}

		// connect and register the joint state interface
		hardware_interface::JointStateHandle state_handle_l("l_wheel_joint", &pos[0], &vel[0], &eff[0]);
		jnt_state_interface.registerHandle(state_handle_l);
		hardware_interface::JointStateHandle state_handle_r("r_wheel_joint", &pos[1], &vel[1], &eff[1]);
		jnt_state_interface.registerHandle(state_handle_r);

		hardware_interface::JointHandle pos_handle_l(jnt_state_interface.getHandle("l_wheel_joint"), &cmd[0]);
		hardware_interface::JointHandle pos_handle_r(jnt_state_interface.getHandle("r_wheel_joint"), &cmd[1]);
		registerInterface(&jnt_state_interface);

		hardware_interface::JointHandle vel_handle_l(jnt_state_interface.getHandle("l_wheel_joint"), &cmd_[0]);
		hardware_interface::JointHandle vel_handle_r(jnt_state_interface.getHandle("r_wheel_joint"), &cmd_[1]);
		registerInterface(&jnt_vel_interface);

		hardware_interface::JointHandle eff_handle_l(jnt_state_interface.getHandle("l_wheel_joint"), &cmd__[0]);
		hardware_interface::JointHandle eff_handle_r(jnt_state_interface.getHandle("r_wheel_joint"), &cmd__[1]);
		registerInterface(&jnt_eff_interface);

		jnt_pos_interface.registerHandle(pos_handle_l);
		jnt_vel_interface.registerHandle(vel_handle_l);
		jnt_eff_interface.registerHandle(eff_handle_l);

		jnt_pos_interface.registerHandle(pos_handle_r);
		jnt_vel_interface.registerHandle(vel_handle_r);
		jnt_eff_interface.registerHandle(eff_handle_r);

		hardware_interface::ImuSensorHandle::Data data;
		data.name="Imu";
		data.frame_id="imu";
		data.orientation=orientation;
		data.orientation_covariance=orientation_covariance;
		data.angular_velocity=&angular_velocity;
		data.angular_velocity_covariance=angular_velocity_covariance;
		data.linear_acceleration=&linear_acceleration;
		data.linear_acceleration_covariance=linear_acceleration_covariance;

		orientation_covariance[0]=1;
		orientation_covariance[1]=2;
		orientation_covariance[2]=3;
		orientation_covariance[3]=4;

		hardware_interface::ImuSensorHandle sensor_handle_imu(data);
		imu_interface.registerHandle(sensor_handle_imu);

		registerInterface(&jnt_pos_interface);
		registerInterface(&jnt_vel_interface);
		registerInterface(&jnt_eff_interface);
		registerInterface(&imu_interface);
		ros::NodeHandle nh;
	    cmd_r = nh.advertise<std_msgs::Int16>("/cmd_vel/right",1); 
        cmd_l =nh.advertise<std_msgs::Int16>("/cmd_vel/left",1);
	}

virtual ~MyRobot()
{
	//
}

void write()
{
	for(int i=0;i<2;i++)
	{

		/*if (pos_fake[i]>=M_PI)
		{
			pos_fake[i]=(pos_fake[i]+cmd_[i]/50)-2*M_PI;
		}
		else
		{
			pos_fake[i]=pos_fake[i]+cmd_[i]/50;//cmd[i];
		}*/
		pos_fake[i]=pos_fake[i]+cmd_[i]/50;
		//pos_fake[i]=cmd[i];
		vel_fake[i]=cmd_[i];
		eff_fake[i]=cmd__[i];
	}
	//std::cout<<"write pos"<<": "<<cmd[0]<<" "<<cmd[1]<<std::endl;
	//std::cout<<"write vel"<<": "<<cmd_[0]<<" "<<cmd_[1]<<std::endl;
	//std::cout<<"write eff"<<": "<<cmd__[0]<<" "<<cmd__[1]<<std::endl;
}

void read()
{
	for(int i=0;i<2;i++)
	{
		pos[i]=pos_fake[i];
		vel[i]=vel_fake[i];
		eff[i]=eff_fake[i];
	}
	//std::cout<<"read pos"<<": "<<pos[0]<<" "<<pos[1]<<std::endl;
	//std::cout<<"read vel"<<": "<<vel[0]<<" "<<vel[1]<<std::endl;
	//std::cout<<"read eff"<<": "<<eff[0]<<" "<<eff[1]<<std::endl;
}
void writepwm()
{
	for(int i=0;i<2;i++)
	{
		pwm_cmd[i]=cmd_[i]*44.5;
	    left.data = pwm_cmd[0];
		right.data = pwm_cmd[1];
		cmd_r.publish(left);
		cmd_l.publish(right);

	}
}
private:
	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::VelocityJointInterface jnt_vel_interface;
	hardware_interface::PositionJointInterface jnt_pos_interface;
	hardware_interface::EffortJointInterface jnt_eff_interface;
	hardware_interface::ImuSensorInterface imu_interface;

	double orientation[4];                    ///< A pointer to the storage of the orientation value: a quaternion (x,y,z,w)
	double orientation_covariance[9];         ///< A pointer to the storage of the orientation covariance value: a row major 3x3 matrix about (x,y,z)
	double angular_velocity;                  ///< A pointer to the storage of the angular velocity value: a triplet (x,y,z)
	double angular_velocity_covariance[9];    ///< A pointer to the storage of the angular velocity covariance value: a row major 3x3 matrix about (x,y,z)
	double linear_acceleration;               ///< A pointer to the storage of the linear acceleration value: a triplet (x,y,z)
	double linear_acceleration_covariance[9]; ///< A pointer to the storage of the linear acceleration covariance value: a row major 3x3 matrix about (x,y,z)

	double cmd[2];   //Zero Ordor
	double cmd_[2];  //First Ordor
	double cmd__[2]; //Second Ordor
	double pos[2];
	double pos_fake[2];
	double vel[2];
	double vel_fake[2];
	double eff[2];
	double eff_fake[2];
	float  pwm_cmd[2];

    std_msgs::Int16 left;
    std_msgs::Int16 right;

    ros::Publisher cmd_r ;
    ros::Publisher cmd_l;
};
