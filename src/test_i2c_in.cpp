#include <ros/ros.h>
#include <std_msgs/String.h>
#include <i2c_comm/I2CIn.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_i2c_in");
	ros::NodeHandle nh;
	ros::ServiceClient inClient = nh.serviceClient<i2c_comm::I2CIn>("i2c_in");
	i2c_comm::I2CIn args;
	args.request.addr = 8;
	args.request.dataLen = 32;
	args.response.data = "";
	if(inClient.call(args))
	{
		std::string::iterator it = args.response.data.begin();
		std::string::iterator end = args.response.data.begin() + args.request.dataLen;
		while(*it != 0 && (uint8_t)*it < 255 && it < end)
			++it;
		if(it < end)
			*it = 0;
		
		//args.response.data += '\0';
		ROS_INFO("Received: %s", args.response.data.c_str());
	}
	else
	{
		ROS_ERROR("Error processing request.");
		return(1);
	}
	
	return(0);
}