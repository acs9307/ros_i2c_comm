#include <ros/ros.h>
#include <std_msgs/String.h>
#include <i2c_comm/I2CIn.h>
#include <vector>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_i2c_in");
	ros::NodeHandle nh;
	ros::ServiceClient inClient = nh.serviceClient<i2c_comm::I2CIn>("i2c_in");
	i2c_comm::I2CIn args;
	args.request.addr = 8;
	args.request.dataLen = 32;
	
	if(argc > 1)
	{
		++argv;
		args.request.addr = atoi(*argv);
	}

	if(inClient.call(args))
	{
		printf("Received %d bytes: \n", args.response.dataLen);
		
		auto it = args.response.data.begin();
		std::string str = "";
		for(; it < args.response.data.begin() + args.response.dataLen && *it; ++it)
			str += (char)*it;
		printf("\tString: %s\n", str.c_str());
		
		printf("\tHex: ");
		it = args.response.data.begin();
		for(; it < args.response.data.begin() + args.response.dataLen; ++it)
			printf("0x%x ", *it);
		printf("\n");
	}
	else
	{
		ROS_ERROR("Error processing request.");
		return(1);
	}
	
	return(0);
}
