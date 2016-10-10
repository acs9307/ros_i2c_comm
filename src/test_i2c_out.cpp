#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

#include <i2c_comm/I2COut.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_i2c_out");
	ros::NodeHandle nh;
	ros::ServiceClient outClient = nh.serviceClient<i2c_comm::I2COut>("i2c_out");
	i2c_comm::I2COut args;
	args.request.addr = 8;
	
	char inBuff[1024] = {0};
	std::cout<<"Type in the data to send.\n";
	
	while(1)
	{
		memset(inBuff, 0, sizeof(inBuff));
		std::fgets(inBuff, sizeof(inBuff), stdin);
		
		/* Remove new line character. */
		char* it = inBuff + sizeof(inBuff) - 1;
		for(; it > inBuff; --it)
		{
			if(*it == '\n' || *it == '\r')
			{
				for(; it > inBuff && (*it == '\n' || *it == '\r'); --it);
				it[1] = 0;
			}
		}
		args.request.data = inBuff;
		
		if(outClient.call(args))
		{
			ROS_INFO("Successfully sent \"%s\".", args.request.data.c_str());
		}
		else
		{
			ROS_INFO("Error occurred sending data.");
		}
	}
	
	return(0);
}
