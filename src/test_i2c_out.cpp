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

	if(argc > 1)
		args.request.addr = atoi(*(++argv));
	else
		args.request.addr = 8;
	
	const size_t buffSize = 64 * 1024;
	char buff[buffSize];
	
	std::cout<<"Type in the data to send.\n";	
	memset(buff, 0, buffSize);
	std::fgets((char*)buff, buffSize, stdin);
	
	/* Remove new line character. */
	for(char* it = (char*)buff; *it != 0 && it < (char*)(buff + buffSize); ++it)
	{
		if(*it == '\n' || *it == '\r')
		{
			*it = 0;
			break;
		}
	}
	std::string strData = buff;
	
	args.request.data = std::vector<uint8_t>((uint8_t*)strData.c_str(), (uint8_t*)(strData.c_str() + strData.length()));
	
	if(outClient.call(args))
	{
		ROS_INFO("Successfully sent %d bytes.", args.request.data.size());
	}
	else
	{
		ROS_INFO("Error occurred sending data.");
	}
	
	return(0);
}
