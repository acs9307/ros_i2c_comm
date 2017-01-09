#include <ros/ros.h>
#include <i2c_comm/I2CIn.h>
#include <i2c_comm/I2COut.h>
#include <std_msgs/String.h>

#include <vector>
#include <iostream>
#include <map>

extern "C"
{
	#include <picom/i2c.h>
	#include <alib-c/alib_error.h>
}

/* Map of file descriptors to I2C devices. 
 * First value is I2C addr, second is the file descriptor. */
std::map<uint8_t, int> fds;

int reconnectSlave(uint8_t addr)
{
	/* Close the old socket if it was open. */
	int fd = fds[addr];
	if(fd > 0)
		close(fd);
	
	fd = connectToSlave(addr);
	if(fd <= 0)
		ROS_INFO("Could not connect to slave %d.", addr);
	else
	{
		fds[addr] = fd;
		ROS_INFO("Connected to slave %d.", addr);
	}
	
	return(fd);
}
int getSlave(uint8_t addr)
{
	int fd = fds[addr];
	if(fd <= 0)
		fd = reconnectSlave(addr);
	
	return(fd);
}

//bool i2c_out(uint8_t addr, std_msgs::String& data)
bool i2c_out(uint8_t addr, std::vector<uint8_t>& data)
{
	/* Get the file descriptor. */
	int fd = getSlave(addr);
	if(fd <= 0)	
		return(false);
	
	/* Write the data. */
	int err = write(fd, &data[0], data.size());
	if(err < 0)
	{
		ROS_INFO("Write failed, attempting to reconnect to slave %d.", addr);
		fd = reconnectSlave(addr);
		if(fd <= 0)
			return(false);
		
		err = write(fd, &data[0], data.size());
		if(err < 0)
		{
			ROS_INFO("Second write attempt to %d failed!", addr);
			return(false);
		}		
	}
	else if(err != data.size())
		ROS_INFO("Wrote %d bytes to slave %d, should have written %d bytes.",
			err, addr, data.size());
	
	return(true);
}
int32_t i2c_in(uint8_t addr, std::vector<uint8_t>& data, uint32_t dataLen)
{
	if(data.size() < dataLen)
		data.resize(dataLen);
	
	int fd = getSlave(addr);
	if(fd <= 0)
		return ALIB_BAD_ARG;
	
	int32_t err = read(fd, &data[0], dataLen);
	if(err < 0)
	{
		fd = reconnectSlave(addr);
		if(fd <= 0 || read(fd, &data[0], dataLen) < 0)
		{
			ROS_INFO("Error reading data from %d.  Errno: %d", addr, errno);
			err = ALIB_UNKNOWN_ERR;
		}
	}
	
	return(err);
}

/* Writes a message to a specific I2C device. 
 * 
 * Arguments:
 * 	Request:
 * 		uint8_t addr - I2C device address. 
 * 		std::vector<uint8_t> data - Data to transmit to slave. 
 *
 * Returns:
 * 		true:  Message successfully sent. 
 * 		false: Message could not be successfully sent. The  
 * 			file descriptor was probably not open or could not be opened. */
bool i2c_out_cb(i2c_comm::I2COut::Request& req, i2c_comm::I2COut::Response& res)
{
	return(i2c_out(req.addr, req.data));
}
/* Reads a message from a specific I2C device. 
 *
 * Arguments:
 * 		Requests:
 *			uint8_t addr     - I2C device address. 
 *			uint32_t dataLen - Number of bytes to read. 
 *		Response:
 *			std::vector<uint8_t> data - The data received from the slave. 
 * 			uint32_t dataLen - The number of bytes read.
 *
 * Returns:
 * 		true:  Message successfully sent. 
 * 		false: Message could not be successfully sent. The  
 * 			file descriptor was probably not open or could not be opened. */
bool i2c_in_cb(i2c_comm::I2CIn::Request& req, 
	i2c_comm::I2CIn::Response& res)
{	
	int32_t result = i2c_in(req.addr, res.data, req.dataLen);
	if(result < 0)
	{
		res.dataLen = 0;
		return(false);
	}
	else
	{
		res.dataLen = result;
		return(true);
	}
}

int main(int argc, char** argv)
{
	/* Init ros. */
	ros::init(argc, argv, "i2c_srv");
	
	/* Init node. */
	ros::NodeHandle nh;
	ros::ServiceServer outService = nh.advertiseService("i2c_out", i2c_out_cb);
	ros::ServiceServer inService = nh.advertiseService("i2c_in", i2c_in_cb);
	
	ROS_INFO("Services started!");
	ros::spin();
	
	return(0);
}