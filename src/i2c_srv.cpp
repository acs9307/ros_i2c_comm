#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <i2c_comm/I2CIn.h>
#include <i2c_comm/I2COut.h>
#include <alib-c/alib_error.h>

extern "C"
{
	#include <picom/i2c.h>
}

/* Map of file descriptors to I2C devices. 
 * First value is I2C addr, second is the file descriptor. */
std::map<uint8_t, int> fds;

int reconnectSlave(uint8_t addr)
{
	#if 1
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
	#else
		return(-1);
	#endif
}
int getSlave(uint8_t addr)
{
	int fd = fds[addr];
	if(fd <= 0)
		fd = reconnectSlave(addr);
	
	return(fd);
}

//bool i2c_out(uint8_t addr, std_msgs::String& data)
bool i2c_out(uint8_t addr, std::string& data)
{
	/* Get the file descriptor. */
	int fd = getSlave(addr);
	if(fd <= 0)	
		return(false);
	
	/* Write the data. */
	int err = write(fd, data.c_str(), data.length() + 1);
	if(err < 0)
	{
		ROS_INFO("Write failed, attempting to reconnect to slave %d.", addr);
		fd = reconnectSlave(addr);
		if(fd <= 0)
			return(false);
		
		err = write(fd, data.c_str(), data.length() + 1);
		if(err < 0)
		{
			ROS_INFO("Second write attempt to %d failed!", addr);
			return(false);
		}		
	}
	else if(err != data.length() + 1)
		ROS_INFO("Wrote %d bytes to slave %d, should have written %d bytes.",
			err, addr, data.length());
	
	return(true);
}
alib_error i2c_in(uint8_t addr, uint64_t readCount, std::string& res)
{
	int fd = getSlave(addr);
	if(fd <= 0)
		return ALIB_BAD_ARG;
	
	alib_error err = ALIB_OK;
	char* dataIn = new char[readCount + 1];
	memset(dataIn, 0, readCount + 1);
	if(dataIn)
	{
		int err = read(fd, dataIn, readCount);
		if(err < 0)
		{
			fd = reconnectSlave(addr);
			if(fd <= 0 || read(fd, dataIn, readCount) < 0)
			{
				ROS_INFO("Error reading data from %d.  Errno: %d", addr, errno);
				err = ALIB_UNKNOWN_ERR;
				goto if_error;
			}
		}
		
		res = dataIn;
		
	if_error:
		delete[] dataIn;
	}
	
	return(err);
}

/* Writes a message to a specific I2C device. 
 * 
 * Arguments:
 * 	Request:
 * 		uint8_t addr - I2C device address. 
 * 		string	data - Data to transmit to slave. 
 * 	Response:
 * 		bool success - True if the operation completed successfully. 
 *
 * Returns:
 * 		true:  Message successfully sent. 
 * 		false: Message could not be successfully sent. The  
 * 			file descriptor was probably not open or could not be opened. */
bool i2c_out_cb(i2c_comm::I2COut::Request& req, i2c_comm::I2COut::Response& res)
{
	res.success = i2c_out(req.addr, req.data);
	return(res.success);
}
/* Reads a message from a specific I2C device. 
 *
 * Arguments:
 * 		Requests:
 *			uint8_t addr     - I2C device address. 
 *			uint64_t dataLen - Number of bytes to read. 
 *		Response:
 *			string data - The data received from the slave. 
 *
 * Returns:
 *		The data received from the slave. */
bool i2c_in_cb(i2c_comm::I2CIn::Request& req, 
	i2c_comm::I2CIn::Response& res)
{
	if(i2c_in(req.addr, req.dataLen, res.data) == ALIB_OK)
		return(true);
	else
		return(false);
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