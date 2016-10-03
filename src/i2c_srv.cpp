#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <picom/i2c.h>


/* Map of file descriptors to I2C devices. 
 * First value is I2C addr, second is the file descriptor. */
std::map<uint8_t, int> fds;

int reconnectSlave(uint8_t addr)
{
	/* Close the old socket if it was open. */
	int fd = fds[addr];
	if(fd > 0)
		close(fd);
	
	fd = connectToSlave(req.addr);
	if(fd <= 0)
		ROS_INFO("Could not connect to slave %d.", addr);
	else
	{
		fds[addr] = fd;
		ROS_INFO("Connected to slave %d.", req.addr);
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

bool i2c_out(uint8_t addr, std_msg::String& data)
{
	/* Get the file descriptor. */
	int fd = getSlave(addr);
	if(fd <= 0)	
		return(false);
	
	/* Write the data. */
	int err = write(fd, data.c_str(), data.length() + 1);
	if(err < 0)
	{
		ROS_INFO("Write failed, attempting to reconnect to slave %d.", addr)
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
std_msg::String i2c_in(uint8_t addr, uint64_t readCount)
{
	int fd = getSlave(addr);
	if(fd <= 0)
		return("");
	
	std_msg::String rval;
	char* dataIn = new char[readCount];
	memset(dataIn, 0, readCount);
	if(dataIn)
	{
		int err = read(fd, dataIn, readCount);
		if(err < 0)
		{
			fd = reconnectSlave(addr);
			if(fd <= 0 || read(fd, dataIn, readCount) < 0)
			{
				ROS_INFO("Error reading data from %d.  Errno: %d", addr, errno);
				goto if_error;
			}
		}
		
		rval = dataIn;
		
	if_error:
		delete[] dataIn;
	}
	
	return(rval);
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
std_msgs::String i2c_in_cb(i2c_comm::I2CIn::Request& req, 
	i2c_comm::I2CIn::Response& res)
{
	res.data = i2c_in(req.addr, req.dataLen);
	return(res.data);
}

int main(int argc, char** argv)
{
	/* Init ros. */
	ros::init(argc, argv, "i2c_srv");
	
	/* Init node. */
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("i2c_out", 1000, i2c_out_cb);
	ros::Subscriber sub = nh.subscribe("i2c_in", 1000, i2c_in_cb);
	ros::spin();
	
	return(0);
}