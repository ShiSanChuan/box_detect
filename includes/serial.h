#ifndef _SERIAL_H
#define _SERIAL_H

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

// ttyTHS2
// 
class serial
{
public:
	serial(std::string _dev_name="/dev/ttyUSB0",int _baudrate=115200);
	~serial();
	void setdev(std::string _dev_name);
	void setbaudrate(int _baudrate);
	size_t send(const char * buf,int len);
	size_t recv(char * buf,int len);
	size_t send(std::string &buf);
	size_t recv(std::string &buf);
	bool init();
	bool close();
	bool config(int baudrate, char data_bits, char parity_bits,
                 char stop_bits, bool testForData);
private:
	const static long long RECV_SIZE=2048;
	std::string dev_name;
	int baudrate;
	int serial_fd;
	char recv_buf[RECV_SIZE];
};

#endif