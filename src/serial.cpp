#include "serial.h"

bool serial::init(){
#ifdef __arm__
	serial_fd = open(dev_name.data(), O_RDWR | O_NONBLOCK);
#elif __x86_64__
	serial_fd = open(dev_name.data(), O_RDWR | O_NOCTTY);
#else
	serial_fd = open(dev_name.data(), O_RDWR | O_NOCTTY);
#endif
	if(serial_fd<0){
		printf("can not open device %s\n",dev_name.data());
		return false;
	}
	if(!config(baudrate, 8, 'N', 1, false)){
		printf("set serial config error\n");
		return false;
	}
	return true;
}
bool serial::close(){
	if(serial_fd!=-1)
	::close(serial_fd);
	serial_fd=-1;
	return true;
}
size_t serial::send(const char *buf, int len){
	if(NULL==buf||len<0)
		return -1;
	else
		return write(serial_fd, buf, len);
}
size_t serial::recv(char *buf, int len){
	if(NULL==buf||len<0)
		return -1;
	else
		return read(serial_fd,buf,len);
}
size_t serial::send(std::string &buf){
	return serial::send(buf.data(),buf.size());
}
size_t serial::recv(std::string &buf){
	memset(recv_buf, 0x0, RECV_SIZE);
	int ret=serial::recv(recv_buf, RECV_SIZE);
	buf=std::string(recv_buf);
	return ret;
}
void serial::setdev(std::string _dev_name){
	dev_name=_dev_name;
}
void serial::setbaudrate(int _baudrate){
	baudrate=_baudrate;
}

bool serial::config(int baudrate, char data_bits, char parity_bits,
             char stop_bits, bool testForData){
	int st_baud[] = { B4800,  B9600,   B19200,  B38400,
                    B57600, B115200, B230400, B921600 };
	int std_rate[] = { 4800,   9600,   19200,   38400,   57600,  115200,
                     230400, 921600, 1000000, 1152000, 3000000 };

	int            i, j;
	struct termios newtio, oldtio;
	/* save current port parameter */
	if (tcgetattr(serial_fd, &oldtio) != 0)
	{
	printf("fail to save current port\n");
	return false;
	}
	memset(&newtio, 0, sizeof(newtio));

	/* config the size of char */
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	/* config data bit */
	switch (data_bits)
	{
	case 7:
	  newtio.c_cflag |= CS7;
	  break;
	case 8:
	  newtio.c_cflag |= CS8;
	  break;
	default:
	  newtio.c_cflag |= CS8;
	  break; //8N1 default config
	}
	/* config the parity bit */
	switch (parity_bits)
	{
	/* odd */
	case 'O':
	case 'o':
	  newtio.c_cflag |= PARENB;
	  newtio.c_cflag |= PARODD;
	  break;
	/* even */
	case 'E':
	case 'e':
	  newtio.c_cflag |= PARENB;
	  newtio.c_cflag &= ~PARODD;
	  break;
	/* none */
	case 'N':
	case 'n':
	  newtio.c_cflag &= ~PARENB;
	  break;
	default:
	  newtio.c_cflag &= ~PARENB;
	  break; //8N1 default config
	}
	/* config baudrate */
	j = sizeof(std_rate) / 4;
	for (i = 0; i < j; ++i)
	{
	if (std_rate[i] == baudrate)
	{
	  /* set standard baudrate */
	  cfsetispeed(&newtio, st_baud[i]);
	  cfsetospeed(&newtio, st_baud[i]);
	  break;
	}
	}
	/* config stop bit */
	if (stop_bits == 1)
	newtio.c_cflag &= ~CSTOPB;
	else if (stop_bits == 2)
	newtio.c_cflag |= CSTOPB;
	else
	newtio.c_cflag &= ~CSTOPB; //8N1 default config
#if __x86_64__
	if (testForData)
	{
	newtio.c_cc[VTIME] = 8;
	newtio.c_cc[VMIN]  = 0;
	}
	else
	{
	newtio.c_cc[VTIME] = 1;
	newtio.c_cc[VMIN]  = 18;
	}
#endif
	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	newtio.c_oflag &= ~OPOST;

	/* flush the hardware fifo */
	tcflush(serial_fd, TCIFLUSH);

	/* activite the configuration */
	if ((tcsetattr(serial_fd, TCSANOW, &newtio)) != 0)
	{
	printf("failed to activate serial configuration\n");
	return false;
	}
	return true;
}



serial::serial(std::string _dev_name,int _baudrate){
	dev_name=_dev_name;
	baudrate=_baudrate;
}