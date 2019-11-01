#include "SerialCon.h"

int SerialCon::open_serial(char* dev_name, int baud, int vtime, int vmin)
{
	int fd;
	struct termios newtio;

	fd = open(dev_name, O_RDWR | O_NOCTTY);
	if (fd < 0)
	{
		dprintf(2, "Device OPEN FAIL %s\n", dev_name);
		return -1;
	}

	memset(&newtio, 0, sizeof(newtio));
	newtio.c_iflag = IGNPAR; // non parity
	newtio.c_oflag = 0;
	newtio.c_cflag = CS8 | CLOCAL | CREAD;

	switch (baud)
	{
	case 115200: newtio.c_cflag |= B115200; break;
	case 57600: newtio.c_cflag |= B57600; break;
	case 38400: newtio.c_cflag |= B38400; break;
	case 19200: newtio.c_cflag |= B19200; break;
	case 9600: newtio.c_cflag |= B9600; break;
	case 4800: newtio.c_cflag |= B4800; break;
	case 2400: newtio.c_cflag |= B2400; break;
	default: newtio.c_cflag |= B115200; break;
	}

	//
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = vtime;
	newtio.c_cc[VMIN] = vmin;

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

	return fd;
}

void SerialCon::close_serial(int fd)
{
	close(fd);
}