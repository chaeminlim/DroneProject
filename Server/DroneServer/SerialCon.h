#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h> 
#include <sys/stat.h> 
#include <termios.h>
#include <fcntl.h>
#include <string.h>

class SerialCon
{
private:
public:
    SerialCon(){}
    int open_serial(char* dev_name, int baud, int vtime, int vmin);
    void close_serial(int fd);
};