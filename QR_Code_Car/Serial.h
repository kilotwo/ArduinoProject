#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <string>
#include <ctype.h>

//此处修改下位机设备地址，终端使用 ls /dev 命令查看设备地址，显示列表中如：ttyUSB0, ttyUSB1 ...
//#define COM_DEVICE "/dev/ttyUSB1"

static int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,};
static int name_arr[] = {115200, 38400,  19200,  9600,  4800,  2400,  1200,  300,};

class Serial
{
public:
    Serial();
    int openSerialDevice(char * COM_DEVICE);
    void closeSerialDevice(int fd);
    void set_speed(int fd, int speed);
    int set_Parity(int fd,int databits,int stopbits,char parity);
    const char* serial_read(int fd);
    int serial_send(int fd, const char *buf, int len);
    void serial_clear();
    ~Serial(void);
private:
    int data_len = 512;
    char cstr[512];
};

#endif // SERIAL_H_INCLUDED
