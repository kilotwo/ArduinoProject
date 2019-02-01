#include "Serial.h"

Serial::Serial(void)
{
}

Serial::~Serial(void)
{
}

int Serial::openSerialDevice(char * COM_DEVICE)
{
    int fd;

    fd = open(COM_DEVICE, O_RDWR);

    return fd;
}

void Serial::closeSerialDevice(int fd)
{
    close(fd);
}

/**
*@brief  设置串口通信速率
*@param  fd     类型 int  打开串口的文件句柄
*@param  speed  类型 int  串口速度
*@return  void
*/
void Serial::set_speed(int fd, int speed)
{
    int   i;
    int   status;
    struct termios   Opt;

    tcgetattr(fd, &Opt);
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {
        if  (speed == name_arr[i]) {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);
            if  (status != 0) {
                perror("tcsetattr fd1");
                return;
            }
            tcflush(fd,TCIOFLUSH);
        }
    }
}

/**
*@brief   设置串口数据位，停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄
*@param  databits 类型  int 数据位   取值 为 7 或者8
*@param  stopbits 类型  int 停止位   取值为 1 或者2
*@param  parity  类型  char  效验类型 取值为N,E,O,,S
*/
int Serial::set_Parity(int fd,int databits,int stopbits,char parity)
{
    struct termios options;
    if  ( tcgetattr( fd,&options)  !=  0) {
        perror("SetupSerial 1");
        return -1;
    }
    options.c_cflag &= ~CSIZE;
    switch (databits) /*设置数据位数*/
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size\n");
        return -2;
    }

    switch (parity)
    {
    case 'n':
    case 'N':
        options.c_cflag &= ~PARENB;   /* Clear parity enable */
        options.c_iflag &= ~INPCK;     /* Enable parity checking */
        break;
    case 'o':
    case 'O':
        options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
        options.c_iflag |= INPCK;             /* Disnable parity checking */
        break;
    case 'e':
    case 'E':
        options.c_cflag |= PARENB;     /* Enable parity */
        options.c_cflag &= ~PARODD;   /* 转换为偶效验*/
        options.c_iflag |= INPCK;       /* Disnable parity checking */
        break;
    case 'S':
    case 's':  /*as no parity*/
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;break;
    default:
        fprintf(stderr,"Unsupported parity\n");
        return -3;
    }
    /* 设置停止位*/
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
       break;
    default:
         fprintf(stderr,"Unsupported stop bits\n");
         return -4;
    }
    options.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
//    options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON|INPCK);
    tcflush(fd,TCIFLUSH);
    options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    if (tcsetattr(fd,TCSANOW,&options) != 0){
        perror("SetupSerial 3");
        return -5;
    }

    return 0;
}

//读取串口
const char* Serial::serial_read(int fd)
{
    //static char cstr[512];
    char buf[1];
    int i = 0;
    while(read(fd, buf, 1) > 0){
        if(buf[0] != '\n'){
            cstr[i] = buf[0];
            i++;
        }else{
            cstr[i] = '\0';
            break;
        }
    }
    //string str = cstr;
    return cstr;
}

//串口发送
int Serial::serial_send(int fd, const char *buf, int len)
{
    int nWrite;

    nWrite = write(fd, buf, len);
    return nWrite;
}

//串口数据清空
void Serial::serial_clear()
{
    for(int i=0;i<data_len;i++)
        cstr[i] = 0;
}






