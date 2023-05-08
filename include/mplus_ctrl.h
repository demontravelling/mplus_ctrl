#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <error.h>
#include <termios.h>
#include <malloc.h>
#include <sys/types.h>
#include <sys/stat.h>
#define IMU_PORT_ID "/dev/ttyS0"

typedef struct termios termios_t;

typedef struct serial_data{

    unsigned char databuf[100];//发送/接受数据
    int serfd;//串口文件描述符

}SerData;




