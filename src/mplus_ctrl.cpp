#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

#include <sensor_msgs/Imu.h>

#include <iostream>

#include "utilcrc16.h"
#include "mplus_ctrl.h"
#include "clear_imu.h"
#define EXP_DOUBLE 1e-6
#define LEN_IMU_DATA 13
#define LEN_IMU_CHANGE 17
#define LEN_RECV_HEAD 3
#define LEN_IMU_CLEAR 8


ros::Publisher imu_pub;
bool flag_snd_clear = false;
char flag_recv_clear = 0;
bool flag_time_up = false;
const unsigned char read_data_buffer[8] = {0x08, 0x03, 0x00, 0x04 ,0x00, 0x04,0x05, 0x51};
const unsigned char read_ack_head[3] = {0x08 , 0x03 , 0x08};
const unsigned char clear0ins[13] = {0x08 ,0x10 ,0x00, 0x04 , 0x00,0x02, 0x04 ,0x43,0x4C ,0x52,0x00 ,0x35 ,0xF3};
const unsigned char clear_fb[8] = {0x08 , 0x10 ,0x00 ,0x04 ,0x00 ,0x02 ,0x00 ,0x90};
int current_level = 0;

unsigned int receive_cnt = LEN_IMU_DATA;

double move_conv[] = {1e-3, 0, 0, 0, 0, 0,
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3};

double static_conv[] = {1e-9, 0, 0, 0, 0, 0,
                        0, 1e-3, 1e-9, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e-9};

bool clear_func(mplus_ctrl::clear_imu::Request& req, mplus_ctrl::clear_imu::Response& resp)
{
    flag_snd_clear = true;
    resp.is_success = 1;
    flag_recv_clear = 0;
    ros::Time then = ros::Time::now();

    while(flag_recv_clear == 0)
    {
        ros::Time cur = ros::Time::now();
        double delta = (cur - then).toSec();
        if(delta > 5.0)
        {
            receive_cnt = LEN_IMU_DATA;
            break;
        }
    }

    resp.is_success = flag_recv_clear;
    return true;
}
void timeCallback(const ros::TimerEvent& event){
    flag_time_up = true;
}
void *sersend(void *arg)//串口发送线程函数
{
    SerData *snd = (SerData *)arg ;
    int ret;
    ros::Time last_time;
    ros::Time current;
    while(1)
    {
       current = ros::Time::now();
       if((current - last_time).toSec() < 0.025)
       {
           continue;
       }

       printf("ready send:\n");
       if(flag_snd_clear == false)
       {
           ret = write(snd->serfd,snd->databuf,8);
       }
       else if(flag_snd_clear == true)
       {
           printf("clear imu data:\n\n\n\n\n\n");
           flag_snd_clear = false;
           ret = write(snd->serfd,clear0ins,13);
           receive_cnt = LEN_IMU_CLEAR;
       }

       if(ret > 0){
           last_time = ros::Time::now();
           printf("send imu data:\n");
           for(int i = 0 ; i < 8 ; i ++)
           {
               printf("%02x ",snd->databuf[i]);
           }
           printf("\n");
       }else{
           printf("send error!\r\n");
       }


    }
}

SerData *rec;
std::vector<unsigned char> pkg;
void treat_read_data()
{
    //尝试构建完整数据包
    if(current_level < 3)
    {
       current_level = (rec->databuf[0] == read_ack_head[current_level])?(current_level + 1) : 0;
    }
    else if(current_level == 3)
    {
       pkg.push_back(read_ack_head[0]);
       pkg.push_back(read_ack_head[1]);
       pkg.push_back(read_ack_head[2]);
       pkg.push_back(rec->databuf[0]);
       current_level ++;
    }
    else
    {
       pkg.push_back(rec->databuf[0]);
       current_level ++;
    }


    //收到完整数据包
    unsigned char crc_buff[LEN_IMU_CHANGE];
    unsigned short crc_cnt;
    unsigned short back_cnt;
    bool flag_complete = false;
    if(current_level == receive_cnt)
    {
        printf("get compelete data:");
        for(int i = 0 ; i < receive_cnt ; i ++)
        {
            crc_buff[i] = pkg.at(i);
            printf("%02x ",crc_buff[i]);
        }
        printf("\n");

        crc_cnt = usMBCRC16(crc_buff, LEN_IMU_DATA -2);
        back_cnt = (crc_buff[LEN_IMU_DATA - 1] << 8) | (crc_buff[LEN_IMU_DATA - 2]);
        pkg.clear();
        current_level = 0;
        flag_complete = true;
    }

    //如果校验通过
    if(back_cnt == crc_cnt && flag_complete == true)
    {
        printf("finish CRC16 check out !!!\n");
        double angle,speed;
        angle = static_cast<double>((short)(crc_buff[3] << 8) | crc_buff[4]) / 10.0;
        speed = static_cast<double>((short)(crc_buff[5] << 8) | crc_buff[6]) / 10.0;

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.angular_velocity.z = speed/180.0*3.14159;
        imu_msg.orientation = tf::createQuaternionMsgFromYaw(angle/180.0*3.14159);

        imu_pub.publish(imu_msg);
        flag_complete = false;
    }
}

void treat_imu_clear()
{
    if(current_level < 8)
    {
      if(rec->databuf[0] == clear_fb[current_level])
      {
          current_level ++;
      }
      else
      {
          //receive_cnt = LEN_IMU_DATA;
          //flag_recv_clear = 1;
          current_level = 0;
      }

    }

    if(current_level == 8)
    {
       receive_cnt = LEN_IMU_DATA;
       current_level = 0;
       flag_recv_clear = 2;
    }
}

void *serrecv(void *arg)//串口发送线程函数
{

    int ret;
    rec= (SerData *)arg ;
    while(1){
       ret = read(rec->serfd,rec->databuf,1);
       if(ret < 0)
       {
           printf("read serial error!!!\n");
           continue;
       }

       if(receive_cnt == LEN_IMU_DATA)
       {
           treat_read_data();
       }
       else if(receive_cnt == LEN_IMU_CLEAR)
       {
           treat_imu_clear();
       }


       usleep(1000);

    }
}

bool IMUInit()
{
    pthread_t pid1,pid2;
    pthread_attr_t *pthread_arr1,*pthread_arr2;
    pthread_arr1 = NULL;
    pthread_arr2 = NULL;
    int serport1fd;


    /*   进行串口参数设置  */
    termios_t *ter_s = (termios_t *)malloc(sizeof(*ter_s));
    printf("ready to init imu ...");
    serport1fd = open(IMU_PORT_ID,O_RDWR | O_NOCTTY | O_NDELAY);//不成为控制终端程序，不受其他程序输出输出影响
    if(serport1fd < 0){
        printf("%s open faild\r\n",IMU_PORT_ID);
        return -1;
    }
    else
    {
        printf("sucessfully open %s\n",IMU_PORT_ID);
    }

    bzero(ter_s,sizeof(*ter_s));

    ter_s->c_cflag |= CLOCAL | CREAD; //激活本地连接与接受使能

    ter_s->c_cflag &= ~CSIZE;//失能数据位屏蔽
    ter_s->c_cflag |= CS8;//8位数据位

    ter_s->c_cflag &= ~CSTOPB;//1位停止位

    ter_s->c_cflag &= ~PARENB;//无校验位

    ter_s->c_cc[VTIME] = 0;
    ter_s->c_cc[VMIN] = 0;


    cfsetispeed(ter_s,B115200);//设置输入波特率
    cfsetospeed(ter_s,B115200);//设置输出波特率

    tcflush(serport1fd,TCIFLUSH);//刷清未处理的输入和/或输出

    if(tcsetattr(serport1fd,TCSANOW,ter_s) != 0){
            printf("com set error!\r\n");
    }


    SerData snd_data;
    SerData rec_data;

    snd_data.serfd = serport1fd;
    rec_data.serfd = serport1fd;

    for(int i = 0; i < 8 ; i ++)
    {
        snd_data.databuf[i] = read_data_buffer[i];
    }


    pthread_create(&pid1,pthread_arr1,sersend,(void *)&snd_data);
    pthread_create(&pid2,pthread_arr2,serrecv,(void *)&rec_data);

    return 1;
}


void imuClearCallBack(const std_msgs::Int8 & msg)
{
    flag_snd_clear = true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_mplus", 1000);

    ros::Subscriber imu_clear_sub = nh.subscribe("/set_imu_clear", 60, imuClearCallBack);
    ros::ServiceServer clear_service = nh.advertiseService("/imu_clear_serv", clear_func);

    //ros::Timer timer= nh.createTimer(ros::Duration(2),timeCallback);

    IMUInit();
    while(ros::ok())
    {

    }



}
