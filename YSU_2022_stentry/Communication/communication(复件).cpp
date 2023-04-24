#include "Main/headfiles.h"
#include "Communication/communication.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
using namespace std;

typedef signed char        int8_t;
typedef short              int16_t;
typedef int                int32_t;
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;

#define BufferLength       255
#define FrameHeader        0xA1



//陀螺仪及申请数据  ，0001类型帧的数据初始化
#define RECIEVE_CONTROLSYSMEGR(DATA_HEAD)\
{\
    Infantry.controlSysmsgR.colourMsg=DataBuff[DATA_HEAD];\
    \
    float_u temp_forFloat;	\
    temp_forFloat.data[3]=DataBuff[DATA_HEAD+1];\
    temp_forFloat.data[2]=DataBuff[DATA_HEAD+2];\
    temp_forFloat.data[1]=DataBuff[DATA_HEAD+3];\
    temp_forFloat.data[0]=DataBuff[DATA_HEAD+4];\
    Infantry.controlSysmsgR.yawAngle=temp_forFloat.f_data;\
    \
    temp_forFloat.data[3]=DataBuff[DATA_HEAD+5];\
    temp_forFloat.data[2]=DataBuff[DATA_HEAD+6];\
    temp_forFloat.data[1]=DataBuff[DATA_HEAD+7];\
    temp_forFloat.data[0]=DataBuff[DATA_HEAD+8];\
    Infantry.controlSysmsgR.yawSpeed=temp_forFloat.f_data;\
    \
    temp_forFloat.data[3]=DataBuff[DATA_HEAD+9];\
    temp_forFloat.data[2]=DataBuff[DATA_HEAD+10];\
    temp_forFloat.data[1]=DataBuff[DATA_HEAD+11];\
    temp_forFloat.data[0]=DataBuff[DATA_HEAD+12];\
    Infantry.controlSysmsgR.pitchAngle=temp_forFloat.f_data;\
    \
    temp_forFloat.data[3]=DataBuff[DATA_HEAD+13];\
    temp_forFloat.data[2]=DataBuff[DATA_HEAD+14];\
    temp_forFloat.data[1]=DataBuff[DATA_HEAD+15];\
    temp_forFloat.data[0]=DataBuff[DATA_HEAD+16];\
    Infantry.controlSysmsgR.pitchSpeed=temp_forFloat.f_data;\
    \
    Infantry.controlSysmsgR.requestMsg=DataBuff[DATA_HEAD+17];\
    }

#define RECIEVE_PCRESPONSEMSG(DATA_HEAD)\
{\
    Infantry.pcRespinsemsg.recieveRsp=DataBuffer[DATA_HEAD];\
    Infantry.pcRespinsemsg.upGimbalRecieveRsp=DataBuffer[DATA_HEAD+1];\
    Infantry.pcRespinsemsg.catchFlag=DataBuffer[DATA_HEAD+2];\
    Infantry.pcRespinsemsg.upGimbalCatchFlag=DataBuffer[DATA_HEAD+3];\
    }

#define RECIEVE_BUFFATTACKMSG(DATA_HEAD)\
{\
    float_u temp_forFloat;	\
    temp_forFloat.data[3]=DataBuffer[DATA_HEAD];\
    temp_forFloat.data[2]=DataBuffer[DATA_HEAD+1];\
    temp_forFloat.data[1]=DataBuffer[DATA_HEAD+2];\
    temp_forFloat.data[0]=DataBuffer[DATA_HEAD+3];\
    Infantry.buffAttackmsg.yawErr=temp_forFloat.f_data;\
    \
    temp_forFloat.data[3]=DataBuffer[DATA_HEAD+4];\
    temp_forFloat.data[2]=DataBuffer[DATA_HEAD+5];\
    temp_forFloat.data[1]=DataBuffer[DATA_HEAD+6];\
    temp_forFloat.data[0]=DataBuffer[DATA_HEAD+7];\
    Infantry.buffAttackmsg.pitchErr=temp_forFloat.f_data;\
    \
    Infantry.buffAttackmsg.shootFlag=DataBuffer[DATA_HEAD+8];\
    }

#define RECIEVE_AMORATTACKMSG(DATA_HEAD)\
{\
    float_u temp_forFloat;	\
    temp_forFloat.data[3]=DataBuffer[DATA_HEAD];	\
    temp_forFloat.data[2]=DataBuffer[DATA_HEAD+1];	\
    temp_forFloat.data[1]=DataBuffer[DATA_HEAD+2];	\
    temp_forFloat.data[0]=DataBuffer[DATA_HEAD+3];	\
    Infantry.amorAttackmsg.yawErr=temp_forFloat.f_data;	\
    \
    temp_forFloat.data[3]=DataBuffer[DATA_HEAD+4];	\
    temp_forFloat.data[2]=DataBuffer[DATA_HEAD+5];	\
    temp_forFloat.data[1]=DataBuffer[DATA_HEAD+6];	\
    temp_forFloat.data[0]=DataBuffer[DATA_HEAD+7];	\
    Infantry.amorAttackmsg.pitchErr=temp_forFloat.f_data;	\
    \
    Infantry.amorAttackmsg.shootFlag=DataBuffer[DATA_HEAD+8];	\
    }
//0006类型的帧的初始化
#define RECIEVE_CONTROLSYSMEGF(DATA_HEAD)\
{\
    float_u temp_forFloat;	\
    temp_forFloat.data[3]=DataBuffer[DATA_HEAD];\
    temp_forFloat.data[2]=DataBuffer[DATA_HEAD+1];\
    temp_forFloat.data[1]=DataBuffer[DATA_HEAD+2];\
    temp_forFloat.data[0]=DataBuffer[DATA_HEAD+3];\
    Infantry.controlSysmsgF.yawAngle=temp_forFloat.f_data;\
    \
    temp_forFloat.data[3]=DataBuffer[DATA_HEAD+4];\
    temp_forFloat.data[2]=DataBuffer[DATA_HEAD+5];\
    temp_forFloat.data[1]=DataBuffer[DATA_HEAD+6];\
    temp_forFloat.data[0]=DataBuffer[DATA_HEAD+7];\
    Infantry.controlSysmsgF.yawSpeed=temp_forFloat.f_data;\
    \
    temp_forFloat.data[3]=DataBuffer[DATA_HEAD+8];\
    temp_forFloat.data[2]=DataBuffer[DATA_HEAD+9];\
    temp_forFloat.data[1]=DataBuffer[DATA_HEAD+10];\
    temp_forFloat.data[0]=DataBuffer[DATA_HEAD+11];\
    Infantry.controlSysmsgF.pitchAngle=temp_forFloat.f_data;\
    \
    temp_forFloat.data[3]=DataBuffer[DATA_HEAD+12];\
    temp_forFloat.data[2]=DataBuffer[DATA_HEAD+13];\
    temp_forFloat.data[1]=DataBuffer[DATA_HEAD+14];\
    temp_forFloat.data[0]=DataBuffer[DATA_HEAD+15];\
    Infantry.controlSysmsgF.pitchSpeed=temp_forFloat.f_data;\
    \
    Infantry.controlSysmsgF.requestMsg=DataBuffer[DATA_HEAD+16];\
    }



unsigned char Communication::Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8 ^ (*pchMessage++);//^按位异或
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return(ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Communication::Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
    return (ucExpected == pchMessage[dwLength - 1]);
}


/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Communication::Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return;
    ucCRC = Get_CRC8_Check_Sum((unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
    pchMessage[dwLength - 1] = ucCRC;
}



/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Communication::Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == 0)
    {
        return 0xFFFF;
    }
    while (dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^
                                                       (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}


/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Communication::Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}


/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Communication::Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum((uint8_t*)pchMessage, dwLength - 2, CRC_INIT);
    pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}
/***********************************    ↑    DJI提供的CRC校检函数   ↑  ***********************************/


Communication::Communication()
{

}
bool Communication::open(const char* portname, int baudrate, char parity, char databit, char stopbit, char synchronizeflag)
{
    pHandle[0] = -1;
    pHandle[0] = ::open(portname,O_RDWR|O_NDELAY );// 阻塞 O_NDELAY    非阻塞 O_NOCTTY

    // 打开失败，则打印失败信息，返回false
    if(pHandle[0] == -1)
    {
        std::cout << portname << " open failed , may be you need 'sudo' permission. OR. Failed to open the serial port. Please check the serial port number!" << std::endl;
        return false;
    }

    // 设置串口参数
    // 创建串口参数对象
    struct termios options;
    // 先获得串口的当前参数
    if(tcgetattr(pHandle[0],&options) < 0)
    {
        std::cout << portname << " open failed , get serial port attributes failed." << std::endl;
        return false;
    }
    bzero(&options, sizeof(options));
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSIZE;
    options.c_lflag &= ~ICANON;//切换模式
    options.c_oflag &= ~ICANON;//切换模式
    // options.c_cc[VMIN]=26;

    // 设置波特率
    switch(baudrate)
    {
    case 4800:
        cfsetispeed(&options,B4800);
        cfsetospeed(&options,B4800);
        break;
    case 9600:
        cfsetispeed(&options,B9600);
        cfsetospeed(&options,B9600);
        break;
    case 19200:
        cfsetispeed(&options,B19200);
        cfsetospeed(&options,B19200);
        break;
    case 38400:
        cfsetispeed(&options,B38400);
        cfsetospeed(&options,B38400);
        break;
    case 57600:
        cfsetispeed(&options,B57600);
        cfsetospeed(&options,B57600);
        break;
    case 115200:
        cfsetispeed(&options,B115200);
        cfsetospeed(&options,B115200);
        break;
    default:
        std::cout << portname << " open failed , unkown baudrate , only support 4800,9600,19200,38400,57600,115200." << std::endl;
        return false;
    }

    // 设置校验位
    switch(parity)
    {
    // 无校验
    case 0:
        options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
        options.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用
        break;
        // 设置奇校验
    case 1:
        options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
        options.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
        options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
        options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
        break;
        // 设置偶校验
    case 2:
        options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
        options.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
        options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
        options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
        break;
    default:
        std::cout << portname << " open failed , unkown parity ." << std::endl;
        return false;
    }

    // 设置数据位
    switch(databit)
    {
    case 5:
        options.c_cflag &= ~CSIZE;//屏蔽其它标志位
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag &= ~CSIZE;//屏蔽其它标志位
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag &= ~CSIZE;//屏蔽其它标志位
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag &= ~CSIZE;//屏蔽其它标志位
        options.c_cflag |= CS8;
        break;
    default:
        std::cout << portname << " open failed , unkown databit ." << std::endl;
        return false;
    }

    // 设置停止位
    switch(stopbit)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;//CSTOPB：使用1位停止位
        break;
    case 2:
        options.c_cflag |= CSTOPB;//CSTOPB：使用2位停止位
        break;
    default:
        std::cout << portname << " open failed , unkown stopbit ." << std::endl;
        return false;
    }


    // 激活新配置
    if((tcsetattr(pHandle[0],TCSANOW,&options))!=0)
    {
        std::cout << portname << " open failed , can not complete set attributes ." << std::endl;
        return false;
    }

    return true;
}

void Communication::close()
{
    if(pHandle[0] != -1)
    {
        ::close(pHandle[0]);
    }
}
int Communication::send(const void *buf,int len)
{
    int sendCount = 0;
    if(pHandle[0] != -1)
    {
        // 将 buf 和 len 转换成api要求的格式
        const char *buffer = (char*)buf;
        size_t length = len;
        // 已写入的数据个数
        ssize_t tmp;

        while(length > 0)
        {
            if((tmp = write(pHandle[0], buffer, length)) <= 0)
            {
                if(tmp < 0&&errno == EINTR)
                {
                    tmp = 0;
                }
                else
                {
                    break;
                }
            }
            length -= tmp;
            buffer += tmp;
        }

        sendCount = len - length;
    }
    return sendCount;
}
int Communication::Receive()
{

    memset(&DataBuffer,0,sizeof(DataBuffer));

    int read_count = 0;


    read(pHandle[0],((unsigned char*)(&DataBuffer)) ,24);

    int com_flag=0;
    while (com_flag<12)
    {
        if(DataBuffer[com_flag]==0xA1)
        {
            for(int i=0;i<12;i++)
            {
                DataBuff[i]=DataBuffer[com_flag+i];
            }
            break;
        }
        else
        {
            com_flag++;
        }
    }
    if(DataBuff[0] == FrameHeader)
    {
    for(int i=0;i<12;i++)
        printf("%x ",DataBuff[i]);//cout<<DataBuffer[i]<<" ";
    cout<<endl;}
    else
    {
        //cout<<"Can not receive."<<endl;
    }
    //    int len,fs_sel;
    //    fd_set fs_read;

    //    struct timeval time;

    //    FD_ZERO(&fs_read);
    //    FD_SET(pHandle[0],&fs_read);

    //    time.tv_sec =1;
    //    time.tv_usec =10000;

    //    //使用select实现串口的多路通信
    //    fs_sel = select(pHandle[0]+1,&fs_read,NULL,NULL,&time);
    // printf("fs_sel = %d\n",fs_sel);
    //    if(fs_sel)
    //    {
    //        len = read(pHandle[0],rcv_buf,data_len);
    //        return len;
    //    }
    //    else
    //    {
    //        return 0;
    //    }
}
int Communication::Read_Data(unsigned char* buff)
{
    int nread = 0;
    int fd_max;
    int nselect;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(pHandle[0],&readfds);
    fd_max = pHandle[0]+1;

    nselect = select(fd_max, &readfds, NULL, NULL, NULL);
    memset(buff, 0, sizeof(buff));

    if(nselect <= 0)
        printf("select failed");
    else if(FD_ISSET(pHandle[0], &readfds) >0)
    {

        nread = read(pHandle[0], buff, 25);
        buff[nread] = '\0';
    }

    int j = 0;
    while(buff[j] != '\0')
    {
        printf("the readable data is 0x%x\n", buff[j]);

        j++;
    }

    return nread;
}


void Communication::clearReadBuff()
{
    tcflush(pHandle[0], TCIOFLUSH);
    // memset(DataBuffer,0,sizeof(DataBuffer));
//    cout << "clean buff" << endl;
}
int Communication::receive(unsigned char *buf,int maxlen)
{
    int receiveCount = read(pHandle[0],buf,maxlen);

    return receiveCount;
}
void Communication::ref_pcResponse(uint8_t recieveRsp, uint8_t upGimbalRecieveRsp, uint8_t catchFlag, uint8_t upGimbalCatchFlag)
{
    uint8_t data_to_send[11];
    data_to_send[0] = 0xA1;//固定头
    //数据段长度
    data_to_send[1] = 2;
    data_to_send[2] = 0;
    //包序号
    data_to_send[3] = 0x00;

    //CRC校验
    Append_CRC8_Check_Sum(data_to_send, 5);
    //帧类型
    data_to_send[5] = 0x0002 & 0xFF;
    data_to_send[6] = (0x0002 >> 8);
    //数据内容
    data_to_send[7] = ((recieveRsp & 0x01) << 7) | ((upGimbalCatchFlag & 0x01) << 6);
    //
    data_to_send[8] = ((catchFlag & 0x01) << 7) | ((upGimbalCatchFlag & 0x01) << 6);

    Append_CRC16_Check_Sum(data_to_send, 11);
    cout << "send yun tai " << endl;
    send(data_to_send, 11);//需要改的接收发送函数

}


void Communication::ref_buffAttackMsg(float yawErr, float pitchErr, uint8_t shootFlag)
{
    uint8_t data_to_send[18];
    data_to_send[0] = 0xA1;//固定头
    //数据段长度
    data_to_send[1] = 9;
    data_to_send[2] = 0;
    //包序号
    data_to_send[3] = 0x00;

    //CRC校验
    Append_CRC8_Check_Sum(data_to_send, 5);
    //帧类型
    data_to_send[5] = 0x0003 & 0xFF;
    data_to_send[6] = (0x0003 >> 8);

    //数据
    float_u temp_forFloat;
    temp_forFloat.f_data = yawErr;
    data_to_send[7] = temp_forFloat.data[0];
    data_to_send[8] = temp_forFloat.data[1];
    data_to_send[9] = temp_forFloat.data[2];
    data_to_send[10] = temp_forFloat.data[3];

    temp_forFloat.f_data = pitchErr;
    data_to_send[11] = temp_forFloat.data[0];
    data_to_send[12] = temp_forFloat.data[1];
    data_to_send[13] = temp_forFloat.data[2];
    data_to_send[14] = temp_forFloat.data[3];

    data_to_send[15] = shootFlag;

    Append_CRC16_Check_Sum(data_to_send, 18);
    cout << "fu fu fu"<< endl;
    send(data_to_send, 18);//需要改的接收发送函数



}
//flag为true时为自瞄数据，为false时为打陀螺数据
void Communication::ref_amorAttackMsg(float yawErr, float pitchErr,float distance, uint8_t shootFlag, uint8_t holderflag, bool flag)
{
    uint8_t data_to_send[21];
    data_to_send[0] = 0xA1;//固定头
    //数据段长度
    data_to_send[1] = 14;
    data_to_send[2] = 0;
    //包序号
    data_to_send[3] = 0x00;

    //CRC校验
    Append_CRC8_Check_Sum(data_to_send, 5);
//    //帧类型
//    if (flag = true)
//    {
//        data_to_send[5] = 0x0004 & 0xFF;
//        data_to_send[6] = (0x0004 >> 8);
//    }
//    else
//    {
//        data_to_send[5] = 0x0005 & 0xFF;
//        data_to_send[6] = (0x0005 >> 8);
//    }

    //数据
    float_u temp_forFloat;
    temp_forFloat.f_data = yawErr;
    // temp_forFloat.f_data = 0;
    data_to_send[5] = temp_forFloat.data[0];
    data_to_send[6] = temp_forFloat.data[1];
    data_to_send[7] = temp_forFloat.data[2];
    data_to_send[8] = temp_forFloat.data[3];

    temp_forFloat.f_data = pitchErr;
  //   temp_forFloat.f_data = 0;
    data_to_send[9] = temp_forFloat.data[0];
    data_to_send[10] = temp_forFloat.data[1];
    data_to_send[11] = temp_forFloat.data[2];
    data_to_send[12] = temp_forFloat.data[3];

    temp_forFloat.f_data = distance;
    data_to_send[13] = temp_forFloat.data[0];
    data_to_send[14] = temp_forFloat.data[1];
    data_to_send[15] = temp_forFloat.data[2];
    data_to_send[16] = temp_forFloat.data[3];

    data_to_send[17] = shootFlag;
    data_to_send[18] = holderflag;

    Append_CRC16_Check_Sum(data_to_send, 21);
    send(data_to_send, 21);//需要改的接收发送函数

}

void Communication::InitCom()
{
    open("/dev/ttyUSB0", 115200, 0, 8, 1);

}

void Communication::UpdateData(double *p_y_err)
{
    Infantry.amorAttackmsg.yawErr=p_y_err[0];
    Infantry.amorAttackmsg.pitchErr=p_y_err[1];
    cout<<"yaw:"<<Infantry.amorAttackmsg.yawErr<<endl;
    cout<<"pitch:"<<Infantry.amorAttackmsg.pitchErr<<endl;
}
void Communication::shoot_err(int shoot)
{
    Infantry.amorAttackmsg.shootFlag=shoot;
    cout<<"shootflag:"<<Infantry.amorAttackmsg.shootFlag<<endl;
}
void Communication::upflag()
{
    Infantry.amorAttackmsg.holderflag=0;
    cout<<"holderflag:"<<Infantry.amorAttackmsg.holderflag<<endl;
}
void Communication::downflag()
{
    Infantry.amorAttackmsg.holderflag=1;
    cout<<"holderflag:"<<Infantry.amorAttackmsg.holderflag<<endl;
}
void Communication::communication(Infantry_Struct Infantry)
{
    uint16_t start_pos = 0;		//当前帧起始位置
    uint16_t next_start_pos=0;  //下一帧起始位置，由于裁判系统有时候会连续发送，因此一次串口空闲中断可能会接收多个帧
    int recieveNum = 0;
    int count = 0;
    //可能你们处理数据的逻辑问题，或者串口初始化细节问题
    //	start_pos=next_start_pos;
    //	w.Read_Data(DataBuffer);
    //ref_amorAttackMsg(Infantry.amorAttackmsg.yawErr, Infantry.amorAttackmsg.pitchErr, Infantry.amorAttackmsg.shootFlag, Infantry.amorAttackmsg.holderflag, true);
    memset(&DataBuff,0,sizeof(DataBuff));
    Receive();
//    ref_amorAttackMsg(52, 48, 21, 0, 0, true);
    ref_amorAttackMsg(Infantry.amorAttackmsg.yawErr, Infantry.amorAttackmsg.pitchErr,0, Infantry.amorAttackmsg.shootFlag, Infantry.amorAttackmsg.holderflag, true);
   // usleep(50000);//xia yun tai man yi dian
   usleep(10000);
    //    ref_amorAttackMsg(Infantry.amorAttackmsg.yawErr, Infantry.amorAttackmsg.pitchErr,0, Infantry.amorAttackmsg.shootFlag, Infantry.amorAttackmsg.holderflag, true);
//    cout<<"yawErr"<<Infantry.amorAttackmsg.yawErr<<endl;
//    cout<<"pitchErr"<<Infantry.amorAttackmsg.pitchErr<<endl;
//    cout<<"shootFlag"<<Infantry.amorAttackmsg.shootFlag<<endl;
//    cout<<"holderFlag"<<Infantry.amorAttackmsg.holderflag<<endl;
    //return;
    //cout<<num<<endl;
    //return;
    //	cout<<"errno is "<<errno<<endl;
    //	cout<<streeor(errno)<<endl;
    //  perror(strerror(errno));
//    if (DataBuff[0] == FrameHeader && Verify_CRC8_Check_Sum(DataBuff, 5))
//    {
//        //		cout<<"start_pos is   " << start_pos << endl;
//        cout << "receive 25 byte" << endl;
//        uint16_t sum_length = ((DataBuff[start_pos + 2] << 8) | DataBuff[start_pos + 1]) + 7;
//        next_start_pos = sum_length + start_pos;
//        //cout<<"next_start_pos is "<<next_start_pos<<endl;
//        if (next_start_pos >= 255)
//        {
//            start_pos = 0;
//            next_start_pos=0;
//        }
//        uint16_t data_startPos = start_pos + 5;   //帧头部份
//        //cout<<"data_startPos is"<<data_startPos<<endl;
//        //cmd_id = ((DataBuffer[start_pos + 6] << 8) | DataBuffer[start_pos + 5]);
//        if (Verify_CRC8_Check_Sum(DataBuff, 5))//帧头部分CRC校验
//        {
//            if (Verify_CRC16_Check_Sum(DataBuff, sum_length))//整包CRC校验
//            {
//                count++;
//                cout<<"count is"<<count<<endl;
////                cmd_id=0x0001;
////                switch (cmd_id)
////                {
////                case 0x0001:
//                    Infantry.pcResponserev.recieveRsp = 1;
//                    RECIEVE_CONTROLSYSMEGR(data_startPos);
//                    cout << "the data is:" <<Infantry.controlSysmsgR.colourMsg<<" "<< Infantry.controlSysmsgR.yawAngle << "   " << Infantry.controlSysmsgR.yawSpeed << "   " << Infantry.controlSysmsgR.pitchAngle << "   " << Infantry.controlSysmsgR.pitchSpeed <<"  "<< Infantry.controlSysmsgR.requestMsg <<endl;
////
////                        //自瞄
//                    cout << "Automatic aiming mode!"<<endl;
//                    ref_amorAttackMsg(Infantry.amorAttackmsg.yawErr, Infantry.amorAttackmsg.pitchErr,0, Infantry.amorAttackmsg.shootFlag, Infantry.amorAttackmsg.holderflag, true);
////                        cout<<"yawErr"<<Infantry.amorAttackmsg.yawErr<<endl;
////                        cout<<"pitchErr"<<Infantry.amorAttackmsg.pitchErr<<endl;
////                        cout<<"shootFlag"<<Infantry.amorAttackmsg.shootFlag<<endl;
////                        cout<<"holderFlag"<<Infantry.amorAttackmsg.holderflag<<endl;
////                        ref_amorAttackMsg(52, 48, 21, 0,0, true);
////                      break;
////                    }
////                    break;

//            }
//            else
//            {
//                //校验错误
////                cout << "CRC verification of the whole package failed!" << endl;
//            }
//        }
//        else
//        {
//            //校验错误
////            cout << "CRC check failure of frame header!" << endl;
//        }
//    }
//    else
//    {
////        cout << "Data format error (length error or frame header error or CRC check failed)!" << endl;
//        clearReadBuff();
//    }

}




