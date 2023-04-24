#include "Main/headfiles.h"
#include "Communication/communication.h"

typedef signed char        int8_t;
typedef short              int16_t;
typedef int                int32_t;
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;

/***********************************    ￄ1�7?    DJI提供的CRC� �检函数   ￄ1�7?  ***********************************/
/*
** Descriptions: CRC8 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint8_t Communication::Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
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
uint32_t Communication::Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
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
/***********************************    ￄ1�7?    DJI提供的CRC� �检函数   ￄ1�7?  ***********************************/


/**********************************由Kirs使用CppLinuxSerial重写************************************/
Communication::Communication()
:serialport("/dev/ttyUSB0", mn::CppLinuxSerial::BaudRate::B_115200, mn::CppLinuxSerial::NumDataBits::EIGHT, mn::CppLinuxSerial::Parity::NONE, mn::CppLinuxSerial::NumStopBits::ONE)
{

}

void Communication::InitCom(const std::string &device, mn::CppLinuxSerial::BaudRate baudRate, mn::CppLinuxSerial::NumDataBits numDataBits, mn::CppLinuxSerial::Parity parity, mn::CppLinuxSerial::NumStopBits numStopBits)
{
    this->serialport.SetDevice(device);
    this->serialport.SetBaudRate(baudRate);
    this->serialport.SetNumDataBits(numDataBits);
    this->serialport.SetParity(parity);
    this->serialport.SetNumStopBits(numStopBits);

    this->serialport.SetTimeout(-1);
}

bool Communication::open()
{
    this->serialport.Open();
    if(this->serialport.getState() == mn::CppLinuxSerial::State::OPEN)
        return true;
    else
        return false;
}

bool Communication::close()
{
    this->serialport.Close();
    if(this->serialport.getState() == mn::CppLinuxSerial::State::CLOSED)
        return true;
    else
        return false;
}

void Communication::sendMsg()
{
    // buyao kan xiamian de nahang zhushi nashi bubin de xieyi
    // 数据ￄ1�7?                               固定ￄ1�7?                数据段长ￄ1�7? 包序ￄ1�7? CRC8  帧类ￄ1�7?                             yerr                perr               dis                  shoot    aim_mode                                    延迟时间      CRC16
    //uint8_t writeBuf[21] = {FrameHeader, 0x00, 0x0A, 0x00,   0x00,  FrameTypePC2MCU, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,    (uint8_t)Infantry.aim_mode, 0x00, 0x00, 0x00, 0x00};
    uint8_t writeBuf[21] = {0xA1, 14, 0, 0x00, 0x00,   \
                            0x00, 0x00, 0x00, 0x00, \
                            0x00, 0x00, 0x00, 0x00,   \
                            0x00, 0x00, 0x00, 0x00,   \
                            0x00, 0x00, \
                            0x00, 0x00};
    Append_CRC8_Check_Sum(writeBuf, 5);

    float_to_byte temp_data_y;
    temp_data_y.f_data = Infantry.amorAttackmsg.yawErr;
    writeBuf[5] = temp_data_y.data[0];
    writeBuf[6] = temp_data_y.data[1];
    writeBuf[7] = temp_data_y.data[2];
    writeBuf[8] = temp_data_y.data[3];

    float_to_byte temp_data_p;
    temp_data_p.f_data = Infantry.amorAttackmsg.pitchErr;
    writeBuf[9] = temp_data_p.data[0];
    writeBuf[10] = temp_data_p.data[1];
    writeBuf[11] = temp_data_p.data[2];
    writeBuf[12] = temp_data_p.data[3];

    float_to_byte temp_data_d;
    temp_data_d.f_data = Infantry.amorAttackmsg.distance;
    writeBuf[13] = temp_data_d.data[0];
    writeBuf[14] = temp_data_d.data[1];
    writeBuf[15] = temp_data_d.data[2];
    writeBuf[16] = temp_data_d.data[3];

    writeBuf[17] = Infantry.amorAttackmsg.shootFlag;
    writeBuf[18] = Infantry.amorAttackmsg.holderFlag;

    Append_CRC16_Check_Sum(writeBuf, 21);
    
    std::vector<uint8_t> writeVec(writeBuf, writeBuf + 21);
    serialport.WriteBinary(writeVec);
}

/*
int Communication::recvMsg()
{
    std::vector<uint8_t> readVec;
    serialport.ReadBinary(readVec);

    for(int i=0; i<=readVec.size()-16; i++){
        if(readVec[i] == FrameHeader){
            uint8_t readbuf[16];
            for(int j=0; j<16; j++){
                readbuf[j] = readVec[i+j];
            }
            uint8_t verifybuf[16];
            memcpy(verifybuf, readbuf, 16);
            Append_CRC16_Check_Sum(verifybuf, 16);
            if(readbuf[14] == verifybuf[14] && readbuf[15] == verifybuf[15]){
                short_to_byte temp_data_s;
                temp_data_s.data[0] = readbuf[6];
                temp_data_s.data[1] = readbuf[7];
                Infantry.shoot_speed = temp_data_s.s_data;
                short_to_byte temp_data_y;
                temp_data_y.data[0] = readbuf[8];
                temp_data_y.data[1] = readbuf[9];
                Infantry.controlSysmsg.yawAngle = temp_data_y.s_data;
                short_to_byte temp_data_p;
                temp_data_p.data[0] = readbuf[10];
                temp_data_p.data[1] = readbuf[11];
                Infantry.controlSysmsg.pitchAngle = temp_data_y.s_data;

                std::cout << "aim_mode: " << Infantry.aim_mode << std::endl;
            }
        }
    }
}
*/

void Communication::UpdateData(double *p_y_err)
{
    Infantry.amorAttackmsg.yawErr=(float)p_y_err[0];
    Infantry.amorAttackmsg.pitchErr=(float)p_y_err[1];
    std::cout<<"yaw:"<<Infantry.amorAttackmsg.yawErr<<std::endl;
    std::cout<<"pitch:"<<Infantry.amorAttackmsg.pitchErr<<std::endl;
}

void Communication::shoot_err(int shoot)
{
    Infantry.amorAttackmsg.shootFlag=shoot;
}

void Communication::upflag()
{
    Infantry.amorAttackmsg.holderFlag=0;
    cout<<"holderflag:"<<Infantry.amorAttackmsg.holderFlag<<endl;
}
void Communication::downflag()
{
    Infantry.amorAttackmsg.holderFlag=1;
    cout<<"holderflag:"<<Infantry.amorAttackmsg.holderFlag<<endl;
}

void Communication::communication(Infantry_Struct Infantry)
{
    sendMsg();
    int timedelay = 5;
    auto t1 = std::chrono::high_resolution_clock::now();
    while (1) {
        auto t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = t2 - t1;
        if(elapsed.count() >= timedelay) break;
    }
}
