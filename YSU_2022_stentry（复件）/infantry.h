#ifndef INFANTRY_H
#define INFANTRY_H
typedef /*_packed*/ struct
{
    float yawAngle;
    float yawSpeed;
    float pitchAngle;
    float pitchSpeed;
    uint8_t requestMsg;
} controlSysMsgR_t;//陀螺仪及请求数据	类型0x0001
/**/
//typedef /*_packed*/ struct
//{
//	uint8_t recieveRsp : 1;
//	uint8_t upGimbalRecieveRsp :1;
//	uint8_t catchFlag : 1;
//	uint8_t upGimbalCatchFlag : 1;
//} pcResponseMsg_t;//PC 状态反馈数据	类型位0x0002   10hz频率发送

typedef /*_packed*/ struct
{
    uint8_t recieveRsp : 1;
    uint8_t upGimbalRecieveRsp : 1;
} pcResponseRev_t;

typedef /*_packed*/ struct
{
    uint8_t catchFlag : 1;
    uint8_t upGimbalCatchFlag : 1;
} pcResponseCatch_t;

typedef /*_packed*/ struct
{
    float yawErr;
    float pitchErr;
    uint8_t shootFlag;
} buffAttackMsg_t;//大符击打数据

typedef /*_packed*/ struct
{
    float yawErr=0;
    float pitchErr=0;
    uint8_t shootFlag=0;
} amorAttackMsg_t;//自瞄、打陀螺数据

typedef /*_packed*/ struct
{
    float yawAngle;
    float yawSpeed;
    float pitchAngle;
    float pitchSpeed;
    uint8_t requestMsg;
} controlSysMsgF_t;//哨兵下云台陀螺仪数据

typedef union
{
    uint8_t data[4];
    float f_data;
}float_u;

typedef struct
{
    controlSysMsgR_t controlSysmsgR;
    //pcResponseMsg_t	pcRespinsemsg;
    buffAttackMsg_t buffAttackmsg;
    amorAttackMsg_t amorAttackmsg;
    controlSysMsgF_t controlSysmsgF;
    pcResponseRev_t pcResponserev;
    pcResponseCatch_t pcResponsecatch;
}Infantry_Struct;
Infantry_Struct Infantry;
#endif // INFANTRY_H
