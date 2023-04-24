#include "Main/headfiles.h"
#include "camera_manager.h"

CameraManager::CameraManager()
{

}

int CameraManager::InitCamera()
{
    iCameraCounts = 1;
    iStatus=-1;

    iplImage = NULL;
    channel=3;
    explore_time=10000;
    picWidth=960;
    picHeight=720;


    sham_img=Mat(picHeight,picWidth,CV_8UC3,Scalar(0,255,0));

    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
    printf("state = %d\n", iStatus);

    printf("count = %d\n", iCameraCounts);
    //没有连接设备

    if(iCameraCounts==0){
        cout<<"industry camra linked failed!!!"<<endl;
        return -1;
    }

    //相机初始化。初始化成功后，才s能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

    //初始化失败
    printf("state = %d\n", iStatus);
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }
    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    CameraPlay(hCamera);

    CameraSetAeState(hCamera,FALSE);//设置为手动曝光模式
    CameraSetExposureTime(hCamera,10000); //曝光时间，单位为微妙    1秒=1000毫秒=1000微妙  曝光时间是快门开始到关闭的时间  5000微妙，1s大概采200次

#if  1

    memset(&sImageSize,0,sizeof(tSdkImageResolution));
    sImageSize.iIndex=0XFF;
    //    sImageSize.iHOffsetFOV=320;//163 242
    //    sImageSize.iVOffsetFOV=272;
    sImageSize.iHOffsetFOV=0;//163 242
    sImageSize.iVOffsetFOV=0;
    sImageSize.iWidthFOV=picWidth;
    sImageSize.iHeightFOV=picHeight;
    sImageSize.iWidth=picWidth;
    sImageSize.iHeight=picHeight;
    CameraSetImageResolution(hCamera,&sImageSize);
#else
    CameraSetImageResolution(hCamera,&tCapability.pImageSizeDesc[0]);
#endif
    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }
}

void CameraManager::SetExplore(int explore)
{
    explore_time=explore;
}

void CameraManager::SetPicSize(int height,int width)
{
    picWidth=width;
    picHeight=height;

}

Mat CameraManager::ReadImage()
{   //cout<<"CameraConnectTest:"<<CameraConnectTest(hCamera)<<endl;

        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
        {//摄像头连接成功，返回读图结果。
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);

            iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,channel);
            cvSetData(iplImage,g_pRgbBuffer,sFrameInfo.iWidth*channel);//此处只是设置指针，无图像块数据拷贝，不需担心转换效率
            //以下两种方式都可以显示图像或者处理图像
 
           Iimag=cv::cvarrToMat(iplImage);
   
            //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
            //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
            CameraReleaseImageBuffer(hCamera,pbyBuffer);
            //cout<<"cols"<<Iimag.cols<<endl;

            return Iimag;
        }
   else{
        cout<<"warning:camera loading failed..."<<endl;//摄像头掉线保护，返回欺骗图
       InitCamera();
       return sham_img;
    }
}
