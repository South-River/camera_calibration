#include "mvsua_camera.h"
#include "../tic_toc.h"
// #include "opencv2/core/types_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>


BYTE *g_readBuf[MVSUA_CAMERA_MAX] = {NULL};
int G_pixel_bit;
std::string G_format;
class LambdaBody : public cv::ParallelLoopBody {
public:
  explicit LambdaBody(const std::function<void(const cv::Range &)> &body) { _body = body; }
  void operator()(const cv::Range &range) const override { _body(range); }

private:
  std::function<void(const cv::Range &)> _body;
};

void __attribute__((weak)) ImageCallback(int camera_id, cv::Mat& image);

void ImageCallback(int camera_id, cv::Mat& image){}

void GrabImageCallback(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead* pFrameHead, PVOID pContext)
{
    TicToc t0;
    if ((G_pixel_bit == 255) && (G_format == "mono8"))//位深255，fomat "mono8" 8位灰度图
    {
        CameraSdkStatus status;
        //将RAW数据转换成挃定格式的图像数据。 默认转换为BGR24格式的图像数据。
        //TODO: 可以直接转存  不通过CameraImageProcess
        status = CameraImageProcess(hCamera, pFrameBuffer, g_readBuf[hCamera-1], pFrameHead);//将获得的相机原始输出图像数据进行处理，叠加饱和度、颜色增益和校正、降噪等处理效果，最后得到RGB888格式的图像数据。
        //转换成功后,使用m_pFrameBuffer中的数据迕行后续处理。
        cv::Mat matImage(cvSize(pFrameHead->iWidth, pFrameHead->iHeight), CV_8UC1, g_readBuf[hCamera-1]);
        ImageCallback(hCamera, matImage);
    }
    else
    {
        ROS_ERROR("bit is not 255 mono8!!! \n");
    }
    // std::cout << "cost: " << t0.toc() << " ms" << std::endl;
    
    
}

namespace camera_driver
{
    mvsua_camera::mvsua_camera(int camera_num, int pixel_bit, std::string format)
    {
        if(camera_num == 0) iCameraCounts_ = MVSUA_CAMERA_MAX;
        else iCameraCounts_ = camera_num;
        pixel_bit_ = pixel_bit;
        G_pixel_bit = pixel_bit;
        format_ = format;
        G_format = format;
        camera_initialized_ = MVSUA_Init();//初始化(第68行)
    }

    mvsua_camera::~mvsua_camera()//析构函数
    {
        ROS_INFO("Release MV Camera");
        camera_initialized_ = false;//初始化状态:false
        MVSUA_UnInit();//反初始化(第114行)
    }

    bool mvsua_camera::MVSUA_Init()//初始化函数
    {
        int iStatus = -1;
        tSdkCameraDevInfo pCameraList[MVSUA_CAMERA_MAX];

        CameraSdkInit(1);//相机SDK初始化，在调用任何SDK其他接口前，必须先调用该接口进行初始化。该函数在整个进程运行期间只需要调用一次。 形参：SDK内部提示信息和界面的语种（1表示中文）
        CameraEnumerateDevice(pCameraList, &iCameraCounts_);//枚举设备，并建立设备列表。 pCameraList:设备列表数组指针  iCameraCounts_:设备个数
        std::cout << "iCameraCounts_: " << iCameraCounts_ << std::endl;//输出设备个数
        if(iCameraCounts_ == 0){//如果设备个数是0（没有找到设备）则提示检查设备连接或者驱动是否成功安装
            ROS_INFO("\033[31mYou should check the connection or /etc/udev/rules.d/88-mvusb.rules\033[31m");
            ROS_ERROR("\033[1;31mNo MVSUA camera find! exit.\033[1;31m");
            return false;
        }
        
        for(int hCamera = 1; hCamera <= iCameraCounts_; hCamera++){//遍历设备
            iStatus = CameraInit(&pCameraList[hCamera-1], -1, -1, &hCamera);//相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口。
            //第1个参数：该相机的设备描述信息，由CameraEnumerateDevice函数获得（程序第74行）
            //第2个参数：相机初始化时使用的参数加载方式。-1表示使用上次退出时的参数加载方式。
            //第3个参数：初始化时使用的参数组。-1表示加载上次退出时的参数组。
            //第4个参数：相机的句柄指针，初始化成功后，该指针返回该相机的有效句柄，在调用其他相机相关的操作接口时，都需要传入该句柄，主要用于多相机之间的区分。
            if (iStatus != CAMERA_STATUS_SUCCESS) {
                ROS_ERROR("Camera %d Init failed!", hCamera);
                return false;
            }

            CameraGetCapability(hCamera, &g_tCapability_[hCamera-1]);//获得相机的特性描述结构体。
            if((pixel_bit_ == 255) && (format_ == "mono8"))
            {
                CameraSetMediaType(hCamera, 1); // 设置相机的输出原始数据格式。
                g_readBuf[hCamera-1] = (unsigned char *) malloc(g_tCapability_[hCamera-1].sResolutionRange.iHeightMax * g_tCapability_[hCamera-1].sResolutionRange.iWidthMax);
		        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);//设置CameraGetImageBuffer函数的图像处理的输出格式
                std::cout << "format: mono8" << std::endl;
            }
            else
            {
                ROS_ERROR("bit is not 255 mono8 \n");
                return false;
            }
            
            
            ROS_INFO("Create MVSUADriver to read image from MVSUA Camera %d.", hCamera);
        }

        return true;
    }

    void mvsua_camera::MVSUA_UnInit()//反初始化
    {
        for(int hCamera = 1; hCamera <= iCameraCounts_; hCamera++){
            CameraUnInit(hCamera);
        }
        ROS_INFO("\033[32mRelease Camera!\033[32m");
    }

    bool mvsua_camera::LoadParam(int camera_id,   // 相机id
                                 int analog_gain, // 模拟增益
                                 int gamma,       // gamma
                                 int saturation,  //图像饱和度
                                 int shaprpness,  //图像锐化参数
                                 int width,  //图像宽度
                                 int height,   //图像高度
                                 int w_offset,   //横向偏移
                                 int h_offset,   //纵向偏移
                                 int speed,   //相机速度
                                 int autoexposure_target,  //自动曝光标志
                                 double exposure_time)  //曝光时间
    {
        if(camera_initialized_ == false) return false; //如果相机初始化失败，返回
        if(camera_id <= 0 || camera_id > iCameraCounts_){ 
            ROS_INFO("\033[33mNo such camera. Please check camera id %d!\033[33m", camera_id);
            return false;
        }

        CameraSetTriggerMode(camera_id,1);   //NOTE:需要在自动曝光前设置软件触发才能使用自动曝光  第二个形参:1 表示软件触发
        int piModeSel;
        if (CameraGetTriggerMode(camera_id, &piModeSel) == CAMERA_STATUS_SUCCESS )//设置相机触发模式
        {
            std::cout << "success Get TriggerMode " << piModeSel << " " << std::endl;//相机节点启动后会输出一次相机触发模式
        }
        
        CameraSetAnalogGain(camera_id, analog_gain);//设置模拟增益
        std::cout << "Gain: " << analog_gain*g_tCapability_[camera_id-1].sExposeDesc.fAnalogGainStep << std::endl;//相机节点启动后会输出一次增益
        //如果是自动曝光模式
        if(exposure_time == 0.0){
                std::cout << "auto exposure " << std::endl;
            
            CameraSetAeState(camera_id, true);//设置曝光模式 true:自动 false:手动
            CameraSetAeTarget(camera_id, autoexposure_target);//设定自动曝光的亮度目标值。设定范围由CameraGetCapability函数获得。
            CameraSetAntiFlick(camera_id, false);//设置自动曝光时抗频闪功能的使能状态。对于手动曝光模式下无效
            // CameraSetLightFrequency(camera_id, 1);//60Hz(室外不设置，室内可设置)
            // if(CameraSetAeState(camera_id, true) == CAMERA_STATUS_SUCCESS)
            //     std::cout << "set auto exposure " << std::endl;
        }
        //如果不是自动曝光模式
        else
        {
            // CameraSetAeState(camera_id, false);
            // CameraSetExposureTime(camera_id, exposure_time);
            if (CameraSetAeState(camera_id, false) == CAMERA_STATUS_SUCCESS && CameraSetExposureTime(camera_id, exposure_time*1000.) == CAMERA_STATUS_SUCCESS)//微秒×1000转换为毫秒
            {
                double pfExposureTime;
                CameraGetExposureTime(camera_id, &pfExposureTime);//获得曝光时间
                std::cout << "success set exposure time " << pfExposureTime*0.001 << "ms" << std::endl;
            }
        }

        // CameraSetGamma(camera_id, gamma);
        CameraSetSaturation(camera_id, saturation);//设置饱和度
        CameraSetSharpness(camera_id, shaprpness);//设置锐化参数
        // CameraSetGain(camera_id, 100, 100, 100);

        // CameraSetMirror(camera_id, 0, false);
        // CameraSetMirror(camera_id, 1, false);
        tSdkImageResolution *p = g_tCapability_[camera_id-1].pImageSizeDesc;
        p->iWidth = width;
        p->iHeight = height;
        p->iHOffsetFOV = w_offset;
        p->iVOffsetFOV = h_offset;
        CameraSetImageResolution(camera_id, p);
        CameraSetFrameSpeed(camera_id, speed);
        CameraSetCallbackFunction(camera_id, GrabImageCallback, (PVOID)0, NULL);

        // //设置扫描方向 用于调整图像
        // if(pixel_bit_ == 4095){
        //     int bEnable = 1;
        //     if (CameraSetHardwareMirror(camera_id, 1, bEnable) == CAMERA_STATUS_SUCCESS )
        //     {
        //         std::cout << "success SetHardwareMirror 1 " << bEnable << " " << std::endl;
        //     }
        //     int pbEnable;
        //     if (CameraGetHardwareMirror(camera_id, 1, &pbEnable) == CAMERA_STATUS_SUCCESS )
        //     {
        //         std::cout << "success GetHardwareMirror 1 " << pbEnable << " " << std::endl;
        //     }
        //     if (CameraGetHardwareMirror(camera_id, 0, &pbEnable) == CAMERA_STATUS_SUCCESS )
        //     {
        //         std::cout << "success GetHardwareMirror 0 " << pbEnable << " " << std::endl;
        //     }
        // }
        if(pixel_bit_ == 255){
            int bEnable = 0;
            if (CameraSetHardwareMirror(camera_id, 1, bEnable) == CAMERA_STATUS_SUCCESS )
            {
                std::cout << "success SetHardwareMirror 1 " << bEnable << " " << std::endl;
            }
            int pbEnable;
            if (CameraGetHardwareMirror(camera_id, 1, &pbEnable) == CAMERA_STATUS_SUCCESS )
            {
                std::cout << "success GetHardwareMirror 1 " << pbEnable << " " << std::endl;
            }
            if (CameraGetHardwareMirror(camera_id, 0, &pbEnable) == CAMERA_STATUS_SUCCESS )
            {
                std::cout << "success GetHardwareMirror 0 " << pbEnable << " " << std::endl;
            }
        }
        
        

        CameraRstTimeStamp(camera_id);  //Reset时间戳
        /*让SDK进入工作模式，开始接收来自相机发送的图像
        数据。如果当前相机是触发模式，则需要接收到
        触发帧以后才会更新图像。    */
        CameraPlay(camera_id);  //开始曝光

        sleep(2);

        ROS_INFO("\033[32mcamera %d load param down! Finish Init\033[32m", camera_id);
        return true;
    }
}

// bool UnpackRaw12(BYTE* pIn, BYTE* pOut, tSdkFrameHead const* pHead)
// {
// 	UINT RawType = pHead->uiMediaType;
// 	if ((RawType & CAMERA_MEDIA_TYPE_EFFECTIVE_PIXEL_SIZE_MASK) != CAMERA_MEDIA_TYPE_OCCUPY12BIT
// 		|| (RawType & CAMERA_MEDIA_TYPE_COLOR_MASK) != CAMERA_MEDIA_TYPE_MONO)
// 	{
// 		return false;
// 	}

// 	BYTE *pTemp = pIn;
// 	USHORT *Pout = (USHORT*)pOut;
// 	UINT loops;
// 	UINT i,j,v;
// 	int iWidth = pHead->iWidth;
// 	int iHeight = pHead->iHeight;

// 	loops = (iWidth * iHeight) / 2;

// 	if (RawType < CAMERA_MEDIA_TYPE_BAYGR12_PACKED_MV)
// 	{
//         // Pout = Pout+iWidth * iHeight;
//         // std::cout << "1" << std::endl;
// 		// for (i = 0,j =0;i < loops; i++)
// 		// {
// 		// 	v = (pTemp[j] << 4) | (pTemp[j + 1] & 0x0f);
// 		// 	Pout[0] = (USHORT)v;
			
// 		// 	v = (pTemp[j + 2] << 4) | ((pTemp[j + 1] & 0xf0) >> 4);
// 		// 	Pout[1] = (USHORT)v;

// 		// 	Pout += 2;
// 		// 	j += 3;
// 		// }
//         //FIXME: 时间长8ms 且有很多噪点：可能是并行时同时写Pout  //不要写同一个内存 如用同一个全局变量，数组不写同一个位置
//         cv::parallel_for_(cv::Range(0,loops), LambdaBody([&](const cv::Range &range) 
//         {
//             for (int i = range.start; i < range.end; i++)
//             {
//                 UINT vv;
//                 vv = (pTemp[3*i] << 4) | (pTemp[3*i + 1] & 0x0f);
//                 Pout[2*i] = (USHORT)vv;
                
//                 vv = (pTemp[3*i + 2] << 4) | ((pTemp[3*i + 1] & 0xf0) >> 4);
//                 Pout[2*i+1] = (USHORT)vv;
//             }
//         }));
//         cv::setNumThreads(2);  //加载线程个数 过多增加cpu负担且速度没有太大提高 过少又慢 
// 	}
// 	else
// 	{
//         // std::cout << "2" << std::endl;

// 		for (i = 0,j =0;i < loops;i++)
// 		{
// 			v = (pTemp[j] << 4) | (pTemp[j + 2] & 0x0f);
// 			Pout[0] = (USHORT)v;

// 			v = (pTemp[j + 1] << 4) | ((pTemp[j + 2] & 0xf0) >> 4);
// 			Pout[1] = (USHORT)v;

// 			Pout += 2;
// 			j += 3;
// 		}
// 	}

// 	return true;
// }
