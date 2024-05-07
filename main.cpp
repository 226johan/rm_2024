#include <iostream>
#include"opencv4/opencv2/opencv.hpp"
#include"include/mindvision/camera_mindvision_running.h"
#include"include/Preprocess/Preprocess.h"
#include"include/Armor/ArmorDetetion.h"
#include"include/Tracker/Tracker.h"
#include "include/Resolver/Resolver.h"
#include"include/KalmanFilter/KalmanFilter.h"
#include "include/SerialPort/SerialPort.h"



Mat src;
Mat Preprocess_image;
Trackers track_obj;         // 追踪器对象
Resolver resolver_obj;      // 解算器对象
Kalmanfilter kalman;        // 卡尔曼对象
MindvisionCamera camera;    // 相机对象
ArmorDetetion bridge_link;  // 自瞄处理器对象
// 串口对象
Uart InfoPort;
int fd_serial =0;
bool serial_state=0;

bool ishave_track= false;

HostComputerData RobotInfo;             // 发送
GroundChassisData MainControlInfo;      // 接收

//EnemyColor g_enemy_color;

float time_=0;
int main() {


    serial_state = InfoPort.Init_serial(fd_serial,115200) +1;

    while(!serial_state)
    {
        std::cout<<"serial state error"<<std::endl;
        return 0;
    }

    // 自动发射用
    RobotInfo.if_real_shoot = 0;
    camera.runCamera();
    while(true)
    {
        Mat temp(image_height,image_width,CV_8UC3,Scalar(0,0,0));
        std::chrono::steady_clock::time_point time1 = std::chrono::steady_clock::now();

        camera.Do();
        camera.getImage(src);
        if(!src.empty())
        {
            bridge_link.Image_init(src);
//            imshow("src",src);

            // 串口读取数据
             InfoPort.GetMode(fd_serial,MainControlInfo);
            /****
             *
             * 模式选择判断
             *
             */

            /// 自瞄模式
            // 图像预处理
            Preprocess_image= Preprocess(src,0  );      // 强制红色模式
//            Preprocess_image= Preprocess(src,1  );      // 强制蓝色模式
//            Preprocess_image= Preprocess(src,static_cast<int>(MainControlInfo.color) );     // 串口接收模式
            // 寻找灯条
            bridge_link.FindLight(Preprocess_image);
            // 寻找装甲板
            bridge_link.FindArmor(Preprocess_image);
            // 识别数字
            bridge_link.Digital_recognition(src, bridge_link.armors_, bridge_link.target_armors);
            if(bridge_link.target_armors.size()!=0) {
                // 确定目标装甲板
                /***
                 * 目标装甲板数量判断，若只有一个则直接选择击打该装甲板
                 * 若有多个装甲板，
                 * 1.评分模式
                 * 2.距离最近模式
                 */
                bridge_link.Target_Confirm(src, bridge_link.target_armors);
                // 追踪器判断

                imshow("src",src);
                track_obj.TrackJudge(src);

                //测量每个目标装甲版的距离和角度
                for (auto &target_armor: bridge_link.target_armors) {
                    resolver_obj.DistanceMeasurer(target_armor);
                }

                // 解算弹道
                resolver_obj.AimTraget(bridge_link.target_armors);

                RobotInfo.Yaw.f = resolver_obj.send_yaw * 1.0f;
                RobotInfo.Pitch.f = resolver_obj.send_pitch * 1.0f;

                track_obj.last_send_yaw=RobotInfo.Yaw.f;
                track_obj.last_send_pitch=RobotInfo.Pitch.f;
            }
            else        // 掉帧模式   貌似存在隐患，待审查   多帧未匹配可能导致程序异常退出
            {
                track_obj.armor_drop_count--;
                if(track_obj.armor_drop_count<=5&&track_obj.armor_drop_count>0)
                {
                    putText(bridge_link.src_, "Lost Frame Mode", cv::Point2f(500, 55), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                    RobotInfo.Yaw.f=track_obj.last_send_yaw;
                    RobotInfo.Pitch.f=track_obj.last_send_pitch;
                }
                if(track_obj.armor_drop_count<0)
                {
                    track_obj.if_first_frame= true;
                    RobotInfo.Yaw.f=0.f;
                    RobotInfo.Pitch.f=0.f;
                    RobotInfo.if_shoot=0;
                }
            }
            RobotInfo.Yaw.f = RobotInfo.Yaw.f * 0.8f;
            RobotInfo.Pitch.f = RobotInfo.Pitch.f;
            std::cout<<"send pitch"<<RobotInfo.Pitch.f<<std::endl;
            std::cout<<"send yaw"<<RobotInfo.Yaw.f<<std::endl;


            /***
             *
             * 是否使用卡尔曼
             *
             */


            // 卡尔曼
            if(CLOSE_KF)
            {
                if(RobotInfo.Yaw.f!=0.0f&&RobotInfo.Pitch.f!=0.0f)
                {
                    resolver_obj.bullet_fight_time= ((resolver_obj.send_distance*0.001f)/30.f);
                    kalman.gain_yaw = MainControlInfo.gain_yaw.f * 1.55;
                    kalman.send_pitch = MainControlInfo.gain_pitch.f * 1.25;

                    kalman.send_yaw = RobotInfo.Yaw.f;
                    kalman.send_pitch = RobotInfo.Pitch.f;

                    if(track_obj.if_first_frame== true)
                    {
                        track_obj.if_first_frame= false;
                        kalman.InitKalman();
                    }
                    else
                    {
                        kalman.UpDate(time_);
                    }
                    kalman.KalmanPredict(resolver_obj.bullet_fight_time *1.2f);

                    RobotInfo.Yaw.f = kalman.send_yaw;
                    RobotInfo.Pitch.f = kalman.send_pitch;
                    RobotInfo.distance.f=resolver_obj.send_distance/1000.0;
                }
            }


            // 死区限制
            if ((fabs(RobotInfo.Yaw.f) < 0.01) || (fabs(RobotInfo.Yaw.f) > 25.0) || (isnan(RobotInfo.Yaw.f)))
            {
                RobotInfo.Yaw.f = 0;
            }
            if ((fabs(RobotInfo.Pitch.f) < 0.1) || (fabs(RobotInfo.Pitch.f) > 25.0) || (isnan(RobotInfo.Pitch.f)))
            {
                RobotInfo.Pitch.f = 0;
            }


            imshow("dst",Preprocess_image);
            waitKey(10);
//            std::cout<<"this"<<std::endl;
            std::chrono::steady_clock::time_point time2 = std::chrono::steady_clock::now();
            std::chrono::duration<double>         time_used = std::chrono::duration_cast<std::chrono::duration<double>>(time2 - time1);
            time_                                           = static_cast<float>(time_used.count());
            double fps = 1.0/time_;
//            std::cout<<time_<<std::endl;

            // 串口发送数据
            InfoPort.TransformTarPos(fd_serial,RobotInfo);

        }
        else
        {
            std::cout<<"frame empty"<<std::endl;
        }


    }
    CameraUnInit(camera.hCamera);
    free(camera.g_pRgbBuffer);

    return 0;
}