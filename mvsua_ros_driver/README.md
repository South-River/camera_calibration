# Critical HIT ICRA 迈德威视相机ROS驱动模块

* [迈德威视官网](http://www.mindvision.com.cn/)
* [MV-SUA133GC-T工业相机](http://www.mindvision.com.cn/cpzx/info_7.aspx?itemid=1656&lcid=21)
* [SDK安装包](http://www.mindvision.com.cn/rjxz/list_12.aspx)

本功能包**主要功能**是迈德威视相机的ROS驱动，可通过`yaml`文件配置相机图像长宽以及图片中心偏置、**帧率**、**曝光**等，并发布相应的`/mvsua_cam/image_rawx`图像话题。

## 1. 文件结构
```bash
mvsua_ros_driver
├── config                              # 相机参数文件(相机内参)
├── image                               # 保存的图片路径
├── launch                              # launch文件
├── video                               # 保存的视频路径
├── mvsua_camera                        # 数据采集节点
│   ├── MVSDK                           # 迈德威视相机SDK
│   ├── mvsua_camera.cpp                # 相机驱动程序
│   └── mvsua_camera.h                  # 数据采集程序
├── camera_driver.cpp                   # 主函数
├── camera_driver.hpp
├── CMakeLists.txt
└── package.xml
```

## 2. 操作说明

### (1) 相机参数配置
相机参数配置主要是依据`config`文件夹下`camra_param1.yaml`文件的参数，注意多相机时必须按照`camra_paramx.yaml`文件名命名`yaml`文件。

`yaml`文件里参数均有注释，请按照自己的需求配置。

`launch`文件里的`camera_num`参数配置为自己所需的相机数量(最多5个，如有更多，请修改`mvsua_camera.h`的`MVSUA_CAMERA_MAX`宏)，请在后面包含各自相机的`yaml`参数配置文件，若未写入，则和最后一个`yaml`文件参数一致。

### (2) 相机录制保存功能
目前仅仅在
| 按键         | 详细描述                           |
| :------------| :----------------------------------|
| q            | 保存图片(单相机下有效)             |
| s            | 开始录像(单相机下有效)             |
| d            | 停止录像(单相机下有效)             |
| t            | ROS图像延迟测试(单相机下有效)      |
| p            | 停止/开始打印FPS                   |
| esc          | 关闭窗口显示                       |

## 3. 运行
单独运行测试：
```bash
roslaunch mvsua_ros_driver camera_driver.launch
```

## 4. 依赖
迈德威视相机驱动

[SDK安装包](http://www.mindvision.com.cn/rjxz/list_12.aspx)

由于安装比较简单，下载后按照readme安装即可，记住最后要重启电脑。