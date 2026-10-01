# 第三方来源

Piper 机械臂的原始 URDF、运动学参数和 STL 网格来自 [AgileX 官方 piper_ros](https://github.com/agilexrobotics/piper_ros/tree/humble) 的 `humble` 分支。本项目修改了碰撞、惯性、控制器和仿真相机等内容。上游 MIT 许可证及 `Copyright (c) 2024 RosenYin` 声明保存在根目录 [LICENSE](LICENSE)，与各 ROS 包已有的 MIT 声明一致。

Gemini 335 参数参考 [Orbbec 官方 Gemini 330 系列 USB 数据手册 V1.7](https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini%20330%20series%20Datasheet-V1.7.pdf)。仓库保留参数来源、核对记录和仿真截图，官方 PDF 通过原始链接下载。

ROS 2、Gazebo Classic、MoveIt 2、MoveIt Task Constructor 和 OpenCV 作为外部依赖安装，本仓库不包含它们的完整源码。手眼矩阵来自作者的本机真机项目；配置中的原始文件路径仅记录来源，仿真运行不要求该项目存在。
