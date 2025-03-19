## 本项目用于betaflight的autopilot

#### 调试
- 如果使用编译好的二进制文件调试，操作与调试正常c++代码一样
- 如果使用launch文件调试，务必需要保证全局状态下（开启一个新的终端后输入`echo $ROS_PACKAGE_PATH`）包含本项目的环境变量，如果在bashrc中无法添加，可以新开终端先source一下然后在这个终端中打开vscode再进行调试
- 如果调试后卡住，可以点击右上角图标调试，选择gdb调试，虽然肯定会报错，但是点击退出后再重新在左边栏点击调试即可正常启动

#### 安装OpenVINS
- 直接clone源码之后可能报错找不到文件opencv2/aruco.hpp，然而系统安装的opencv是二进制格式且不包含这个包，所以只能在用户级别下包含这个包并重新源码编译opencv，该包位于pencv_contrib库中
- 具体操作是参考[这里](https://docs.openvins.com/gs-installing.html#gs-install-opencv)：
    - `git clone https://github.com/opencv/opencv/`和`git clone https://github.com/opencv/opencv_contrib/`，注意二者的版本号必须相同
    - 然后是正常源码编译流程，只是cmake..变成`cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..`
    - 查看路径`/usr/local/include/opencv4/opencv2`下是否有之前报错缺失的hpp文件
    - 在/OpenVINS_ws/src/open_vins/ov_core路径下的CMakeLists文件前面添加`set(Opencv_DIR /user/local/lib/cmake/opencv4)`，以使得构建时不链接系统级别下的opencv，而是链接用户级别下的opencv

#### OpenVINS漂移
- 电机振动或者高速晃动等高频噪声导致openins里程计严重的漂移，使用vins-fusion没有这种问题但性能要求高，换用nx

#### OpenVINS和VINS-fusion都不够稳定
- 经过实机测试，二者都不稳定，起飞瞬间可能导致漂移，有可能是相机外参或者室内环境不够理想，但如果当定位稳定工作时，可以实现定点
