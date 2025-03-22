### 调试注意事项

- 使用terminal启动vscode，然后在terminal里source，才可以正常debug
- 使用`sudo gedit /etc/sudoers`修改为这句话`sudo ALL=NOPASSWD:ALL`，使得打开终端时候不需要输入sudo密码，防止多节点调试时需要输入密码导致debug卡死
- 每个package必须都设置为Debug编译模式，并关闭编译优化，即在CMakeList.txt添加：`set(CMAKE_CXX_FLAGS "-g -O0")`和`set(CMAKE_BUILD_TYPE "Debug")`，同时检查是否会被后面的语句覆盖
- 如果以上操作全部执行然后debug还卡的话，这是vscode的问题，打开任意一个c/cpp文件并点击左上角的小三角调试，选择gdb，一定会报错，点击终止即可，然后在重新启动debug即可正常
- 终止debug，需要点击红色方可，然后并逐个关闭右下角栏目的cppdbg终端和gdb-server

### 编译注意事项

- 在使用catkin_make的时候，会按照默认的顺序对功能包进行编译，这导致了有的被依赖的包尚未被编译，未生成xxxConfig.cmake文件，因此也无法被其他包的findpackage()命令找到
- 需要在package.xml文件里声明依赖就可以避免这种问题
- 由于头文件互相引用的问题，更新内容后可能出现找不到头文件的问题，记得不要仅仅修改新增内容的cmakelist，依赖它的package的cmakelist也要修改
- CMakeLists.txt如果使用`file(GLOB SRC_FILES "src/*cpp")`，再添加新文件后并不会自动更新cpp文件列表，需要修改CMakeLists.txt，比如加一个回车，这样完全重新编译，才会重新寻找cpp文件
- 编译时注意要使用release模式，并开启o3编译优化，否则定时器回调的触发频率会远低于设定频率

###
- 一定要使用[GPU版本的Vins-Fusion](https://github.com/pjrambo/VINS-Fusion-gpu)，否则使用CPU版本的漂移非常严重导致经常炸机，下面是配置步骤
  - 配置GPU版本的OpenCV
    - 下载[OpenCV源码](https://github.com/opencv/opencv), 和[OpenCV_contrib]源码(https://github.com/opencv/opencv_contrib)，并放在统一目录下
    - 终端进入OpenCV目录，`mkdir build && cd build`，然后使用cmake进行构建：
    ```cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local/ \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.6.0/modules \
        -D WITH_CUDA=ON \
        -D CUDA_ARCH_BIN=8.7 \
        -D CUDA_ARCH_PTX="" \
        -D ENABLE_FAST_MATH=ON \
        -D CUDA_FAST_MATH=ON \
        -D WITH_CUBLAS=ON \
        -D WITH_LIBV4L=ON \
        -D WITH_GSTREAMER=ON \
        -D WITH_GSTREAMER_0_10=OFF \
        -D WITH_QT=ON \
        -D WITH_OPENGL=ON \
        -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" \
        -D WITH_TBB=ON \
        ..
    ```
    其中CMAKE_INSTALL_PREFIX是OpenCV安装路径，CUDA_ARCH_BIN是GPU算力量，需要根据板子适配，8.7是Orin NX的，**注意版本要选择自己下载的版本**，名字要匹配
    - 使用`sudo make install -j8`编译，这个要等几十分钟甚至更九，非常慢
    - 使用jtop判断GPU版本的OpenCV是否安装，如果成功，在7INFO一栏里可以看到`OpenCV: 4.6.0 with CUDA: YES `

  - 使用CV_bridge构建OpenCV与ROS的连接
    - 下载[CV_bridge源码](https://github.com/ros-perception/vision_opencv/tree/noetic)，**注意选择自己的ROS版本**，并使用catkin构建，需要配置好工作空间
    - 在CMakeList.txt中修改为自己的GPU版本OpenCV路径`include("/usr/local/lib/cmake/opencv4/OpenCVConfig.cmake")`，然后使用`catkin_make`构建并编译
    - 如果可能存在与已有的CV_bridge重名，可以考虑修改在CMakeList.txt和Package.xml中修改名字，这样后面编译Vins的时候findpackage就不是cv_bridge而是自己定义的名字了

  - 编译Vins-Fsuion-GPU
    - 安装依赖`sudo apt-get install liblapack-dev libsuitesparse-dev libgflags-dev libgoogle-glog-dev libgtest-dev libcxsparse3 -y`，
    - 安装ceres: `wget ceres-solver.org/ceres-solver-1.14.0.tar.gz`,`tar -zxvf ceres-solver-1.14.0.tar.gz`，然后使用cmake进行源码编译
    - 下载[GPU版本的Vins-Fusion](https://github.com/pjrambo/VINS-Fusion-gpu)并：
      - 修改vins_estimator/CMakeLists.txt和loop_fusion/CMakeLists.txt 文件，将findpackage里的cv_bridge改为自己的名字如cv_bridge_454（如果没有自定义就不用），然后修改OpenCV和CV-bridge的路径
      ```
        set(cv_bridge_454_DIR /home/nyf/dependencies/cv_bridge_454/devel/share/cv_bridge_454/cmake)
        set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
      ```
    - 终端进入src/VINS-Fusion-gpu文件夹，使用正则表达式修改源码中的一些内容
    ```
      sed -i 's/CV_FONT_HERSHEY_SIMPLEX/cv::FONT_HERSHEY_SIMPLEX/g' `grep CV_FONT_HERSHEY_SIMPLEX -rl ./`
      sed -i 's/CV_LOAD_IMAGE_GRAYSCALE/cv::IMREAD_GRAYSCALE/g' `grep CV_LOAD_IMAGE_GRAYSCALE -rl ./`
      sed -i 's/CV_BGR2GRAY/cv::COLOR_BGR2GRAY/g' `grep CV_BGR2GRAY -rl ./`
      sed -i 's/CV_RGB2GRAY/cv::COLOR_RGB2GRAY/g' `grep CV_RGB2GRAY -rl ./`
      sed -i 's/CV_GRAY2RGB/cv::COLOR_GRAY2RGB/g' `grep CV_GRAY2RGB -rl ./`
      sed -i 's/CV_GRAY2BGR/cv::COLOR_GRAY2BGR/g' `grep CV_GRAY2BGR -rl ./`
      sed -i 's/CV_CALIB_CB_ADAPTIVE_THRESH/cv::CALIB_CB_ADAPTIVE_THRESH/g' `grep CV_CALIB_CB_ADAPTIVE_THRESH -rl ./`
      sed -i 's/CV_CALIB_CB_NORMALIZE_IMAGE/cv::CALIB_CB_NORMALIZE_IMAGE/g' `grep CV_CALIB_CB_NORMALIZE_IMAGE -rl ./`
      sed -i 's/CV_CALIB_CB_FILTER_QUADS/cv::CALIB_CB_FILTER_QUADS/g' `grep CV_CALIB_CB_FILTER_QUADS -rl ./`
      sed -i 's/CV_CALIB_CB_FAST_CHECK/cv::CALIB_CB_FAST_CHECK/g' `grep CV_CALIB_CB_FAST_CHECK -rl ./`
      sed -i 's/CV_ADAPTIVE_THRESH_MEAN_C/cv::ADAPTIVE_THRESH_MEAN_C/g' `grep CV_ADAPTIVE_THRESH_MEAN_C -rl ./`
      sed -i 's/CV_THRESH_BINARY/cv::THRESH_BINARY/g' `grep CV_THRESH_BINARY -rl ./`
      sed -i 's/CV_SHAPE_CROSS/cv::MORPH_CROSS/g' `grep CV_SHAPE_CROSS -rl ./`
      sed -i 's/CV_SHAPE_RECT/cv::MORPH_RECT/g' `grep CV_SHAPE_RECT -rl ./`
      sed -i 's/CV_TERMCRIT_EPS/cv::TermCriteria::EPS/g' `grep CV_TERMCRIT_EPS -rl ./`
      sed -i 's/CV_TERMCRIT_ITER/cv::TermCriteria::MAX_ITER/g' `grep CV_TERMCRIT_ITER -rl ./`
      sed -i 's/CV_RETR_CCOMP/cv::RETR_CCOMP/g' `grep CV_RETR_CCOMP -rl ./`
      sed -i 's/CV_CHAIN_APPROX_SIMPLE/cv::CHAIN_APPROX_SIMPLE/g' `grep CV_CHAIN_APPROX_SIMPLE -rl ./`
      sed -i 's/CV_AA/cv::LINE_AA/g' `grep CV_AA -rl ./`
      sed -i 's/CV_LOAD_IMAGE_UNCHANGED/cv::IMREAD_UNCHANGED/g' `grep CV_LOAD_IMAGE_UNCHANGED -rl ./`
      sed -i 's/CV_MINMAX/cv::NORM_MINMAX/g' `grep CV_MINMAX -rl ./`
    ```
    - 使用catkin_make编译
    - 修改yaml文件，增加以下两句启用GPU加速`use_gpu: 1`和`use_gpu_acc_flow: 1`
    