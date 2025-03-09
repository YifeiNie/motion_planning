### 调试注意事项

- 使用terminal启动vscode，然后在terminal里source，才可以正常debug
- 使用`sudo gedit /etc/sudoers`修改为这句话`sudo ALL=NOPASSWD:ALL`，使得打开终端时候不需要输入sudo密码，防止多节点调试时需要输入密码导致debug卡死
- 每个package必须都设置为Debug编译模式，并关闭编译优化，即在CMakeList.txt添加：`set(CMAKE_CXX_FLAGS "-g -O0")`和`set(CMAKE_BUILD_TYPE "Debug")`，同时检查是否会被后面的语句覆盖
- 如果以上操作全部执行然后debug还卡的话，这是vscode的问题，打开任意一个c/cpp文件并点击左上角的小三角调试，选择gdb，一定会报错，点击终止即可，然后在重新启动debug即可正常
- 终止debug，需要点击红色方可，然后并逐个关闭右下角栏目的cppdbg终端和gdb-server

### 编译注意事项

- 在使用catkin_make的时候，会按照默认的顺序对功能包进行编译，这导致了有的被依赖的包尚未被编译，未生成xxxConfig.cmake文件，因此也无法被其他包的findpackage()命令找到
- 需要在package.xml文件里声明依赖就可以避免这种问题
- 由于头文件互相引用的问题，工程新内容后可能出现找不到头文件的问题，记得不要仅仅修改新增内容的cmakelist，依赖它的package的cmakelist也要修改