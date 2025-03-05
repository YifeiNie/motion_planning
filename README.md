### 调试注意事项

- 使用terminal启动vscode，然后在terminal里source，才可以正常debug
- 使用`sudo gedit /etc/sudoers`修改为这句话`sudo ALL=NOPASSWD:ALL`，使得打开终端时候不需要输入sudo密码，防止多节点调试时需要输入密码导致debug卡死
- 每个package必须都设置为Debug编译模式，并关闭编译优化，即在CMakeList.txt添加：`set(CMAKE_CXX_FLAGS "-g -O0")`和`set(CMAKE_BUILD_TYPE "Debug")`，同时检查是否会被后面的语句覆盖
- 如果以上操作全部执行然后debug还卡的话，这是vscode的问题，打开任意一个c/cpp文件并点击左上角的小三角调试，选择gdb，一定会报错，点击终止即可，然后在重新启动debug即可正常
- 终止debug，需要点击红色方可，然后并逐个关闭右下角栏目的cppdbg终端和gdb-server