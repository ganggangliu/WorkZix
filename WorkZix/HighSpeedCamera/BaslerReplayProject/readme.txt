Basler高速相机工程
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
src文件夹
所有源文件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ProjectWin32文件夹
Win32，vs2010工程
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ProjectWin32/BaslerReplay.vcxproj
功能：2个Balser和1个PointGrey相机同步播放程序。
运行参数：数据文件夹目录，例如“E:\个人文件\zix\20170829走行-cut\久が原6－1中央線あり\南→西（左折）”
操作：空格键停止/启动自动播放，‘A’‘D’分别为上/下一帧单步播放
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ProjectWin32/BaslerProcess.vcxproj
功能：此工程要求系统装有显卡及其驱动，对数据中Balser图像进行数据处理，输出视觉光流结果保存到数据文件夹目录的result文件夹中
运行参数：
参数1，数据文件夹目录，例如“E:\个人文件\zix\20170829走行-cut\久が原6－1中央線あり\南→西（左折）”
参数2，处理模式：0：预读5000图片到内存，然后以最快速度处理数据 1：逐帧读取图片并处理 2：逐帧读取图片并处理并显示
操作：空格键停止/启动自动播放，‘D’为下一帧单步播放
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ProjectWin32/BaslerImageEval.vcxproj
功能：评价图像质量，获取图片特征点个数及强度值
运行参数：Basler数据目录，例如“E:\个人文件\zix\20170829走行-cut\久が原6－1中央線あり\南→西（左折）\left”
操作：空格键停止/启动自动播放，‘D’为下一帧单步播放
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ProjectWin32/BaslerReplayStartUp/BaslerReplayStartUp.vcxproj
功能：BaslerReplay.vcxproj的界面程序
运行参数：无
操作：无
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ProjectWin32/BaslerEvaluate.vcxproj
功能：Basler高速相机轨迹生成算法测试及评价
运行参数：数据文件夹目录，例如“E:\个人文件\zix\20170829走行-cut\久が原6－1中央線あり\南→西（左折）”
操作：无
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ProjectWin32/KmlTransfer.vcxproj
功能：将ublox数据转换为kml文件
运行参数：数据文件夹目录，例如“E:\个人文件\zix\20170829走行-cut\久が原6－1中央線あり\南→西（左折）”
操作：无
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ProjectWin32/GoogleEarthDisplay/GoogleEarthDisplay.vcxproj
功能：GoogleEarth显示kml轨迹程序
运行参数：无
操作：导入kml文件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ProjectWin32/HD_MapReplay/HD_MapReplay.vcxproj
功能：向导航程序通过LCM发送定位数据
运行参数：ublox文件夹目录，例如：“D:\data\CameraVideo\CameraVideo\CameraVideo\ublox”
操作：无
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ProjectWin32/UbloxReplay.vcxproj
功能：单独播放Ublox数据
运行参数：ublox文件夹目录，例如：“D:\data\CameraVideo\CameraVideo\CameraVideo\ublox”
操作：空格键停止/启动自动播放，‘A’‘D’分别为上/下一帧单步播放


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
环境配置
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
编译环境：win7 32位，vs2010，opencv2.4.11
1：OpenCV官网，进入RELEASES页面，下载2.4.11 Win pack
2：D盘下新建opencv2411目录，将下载的opencv解压到D:\opencv2411
3：用vs2010打开BaslerReplay.sln，debug/release编译，保证无错误
4：将D:\opencv2411\opencv\build\x86\vc10\bin目录下的opencv*2411d.dll拷贝到BaslerReplayProject\BaslerReplay\Debug下
5：将D:\opencv2411\opencv\build\x86\vc10\bin目录下的opencv*2411.dll拷贝到BaslerReplayProject\BaslerReplay\Release下
6：将要运行的工程设置为启动项，设置运行参数后运行