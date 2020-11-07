LOAM algorithm notes
===
　　基于在GitHub上的[LOAM源码](https://github.com/laboshinl/loam_velodyne)，在自己的理解上进行了中文注释，同时参考了另外一个项目[cuitaixiang/LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED)。<br/>


关于原项目存在的一个问题
===
　　原项目[laboshinl/loam_velodyne](https://github.com/laboshinl/loam_velodyne)，实现了LOAM算法，clone下来，编译该项目，运行会产生错误"[multiScanRegistration-1] process has died [pid 18786, exit code -11," 该项目下的issue也有提问[issue7](https://github.com/laboshinl/loam_velodyne/issues/7)、[issue71](https://github.com/laboshinl/loam_velodyne/issues/71)、[issue126](https://github.com/laboshinl/loam_velodyne/issues/126)，作者在README中给出的解决方案是重新编译pcl源码，但在issue126中的[comment](https://github.com/laboshinl/loam_velodyne/issues/126#issuecomment-660618745)给出了更好的解决方案，即注释掉"loam_velodyne/CMakeLists.txt"中的"#add_definitions( -march=native )"，可使程序正常运行。<br/>



论文解读
===
- 0 原文[Low-drift and real-time lidar odometry and mapping](https://link.springer.com/article/10.1007/s10514-016-9548-2)<br/>
- 1 约定与解释<br/>
	- 1.1 sweep，所有激光测距单元在一个运动周期内扫描输出的点云<br/>
	- 1.2 scan，单个激光测距单元在一个运动周期内扫描输出的点云<br/>
- 2 坐标系设定：激光雷达坐标系L，位于雷达的几何中心，Z轴朝前，XZ平面计算azimuth angle；世界坐标系W(与初始的雷达坐标系L重合) <br/>
- 3 对每个scan中的点计算曲率并排序，曲率大的视为边缘点edge points，曲率小的视为平面点planar points，计算方法为公式1 <br/>
- 4 将每个scan四等分，每个等分区域提取两个边缘点和4个平面点，特征点需要避免两种特殊情况，一种是点所在平面几乎平行于激光束，另一种是点位于闭塞区域的边缘 <br/>
- 5 在相邻两个swee寻找对应的特征点（边缘点和平面点），Pk-1和Pk，注意，Pk-1需要投影到Pk的初始时刻 <br/>
	- 5.1 寻找边缘点对应点，对应Pk中的边缘点i，寻找Pk-1中与之距离最近的点j，再寻找Pk-1中j所在scan的前后两个scan中与i最近的点l，j和l两点就表示Pk-1中的边缘线，i就是这条边缘线在Pk中对应边缘线上的点<br/>
	- 5.2 寻找平面点对应点，和上面寻找边缘点的方法系统，但是构成平面需要三个点，因此要再找一个和j同一个scan的距离i最近点m，j、l和m三个点构成了Pk-1中的平面，i就是做个平面在Pk中对应平面上的点。 <br/>
	- 5.3 为了确保以上的点都是边缘点或平面点，需要对点的曲率设定阈值，大于或小于该阈值才能视为边缘点或平面点。 <br/>
- 6 计算Pk中边缘点/平面点i到Pk-1中对应边缘线/平面的距离 <br/>
	- 6.1 边缘点到边缘线的距离，利用i到j、l的两个向量构成的平行四边形的面积计算，两个向量叉乘即为平行四边形的面积，同时面积也可表示出底乘以高，高即是要求的距离；公式2 <br/>
	- 6.2 平面点到平面的距离，与计算点到线的距离类似，利用了四面体的计算体积的不同方式求得；公式3 <br/>
	- 6.3 参考链接：[向量积_百度百科](https://baike.baidu.com/item/向量积/4601007)、[四面体 - 维基百科](https://zh.wikipedia.org/wiki/四面体) <br/>
- 7 运动估计，假设在一个sweep内lidar是匀速运动的，初始时刻tk，某时刻t相对于tk的位姿变换为T，因此可对在[tk,t]之间的点进行线性插值而获得其相对于该sweep初始时刻的位姿变换，见公式5；因此可以将sweep内具有不同时间戳的点统一投影到该sweep的初始时刻，再将前一个sweep的也投影到该sweep的初始时刻，在这两个投影到同一个时刻的点云中根据第4步的做法求取距离，理论上如果位姿变换T是正确的，该距离应该为0，因此可联合距离求取和位姿变换建立非线性方程组，通过数值迭代的方法最小化距离至0而得到一个位姿变换T，并将该sweep内的点都统一投影至初始时刻，即去除了点云的畸变。详细的算法描述在Algorithm 1: Lidar Odometry <br/>
- 8 ……



算法源码解析
=======
- 0 参考：
	- 0.1 [LOAM笔记及A-LOAM源码阅读 - WellP.C - 博客园](https://www.cnblogs.com/wellp/p/8877990.html)
	- 0.2 [3D 激光SLAM ->loam_velodyne论文与代码解析Lidar Odometry and Mapping_Nksjc的博客-CSDN博客](https://blog.csdn.net/nksjc/article/details/76401092)
	- 0.3 [LOAM细节分析 - 知乎](https://zhuanlan.zhihu.com/p/57351961)  
- 1 MultiScanRegistration(multi_scan_registration_node.cpp , MultiScanRegistration.cpp , ScanRegistration.cpp , BasicScanRegistration.cpp)，初始化，设定激光雷达的线数，垂直角度，及各种参数；订阅输入点云数据Topic(/velodyne_points，数据类型为PointCloud2)<br/>
	- 1.1 为什么不直接解析原始的PointCloud2数据，该数据里面包含有x、y、z、intensity和ring字段，ring字段就是该点所属的线束ID?<br/>
	- 1.2 先不考虑IMU，将该部分代码中关于IMU的都注释掉，基本流程是: <br/>
		- 1.2.1 MultiScanRegistration::setup()<br/>
		- 1.2.2 MultiScanRegistration::setupROS() <br/>
		- 1.2.3 MultiScanRegistration::handleCloudMessage()/process()，划分线束，将一个sweep按照线束编号分组_laserCloudScans[scanID]，同时根据同一线束下的点的水平方位角以及前后顺序确定该点的时间<br/>
		- 1.2.4 BasicScanRegistration::processScanlines()，将前面分好组的点云集合到一个点云pcl::PointCloud<pcl::PointXYZI> _laserCloud之中，并记录下每组的起始与结束编号extractFeatures()，提取特征，通过setScanBuffersFor()剔除不可靠的两类点，通过setRegionBuffersFor()函数计算曲率，选择edge point和planar point，分别放入_cornerPointsSharp、_cornerPointsLessSharp、_surfacePointsFlat、surfPointsLessFlatScan四个点云中，带less相比较而言选取的点的数量更多<br/>
		- 1.2.5 ScanRegistration::publishResult()()，发布上述四种点云的话题，以及分好组后集合到一起的点云的话题<br/>
- 2 LaserOdometry(laser_odometry_node.cpp , LaserOdometry.cpp , BasicLaserOdometry.cpp)，仅凭点云进行运动估计<br/> 
	- 2.1 …… <br/>
	- 2.2 代码基本流程： <br/>
		- 2.2.1 LaserOdometry::setup()，从参数服务器获取参数，订阅话题MultiScanRegistration发布的点云话题 <br/>
		- 2.2.2 LaserOdometry::spin()，设定执行LaserOdometry::process()函数100Hz <br/>
		- 2.2.3LaserOdometry::process()，使用函数LaserOdometry::hasNewData()判断是否收到了Topic的数据，收到了就执行BasicLaserOdometry::process()，否则返回 <br/>
		- 2.2.4 BasicLaserOdometry::process()， 实现过程参考论文"ow-drift and real-time lidar odometry and mapping"的5.3节——Motion estimation、Algorithm 1：Lidar Odometry以及代码注释，这部分的参考资料如下：<br/>
			- 2.2.4.1 [如何理解LOAM中的雅各比矩阵推导_Yukina的博客-CSDN博客](https://blog.csdn.net/weixin_39781401/article/details/105275758) <br/>
			- 2.2.4.2 [LOAM论文和程序代码的解读_robinvista的专栏-CSDN博客](https://blog.csdn.net/robinvista/article/details/104379087) —— 这里最后对欧拉角对应的变换矩阵求导过程不太能理解，我自己推导的雅克比矩阵和此文中的以及源码中的，元素形式是一致的，只是有几个的正负号不一样，不知道是不是我求导的过程存在错误？ <br/>
			- 2.2.4.3 [Euler angles - Wikipedia](https://en.wikipedia.org/wiki/Euler_angles)，在5.1节的Rotation Matrix中有关于欧拉角转换为旋转矩阵的描述，同时可参考[欧拉角 图解释_linuxheik的专栏-CSDN博客](https://blog.csdn.net/linuxheik/article/details/78842428)<br/>
			- 2.2.4.4 [高斯-牛顿法(Guass-Newton Algorithm)与莱文贝格-马夸特方法(Levenberg–Marquardt algorithm)求解非线性最小二乘问题 - Fun With GeometryFun With Geometry](http://www.whudj.cn/?p=1122)<br/>
			- 2.2.4.5 [loam中激光里程计部分accumulateRotation()函数，PluginIMURotation函数以及误差函数对欧拉角偏导的数学推导-热备资讯](https://www.hotbak.net/key/PluginIMURotation%E5%87%BD%E6%95%B0%E7%9A%84%E6%95%B0%E5%AD%A6%E6%8E%A8%E5%AF%BCslaml1323%E7%9A%84%E5%8D%9A%E5%AE%A2CSDN%E5%8D%9A%E5%AE%A2.html)
		- 2.2.5 void LaserOdometry::publishResult()，发布当前帧点云的特征点话题<br/>
- 3 ……
