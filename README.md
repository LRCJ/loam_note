LOAM algorithm notes
===
　　基于在GitHub上的[LOAM源码](https://github.com/laboshinl/loam_velodyne)，在自己的理解上进行了中文注释，同时参考了另外一个项目[cuitaixiang/LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED)。<br/>


关于原项目存在的一个问题
===
　　原项目[laboshinl/loam_velodyne](https://github.com/laboshinl/loam_velodyne)，实现了LOAM算法，clone下来，编译该项目，运行会产生错误"[multiScanRegistration-1] process has died [pid 18786, exit code -11," 该项目下的issue也有提问[issue7](https://github.com/laboshinl/loam_velodyne/issues/7)、[issue71](https://github.com/laboshinl/loam_velodyne/issues/71)、[issue126](https://github.com/laboshinl/loam_velodyne/issues/126)，作者在README中给出的解决方案是重新编译pcl源码，但在issue126中的[comment](https://github.com/laboshinl/loam_velodyne/issues/126#issuecomment-660618745)给出了更好的解决方案，即注释掉"loam_velodyne/CMakeLists.txt"中的"#add_definitions( -march=native )"，可使程序正常运行。<br/>



论文解读
===
- **0** **原文**[Low-drift and real-time lidar odometry and mapping](https://link.springer.com/article/10.1007/s10514-016-9548-2)<br/>
- **1** **约定与解释**<br/>
	- **1.1** sweep，所有激光测距单元在一个运动周期内扫描输出的点云<br/>
	- **1.2** scan，单个激光测距单元在一个运动周期内扫描输出的点云<br/>
	- **1.3** 坐标系设定：激光雷达坐标系L，位于雷达的几何中心，Z轴朝前，XZ平面计算azimuth angle；世界坐标系W(与初始的雷达坐标系L重合) <br/>
- **2** **点云预处理(点云注册/Registration)** <br/>
	- **2.1** 对每个scan中的点计算曲率并排序，曲率大的视为边缘点edge points，曲率小的视为平面点planar points，计算方法为公式1 <br/>
	- **2.2** 将每个scan四等分，每个等分区域提取两个边缘点和4个平面点，特征点需要避免两种特殊情况，一种是点所在平面几乎平行于激光束，另一种是点位于闭塞区域的边缘 <br/>
- **3** **运动估计(odometry)** <br/>
	- **3.1** 通过点云预处理，可以获得特征点云。这部分有个重要的假设——匀速假设，在一个sweep内，Lidar的角速度和线速度恒定(加速度为0)。另外需要注意，根据Lidar的原理，其所有的点并非是同一时刻同时测量获得的，而是在其一个周期内依次测量获得的，所以如果Lidar在测量的过程中运动了，那这个sweep内的点是在不同的基准坐标系中测得的，即点云畸变。假设现在有第k帧点云Pk，点云初始时刻tk，结束时刻tk+1，以及前一帧点云P~k-1(前一帧点云Pk-1被投影到其结束时刻tk)。根据前述假设，在Pk所经历的时间[tk,tk+1]内Lidar的位置和角度是线性变化的，直接沿用上一帧的位置和角度变化(由上一帧点云进行运动估计得到)并进行插值，见公式5，就可以得到每一个点进行测量时的基准坐标相对于第一个点(也就是该sweep的初始时刻)的位姿变换，然后将所有点统一变换到初始时刻，得到P~k，其与被投影到其结束时刻的前一帧点云P~k-1是处于同一个坐标系下的(前一帧点云的结束时刻即是当前帧的初始时刻) <br/>
	- **3.2** 然后在这相邻两个sweep P~k和P~k-1寻找对应的特征点（边缘点和平面点） <br/>
		- **3.2.1** 寻找边缘点对应点，对P~k中的边缘点i，寻找P~k-1中与之距离最近的点j，再寻找P~k-1中j所在scan的前后两个scan中与i最近的点l，j和l两点就表示P~k-1中的边缘线，i就是这条边缘线在P~k中对应边缘线上的点 <br/>
		- **3.2.2** 寻找平面点对应点，和上面寻找边缘点的方法系统，但是构成平面需要三个点，因此要再找一个和j同一个scan的距离i最近点m，j、l和m三个点构成了Pk-1中的平面，i就是做个平面在P~k中对应平面上的点 <br/>
		- **3.2.3** 为了确保以上的点都是边缘点或平面点，需要对点的曲率设定阈值，大于或小于该阈值才能视为边缘点或平面点 <br/>
	- **3.3** 计算P~k中边缘点/平面点i到P~k-1中对应边缘线/平面的距离 <br/>
		- **3.3.1** 边缘点到边缘线的距离，利用i到j、l的两个向量构成的平行四边形的面积计算，两个向量叉乘即为平行四边形的面积，同时面积也可表示出底乘以高，高即是要求的距离；公式2 <br/>
		- **3.3.2** 平面点到平面的距离，与计算点到线的距离类似，利用了点与面构成的四面体的计算体积的不同方式求得；公式3  <br/>
		- **3.3.3** 参考链接：[向量积_百度百科](https://baike.baidu.com/item/%E5%90%91%E9%87%8F%E7%A7%AF/4601007?fromtitle=%E5%8F%89%E4%B9%98&fromid=8251892&fr=aladdin)、[四面体 - 维基百科，自由的百科全书](https://zh.wikipedia.org/wiki/%E5%9B%9B%E9%9D%A2%E9%AB%94%20%C2%A0%C2%A0) <br/>
	- **3.4** 通过上述公式计算的距离理论上应该为0，因为P~k和P~k-1处于同一坐标系下，但实际不为0，因为匀速假设是不准确的。然后需要通过数值迭代的方式去优化Pk所经历的时间[tk,tk+1]内Lidar的位置和位姿变化使距离逼近0。也就是说，匀速假设相当于给定了数值迭代的初始值。数值迭代使用的目标函数可以表达为f(X,T)=d，X为当前帧P~k的特征点，T为[tk,tk+1]内Lidar的位姿变化(三个位置参数和三个角度参数)，f为上述求解距离的公式；使用的迭代方法是LM法，需要求解雅克比矩阵来，代码中用的是高斯牛顿法。通过迭代优化可以得到较好的[tk,tk+1]内Lidar的位姿变化Tk，在知道Tk后即可将Pk直接变换到本帧的初始时刻或者结束时刻(根据匀速假设插值)，而Tk以及被变换到结束时刻的点云又作为下一帧点云的初始条件 <br/>
	- **3.5** 该部分实际上将去除点云畸变和估计一帧点云经历内Lidar的运动(位姿变化)耦合了起来，估计好了运动，就能去除点云畸变，或者想要去除点云畸变，就必须要知道一帧之内的运动。<br/>
- **4** **建图(mapping)** <br/>
	- **4.1** 该部分将运动估计部分输出的无畸变的点云统一变换到世界坐标系下，然后再将这部分的点云和世界坐标系下已经存在的点云按照上述的计算点到线/面的距离、运动估计的过程再实施一次，但其所需要的特征点的数量是前面单帧点云运动估计的10倍，通过这种做法，可以获得精度更高的位姿信息，从而获得更高精度的地图。此外，与上述寻找特征点对应的线/面的方法不同，这里首先对当前点云帧中的特征点，在世界坐标系中寻找临近点(特征点附近10cm×10cm×10cm的方块内)，并只选择edge point或者planar point(通过曲率确定)，然后被选中的这些点组成一个集合S，对该集合求协方差矩阵M，并求该矩阵的特征向量E和特征值V，对于由edge point构成的M而言，如果特征向量E中有1个特征值远大于另外两个，则表示最大特征值对应的特征向量E代表了该特征点所对应的edge line的方向；对于planar point，其对应的协方差矩阵M和特征向量E、特征值V，如果特征值V包含了两个大的特征值和一个相对小的特征值，则与最小的特征值对应的特征向量表示该特征点对应的planar patch的方向。而edge line或planar patch的位置则通过计算集合S的质心来获得 <br/>
	- **4.2** 这里说明一下增加特征点数量的实现方式，代码中的实现方式是在提取特征点时，根据曲率大小排序后，曲率最大的2个作为edge points，而曲率最大的10个作为less edge points，planar points同理；在odometry中，寻找当前帧特征点在前一帧的对应的线/面不是搜索前一帧所有的点，而是只搜索前一帧的less edge/planar points；在mapping中，也是利用less edge/planar points去和世界坐标系下已经存在的点云进行匹配 <br/>
- **5** **里程计融合(transformMaintenance)**
	- **5.1** odometry过程计算量小，可以跟上激光雷达10Hz的输出频率；而mapping过程计算量大，一般无法跟上激光雷达10Hz的输出频率，导致处理完一帧点云数据后，下一帧或下几帧点云数据都被跳过，但依然可以使用处理完的这一帧输出的修正量去修正odometry节点输出的10Hz位姿信息，最终得到一个较精准的位姿。而这个修正量并不是每帧点云都有的，受限于mapping的执行速度，一般需要间隔2~3帧才能有一次修正。当然，论文中mapping假定只有1Hz，而实际可以达到5Hz。[LOAM_velodyne学习（四） - 归向 - 博客园](https://www.cnblogs.com/luojs/p/10817091.html) <br/>

算法源码解析
=======
- **0** **参考**：
	- **0.1** [LOAM笔记及A-LOAM源码阅读 - WellP.C - 博客园](https://www.cnblogs.com/wellp/p/8877990.html)
	- **0.2** [3D 激光SLAM ->loam_velodyne论文与代码解析Lidar Odometry and Mapping_Nksjc的博客-CSDN博客](https://blog.csdn.net/nksjc/article/details/76401092)
	- **0.3** [LOAM细节分析 - 知乎](https://zhuanlan.zhihu.com/p/57351961)  
- **1 MultiScanRegistration(multi_scan_registration_node.cpp , MultiScanRegistration.cpp , ScanRegistration.cpp , BasicScanRegistration.cpp)**<br/>
	- **1.1** 初始化，设定激光雷达的线数，垂直角度，及各种参数；订阅输入点云数据Topic(/velodyne_points，数据类型为PointCloud2)。还有为什么不直接解析原始的PointCloud2数据，该数据里面包含有x、y、z、intensity和ring字段，ring字段就是该点所属的线束ID?<br/>
	- **1.2** 先不考虑IMU，将该部分代码中关于IMU的都注释掉，基本流程是: <br/>
		- **1.2.1** MultiScanRegistration::setup()<br/>
		- **1.2.2** MultiScanRegistration::setupROS() <br/>
		- **1.2.3** MultiScanRegistration::handleCloudMessage()/process()，划分线束，将一个sweep按照线束编号分组_laserCloudScans[scanID]，同时根据同一线束下的点的水平方位角以及前后顺序确定该点的时间<br/>
		- **1.2.4** BasicScanRegistration::processScanlines()，将前面分好组的点云集合到一个点云pcl::PointCloud<pcl::PointXYZI> _laserCloud之中，并记录下每组的起始与结束编号extractFeatures()，提取特征，通过setScanBuffersFor()剔除不可靠的两类点，通过setRegionBuffersFor()函数计算曲率，选择edge point和planar point，分别放入_cornerPointsSharp、_cornerPointsLessSharp、_surfacePointsFlat、surfPointsLessFlatScan四个点云中，带less相比较而言选取的点的数量更多<br/>
		- **1.2.5** ScanRegistration::publishResult()()，发布上述四种点云的话题，以及分好组后集合到一起的点云的话题<br/>
- **2 LaserOdometry(laser_odometry_node.cpp , LaserOdometry.cpp , BasicLaserOdometry.cpp)，仅凭点云进行运动估计**<br/> 
	- **2.1** …… <br/>
	- **2.2** **代码基本流程：** <br/>
		- **2.2.1** LaserOdometry::setup()，从参数服务器获取参数，订阅话题MultiScanRegistration发布的点云话题 <br/>
		- **2.2.2** LaserOdometry::spin()，设定执行LaserOdometry::process()函数100Hz <br/>
		- **2.2.3** LaserOdometry::process()，使用函数LaserOdometry::hasNewData()判断是否收到了Topic的数据，收到了就执行BasicLaserOdometry::process()，否则返回 <br/>
		- **2.2.4** BasicLaserOdometry::process()， 实现过程参考论文"[low-drift and real-time lidar odometry and mapping](https://link.springer.com/article/10.1007%2Fs10514-016-9548-2)"的5.3节——Motion estimation、Algorithm 1：Lidar Odometry以及代码注释，这部分的参考资料如下：<br/>
			- **2.2.4.1** [如何理解LOAM中的雅各比矩阵推导_Yukina的博客-CSDN博客](https://blog.csdn.net/weixin_39781401/article/details/105275758) <br/>
			- **2.2.4.2** [LOAM论文和程序代码的解读_robinvista的专栏-CSDN博客](https://blog.csdn.net/robinvista/article/details/104379087) —— 这里最后对欧拉角对应的变换矩阵求导过程不太能理解，我自己推导的雅克比矩阵和此文中的以及源码中的，元素形式是一致的，只是有几个的正负号不一样，不知道是不是我求导的过程存在错误？ <br/>
			- **2.2.4.3** [Euler angles - Wikipedia](https://en.wikipedia.org/wiki/Euler_angles)，在5.1节的Rotation Matrix中有关于欧拉角转换为旋转矩阵的描述，同时可参考[欧拉角 图解释_linuxheik的专栏-CSDN博客](https://blog.csdn.net/linuxheik/article/details/78842428)<br/>
			- **2.2.4.4** [高斯-牛顿法(Guass-Newton Algorithm)与莱文贝格-马夸特方法(Levenberg–Marquardt algorithm)求解非线性最小二乘问题 - Fun With GeometryFun With Geometry](http://www.whudj.cn/?p=1122)<br/>
			- **2.2.4.5** [loam中激光里程计部分accumulateRotation()函数，PluginIMURotation函数以及误差函数对欧拉角偏导的数学推导-热备资讯](https://www.hotbak.net/key/PluginIMURotation%E5%87%BD%E6%95%B0%E7%9A%84%E6%95%B0%E5%AD%A6%E6%8E%A8%E5%AF%BCslaml1323%E7%9A%84%E5%8D%9A%E5%AE%A2CSDN%E5%8D%9A%E5%AE%A2.html)
		- **2.2.5** void LaserOdometry::publishResult()，发布当包含前点云帧中特征点以及所有点云的话题("/laser_cloud_corner_last","/laser_cloud_surf_last","/velodyne_cloud_3")，以及位姿变换话题("/laser_odom_to_init") <br/>
- **3 laserMapping(laser_mapping_node.cpp , LaserMapping.cpp , BasicLaserMapping.cpp)**<br/>
	- **3.1 代码基本流程：**<br/>
		- **3.1.1** LaserMapping::setup()，获取参数服务器scanPeriod、maxIterations、deltaTAbort、deltaRAbort、cornerFilterSize、surfaceFilterSize、mapFilterSize参数(实际上由于launch文件中根本没有定义这些参数，所以是获取不到的，这些参数在实例化BasicLaserMapping类时已经赋值好了)，订阅LaserOdometry节点发布的四个话题。 <br/>
		- **3.1.2** LaserMapping::spin()，设定执行LaserMapping::process()函数100Hz <br/>
		- **3.1.3** LaserMapping::process()，判断是否同时接收到上一个节点LaserOdometry发布的四个话题，是则执行BasicLaserMapping::process()函数，否则退出。<br/>
		- **3.1.4** BasicLaserMapping::process()，理论参考论文"[low-drift and real-time lidar odometry and mapping](https://link.springer.com/article/10.1007%2Fs10514-016-9548-2)"的第6节——Lidar mapping，更详细的说明见代码注解<br/>
		- **3.1.5** LaserMapping::publishResult()，publish经过优化的里程计<br/>
- **4 transformMaintenance(transform_maintenance_node.cpp , TransformMaintenance.cpp , BasicTransformMaintenance.cpp)** <br/>
	- **4.1 参考：**<br/>
		- 4.1.1 [LOAM_velodyne学习（四） - 归向 - 博客园](https://www.cnblogs.com/luojs/p/10817091.html)<br/>
	- **4.2 代码基本流程：**<br/>
		- **4.2.1** TransformMaintenance::setup()，订阅数据来自odometry节点和mapping节点的位姿数据topic ，"/aft_mapped_to_init"(来自laserMapping节点)，"/laser_odom_to_init"(来自laserOdometry节点)，以及新建一个publisher——_pubLaserOdometry2，发布topic"/integrated_to_init"，该topic包含odometry节点和mapping节点合成的位姿信息 <br/>
		- **4.2.2** TransformMaintenance::laserOdometryHandler()，处理来自odometry节点的位姿数据 <br/>
			- **4.2.2.1** BasicTransformMaintenance::updateOdometry()，使用odometry节点发布的位姿信息更新_transformSum变量中的位姿信息 <br/>
			- **4.2.2.2** BasicTransformMaintenance::transformAssociateToMap()，将mapping节点输出的优化叠加上odometry输出的里程计，如果没有则直接输出odometry的里程计<br/>
			- **4.2.2.3** _pubLaserOdometry2.publish(_laserOdometry2)，发布前面两节点输出位姿信息的融合位姿信息 <br/>
		- **4.2.3** TransformMaintenance::odomAftMappedHandler()<br/>
			- **4.2.3.1** BasicTransformMaintenance::updateMappingTransform()，使用mapping节点发布的位姿信息更新__transformBefMapped和_transformAftMapped变量中的位姿信息，由BasicTransformMaintenance::transformAssociateToMap()使用<br/>