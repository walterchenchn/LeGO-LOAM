# 对于LEGO-LOAM的一些理解
## 1. imageProjection图像投影
#### 1.1 首先检测地面,通过前后两个环上同角度的点,计算z方向的夹角,小于10度就是地面点
#### 1.2 点云分割的时候,计算每个点周围四个点,如果夹角小于30度,且具有一定数目,则可以认为这个点可以认为是特征点(主要是面和线)
## 2. featureAssociation特征提取
#### 2.1 计算每隔五个点的曲率,然后按照曲率大小来区别特征点是面还是线
#### 2.2 然后寻找点云中的面特征和线特征,然后使用点对面特征和线特征做位置变换计算,并融入IMU信息
## 3. mapOptmization图优化
#### 3.1 提取关键帧(机器人的位姿点),并保存关键帧周围的点云
#### 3.2 使用现有地图与最新扫描点云进行配准,精确计算机器人位姿(过程与里程计类似,计算点线,点面距离)
#### 3.3 saveKeyFramesAndFactor()根据更新后的机器人位姿更新位姿图
#### 3.4 loopClosureThread()是以1Hz在后台进行回环检测,如果检测成功就对当前点云和关键帧点云进行ICP,更新回环检测后的位姿图
## 4. transformFusion位姿融合
#### 4.1 使用/laser_odom_to_init卡尔曼粗定位
#### 4.2 使用/aft_mapped_to_init卡尔曼精定位