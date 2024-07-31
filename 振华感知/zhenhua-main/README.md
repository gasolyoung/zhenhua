# 振华激光感知部分
本工程基于autoware+openplanner 2.5版本的框架进行更改

环境配置:参考官方autoware环境配置

zh.sh：
+ 更改 bag 的路径
+ rviz选择合适.rviz配置文件
+ 默认点云输入话题为：/points_raw
+ 根据实际需要更改zh_lidar.launch中的base_frame

[lidar_euclidean_cluster_detect更改以显示矩形包围框](https://blog.csdn.net/crazty/article/details/128115139?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522172127058616800184178891%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=172127058616800184178891&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-3-128115139-null-null.142^v100^control&utm_term=autoware_msgs%3A%3ACloudCluster&spm=1018.2226.3001.4187)