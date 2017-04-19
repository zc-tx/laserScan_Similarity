## A simple and stupid method for loop closure detection in 3D LIDAR data:

Rohling T, Mack J, Schulz D. A fast histogram-based similarity measure for detecting loop closures in 3-D LIDAR data[C]// Ieee/rsj International Conference on Intelligent Robots and Systems. IEEE, 2015:736-741.

## Images

Our own data in Yuquan Campus, from Velodyen VLP16.

![](https://github.com/ZJUYH/laserScan_Similarity/raw/master/image/Campus.png)

two loop closures on 2017.03.07 in Zhejiang University:

![](https://github.com/ZJUYH/laserScan_Similarity/raw/master/image/round0&2.jpg)

Bins of single scan:

![](https://github.com/ZJUYH/laserScan_Similarity/raw/master/image/0.png)

Similarity Matrix:

![](https://github.com/ZJUYH/laserScan_Similarity/raw/master/image/1.png)

Test on kitti_odom_benchmark 05:

![](https://github.com/ZJUYH/laserScan_Similarity/raw/master/image/kitti_05.png)

The ground truth poses in SM-Matrix:

![](https://github.com/ZJUYH/laserScan_Similarity/raw/master/image/smtrue.jpg)

PR-Curve of EMD & Cos:

![](https://github.com/ZJUYH/laserScan_Similarity/raw/master/image/PRCurve05.jpg)

ROC-Curve of EMD & Cos:

![](https://github.com/ZJUYH/laserScan_Similarity/raw/master/image/ROCCurve05.jpg)

## Thanks:

Ethz_asl for libpointmatcher

## IS GOING ON

start to use deep learning on Place Recognition
