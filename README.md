# EnhancedLiDAR
Enhance LiDAR pointcloud based on stereo image.

## Flow diagram
![flow diagram](https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/flow%20diagram.png)

## Result
### original point
It is the pointcloud given by HDL-64E from KITTI database.
![original](https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/original_point.png)

### ground point
It is the ground pointcloud extracted by RANSAC.
![ground](https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/ground_point.png)

### object point
It is the object pointcloud which doesn't contain ground points.
![object](https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/object_point.png)

### pseudo point
It is the pseudo-LiDAR pointcloud based on stereo image depth estimation.
![pseudo](https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/pseudo_point.png)

### denser point
It is the final result pointcloud (only objects) with higher density.
The color points come from original object pointcloud while the white points represent the denser points produced by this algorithm.
![denser1](https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/fusion_point.png)
![denser1](https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/fusion_point2.png)
