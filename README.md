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

## Experiment
### Aggregate View Object Detection
1. link:https://github.com/KID-22/avod
2. results:

|              |   |           |        AP-3D |           |   |           |       AP-BEV |           |
|:------------:|---|:---------:|:------------:|:---------:|---|:---------:|:------------:|:---------:|
|   **Method** |   |  **Easy** | **Moderate** |  **Hard** |   |  **Easy** | **Moderate** |  **Hard** |
|         AVOD |   |   73.59   |      65.78   |   58.38   |   |   86.80   |      85.44   |   77.73   |
|     AVOD-FPN |   |   81.94   |    71.88     |   66.38   |   |   88.53   |      83.79   |   77.90   |
| Ours <br> (EnhancedLiDAR + AVOD-FPN)|   |     81.19 |    73.21    |  67.62     |   |   89.18   |      86.92   | 86.53     |

3. PR curve

<center> &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; Car_3D &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; Car_BEV</center>
<div align="center">

<img src="https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/car_3D_AVOD-FPN.png" height="300px" ><img src="https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/car_BEV%20_AVOD-FPN.png" height="300px" >

</div>
