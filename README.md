# EnhancedLiDAR
Enhance LiDAR pointcloud based on stereo image.

## Flow diagram
![flow diagram](https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/flow_diagram.png)

## Result
### Ground remove
The following picture shows the ground remove results based on iterated RANSAC method we designed and normal RANSAC method.
![ground remove](https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/ground_remove.png)
The iterated RANSAC flow diagram is shown below.<br/>
<img src="https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/iterated_RANSAC.png" height = "500px" div align=center/>

### Enhanced LiDAR point cloud
The enhanced LiDAR point cloud with higher density is shown below. 

<div align=center> ![point cloud](https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/enhanced_pointcloud.png)
<div align=left>


## Experiment on 3D object detection
1. Method: AVOD and AVOD-FPN
2. Results:
![3D detection](https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/3D_detection.png)
3. PR curve
<center> &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; Car_3D &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; Car_BEV</center>
<div align="center">

<img src="https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/AP_3D.png" height="300px" ><img src="https://github.com/jerry99s/EnhancedLiDAR/blob/master/pic/AP_BEV.png" height="300px" >

</div>
