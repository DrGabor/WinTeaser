# WinTeaser
Teaser++[1] is a well-performed point cloud registration algorithm which can be used in computer vision and robotics. However, the original Teaser++ operates in Linux (https://github.com/MIT-SPARK/TEASER-plusplus). This reporsitory configures Teaser++ in Windows with the help of Visual Studio 2015. The main procedures are as follows:

1. Install PCL-all-in-one-installer_1.8.1. 

2. Configure PCLRelease/PCLDebug.props and Teaser.props in VS 2015. 

3. Compile the whole project. The main() lies in Teaser.cpp. 

4. run visResult.m using MATLAB to see the quality of registration. 

Enjoy it! 

PS: One of the major difference between WinTeaser and Teaser++ lies in the way of computing the FPFH descriptors. Teaser++ computes one FPFH descriptor for each point in point cloud. which is quite computationally inefficiency. Therefore in WinTeaser, the raw cloud is firstly subsampled into cloud_key_points, which is sparse than original cloud. Subsequently, FPFH descriptor is computed for each point in cloud_key_point. Notice that when computing FPFH, the input surface of FPFH_Estimator is still cloud instead of sub-sampled cloud_key_point, which ensures the accuracy of FPFH descriptor has no difference with Teaser++. The modified FPFH computation is in teaser/include/teaser_fpfh.h. 

Disclaimer: I strongly recommend original TEASER++ in Linux since it is well maintained and has rich document. WinTeaser is mainly used for users which are unfamilar with Linux OS and want to quickly test TEASER++ on their own dataset. 

[1] Yang, H., Shi, J., & Carlone, L. (2020). TEASER: Fast and Certifiable Point Cloud Registration. IEEE Transactions on Robotics.
