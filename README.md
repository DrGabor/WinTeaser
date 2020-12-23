# WinTeaser
Teaser++[1] is a well-performed point cloud registration algorithm which can be used in computer vision and robotics. However, the original Teaser++ operates in Linux (https://github.com/MIT-SPARK/TEASER-plusplus). This reporsitory configures Teaser++ in Windows with the help of Visual Studio 2015. The linux_lib contains <sys/time.h>, <unistd.h>, <getopt.h> and <getopt.c> which not exist in Windows OS. If you want to explore more linux libaries, please check https://github.com/robinrowe/libunistd. 

The main procedures are as follows:

1. Install PCL-all-in-one-installer_1.8.1. 

2. Configure PCLRelease/PCLDebug.props and Teaser.props in VS 2015. 

3. Compile the whole project. The main() lies in Teaser.cpp. 

4. run visResult.m using MATLAB to see the quality of registration. 

Enjoy it! 

PS: One of the major difference between WinTeaser and Teaser++ lies in the way of computing the FPFH descriptors. Teaser++ computes one FPFH descriptor for each point in point cloud. which is quite computationally inefficiency. Therefore in WinTeaser, the raw cloud is firstly subsampled into cloud_key_points, which is sparse than original cloud. Subsequently, FPFH descriptor is computed for each point in cloud_key_point. Notice that when computing FPFH, the input surface of FPFH_Estimator is still cloud instead of sub-sampled cloud_key_point, which ensures the accuracy of FPFH descriptor has no difference with Teaser++. The modified FPFH computation is in teaser/include/teaser_fpfh.h. 

PPS: There are some other issues worthy to be mentioned. Firstly, the pmc library seems to be wrong when the number of point pairs is too large. This is because pmc uses omp library. If you meet this issue, just use ordinary for loop instead of omp for loop. Secondly, the running time of teaser can be hours since the maximum clique searching in pmc library. To save the time, you can change the configuration of Teaser. There is a maximum pmc search time
in the constructor of Teaser. Both of the above issues may be occurred when the number of input point pairs is large (I think it is around 10000). Therefore, you can enlarge the feature_radius to reduce the point pair number.

Disclaimer: I strongly recommend original TEASER++ in Linux since it is well maintained and has rich document. WinTeaser is mainly used for users which are unfamilar with Linux OS and want to quickly test TEASER++ on their own dataset. 

[1] Yang, H., Shi, J., & Carlone, L. (2020). TEASER: Fast and Certifiable Point Cloud Registration. IEEE Transactions on Robotics.
