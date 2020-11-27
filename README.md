# WinTeaser
Teaser++[1] is a well-performed point cloud registration algorithm which can be used in computer vision and robotics. However, the original Teaser++ operates in Linux. This reporsitory configures Teaser++ in Windows with the help of Visual Studio 2015. The main procedures are as follows:

1. Install PCL-all-in-one-installer_1.8.1. 

2. Configure PCLRelease/PCLDebug.props and Teaser.props in VS 2015. 

3. Compile the whole project. The main() lies in Teaser.cpp. 

4. run visResult.m using MATLAB to see the quality of registration. 

Enjoy it! 

[1] Yang, H., Shi, J., & Carlone, L. (2020). TEASER: Fast and Certifiable Point Cloud Registration. IEEE Transactions on Robotics.
