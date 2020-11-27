// Teaser.cpp : 定义控制台应用程序的入口点。
// An example showing TEASER++ registration with FPFH features with the Stanford bunny model

#include "stdafx.h"
#include <TRO_Utilities.h>
#include <teaser/teaser_utility.h>
#include <teaser/CTeaser.h>

void load_kitti(string& sFile_src, string& sFile_tgt) {
	string sDataRoot = "example_data\\kitti\\";
	sFile_src = sDataRoot + "raw_000000.pcd";
	sFile_tgt = sDataRoot + "raw_001100.pcd";
}

void load_bunny(string& sFile_src, string& sFile_tgt) {
	string sDataRoot = "example_data\\bunny\\";
	sFile_src = sDataRoot + "bun000.pcd";
	sFile_tgt = sDataRoot + "bun090.pcd";
}

void load_smoothNet(string& sFile_src, string& sFile_tgt) {
	string sDataRoot = "example_data\\3dmatch_sample\\";
	sFile_src = sDataRoot + "cloud_bin_2.ply";
	sFile_tgt = sDataRoot + "cloud_bin_36.ply";
}
//// compute the spatial resolution of point cloud. 
double computeModelResFun(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud) {
	pcl::KdTreeFLANN<pcl::PointNormal>::Ptr kd_tree(new pcl::KdTreeFLANN<pcl::PointNormal>()); 
	kd_tree->setInputCloud(cloud); 
	double dDist = 0.0; 
	for (int i = 0; i < cloud->points.size(); i++) {
		pcl::PointNormal pt = cloud->points[i]; 
		vector<int> vIdx; 
		vector<float> vDist_sq;
		kd_tree->nearestKSearch(pt, 2, vIdx, vDist_sq);
		dDist += sqrt(vDist_sq[1]); 
	}
	double dRes = dDist / cloud->points.size(); 
	return dRes; 
}

int main() {
	// Load the .ply file. Both load_bunny() and load_kitti() are OK. 
	string sFile_src, sFile_tgt; 
	load_bunny(sFile_src, sFile_tgt);
	pcl::PointCloud<pcl::PointNormal>::Ptr pcl_cloud_src(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::PointNormal>::Ptr pcl_cloud_tgt(new pcl::PointCloud<pcl::PointNormal>());
	read_cloud(sFile_src, pcl_cloud_src);
	read_cloud(sFile_tgt, pcl_cloud_tgt);
	//// compute normals if necessary. the default method is KNN-based normal computation.
	if ( false == CNormalEstimator::is_valid_normal(pcl_cloud_src) ) {
		CNormalEstimator neEstimator; 
		printf_s("compute normals, ptsNum = %05d....\n", pcl_cloud_src->points.size() );
		neEstimator.loadInPlace(pcl_cloud_src);
	}
	if ( false == CNormalEstimator::is_valid_normal(pcl_cloud_tgt) ) {
		CNormalEstimator neEstimator;
		printf_s("compute normals, ptsNum = %05d....\n", pcl_cloud_tgt->points.size());
		neEstimator.loadInPlace(pcl_cloud_tgt);
	}
	//// start match. 
	boost::shared_ptr<CTEASER> pTeaser(new CTEASER());
	pTeaser->setParams( string("Teaser") ); 
	//// set the sampling and feature radius. 
	double dRes = computeModelResFun(pcl_cloud_tgt);
	printf_s("model resolution = %.4f\n", dRes); 
	pTeaser->setRadius(5.0 * dRes, 10.0 * dRes); 
	//// set input cloud. 
	pTeaser->setInputSource(pcl_cloud_src);
	pTeaser->setInputTarget(pcl_cloud_tgt); 
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointNormal>());
	Eigen::Matrix4f Tf0 = Eigen::Matrix4f::Identity(); 
	pTeaser->align(*cloud_aligned, Tf0); 

	//// save for visualization. 
	pcl::PCDWriter pcdWriter;
	string sSaveFile = "cloud_src.pcd";
	pcdWriter.writeBinary(sSaveFile, *pcl_cloud_src);
	sSaveFile = "cloud_tgt.pcd";
	pcdWriter.writeBinary(sSaveFile, *pcl_cloud_tgt);
	sSaveFile = "cloud_aft.pcd";
	pcdWriter.writeBinary(sSaveFile, *cloud_aligned);
	return 0; 
}