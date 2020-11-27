#pragma once
#ifndef H_TRO_UTILITIES_H
#define H_TRO_UTILITIES_H
#include "stdafx.h"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <direct.h>
#include <string>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
//
inline void stringSplit(std::string s, 
	std::string delimiter, std::vector<std::string>& vToken) {
	vToken.clear();
	size_t pos = 0;
	std::string token;
	while ((pos = s.find(delimiter)) != std::string::npos) {
		token = s.substr(0, pos);
		// std::cout << token << std::endl;
		s.erase(0, pos + delimiter.length());
		vToken.push_back(token);
	}
	if (false == s.empty()) {
		vToken.push_back(s);
	}
}

inline void tf2quatFun(const Eigen::Matrix4f& Tf, Eigen::Vector4f& coeff, Eigen::Vector3f& dT) {
	Eigen::Quaternionf quat;
	quat = Tf.block<3, 3>(0, 0);
	coeff = quat.coeffs();
	dT = Tf.block(0, 3, 3, 1);
}

inline void quat2tfFun(const std::vector<float>& vq, const std::vector<float>& vT, Eigen::Matrix4f& Tf) {
	Eigen::Quaternionf q0;
	q0.w() = vq[0];
	q0.x() = vq[1];
	q0.y() = vq[2];
	q0.z() = vq[3];

	Eigen::Matrix3f rot0 = q0.normalized().toRotationMatrix();
	Eigen::Vector3f trt0;
	trt0 << vT[0], vT[1], vT[2];
	Eigen::Matrix4f Tf0 = Eigen::Matrix4f::Identity();
	Tf.block<3, 3>(0, 0) = rot0;
	Tf.block<3, 1>(0, 3) = trt0;
}

template <typename T>
inline void randSampleFun(const boost::shared_ptr<pcl::PointCloud<T>>& cloud_in,
	boost::shared_ptr<pcl::PointCloud<T>>& cloud_out,
	int nCnt) {
	cloud_out.reset(new pcl::PointCloud<T>());
	if ( cloud_in->points.empty()  )
	{
		return; 
	}
	pcl::copyPointCloud(*cloud_in, *cloud_out);
	float fRatio = nCnt / float(cloud_in->points.size());
	if (fRatio >= 1.0) {
		return;
	}
	//// the following is samplling without replacement. 
	std::random_device rd;
	std::mt19937 g(rd());
	std::shuffle(cloud_out->points.begin(), cloud_out->points.end(), g);
	cloud_out->points.erase(cloud_out->points.begin() + nCnt, cloud_out->points.end()); 
	cloud_out->width = 1;
	cloud_out->height = cloud_out->points.size();
	cloud_out->is_dense = true;
}

template <typename T>
inline void randSampleInPlaceFun(boost::shared_ptr<pcl::PointCloud<T>>& cloud_in,
	int nCnt) {
	boost::shared_ptr<pcl::PointCloud<T>> cloud_out(new pcl::PointCloud<T>());
	randSampleFun(cloud_in, cloud_out, nCnt); 
	cloud_in.swap(cloud_out); 
}

template <typename T>
inline void voxelSampleFun( boost::shared_ptr<pcl::PointCloud<T>>& cloud,
	boost::shared_ptr<pcl::PointCloud<T>>& cloud_out,
	double dGridSize) {
	if (dGridSize < 0) {
		pcl::copyPointCloud(*cloud, *cloud_out); 
		return;
	}
	pcl::ApproximateVoxelGrid<T> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(dGridSize, dGridSize, dGridSize);
	approximate_voxel_filter.setInputCloud(cloud);
	approximate_voxel_filter.filter(*cloud_out);
	cloud_out->sensor_origin_      = cloud->sensor_origin_;
	cloud_out->sensor_orientation_ = cloud->sensor_orientation_;
}

template <typename T>
inline void voxelSampleInPlaceFun(boost::shared_ptr<pcl::PointCloud<T>>& cloud,
	double dGridSize) {
	boost::shared_ptr<pcl::PointCloud<T>> cloud_out(new pcl::PointCloud<T>());
	voxelSampleFun(cloud_in, cloud_out, dGridSize);
	cloud.swap(cloud_out);
}

template <typename T>
inline void scaleCloudFun(boost::shared_ptr<pcl::PointCloud<T>>& cloud, 
	double dScale) {
	for (int i = 0; i < cloud->points.size(); i++) {
		T pt = cloud->points[i];
		pt.x /= dScale;
		pt.y /= dScale;
		pt.z /= dScale;
		cloud->points[i] = pt;
	}
}

inline void catCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_xyz,
	const pcl::PointCloud<pcl::Normal>::Ptr& cloud_nor,
	pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{
	if (cloud_xyz->points.size() != cloud_nor->points.size())
	{
		cout << "Error in catCloud(), the number of xyz and nor is not the same!\n";
		exit(-1);
	}
	cloud_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud_xyz, *cloud_normals);
	pcl::copyPointCloud<pcl::Normal, pcl::PointNormal>(*cloud_nor, *cloud_normals);
}

inline void decoupleCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_xyz,
	pcl::PointCloud<pcl::Normal>::Ptr&   cloud_nor)
{
	cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_nor.reset(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*cloud_normals, *cloud_xyz);
	pcl::copyPointCloud<pcl::PointNormal, pcl::Normal>(*cloud_normals, *cloud_nor);
}

inline void decoupleCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_xyz)
{
	cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_nor(new pcl::PointCloud<pcl::Normal>());
	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*cloud_normals, *cloud_xyz);
	pcl::copyPointCloud<pcl::PointNormal, pcl::Normal>(*cloud_normals, *cloud_nor);
}

template <typename T>
inline void read_cloud(std::string& sFileName,
	boost::shared_ptr< pcl::PointCloud<T>> & cloud_normal) {
	cloud_normal.reset(new pcl::PointCloud<T>() );
	if (0 != access(sFileName.c_str(), 0) ) {
		printf_s("%s does not exist!\n", sFileName.c_str()); 
		exit(-1); 
	}
	std::string sSuffix = sFileName.substr(sFileName.length() - 3, 3);
	if (0 == sSuffix.compare(std::string("ply"))) {
		pcl::PLYReader plyReader;
		plyReader.read(sFileName.c_str(), *cloud_normal);
	}
	if (0 == sSuffix.compare(std::string("pcd"))) {
		pcl::PCDReader pcdReader;
		pcdReader.read(sFileName.c_str(), *cloud_normal);
	}
}

void map_truncateFun(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_map_color,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& map_disp_sparse,
	const Eigen::Matrix4f& rstTf,
	const double& dLen,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& map_display); 

class CNormalEstimator
{
public:
	////
	CNormalEstimator() {
		m_sNeighbor = "knn";
		m_dNormRad  = 0.5; 
		clear();
	}
	//// 
	~CNormalEstimator() {
	}

	void setRadius(double dNormRad) {
		m_sNeighbor = "radius"; 
		m_dNormRad  = dNormRad; 
	}
	//// 
	template <typename T>
	void load(const boost::shared_ptr<pcl::PointCloud<T>>& cloud_in) {
		clear();
		//// load input point cloud. 
		for (auto i = 0; i < cloud_in->points.size(); i++) {
			T pt_src = cloud_in->points[i];
			pcl::PointXYZ pt_dst;
			pt_dst.x = pt_src.x;
			pt_dst.y = pt_src.y;
			pt_dst.z = pt_src.z;
			m_cloud_xyz->points.push_back(pt_dst);
		}
		m_cloud_xyz->is_dense = true;
		m_cloud_xyz->width = 1;
		m_cloud_xyz->height = m_cloud_xyz->points.size();
		//// set kd_tree. 
		m_kd_tree->setInputCloud(m_cloud_xyz);
		//// compute normals. 
		compute_normal();
	}
	void get_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out) const {
		pcl::copyPointCloud(*m_cloud_out, *cloud_out);
	}
	//// this will change cloud_out. 
	void loadInPlace(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out) {
		load(cloud_out);
		get_cloud(cloud_out);
	}
	static bool is_valid_normal(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_in) {
		//// randomly check the feasibility. 
		for (auto i = 0; i < cloud_in->points.size(); i += 100) {
			pcl::PointNormal pt = cloud_in->points[i];
			double dist = pow(pt.normal_x, 2) + pow(pt.normal_y, 2) + pow(pt.normal_z, 2);
			if (dist <= 0.99) {
				return false;
			}
		}
		return true;
	}
	//// make sure that this class is user-security. 
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_xyz;
	pcl::PointCloud<pcl::Normal>::Ptr m_cloud_nor;
	pcl::PointCloud<pcl::PointNormal>::Ptr m_cloud_out;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr m_kd_tree;
	std::string m_sNeighbor;
	double m_dNormRad; 

	//// 
	void clear() {
		m_cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>());
		m_cloud_nor.reset(new pcl::PointCloud<pcl::Normal>());
		m_cloud_out.reset(new pcl::PointCloud<pcl::PointNormal>());
		m_kd_tree.reset(new pcl::search::KdTree<pcl::PointXYZ>());
	}
	//// 
	void delete_nan_element() {
		pcl::Normal norm_pt;
		boost::shared_ptr<std::vector<int> > valid_indices(new std::vector<int>);
		for (auto i = 0; i < m_cloud_nor->points.size(); i++) {
			norm_pt = m_cloud_nor->points[i];
			if (!isnan(norm_pt.normal_x)) {
				valid_indices->push_back(i);
			}
		}
		if (m_cloud_nor->points.size() != valid_indices->size()) {
			printf_s("Delete nan normals: %02d, %05d/%05d\n",
				m_cloud_nor->points.size() - valid_indices->size(),
				valid_indices->size(), m_cloud_nor->points.size());
			pcl::ExtractIndices<pcl::PointXYZ>::Ptr Extractor_Point(new pcl::ExtractIndices<pcl::PointXYZ>);
			Extractor_Point->setInputCloud(m_cloud_xyz);
			Extractor_Point->setIndices(valid_indices);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>());
			Extractor_Point->filter(*cloud_tmp);
			m_cloud_xyz.swap(cloud_tmp);
			//// 
			pcl::ExtractIndices<pcl::Normal>::Ptr extractor_normal(new pcl::ExtractIndices<pcl::Normal>);
			extractor_normal->setInputCloud(m_cloud_nor);
			extractor_normal->setIndices(valid_indices);
			pcl::PointCloud<pcl::Normal>::Ptr normals_tmp(new pcl::PointCloud<pcl::Normal>);
			extractor_normal->filter(*normals_tmp);
			m_cloud_nor.swap(normals_tmp);
		}
	}
	//// 
	void compute_normal() {
		//// normal estimator. 
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_filter;
		ne_filter.setInputCloud(m_cloud_xyz);
		ne_filter.setSearchMethod(m_kd_tree);
		ne_filter.setNumberOfThreads(8);
		if (0 == m_sNeighbor.compare("knn")) {
			ne_filter.setKSearch(20);
		}
		if (0 == m_sNeighbor.compare("radius")) {
			ne_filter.setRadiusSearch(m_dNormRad);
		}
		ne_filter.compute(*m_cloud_nor);
		//// delete invalid element. 
		delete_nan_element();
		//// output the cloud. 
		catCloud(m_cloud_xyz, m_cloud_nor, m_cloud_out);
	}
};

#endif 