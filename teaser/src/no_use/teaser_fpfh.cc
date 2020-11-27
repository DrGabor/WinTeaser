/**
 * Copyright 2020, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Jingnan Shi, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#include <teaser/teaser_fpfh.h>

//teaser::FPFHCloudPtr teaser::FPFHEstimation::computeFPFHFeatures (
//	const pcl::PointCloud <pcl::PointNormal>::Ptr& input_cloud, 
//	double fpfh_search_radius) {
//	// Intermediate variables
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	teaser::FPFHCloudPtr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	
//
//	// decouple point cloud and normals
//	decoupleCloud(input_cloud, pcl_input_cloud, normals); 
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
//	kdtree->setInputCloud(pcl_input_cloud);
//
//	// Estimate FPFH
//	// setInputCloud(pcl_input_cloud);
//	fpfh_estimation_->setInputCloud(pcl_input_cloud);
//	fpfh_estimation_->setInputNormals(normals);
//	// setInputNormals(normals);
//	// setSearchMethod(kdtree);
//	// fpfh_estimation_->setSearchMethod(kdtree);
//	fpfh_estimation_->setRadiusSearch(fpfh_search_radius);
//	// setRadiusSearch(fpfh_search_radius);
//	// compute(*descriptors);
//	fpfh_estimation_->compute(*descriptors);
//
//	return descriptors;
//}


//void teaser::FPFHEstimation::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
//  fpfh_estimation_->setInputCloud(input_cloud);
//}
//
//void teaser::FPFHEstimation::setInputNormals(pcl::PointCloud<pcl::Normal>::Ptr input_normals) {
//  fpfh_estimation_->setInputNormals(input_normals);
//}
//
//void teaser::FPFHEstimation::setSearchMethod(
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method) {
//  fpfh_estimation_->setSearchMethod(search_method);
//}
//
//void teaser::FPFHEstimation::compute(pcl::PointCloud<pcl::FPFHSignature33>& output_cloud) {
//  fpfh_estimation_->compute(output_cloud);
//}
//void teaser::FPFHEstimation::setRadiusSearch(double r) { fpfh_estimation_->setRadiusSearch(r); }
