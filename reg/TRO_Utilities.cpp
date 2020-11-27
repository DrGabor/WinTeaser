#include "stdafx.h"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <direct.h>
#include <string>
#include <vector>

#include "TRO_Utilities.h"

void map_truncateFun(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_map_color,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& map_disp_sparse,
	const Eigen::Matrix4f& rstTf,
	const double& dLen,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& map_display)
{
	map_display.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	Eigen::Matrix4f invTf = rstTf.inverse();
	double x_center = rstTf(0, 3);
	double y_center = rstTf(1, 3);
	for (int i = 0; i < cloud_map_color->points.size(); i++)
	{
		pcl::PointXYZRGB pt = cloud_map_color->points[i];
		if (abs(pt.x - x_center) <= dLen && abs(pt.y - y_center) <= dLen)
		{
			map_display->points.push_back(pt);
		}
	}
	for (int i = 0; i < map_disp_sparse->points.size(); i++)
	{
		map_display->points.push_back(map_disp_sparse->points[i]);
	}
	map_display->width = 1;
	map_display->height = map_display->points.size();
	map_display->is_dense = true;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::transformPointCloud(*map_display, *cloud_tmp, invTf);
	cloud_tmp.swap(map_display); 
}

//std::vector<float> CalQualityFun(pcl::PointCloud<pcl::PointNormal>::Ptr& source_cloud,
//	pcl::PointCloud<pcl::PointNormal>::Ptr& target_cloud,
//	pcl::KdTreeFLANN<pcl::PointNormal>::Ptr & kd_tree,
//	Eigen::Matrix4f rstTf,
//	double maxDist)
//{
//	std::vector<float> vRatio;
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
//	pcl::transformPointCloud(*source_cloud, *cloud, rstTf);
//
//	float ratio = 0.0;
//	pcl::PointNormal aftPt;
//	pcl::PointNormal refPt;
//	std::vector<int> vIdx;
//	std::vector<float> vDist;
//	int nCnt_point = 0, nCnt_plane = 0;
//	for (int i = 0; i < cloud->points.size(); i++)
//	{
//		aftPt = cloud->points[i];
//		kd_tree->nearestKSearch(aftPt, 1, vIdx, vDist);
//		int nIdx = vIdx[0];
//		refPt = target_cloud->points[nIdx];
//		float dx = aftPt.x - refPt.x;
//		float dy = aftPt.y - refPt.y;
//		float dz = aftPt.z - refPt.z;
//		float nx = refPt.normal_x, ny = refPt.normal_y, nz = refPt.normal_z;
//		float fDist_pl = abs(dx*nx + dy*ny + dz*nz);
//		if (fDist_pl <= maxDist)
//		{
//			nCnt_plane++;
//		}
//
//		float fDist_p = dx*dx + dy*dy + dz*dz;
//		if (fDist_p <= maxDist*maxDist)
//		{
//			nCnt_point++;
//		}
//	}
//	float ratio0 = float(nCnt_plane) / cloud->points.size();
//	float ratio1 = float(nCnt_point) / cloud->points.size();
//	vRatio.push_back(ratio0);
//	vRatio.push_back(ratio1);
//	return vRatio;
//}
//
//std::vector<float> CalQualityFun(pcl::PointCloud<pcl::PointNormal>::Ptr& source_cloud,
//	pcl::PointCloud<pcl::PointNormal>::Ptr& target_cloud,
//	Eigen::Matrix4f rstTf,
//	double maxDist)
//{
//	pcl::KdTreeFLANN<pcl::PointNormal>::Ptr kd_tree(new pcl::KdTreeFLANN<pcl::PointNormal>());
//	kd_tree->setInputCloud(target_cloud);
//	return CalQualityFun(source_cloud, target_cloud, kd_tree, rstTf, maxDist);
//}

//namespace CURVATURE_SELECTOR
//{
//	CURVATURE_SELECTOR::sLocFV LocFVExtract(const pcl::PointCloud<pcl::PointNormal>::Ptr& Cloud, 
//		std::vector<int>& k_Ind)
//	{
//		CURVATURE_SELECTOR::sLocFV TmpFV;
//		Eigen::Matrix3d covMat;
//		Eigen::Matrix<double, 4, 1> centroid;
//		pcl::compute3DCentroid(*Cloud, k_Ind, centroid);
//		pcl::computeCovarianceMatrixNormalized(*Cloud, k_Ind, centroid, covMat);
//		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> EigenSolve(covMat, Eigen::ComputeEigenvectors);
//		TmpFV.m_eVect = EigenSolve.eigenvectors();
//		TmpFV.m_eVal = EigenSolve.eigenvalues(); // MaxVal - (2) --- MinVal - (0)
//		TmpFV.m_Curve = TmpFV.m_eVal(0) / TmpFV.m_eVal.sum();
//		return TmpFV;
//	}
//
//	void SelLargeCurvatureFun(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_input,
//		pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_sig,
//		int nSampleNum,
//		double Radius)
//	{
//		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_seed(new pcl::PointCloud<pcl::PointNormal>);
//		randSampleFun(cloud_input, cloud_seed, nSampleNum);
//
//		pcl::search::KdTree<pcl::PointNormal>::Ptr SearchTreePtr(new pcl::search::KdTree<pcl::PointNormal>);
//		SearchTreePtr->setInputCloud(cloud_input);
//		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_grd(new pcl::PointCloud<pcl::PointNormal>);
//		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_obs(new pcl::PointCloud<pcl::PointNormal>);
//
//		pcl::PointNormal searchPoint;
//		int nLen = cloud_seed->points.size();
//		for (int nId = 0; nId < nLen; nId++)
//		{
//			searchPoint = cloud_seed->points[nId];
//			std::vector<int> k_Ind;
//			std::vector<float> k_Dist;
//			int nNum = SearchTreePtr->radiusSearch(searchPoint, Radius, k_Ind, k_Dist);
//			if (nNum < 10)
//			{
//				continue;
//			}
//			sLocFV TmpFV = LocFVExtract(cloud_input, k_Ind);
//
//			if (TmpFV.m_Curve >= 0.01)
//			{
//				cloud_obs->points.push_back(searchPoint);
//			}
//			else
//			{
//				cloud_grd->points.push_back(searchPoint);
//			}
//		}
//		cloud_obs->width = cloud_obs->size();
//		cloud_obs->height = 1;
//		cloud_obs->is_dense = true;
//
//		cloud_grd->width = cloud_grd->size();
//		cloud_grd->height = 1;
//		cloud_grd->is_dense = true;
//
//		int nCnt = 0.2 * cloud_obs->points.size();
//		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_grd_filter(new pcl::PointCloud<pcl::PointNormal>);
//		randSampleFun(cloud_grd, cloud_grd_filter, nCnt);
//		cloud_sig.swap(cloud_obs);
//		cloud_sig->points.insert(cloud_sig->points.begin(), cloud_grd_filter->points.begin(), cloud_grd_filter->points.end());
//		cloud_sig->width = cloud_sig->points.size();
//		cloud_sig->height = 1;
//	}
//}


//void trimRadiusFun(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud, pcl::PointNormal QuerryPt, double maxRadius)
//{
//	double maxRad2 = pow(maxRadius, 2); 
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointNormal>);
//	for (int i = 0; i < cloud->points.size(); i++)
//	{
//		pcl::PointNormal pt = cloud->points[i]; 
//		double dx = QuerryPt.x - pt.x; 
//		double dy = QuerryPt.y - pt.y; 
//		if ( abs(dx) >= maxRadius || abs(dy) >= maxRadius )
//		{
//			continue; 
//		}
//		double dDist = pow(dx, 2) + pow(dy, 2); 
//		if (dDist <= maxRad2)
//		{
//			cloud_tmp->points.push_back(pt); 
//		}
//	}
//	cloud_tmp->width = 1; 
//	cloud_tmp->height = cloud_tmp->size(); 
//	cloud_tmp->is_dense = true; 
//	cloud.swap(cloud_tmp); 
//}

//void KNNSearchFun(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_map,
//	pcl::search::KdTree<pcl::PointNormal>::Ptr search_method_map,
//	pcl::PointNormal pt,
//	int nK, 
//	pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out)
//{
//	cloud_out.reset(new pcl::PointCloud<pcl::PointNormal>);
//	vector<int> vIdx;
//	vector<float> vDist;
//	search_method_map->nearestKSearch(pt, nK, vIdx, vDist);
//	for (int i = 0; i < vIdx.size(); i++)
//	{
//		int nId = vIdx[i];
//		pt = cloud_map->points[nId];
//		cloud_out->points.push_back(pt);
//	}
//	cloud_out->width = 1;
//	cloud_out->height = cloud_out->points.size();
//	cloud_out->is_dense = true;
//}
//
//void RadiusSearchFun(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_map,
//	pcl::search::KdTree<pcl::PointNormal>::Ptr search_method_map,
//	pcl::PointNormal pt,
//	double dRadius,
//	pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out)
//{
//	cloud_out.reset(new pcl::PointCloud<pcl::PointNormal>);
//	vector<int> vIdx;
//	vector<float> vDist;
//	search_method_map->radiusSearch(pt, dRadius, vIdx, vDist, 0);
//	for (int i = 0; i < vIdx.size(); i++)
//	{
//		int nId = vIdx[i];
//		pt = cloud_map->points[nId];
//		cloud_out->points.push_back(pt);
//	}
//	cloud_out->width = 1;
//	cloud_out->height = cloud_out->points.size();
//	cloud_out->is_dense = true;
//}