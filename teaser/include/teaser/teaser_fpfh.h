/**
 * Copyright 2020, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Jingnan Shi, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#pragma once
#ifndef H_TEASER_FPFH_H
#define H_TEASER_FPFH_H
#include <vector>
#include <teaser/teaser_geometry.h> 
#include <teaser/teaser_utils.h>
#include <TRO_Utilities.h>

using namespace std;

namespace gabor {
	using FEATURE_TYPE = pcl::FPFHSignature33;

	class CFPFH {
	public:
		pcl::search::KdTree<pcl::PointXYZ>::Ptr m_searchMethod;
		// pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
		// pcl::PointCloud<pcl::Normal>::Ptr m_cloud_normal;
		pcl::PointCloud<pcl::PointNormal>::Ptr m_cloud_raw;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr m_cloud_feature;
		boost::shared_ptr<std::vector<int> > m_vkeypoint_indices;
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_keypoints;
		double m_dSampleRadius;
		double m_dFeatureRadius;

		CFPFH() {
			m_dFeatureRadius = 3.0;
			m_dSampleRadius  = m_dFeatureRadius / 2;
			m_cloud_raw.reset(new pcl::PointCloud<pcl::PointNormal>());
		}
		~CFPFH() {}

		void setRadius(double dSampleRadius, double dFeatureRadius) {
			m_dSampleRadius = dSampleRadius;
			m_dFeatureRadius = dFeatureRadius;
		}
		//// 
		void setInputCloud(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud) {
			pcl::copyPointCloud(*input_cloud, *m_cloud_raw);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
			decoupleCloud(m_cloud_raw, cloud_xyz);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method(
				new pcl::search::KdTree<pcl::PointXYZ>);
			search_method->setInputCloud(cloud_xyz);
			m_searchMethod.swap(search_method);
		}
		//// 
		void computeKeypoints() {
			m_vkeypoint_indices.reset(new std::vector<int>());
			m_cloud_keypoints.reset(new pcl::PointCloud<pcl::PointXYZ>());
			//// key_points calculation.
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
			decoupleCloud(m_cloud_raw, cloud_xyz);
			voxelSampleFun(cloud_xyz, m_cloud_keypoints, m_dSampleRadius);
			char tmpChar[2000] = " ";
			sprintf(tmpChar, "%s", "uniform sampling");

			//// key_points index calculation.
			vector<int> pointIdxNKNSearch;
			vector<float> pointNKNSquaredDistance;
			pcl::PointXYZ querryPt, searchPoint;
			int nLen = m_cloud_keypoints->points.size();
			for (int i = 0; i < nLen; i++) {
				pcl::PointXYZ pt = m_cloud_keypoints->points[i];
				m_searchMethod->nearestKSearch(pt, 1, pointIdxNKNSearch,
					pointNKNSquaredDistance);
				m_vkeypoint_indices->push_back(pointIdxNKNSearch[0]);
			}
		}
		void computeFPFH() {
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(
				new pcl::PointCloud<pcl::FPFHSignature33>);
			features.reset(new pcl::PointCloud<pcl::FPFHSignature33>());
			//// calculate descriptor.
			pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal,
				pcl::FPFHSignature33>::Ptr
				fpfh_estimation(new pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal,
					pcl::FPFHSignature33>);
			// fpfh_estimation->setSearchSurface(cloud);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
			decoupleCloud(m_cloud_raw, cloud, normals);
			fpfh_estimation->setInputCloud(cloud);
			fpfh_estimation->setInputNormals(normals);
			fpfh_estimation->setSearchMethod(m_searchMethod);
			fpfh_estimation->setRadiusSearch(m_dFeatureRadius);
			fpfh_estimation->setNumberOfThreads(8);

			fpfh_estimation->setIndices(m_vkeypoint_indices);
			fpfh_estimation->compute(*features);
			//// 
			m_cloud_feature.swap(features);
		}

		void detectZeroFeatures(
			const pcl::PointCloud<FEATURE_TYPE>::Ptr& cloud_features,
			pcl::IndicesPtr& indices) {
			indices->clear();
			FEATURE_TYPE tmpFeature;
			double dDist2 = 0.0;
			for (size_t id = 0; id < cloud_features->points.size(); id++) {
				tmpFeature = cloud_features->points[id];
				dDist2 = 0.0;
				int nLen = tmpFeature.descriptorSize();
				for (int i = 0; i < nLen; i++) {
					// dDist2 += pow(tmpFeature.descriptor[i], 2);
					dDist2 += pow(tmpFeature.histogram[i], 2);
				}
				if (dDist2 >= 1e-4) {
					indices->push_back(id);
				}
			}
		}
		void featureFilter(
			const pcl::PointCloud<FEATURE_TYPE>::Ptr& cloud_descriptors,
			const pcl::IndicesPtr& indices,
			pcl::PointCloud<FEATURE_TYPE>::Ptr& cloud_descriptors_new) {
			FEATURE_TYPE descriptor;
			for (size_t i = 0; i < indices->size(); i++) {
				int nId = (*indices).at(i);
				descriptor = cloud_descriptors->points[nId];
				cloud_descriptors_new->push_back(descriptor);
			}
			cloud_descriptors_new->width = cloud_descriptors_new->points.size();
			cloud_descriptors_new->height = 1;
		}

		void deleteInvalidFeature() {
			pcl::IndicesPtr indices(new vector<int>());
			detectZeroFeatures(m_cloud_feature, indices);
			if (m_cloud_feature->size() != indices->size()) {
				if (indices->size() == 0) {
					printf("Error: no valid descriptors!\n");
					exit(-1);
				}
				pcl::ExtractIndices<pcl::PointXYZ>::Ptr extractor_pointCloud(
					new pcl::ExtractIndices<pcl::PointXYZ>);
				extractor_pointCloud->setInputCloud(m_cloud_keypoints);
				extractor_pointCloud->setIndices(indices);
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(
					new pcl::PointCloud<pcl::PointXYZ>);
				extractor_pointCloud->filter(*cloud_tmp);
				m_cloud_keypoints.swap(cloud_tmp);

				pcl::PointCloud<FEATURE_TYPE>::Ptr cloud_feature_tmp(
					new pcl::PointCloud<FEATURE_TYPE>);
				featureFilter(m_cloud_feature, indices, cloud_feature_tmp);
				m_cloud_feature.swap(cloud_feature_tmp);
			}
			PCL_INFO("compute descriptors done, keypoints = %05d/%05d\n",
				m_cloud_feature->points.size(), m_cloud_raw->points.size());
		}

		template<typename T>
		inline void cvt2Teaser_cloud(const boost::shared_ptr<pcl::PointCloud<T>>& cloud_pcl,
			teaser::PointCloud& cloud_teaser) {
			cloud_teaser.clear();
			for (int i = 0; i < cloud_pcl->points.size(); i++) {
				T pt = cloud_pcl->points[i];
				cloud_teaser.push_back({ pt.x, pt.y, pt.z });
			}
		}

		void compute(
			const pcl::PointCloud<pcl::PointNormal>::Ptr& input_cloud,
			teaser::PointCloud& teaser_cloud,
			teaser::FPFHCloudPtr& fpfh_cloud) {
			setInputCloud(input_cloud);
			computeKeypoints();
			computeFPFH();
			deleteInvalidFeature();
			//// convert to teaser format. 
			teaser_cloud.clear();
			cvt2Teaser_cloud(m_cloud_keypoints, teaser_cloud); 
			fpfh_cloud.reset(new teaser::FPFHCloud());
			pcl::copyPointCloud(*m_cloud_feature, *fpfh_cloud);
		}
	};
}

#endif 
