#pragma once

#ifndef H_TEASER_H
#define H_TEASER_H

#include <RegBase.h>
#include <teaser/teaser_matcher.h>
#include <teaser/teaser_registration.h>

class CTEASER : public CRegBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	double m_dSampleRadius; 
	double m_dFeatureRadius;
	virtual void compute_statistics_target() {

	}

	virtual void compute_statistics_source() {

	}

	virtual void setParams(string& sMethod) {
		m_dSampleRadius  = 1.0; 
		m_dFeatureRadius = 1.0; 
	}

	virtual int getTCIterations() {
		return 0;
	}
	void setRadius(double dSampleRadius, double FeatureRadius) {
		m_dSampleRadius  = dSampleRadius; 
		m_dFeatureRadius = FeatureRadius; 
	}
	void setInputTarget(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_target,
		const boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointNormal>> kd_tree_ptr = nullptr) {
		pcl::copyPointCloud(*cloud_target, *m_cloud_target);
		//// m_kd_tree is used to calculate ratio score. 
		if (kd_tree_ptr == nullptr) {
			m_kd_tree.reset(new pcl::KdTreeFLANN<pcl::PointNormal>());
			m_kd_tree->setInputCloud(m_cloud_target);
		} else {
			m_kd_tree = kd_tree_ptr;
		}
	}

	void setInputSource(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_source) {
		pcl::copyPointCloud(*cloud_source, *m_cloud_source);
		compute_statistics_source();
	}

	virtual void setInputSurface(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_source) {

	}

	Eigen::Matrix4f getFinalTransformation() const {
		return m_rstTf.matrix().cast<float>();
	}

	//// need to be implemented by derived class. 
	virtual void align(pcl::PointCloud<pcl::PointNormal>& cloud_aligned,
		Eigen::Matrix4f Tf0 = Eigen::Matrix4f::Identity()) {
		clock_t tBeg, tEnd; 
		tBeg = clock(); 
		gabor::CFPFH fpfhEstimator;
		fpfhEstimator.setRadius(m_dSampleRadius, m_dFeatureRadius);
		teaser::PointCloud src_cloud, tgt_cloud;
		teaser::FPFHCloudPtr src_descriptors(new teaser::FPFHCloud());
		fpfhEstimator.compute(m_cloud_source, src_cloud, src_descriptors);
		teaser::FPFHCloudPtr tgt_descriptors(new teaser::FPFHCloud());
		fpfhEstimator.compute(m_cloud_target, tgt_cloud, tgt_descriptors);
		tEnd = clock(); 
		int nTime_fpfh = tEnd - tBeg; 
		//// compute correspondence.  
		tBeg = clock(); 
		teaser::Matcher matcher;
		bool use_absolute_scale = true,
			use_crosscheck      = true,
			use_tuple_test      = true; // false;
		float tuple_scale       = 0.85;

		auto correspondences = matcher.calculateCorrespondences(
			src_cloud, tgt_cloud,
			*src_descriptors, *tgt_descriptors,
			use_absolute_scale, use_crosscheck, use_tuple_test, tuple_scale);
		tEnd = clock(); 
		int nTime_corres = tEnd - tBeg; 
		// Run TEASER++ registration
		// Prepare solver parameters
		tBeg = clock(); 
		teaser::RobustRegistrationSolver::Params params;
		//// noise_bound is very important. 
		//// Using Gaussian noise for 3D point cloud, noise_bound = sqrt(3) * sigma. 
		params.noise_bound = m_dSampleRadius / 2;
		params.cbar2 = 1.0; //
		params.estimate_scaling = false;
		params.rotation_max_iterations = 100; // 100
		params.rotation_gnc_factor = 1.4;
		params.rotation_estimation_algorithm =
			teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
		params.rotation_cost_threshold = 0.005;

		// Solve with TEASER++
		teaser::RobustRegistrationSolver solver(params);
		solver.solve(src_cloud, tgt_cloud, correspondences);
		tEnd = clock(); 
		int nTime_reg = tEnd - tBeg; 

		auto solution = solver.getSolution();

		//// save registration result. 
		m_rstTf = Eigen::Matrix4d::Identity();
		m_rstTf.block(0, 0, 3, 3) = solution.rotation.cast<double>();
		m_rstTf.block(0, 3, 3, 1) = solution.translation.cast<double>();
		pcl::transformPointCloud(*m_cloud_source, cloud_aligned, m_rstTf);
		printf_s("time_fpfh = %04dms, time_corres = %04dms, time_reg = %04dms, corresNum = %04d\n", 
			nTime_fpfh, nTime_corres, nTime_reg, correspondences.size() ); 
		bool bTest = true; 
	}

private:


};

#endif // !H_TEASER_H

