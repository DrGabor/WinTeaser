#pragma once
#ifndef H_REG_BASE_H
#define H_REG_BASE_H
#include "stdafx.h"
#include <vector>
#include <string>

using namespace std; 

//// Use key word of virtual in order to be polymoriphsm. 
class CRegBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	//// point set. 
	pcl::PointCloud<pcl::PointNormal>::Ptr  m_cloud_source;
	pcl::PointCloud<pcl::PointNormal>::Ptr  m_cloud_target;
	pcl::KdTreeFLANN<pcl::PointNormal>::Ptr m_kd_tree;

	Eigen::Matrix3Xd m_Mov0;
	Eigen::Matrix3Xd m_Ref0;
	Eigen::Matrix3Xd m_RefNorm0;
	Eigen::Matrix3Xd m_tmpAft;
	Eigen::Matrix3Xd m_tmpRef;
	Eigen::Matrix3Xd m_tmpNor;
	std::vector<std::pair<int, int>> m_vInd; 
	vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> m_vOmega;

	//// transformations. 
	Eigen::Matrix4d m_Tf0;
	Eigen::Matrix4d m_rstTf;
	void init_cloud() {
		m_cloud_target.reset(new pcl::PointCloud<pcl::PointNormal>());
		m_cloud_source.reset(new pcl::PointCloud<pcl::PointNormal>());
		m_kd_tree.reset(new pcl::KdTreeFLANN<pcl::PointNormal>());
	}
	//// 
	CRegBase() {
		init_cloud(); 
	}
	//// 
	virtual ~CRegBase() {

	}
	//// tmpAft must be specified. 
	void compute_NN(double max_dist_sq = 1e8) {
		Eigen::Matrix3Xd Aft = m_tmpAft; 
		vector<int> vIndMov(Aft.cols(), -1); 
		vector<int> vIndRef(Aft.cols(), -1);
#pragma omp parallel for
		for (int i = 0; i < Aft.cols(); ++i) {
			pcl::PointNormal querryPt;
			querryPt.x = Aft(0, i);
			querryPt.y = Aft(1, i);
			querryPt.z = Aft(2, i);
			std::vector<int> vIdx;
			std::vector<float> vDist2; 
			m_kd_tree->nearestKSearch(querryPt, 1, vIdx, vDist2);
			if (vDist2[0] < max_dist_sq) {
				vIndMov[i] = i; 
				vIndRef[i] = vIdx[0]; 
			}
		}
		m_vInd.clear(); 
		for (int i = 0; i < vIndMov.size(); i++) {
			if (vIndMov[i] != -1) {
				m_vInd.push_back( std::make_pair(vIndMov[i], vIndRef[i]) );
			}
		}
		//// store the reference in m_tmpRef. 
		m_tmpRef.resize(3, m_vInd.size());
		m_tmpAft.resize(3, m_vInd.size());
		//// the normal info should also be retrived. 
		bool bUseNorm = m_RefNorm0.size() > 0; 
		if (bUseNorm) {
			m_tmpNor.resize(3, m_vInd.size());
		}
		for (int i = 0; i < m_vInd.size(); i++) {
			int nMov = m_vInd[i].first;
			int nRef = m_vInd[i].second;
			m_tmpAft.col(i) = Aft.col(nMov);
			m_tmpRef.col(i) = m_Ref0.col(nRef);
			if (bUseNorm) {
				m_tmpNor.col(i) = m_RefNorm0.col(nRef);
			}
		}
	}
	//// compute the point-to-plane distance. 
	double compute_dist(const pcl::PointNormal& pt_source, const pcl::PointNormal& pt_target) {
		Eigen::Vector3d pt0, pt1; 
		pt0 << pt_source.x, pt_source.y, pt_source.z; 
		pt1 << pt_target.x, pt_target.y, pt_target.z; 
		Eigen::Vector3d nor; 
		nor << pt_target.normal_x, pt_target.normal_y, pt_target.normal_z; 
		double dist = abs( nor.dot(pt0 - pt1) );
		return pow(dist, 2); 
	}
	//// tmpAft must be specified. 
	void compute_NN_reciprocal(const pcl::KdTreeFLANN<pcl::PointNormal>::Ptr& kd_tree_source, 
		const Eigen::Matrix4d& rstTf, double max_dist_sq) {
		Eigen::Matrix4d invTf = rstTf.inverse(); 
		Eigen::Matrix3Xd Aft = m_tmpAft;
		vector<int> vIndMov(Aft.cols(), -1);
		vector<int> vIndRef(Aft.cols(), -1);
#pragma omp parallel for
		for (int i = 0; i < Aft.cols(); ++i) {
			pcl::PointNormal querryPt;
			querryPt.x = Aft(0, i);
			querryPt.y = Aft(1, i);
			querryPt.z = Aft(2, i);
			std::vector<int> vIdx;
			std::vector<float> vDist2;
			m_kd_tree->nearestKSearch(querryPt, 1, vIdx, vDist2);
			int nId_target = vIdx[0]; 
			double dist_target = vDist2[0];
			if (dist_target < max_dist_sq) {
				pcl::PointNormal targetPt = m_cloud_target->points[nId_target];
				Eigen::Vector4d pt; 
				pt << targetPt.x, targetPt.y, targetPt.z, 1; 
				pt.noalias() = invTf * pt; 
				targetPt.x = pt(0); 
				targetPt.y = pt(1); 
				targetPt.z = pt(2); 
				kd_tree_source->nearestKSearch(targetPt, 1, vIdx, vDist2);
				int nId_source = vIdx[0];
				double dist_source = vDist2[0];
				double dist_diff = abs(dist_source - dist_target); 
				if (dist_source < max_dist_sq && dist_diff <= 0.5) { // nId_source == i
				
					vIndMov[i] = i;
					vIndRef[i] = nId_target;
				}
			}
		}
		m_vInd.clear();
		for (int i = 0; i < vIndMov.size(); i++) {
			if (vIndMov[i] != -1) {
				m_vInd.push_back(std::make_pair(vIndMov[i], vIndRef[i]));
			}
		}
		//// store the reference in m_tmpRef. 
		m_tmpRef.resize(3, m_vInd.size());
		m_tmpAft.resize(3, m_vInd.size());
		bool bUseNorm = m_RefNorm0.size() > 0;
		if (bUseNorm) {
			m_tmpNor.resize(3, m_vInd.size());
		}
		for (int i = 0; i < m_vInd.size(); i++) {
			int nMov = m_vInd[i].first;
			int nRef = m_vInd[i].second;
			m_tmpAft.col(i) = Aft.col(nMov);
			m_tmpRef.col(i) = m_Ref0.col(nRef);
			if (bUseNorm) {
				m_tmpNor.col(i) = m_RefNorm0.col(nRef);
			}
		}
	}

	virtual void compute_statistics_target() {

	}

	virtual void compute_statistics_source() {

	}

	virtual void setParams(string& sMethod) {

	}

	virtual int getTCIterations() {
		return 0; 
	}

	void setInputTarget(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_target, 
		const boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointNormal>> kd_tree_ptr = nullptr ) {
		pcl::copyPointCloud<pcl::PointNormal, pcl::PointNormal>(*cloud_target, *m_cloud_target);
		if (kd_tree_ptr == nullptr) {
			m_kd_tree.reset(new pcl::KdTreeFLANN<pcl::PointNormal>()); 
			m_kd_tree->setInputCloud(m_cloud_target);
		} else {
			m_kd_tree = kd_tree_ptr; 
		}
		compute_statistics_target(); 
		m_Ref0.resize(3, m_cloud_target->points.size());
		m_RefNorm0.resize(3, m_cloud_target->points.size());
		for (int i = 0; i < m_cloud_target->points.size(); i++) {
			pcl::PointNormal pt = m_cloud_target->points[i];
			m_Ref0.col(i) << pt.x, pt.y, pt.z;
			m_RefNorm0.col(i) << pt.normal_x, pt.normal_y, pt.normal_z;
		}
	}

	void setInputSource(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_source) {
		pcl::copyPointCloud<pcl::PointNormal, pcl::PointNormal>(*cloud_source, *m_cloud_source);
		m_Mov0.resize(3, m_cloud_source->points.size());
		for (int i = 0; i < m_cloud_source->points.size(); i++) {
			pcl::PointNormal pt = m_cloud_source->points[i];
			m_Mov0.col(i) << pt.x, pt.y, pt.z;
		}
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
		printf_s("hi, its align() in CICPBase!\n");
		exit(-1); 
	}
	//// by default, the outlier and inlier is discerned by distance. 
	virtual void outlier_awareness(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_source,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ins,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ous) {
		cloud_ins.reset(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ous.reset(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_aft(new pcl::PointCloud<pcl::PointNormal>());
		pcl::transformPointCloud(*cloud_source, *cloud_aft, m_rstTf);
		// std::cout << "No outlier_awareness is available!\n" <<std::endl; 
		for (auto i = 0; i < cloud_aft->points.size(); i++) {
			pcl::PointNormal querryPt = cloud_aft->points[i];
			vector<float> vDist2; 
			m_kd_tree->nearestKSearch(querryPt, 1, vector<int>(), vDist2);
			pcl::PointXYZ pt; 
			pt.x = querryPt.x; 
			pt.y = querryPt.y; 
			pt.z = querryPt.z; 
			if ( vDist2[0] >= pow(0.2, 2) ) {
				cloud_ous->points.push_back(pt); 
			} else {
				cloud_ins->points.push_back(pt);
			}
		}
		cloud_ins->is_dense = true;
		cloud_ins->width    = 1;
		cloud_ins->height   = cloud_ins->points.size();
		cloud_ous->is_dense = true;
		cloud_ous->width    = 1;
		cloud_ous->height   = cloud_ous->points.size();
	}

	std::vector<float> computeRatioFun(const pcl::PointCloud<pcl::PointNormal>::Ptr& source_cloud,
		const Eigen::Matrix4f& rstTf,
		double max_dist ) {
		std::vector<float> vRatio(2, 0.0);
		if (source_cloud->points.empty()) {
			return vRatio; 
		}
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointNormal>);
		pcl::transformPointCloud(*source_cloud, *cloud_aligned, rstTf);
		double max_dist2 = pow(max_dist, 2);
		std::vector<int> vPT(cloud_aligned->points.size(), 0), vPL(cloud_aligned->points.size(), 0); 
#pragma omp parallel for
		for (int i = 0; i < cloud_aligned->points.size(); i++) {
			pcl::PointNormal aftPt = cloud_aligned->points[i];
			std::vector<int> vIdx;
			m_kd_tree->nearestKSearch(aftPt, 1, vIdx, std::vector<float>());
			int nIdx = vIdx[0];
			pcl::PointNormal refPt = m_cloud_target->points[nIdx];
			float dx = aftPt.x - refPt.x;
			float dy = aftPt.y - refPt.y;
			float dz = aftPt.z - refPt.z;
			float nx = refPt.normal_x, ny = refPt.normal_y, nz = refPt.normal_z;
			float fDist_pl = abs(dx*nx + dy*ny + dz*nz);
			if (fDist_pl <= max_dist) {
				vPL[i] = 1; 
			}
			float fDist_p = dx*dx + dy*dy + dz*dz;
			if (fDist_p <= max_dist2) {
				vPT[i] = 1; 
			}
		}
		int nCnt_PL = std::accumulate(vPL.begin(), vPL.end(), 0);
		int nCnt_PT = std::accumulate(vPT.begin(), vPT.end(), 0); 
		float ratio0 = float(nCnt_PL) / cloud_aligned->points.size();
		float ratio1 = float(nCnt_PT) / cloud_aligned->points.size();
		vRatio[0] = ratio0;
		vRatio[1] = ratio1;
		return vRatio;
	}

};

#endif // !H_ICP_BASE_H
