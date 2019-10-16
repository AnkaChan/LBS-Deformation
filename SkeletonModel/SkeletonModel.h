#ifndef _SKELETON_MODEL_H_
#define _SKELETON_MODEL_H_

#ifndef EIGEN_STACK_ALLOCATION_LIMIT
#define EIGEN_STACK_ALLOCATION_LIMIT 10000000
#endif
#include "Eigen/Core"
#include <Eigen/StdVector>

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "ceres/rotation.h"
#include "tinycolormap.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <vector>
#include <map>
#include <fstream>

#define JOINT_ROT_DIMENSION 3

#ifndef SKEL_ROT_AXIS_ANGLE
#define SKEL_ROT_EULER_ANGLE

#endif // !SKEL_ROT_EULER_ANGLE


template <typename T1, typename T2, size_t rows, size_t cols>
static void copyToEigenFixedSizeMat(const std::vector<std::vector<T1>> & m, Eigen::Matrix<T2, rows, cols> & em) {
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++) 
		{
			em(i, j) = m[i][j];
		}
	}
}

template <typename T1, typename T2>
static void copyToEigenDynamicSizeMat(const std::vector<std::vector<T1>>& m, Eigen::Matrix<T2, Eigen::Dynamic, Eigen::Dynamic>& em) {
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			em(i, j) = m[i][j]
		}
	}
}


template <size_t nVerts, size_t nJoints, typename T>
struct SkeletonStaticData
{
public:
	Eigen::Matrix<T, 4, nVerts> vertsRestPose_;
	Eigen::Matrix<T, 4, nJoints> jointPos_;
	Eigen::Matrix<T, nJoints, nVerts> vertsWeights_;
	std::array<std::array<int, nJoints>, 2> kintree_table;
	std::map<size_t, size_t> parents;
	std::vector<std::vector<int>> faces;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	void readJsonSkelData(std::string jsonFilePath) {
		std::vector<std::vector<double>> vertsRestPoseMat;
		std::vector<std::vector<double>> vertsWeightsMat;
		std::vector<std::vector<double>> jointPosMat;
		//std::vector<std::vector<int>> kintree_tableMat;
		std::vector<std::vector<int>> faceMat;

		std::ifstream is(jsonFilePath);
		nlohmann::json j;
		is >> j;
		
		vertsRestPoseMat = j["VTemplate"].get<std::vector<std::vector<double>>>();
		copyToEigenFixedSizeMat<double, T, 4, nVerts>(vertsRestPoseMat, vertsRestPose_);

		vertsWeightsMat = j["Weights"].get<std::vector<std::vector<double>>>();
		copyToEigenFixedSizeMat<double, T, nJoints, nVerts>(vertsWeightsMat, vertsWeights_);

		jointPosMat = j["JointPos"].get<std::vector<std::vector<double>>>();
		copyToEigenFixedSizeMat<double, T, 4, nJoints>(jointPosMat, jointPos_);

		kintree_table = j["KintreeTable"].get<std::array<std::array<int, nJoints>, 2>>();

		std::map<std::string, size_t> parentsRaw = j["Parents"].get<std::map<std::string, size_t>>();
		parents.clear();
		for (auto & p : parentsRaw)
		{
			parents.insert(std::pair<size_t, size_t>(std::atoi(p.first.c_str()), p.second));
		}
		
		faces = j["Faces"].get<std::vector<std::vector<int>>>();
	}


};

template <size_t nVerts, size_t nJoints, typename T>
struct SkeletonModel {
public:
	SkeletonModel() {};

	SkeletonModel(SkeletonStaticData<nVerts, nJoints, T>* skelStaticData) {
		setVertsRestPose(&(skelStaticData->vertsRestPose_));
		setJointPos(&(skelStaticData->jointPos_));
		setVertsWeights(&(skelStaticData->vertsWeights_));
		setJointAngles(&(skelStaticData->jointAngles_));
		setKintree_table(&(skelStaticData->kintree_table));
		setParents(&(skelStaticData->parents));
		setGlobalTranslation(&(skelStaticData->gTranslation));
	}

	SkeletonModel(Eigen::Matrix<T, 4, nVerts> *vertsRestPose, Eigen::Matrix<T, 4, nJoints>* jointPos, std::array<std::array<int, nJoints>, 2>* kintree_table,
	std::map<size_t, size_t>* parents, Eigen::Matrix<T, nJoints, nVerts>* vertsWeights, const T* const jointAngles, const T* const gTranslation) 
		: jointAngles_(jointAngles), globalTranslation_(gTranslation)
	{
		setVertsRestPose(vertsRestPose);
		setJointPos(jointPos);
		setVertsWeights(vertsWeights);
		setKintree_table(kintree_table);
		setParents(parents);
		//setJointAngles(jointAngles);
		//setGlobalTranslation(gTranslation);
	}

	void setVertsRestPose(Eigen::Matrix<T, 4, nVerts> *vertsRestPose) {
		vertsRestPose_ = vertsRestPose;
	}
	void setJointPos(Eigen::Matrix<T, 4, nJoints> *jointPos) {
		jointPos_ = jointPos;
	}
	void setVertsWeights(Eigen::Matrix<T, nJoints, nVerts> *vertsWeights) {
		vertsWeights_ = vertsWeights;
	}
	void setParents(std::map<size_t, size_t>* parents) {
		parents_ = parents;
	}
	void setKintree_table(std::array<std::array<int, nJoints>, 2> * kintree_table) {
		kintree_table_ = kintree_table;
	}
	/*void setJointAngles(const T* const jointAngles) {
		jointAngles_ = jointAngles;
	}
	void setGlobalTranslation(const T* const globalTranslation) {
		globalTranslation_ = globalTranslation;
	}*/

	void setFaces(std::vector<std::vector<int>>* newFaces) {
		faces = newFaces;
	}

	static void RtToAffine(const T* const R, const T* t, Eigen::Matrix<T, 4, 4> & Tran) {
		Tran = Eigen::Matrix<T, 4, 4>::Identity();
		T Re[9];

#ifdef SKEL_ROT_EULER_ANGLE
		ceres::EulerAnglesToRotationMatrix<T>(R, 0, Re);
		Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::RowMajor>> Rm(Re);
#elif defined SKEL_ROT_AXIS_ANGLE
		ceres::AngleAxisToRotationMatrix<T>(R, Re);
		Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::ColMajor>> Rm(Re);
#endif

		Eigen::Map<const Eigen::Matrix<T, 3, 1>> tm(t);

		Tran.block<3, 3>(0, 0) = Rm;
		Tran.block<3, 1>(0, 3) = tm;

		//Eigen::Map<const Eigen::Matrix<T, 3, 3>> Rmcm(Re);
		//std::cout << "Rm row major:\n" << Rm << std::endl;
		//std::cout << "Rm col major:\n" << Rmcm << std::endl;
		//std::cout << "T col major:\n" << Tran << std::endl;
		//getchar();
	}

	static void RtToAffine(const T* R, const Eigen::Matrix<T, 3, 1> t, Eigen::Matrix<T, 4, 4> & Tran) {
		Tran = Eigen::Matrix<T, 4, 4>::Identity();
		T Re[9];

#ifdef SKEL_ROT_EULER_ANGLE
		ceres::EulerAnglesToRotationMatrix<T>(R, 0, Re);
		Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::RowMajor>> Rm(Re);
		//std::cout << "Use Euler Angle.\n";
#elif defined SKEL_ROT_AXIS_ANGLE
		ceres::AngleAxisToRotationMatrix<T>(R, Re);
		Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::ColMajor>> Rm(Re);
		//std::cout << "Use Axis Angle.\n";
#endif


		Tran.block<3, 3>(0, 0) = Rm;
		Tran.block<3, 1>(0, 3) = t;

		//Eigen::Map<const Eigen::Matrix<T, 3, 3>> Rmcm(Re);
		//std::cout << "Rm row major:\n" << Rm << std::endl;
		//std::cout << "Rm col major:\n" << Rmcm << std::endl;
		//std::cout << "T col major:\n" << Tran << std::endl;
		//getchar();
	}

	void update(bool log = false) {
		//std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>  trans(nJoints);
		
		// Transformation for root node
		RtToAffine(jointAngles_, (*jointPos_).block<3,1>(0,0), trans[0]);
		
		// Update joint world transformation
		for (size_t i = 1; i < nJoints; i++)
		{
			const T* pRj = jointAngles_ + JOINT_ROT_DIMENSION * i;
			int parentId = (*parents_)[i];

			Eigen::Matrix<T, 4, 4> jointTrans;

			Eigen::Matrix<T, 3, 1> jointTranslation = (*jointPos_).block<3, 1>(0, i) - (*jointPos_).block<3, 1>(0, parentId);
			/*std::cout << "(*jointPos_).block<3, 1>(0, i): \n" << (*jointPos_).block<3, 1>(0, i) << std::endl;
			std::cout << "(*jointPos_).block<3, 1>(0, iparentId): \n" << (*jointPos_).block<3, 1>(0, parentId) << std::endl;*/
			RtToAffine(pRj, jointTranslation, jointTrans);
			trans[i] = trans[parentId] * jointTrans;
		}

		// Update point coordinates
		// Try non-tensor version
		for (size_t i = 0; i < nJoints; i++)
		{
			// Apply the transformation that transforms the world coordinate system to joint's local system
			Eigen::Matrix<T, 4, 1> jointNo1 = (*jointPos_).block<4, 1>(0, i);
			jointNo1(3, 0) = T(0);
			Eigen::Matrix<T, 4, 1> inverseJ = trans[i] * jointNo1;
			trans[i].block<3, 1>(0, 3) = trans[i].block<3, 1>(0, 3) - inverseJ.block<3, 1>(0, 0);
		}

		Eigen::Map<const Eigen::Matrix<T, 3, 1>> globalTranslationMat(globalTranslation_);
		for (size_t iV = 0; iV < nVerts; iV++)
		{
			Eigen::Matrix<T, 4, 4> vertTrans = Eigen::Matrix<T, 4, 4>::Zero();
			//Speed up to only loop for 4 times
			for (size_t iJ = 0; iJ < nJoints; iJ++)
			{
				vertTrans = vertTrans + (*vertsWeights_)(iJ, iV) * trans[iJ];
				//std::cout << "weights for joint: " << (*vertsWeights_)(iJ, iV) << std::endl;
				//std::cout << "trans for joint: " << iJ << "\n" << trans[iJ] << std::endl;

			}
			/*std::cout << "vertTrans for v: " << iV << "\n" << vertTrans << std::endl;
			getchar();*/
			Eigen::Matrix<T, 4, 1> vt = vertTrans * (*vertsRestPose_).block<4, 1>(0, iV);
			vertsTransformed_.block<4, 1>(0, iV) = vt;
			vertsTransformed_.block<3, 1>(0, iV) = vertsTransformed_.block<3, 1>(0, iV) + globalTranslationMat;
		}
		if (log) std::cout << "GlobalTranslationMat:\n" << globalTranslationMat << std::endl;
	}

	void saveAsObj(const std::string filePath) {
		std::ofstream ofs(filePath);
		nlohmann::json j;
		for (size_t i = 0; i < nVerts; i++)
		{
			ofs << "v " << " " << vertsTransformed_(0, i) << " " << vertsTransformed_(1, i) << " " << vertsTransformed_(2, i) << "\n";
		}
		if (faces == NULL) return;
		for (size_t i = 0; i < faces->size(); i++)
		{
			ofs << "f ";
			for (size_t j = 0; j < (*faces)[i].size(); j++)
			{
				ofs << " " << (*faces)[i][j] + 1;
			}
			ofs << "\n";
		}
		ofs.close();
	}

	void saveAsObjWithColor(const std::string filePath, std::vector<double> errs, double maxErr = 10) {
		std::ofstream ofs(filePath);
		nlohmann::json j;
		for (size_t i = 0; i < nVerts; i++)
		{
			const tinycolormap::Color color = tinycolormap::GetColor(errs[i] / maxErr, tinycolormap::ColormapType::Jet);

			ofs << "v " << " " << vertsTransformed_(0, i) << " " << vertsTransformed_(1, i) << " " << vertsTransformed_(2, i) << " "
				<< color.r() << " " << color.g() << " " << color.b() << "\n";
		}
		if (faces == NULL) return;
		for (size_t i = 0; i < faces->size(); i++)
		{
			ofs << "f ";
			for (size_t j = 0; j < (*faces)[i].size(); j++)
			{
				ofs << " " << (*faces)[i][j] + 1;
			}
			ofs << "\n";
		}
		ofs.close();
	}
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Static Parameters
	Eigen::Matrix<T, 4, nVerts> *vertsRestPose_ = NULL;
	Eigen::Matrix<T, 4, nJoints> *jointPos_ = NULL;
	Eigen::Matrix<T, nJoints, nVerts> *vertsWeights_ = NULL;
	std::array<std::array<int, nJoints>, 2> *kintree_table_;
	std::map<size_t, size_t> *parents_;

	// Deformatiom Parameters
	const T * const jointAngles_ = NULL;
	const T * const globalTranslation_ = NULL;
	// Deformation Dependencies
	Eigen::Matrix<T, 4, 4> trans[nJoints];

	// Deformation results
	Eigen::Matrix<T, 4, nVerts> vertsTransformed_;
	// Faces
	std::vector<std::vector<int>> *faces = NULL;

	
};

#endif // _SKELETON_MODEL_H_
