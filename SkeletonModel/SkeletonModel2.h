#ifndef _SKELETON_MODEL_2_H_
#define _SKELETON_MODEL_2_H_

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


#ifndef SKEL_ROT_AXIS_ANGLE 
	#ifndef SKEL_ROT_QUARTERNION
		#define SKEL_ROT_EULER_ANGLE
	#endif // !SKEL_ROT_QUARTERNION
#endif // !SKEL_ROT_EULER_ANGLE

#ifdef SKEL_ROT_QUARTERNION
#define JOINT_ROT_DIMENSION 4
#else
#define JOINT_ROT_DIMENSION 3
#endif // SKEL_ROT_QUARTERNION

#define JOINT_TRANSLATION_DIMENSION 3



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

	std::vector<std::vector<int>> activeBonesTable_;
	std::array<std::array<int, nJoints>, 2> kintree_table_;
	std::map<size_t, size_t> parents_;
	std::vector<std::vector<int>> faces_;

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

		//kintree_table_ = j["KintreeTable"].get<std::array<std::array<int, nJoints>, 2>>();

		std::map<std::string, size_t> parentsRaw = j["Parents"].get<std::map<std::string, size_t>>();
		parents_.clear();
		for (auto & p : parentsRaw)
		{
			parents_.insert(std::pair<size_t, size_t>(std::atoi(p.first.c_str()), p.second));
		}
		
		faces_ = j["Faces"].get<std::vector<std::vector<int>>>();

		try
		{
			// try to write at a nonexisting key
			activeBonesTable_ = j.at("ActiveBoneTable").get<std::vector<std::vector<int>>>();
		}
		catch (nlohmann::json::out_of_range& e)
		{
			std::cout << e.what() << '\n';
			std::cout << "activeBonesTable_ will be set to empty." << std::endl;
		}
	}

};

template <size_t nVerts, size_t nJoints, typename T>
struct SkeletonModel {
public:
	SkeletonModel() {};

	SkeletonModel(const SkeletonStaticData<nVerts, nJoints, T>* skelStaticData, T const * const* jointAnglesStructured, const T* gTranslation, Eigen::Matrix<T, 4, nVerts>* vertsTransformed)
		: jointAnglesStructured_(jointAnglesStructured), globalTranslation_(gTranslation), vertsTransformed_(vertsTransformed)
	{
		setVertsRestPose(&(skelStaticData->vertsRestPose_));
		setJointPos(&(skelStaticData->jointPos_));
		setVertsWeights(&(skelStaticData->vertsWeights_));
		setKintree_table(&(skelStaticData->kintree_table_));
		setParents(&(skelStaticData->parents_));
		activeBonesTable_ = &(skelStaticData->activeBonesTable_);
		setFaces(&(skelStaticData->faces_));
	}

	SkeletonModel(const SkeletonStaticData<nVerts, nJoints, T> *skelStaticData, const T* jointAngles, const T* gTranslation, Eigen::Matrix<T, 4, nVerts>* vertsTransformed)
		: jointAngles_(jointAngles), globalTranslation_(gTranslation), vertsTransformed_(vertsTransformed)
	{
		setVertsRestPose(&(skelStaticData->vertsRestPose_));
		setJointPos(&(skelStaticData->jointPos_));
		setVertsWeights(&(skelStaticData->vertsWeights_));
		setKintree_table(&(skelStaticData->kintree_table_));
		setParents(&(skelStaticData->parents_));
		activeBonesTable_ = &(skelStaticData->activeBonesTable_);
		setFaces(&(skelStaticData->faces_));

	}

	SkeletonModel(const Eigen::Matrix<T, 4, nVerts>* vertsRestPose, const Eigen::Matrix<T, 4, nJoints>* jointPos, const std::array<std::array<int, nJoints>, 2> * kintree_table,
		const std::map<size_t, size_t>* parents, const Eigen::Matrix<T, nJoints, nVerts>* vertsWeights, const std::vector<std::vector<int>>* activeBonesTable,
		T const* const* jointAnglesStructured, const T* gTranslation, Eigen::Matrix<T, 4, nVerts>* vertsTransformed)
		: jointAnglesStructured_(jointAnglesStructured), globalTranslation_(gTranslation), vertsTransformed_(vertsTransformed), activeBonesTable_(activeBonesTable)
	{
		setVertsRestPose(vertsRestPose);
		setJointPos(jointPos);
		setVertsWeights(vertsWeights);
		setKintree_table(kintree_table);
		setParents(parents);
		//setJointAngles(jointAngles);
		//setGlobalTranslation(gTranslation);
	}

	SkeletonModel(const Eigen::Matrix<T, 4, nVerts> *vertsRestPose, const Eigen::Matrix<T, 4, nJoints>* jointPos, const std::array<std::array<int, nJoints>, 2>* kintree_table,
		const std::map<size_t, size_t>* parents, const Eigen::Matrix<T, nJoints, nVerts>* vertsWeights, const std::vector<std::vector<int>> * activeBonesTable,
		const T*  jointAngles, const T* gTranslation, Eigen::Matrix<T, 4, nVerts> * vertsTransformed)
		: jointAngles_(jointAngles), globalTranslation_(gTranslation), vertsTransformed_(vertsTransformed), activeBonesTable_(activeBonesTable)
	{
		setVertsRestPose(vertsRestPose);
		setJointPos(jointPos);
		setVertsWeights(vertsWeights);
		setKintree_table(kintree_table);
		setParents(parents);
		//setJointAngles(jointAngles);
		//setGlobalTranslation(gTranslation);
	}

	void setVertsRestPose(const Eigen::Matrix<T, 4, nVerts> *vertsRestPose) {
		vertsRestPose_ = vertsRestPose;
	}
	void setJointPos(const Eigen::Matrix<T, 4, nJoints> *jointPos) {
		jointPos_ = jointPos;
	}
	void setVertsWeights(const Eigen::Matrix<T, nJoints, nVerts> *vertsWeights) {
		vertsWeights_ = vertsWeights;
	}
	void setParents(const std::map<size_t, size_t>* parents) {
		parents_ = parents;
	}
	void setKintree_table(const std::array<std::array<int, nJoints>, 2> *kintree_table) {
		kintree_table_ = kintree_table;
	}
	void setJointAngles(const T* jointAngles) {
		jointAngles_ = jointAngles;
	}
	void setGlobalTranslation(const T* globalTranslation) {
		globalTranslation_ = globalTranslation;
	}

	void setOutputMat(Eigen::Matrix<T, 4, nVerts>* vertsTransformed) {
		vertsTransformed_ = vertsTransformed;
	}

	void setFaces(const std::vector<std::vector<int>>* const newFaces) {
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
#elif defined SKEL_ROT_QUARTERNION
		ceres::QuaternionToRotation(R, Re);
		Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::RowMajor>> Rm(Re);
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
#elif defined SKEL_ROT_AXIS_ANGLE
		ceres::AngleAxisToRotationMatrix<T>(R, Re);
		Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::ColMajor>> Rm(Re);
#elif defined SKEL_ROT_QUARTERNION
		ceres::QuaternionToScaledRotation(R, Re);
		Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::RowMajor>> Rm(Re);
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
		if (jointAngles_ == NULL || globalTranslation_ == NULL)
		{
			std::cout << "Invalid jointAngles_ or globalTranslation_!" << std::endl;
			return;
		}
		
		// Transformation for root node
		RtToAffine(jointAngles_, (*jointPos_).block<3,1>(0,0), trans[0]);
		
		// Update joint world transformation
		for (size_t i = 1; i < nJoints; i++)
		{
			const T* pRj = jointAngles_ + JOINT_ROT_DIMENSION * i;
			int parentId = (*parents_).at(i);

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
			//for (size_t iJ = 0; iJ < nJoints; iJ++)
			for (size_t iJ = 0; iJ < (*activeBonesTable_)[iV].size(); iJ++)
			{
				size_t jointId = (*activeBonesTable_)[iV][iJ];
				vertTrans = vertTrans + (*vertsWeights_)(jointId, iV) * trans[jointId];
				//std::cout << "weights for joint: " << (*vertsWeights_)(iJ, iV) << std::endl;
				//std::cout << "trans for joint: " << iJ << "\n" << trans[iJ] << std::endl;

			}
			/*std::cout << "vertTrans for v: " << iV << "\n" << vertTrans << std::endl;
			getchar();*/
			Eigen::Matrix<T, 4, 1> vt = vertTrans * (*vertsRestPose_).block<4, 1>(0, iV);
			vertsTransformed_->block<4, 1>(0, iV) = vt;
			vertsTransformed_->block<3, 1>(0, iV) = vertsTransformed_->block<3, 1>(0, iV) + globalTranslationMat;
		}
		if (log) std::cout << "GlobalTranslationMat:\n" << globalTranslationMat << std::endl;
	}

	void updateJointPoseWithStructuredParam() {
		//std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>  trans(nJoints);
		if (jointAnglesStructured_ == NULL)
		{
			std::cout << "Invalid jointAnglesStructured_ or globalTranslation_!" << std::endl;
			return;
		}

		// Transformation for root node
		const T* const firstRotParam = *jointAnglesStructured_;
		RtToAffine(firstRotParam, (*jointPos_).block<3, 1>(0, 0), trans[0]);

		// Update joint world transformation
		for (size_t i = 1; i < nJoints; i++)
		{
			const T* pRj = jointAnglesStructured_[i];
			int parentId = (*parents_).at(i);

			Eigen::Matrix<T, 4, 4> jointTrans;

			Eigen::Matrix<T, 3, 1> jointTranslation = (*jointPos_).block<3, 1>(0, i) - (*jointPos_).block<3, 1>(0, parentId);
			/*std::cout << "(*jointPos_).block<3, 1>(0, i): \n" << (*jointPos_).block<3, 1>(0, i) << std::endl;
			std::cout << "(*jointPos_).block<3, 1>(0, iparentId): \n" << (*jointPos_).block<3, 1>(0, parentId) << std::endl;*/
			RtToAffine(pRj, jointTranslation, jointTrans);
			trans[i] = trans[parentId] * jointTrans;
		}

		// Try non-tensor version
		for (size_t i = 0; i < nJoints; i++)
		{
			// Apply the transformation that transforms the world coordinate system to joint's local system
			Eigen::Matrix<T, 4, 1> jointNo1 = (*jointPos_).block<4, 1>(0, i);
			jointNo1(3, 0) = T(0);
			Eigen::Matrix<T, 4, 1> inverseJ = trans[i] * jointNo1;
			trans[i].block<3, 1>(0, 3) = trans[i].block<3, 1>(0, 3) - inverseJ.block<3, 1>(0, 0);
		}
	}

	void getVertTransformation(size_t iV, Eigen::Matrix<T, 4, 4> & vT) {
		//Speed up to only loop for 4 times
		//for (size_t iJ = 0; iJ < nJoints; iJ++)
		vT = Eigen::Matrix<T, 4, 4>::Zero();
		//double sumOfWeights = 0;
		for (size_t iJ = 0; iJ < (*activeBonesTable_)[iV].size(); iJ++)
		{
			size_t jointId = (*activeBonesTable_)[iV][iJ];
			vT = vT + (*vertsWeights_)(jointId, iV) * trans[jointId];
			//std::cout << "weights for joint: " << (*vertsWeights_)(jointId, iV) << std::endl;
			//std::cout << "trans for joint: " << iJ << "\n" << trans[iJ] << std::endl;
			//sumOfWeights += (*vertsWeights_)(jointId, iV);
		}
		//std::cout << "Sum of all weights: " << sumOfWeights << std::endl;
		Eigen::Map<const Eigen::Matrix<T, 3, 1>> globalTranslationMat(globalTranslation_);
		vT.block<3, 1>(0, 3) = vT.block<3, 1>(0, 3) + globalTranslationMat;
		/*std::cout << "vertTrans for v: " << iV << "\n" << vertTrans << std::endl;
		getchar();*/
		/*Eigen::Matrix<T, 4, 1> vt = vT * (*vertsRestPose_).block<4, 1>(0, iV);
		vertsTransformed_->block<4, 1>(0, iV) = vt;
		vertsTransformed_->block<3, 1>(0, iV) = vertsTransformed_->block<3, 1>(0, iV) + globalTranslationMat;*/
	}

	void posForOnePointWithStructuredParam(size_t vid, T * vData) {
		//std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>  trans(nJoints);
		if (jointAnglesStructured_ == NULL || globalTranslation_ == NULL)
		{
			std::cout << "Invalid jointAnglesStructured_ or globalTranslation_!" << std::endl;
			return;
		}

		updateJointPoseWithStructuredParam();

		Eigen::Map<const Eigen::Matrix<T, 3, 1>> globalTranslationMat(globalTranslation_);
		Eigen::Matrix<T, 4, 4> vertTrans = Eigen::Matrix<T, 4, 4>::Zero();
		for (size_t iJ = 0; iJ < (*activeBonesTable_)[vid].size(); iJ++)
		{
			size_t jointId = (*activeBonesTable_)[vid][iJ];
			vertTrans = vertTrans + (*vertsWeights_)(jointId, vid) * trans[jointId];
			//std::cout << "weights for joint: " << (*vertsWeights_)(iJ, iV) << std::endl;
			//std::cout << "trans for joint: " << iJ << "\n" << trans[iJ] << std::endl;

		}
		/*std::cout << "vertTrans for v: " << iV << "\n" << vertTrans << std::endl;
		getchar();*/
		Eigen::Matrix<T, 4, 1> vt = vertTrans * (*vertsRestPose_).block<4, 1>(0, vid);
		
		vData[0] = vt(0);
		vData[1] = vt(1);
		vData[2] = vt(2);
	}

	void transformVerts() {
		Eigen::Map<const Eigen::Matrix<T, 3, 1>> globalTranslationMat(globalTranslation_);
		for (size_t iV = 0; iV < nVerts; iV++)
		{
			Eigen::Matrix<T, 4, 4> vertTrans = Eigen::Matrix<T, 4, 4>::Zero();
			//Speed up to only loop for 4 times
			//for (size_t iJ = 0; iJ < nJoints; iJ++)
			for (size_t iJ = 0; iJ < (*activeBonesTable_)[iV].size(); iJ++)
			{
				size_t jointId = (*activeBonesTable_)[iV][iJ];
				vertTrans = vertTrans + (*vertsWeights_)(jointId, iV) * trans[jointId];
				//std::cout << "weights for joint: " << (*vertsWeights_)(iJ, iV) << std::endl;
				//std::cout << "trans for joint: " << iJ << "\n" << trans[iJ] << std::endl;

			}
			/*std::cout << "vertTrans for v: " << iV << "\n" << vertTrans << std::endl;
			getchar();*/
			Eigen::Matrix<T, 4, 1> vt = vertTrans * (*vertsRestPose_).block<4, 1>(0, iV);
			vertsTransformed_->block<4, 1>(0, iV) = vt;
			vertsTransformed_->block<3, 1>(0, iV) = vertsTransformed_->block<3, 1>(0, iV) + globalTranslationMat;
		}
	}

	void updateWithStructuredParam(bool log = false) {
		//std::vector<Eigen::Matrix<T, 4, 4>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 4>>>  trans(nJoints);
		if (jointAnglesStructured_ == NULL || globalTranslation_ == NULL)
		{
			std::cout << "Invalid jointAnglesStructured_ or globalTranslation_!" << std::endl;
			return;
		}

		// Transformation for root node
		const T * const firstRotParam = *jointAnglesStructured_;
		RtToAffine(firstRotParam, (*jointPos_).block<3,1>(0,0), trans[0]);
		
		// Update joint world transformation
		for (size_t i = 1; i < nJoints; i++)
		{
			const T* pRj = jointAnglesStructured_[i];
			int parentId = (*parents_).at(i);

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
			//for (size_t iJ = 0; iJ < nJoints; iJ++)
			for (size_t iJ = 0; iJ < (*activeBonesTable_)[iV].size(); iJ++)
			{
				size_t jointId = (*activeBonesTable_)[iV][iJ];
				vertTrans = vertTrans + (*vertsWeights_)(jointId, iV) * trans[jointId];
				//std::cout << "weights for joint: " << (*vertsWeights_)(iJ, iV) << std::endl;
				//std::cout << "trans for joint: " << iJ << "\n" << trans[iJ] << std::endl;

			}
			/*std::cout << "vertTrans for v: " << iV << "\n" << vertTrans << std::endl;
			getchar();*/
			Eigen::Matrix<T, 4, 1> vt = vertTrans * (*vertsRestPose_).block<4, 1>(0, iV);
			vertsTransformed_->block<4, 1>(0, iV) = vt;
			vertsTransformed_->block<3, 1>(0, iV) = vertsTransformed_->block<3, 1>(0, iV) + globalTranslationMat;
		}
		if (log) std::cout << "GlobalTranslationMat:\n" << globalTranslationMat << std::endl;
	}

	void saveAsObj(const std::string filePath) {
		std::ofstream ofs(filePath);
		nlohmann::json j;
		for (size_t i = 0; i < nVerts; i++)
		{
			ofs << "v " << " " << (*vertsTransformed_)(0, i) << " " << (*vertsTransformed_)(1, i) << " " << (*vertsTransformed_)(2, i) << "\n";
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

			ofs << "v " << " " << (*vertsTransformed_)(0, i) << " " << (*vertsTransformed_)(1, i) << " " << (*vertsTransformed_)(2, i) << " "
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
	const Eigen::Matrix<T, 4, nVerts> * vertsRestPose_ = NULL;
	const Eigen::Matrix<T, 4, nJoints> * jointPos_ = NULL;
	const Eigen::Matrix<T, nJoints, nVerts> * vertsWeights_ = NULL;
	const std::array<std::array<int, nJoints>, 2> * kintree_table_;
	const std::map<size_t, size_t> * parents_;
	const std::vector<std::vector<int>> * activeBonesTable_;

	// Deformatiom Parameters
	const T * jointAngles_ = NULL;
	T const* const* jointAnglesStructured_ = NULL;
	const T * globalTranslation_ = NULL;
	// Deformation Dependencies
	Eigen::Matrix<T, 4, 4> trans[nJoints];

	// Deformation results
	Eigen::Matrix<T, 4, nVerts> * vertsTransformed_ = NULL;
	// Faces
	const std::vector<std::vector<int>> *faces = NULL;

};

#endif // _SKELETON_MODEL_H_
