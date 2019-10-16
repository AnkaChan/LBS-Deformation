#define EIGEN_STACK_ALLOCATION_LIMIT 100000000000

#define SKEL_ROT_AXIS_ANGLE
#include "../SkeletonModel/SkeletonModel.h"
#include "../SkeletonModel/SMPLModel.h"
#include <iostream>



int main(int argc, char** argv) {
	SkeletonStaticData<6890, 24, double>* skelData = new SkeletonStaticData<6890, 24, double>;

	skelData->readJsonSkelData("SkelData.json");

	std::cout << skelData->vertsRestPose_.block<3, 10>(0, 0) << std::endl;
	std::cout << skelData->jointPos_.block<3, 24>(0, 0) << std::endl;
	std::cout << skelData->vertsWeights_.block<3, 10>(0, 0) << std::endl;

	double jointAngles[72] = {
		 0.06657933,  0.03576395, -0.07727001,
		-0.00041919,  0.0310668 , -0.03600428,
		 0.01327558,  0.00542743,  0.00021457,
		-0.00873001,  0.02165131,  0.06015187,
		-0.04825328,  0.0514137 ,  0.01143151,
		 0.02225688, -0.05683011,  0.00675684,
		 0.07422685, -0.05399024, -0.09888641,
		-0.08716861,  0.01330351,  0.11924837,
		 0.05618456,  0.08363111,  0.00495746,
		 0.06989982, -0.0135624 ,  0.03066021,
		-0.01336586, -0.02746545,  0.00663541,
		-0.0238071 ,  0.06542365,  0.00975066,
		 0.0200105 , -0.01688162,  0.06282361,
		-0.03659848,  0.03301158, -0.01754359,
		-0.04697167, -0.02446686, -0.04022956,
		-0.01063488, -0.01695701,  0.0156085 ,
		 0.02825763, -0.00737101, -0.00129527,
		 0.01445471, -0.02699395,  0.035408  ,
		 0.04211124,  0.01017904,  0.11973518,
		 0.04587295, -0.00561362, -0.01810902,
		-0.01160911, -0.02508644,  0.05643926,
		-0.0348905 , -0.00405611, -0.0264648 ,
		 0.05230914, -0.0709278 , -0.01812496,
		-0.00609528,  0.01596782,  0.02304515
	};
	for (size_t i = 0; i < 72; i++)
	{
		jointAngles[i] = 10 * jointAngles[i];
	}

	double t[3] = { 0,0,1 };

	SkeletonModel<6890, 24, double> skelModel(&skelData->vertsRestPose_, &skelData->jointPos_, &skelData->kintree_table, &skelData->parents, &skelData->vertsWeights_, jointAngles, t);
	skelModel.update();

	std::ofstream ofs("output.obj");
	nlohmann::json j;
	for (size_t i = 0; i < 6890; i++)
	{
		ofs << "v " << " " << skelModel.vertsTransformed_(0, i) << " " << skelModel.vertsTransformed_(1, i) << " " << skelModel.vertsTransformed_(2, i) << "\n";
	}



	return 0;
}