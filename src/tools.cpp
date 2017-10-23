#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations,
							  const vector<VectorXd>& ground_truth) {
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
		cout << "ERROR: invalid data to calculate RMSE" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {
		rmse += (estimations[i] - ground_truth[i]).array().pow(2).matrix();
	}

	rmse /= (const double)estimations.size();
	rmse = rmse.array().sqrt();
	return rmse;
}

double Tools::ConstrainRadian(double x) {
	if (x > M_PI)
		return fmod(x - M_PI, 2 * M_PI) - M_PI;
	else if (x < -M_PI)
		return fmod(x + M_PI, 2 * M_PI) + M_PI;
	return x;
}
