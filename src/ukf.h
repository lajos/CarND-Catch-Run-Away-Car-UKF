#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:
	///* if this is false, laser measurements will be ignored (except for init)
	bool use_laser_;

	///* if this is false, radar measurements will be ignored (except for init)
	bool use_radar_;

	///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
	VectorXd x_;

	///* state covariance matrix
	MatrixXd P_;

	// process noise covariance matrix
	MatrixXd Q_;

	///* time when the state is true, in us
	long long time_us_;

	///* Process noise standard deviation longitudinal acceleration in m/s^2
	double std_a_;

	///* Process noise standard deviation yaw acceleration in rad/s^2
	double std_yawdd_;

	///* Laser measurement noise standard deviation position1 in m
	double std_laspx_;

	///* Laser measurement noise standard deviation position2 in m
	double std_laspy_;

	///* Radar measurement noise standard deviation radius in m
	double std_radr_;

	///* Radar measurement noise standard deviation angle in rad
	double std_radphi_;

	///* Radar measurement noise standard deviation radius change in m/s
	double std_radrd_ ;

	///* Weights of sigma points
	VectorXd weights_;

	///* State dimension
	int n_x_;

	///* Augmented state dimension
	int n_aug_;

	///* Sigma point spreading parameter
	double lambda_;

	// augmented state vector
	VectorXd x_aug_;

	// augmented state covariance
	MatrixXd P_aug_;

	// augmented sigma point matrix
	MatrixXd Xsig_aug_;

	// predicted sigma points
	MatrixXd Xsig_pred_;

	//radar measurement dimension (r, phi, r_dot)
	int n_z_radar_;

	// radar predicted mean
	VectorXd z_pred_radar_;

	// radar predicted covariance
	MatrixXd S_radar_;

	// radar measurement noise covariance matrix
	MatrixXd R_radar_;

	// sigma points in radar measurement space
	MatrixXd Zsig_radar_;

	// number of radar measurements
	long num_radar_measurement_;

	// number of radar measurements above 95%
	long num_radar_measurement_95_;

	// radar measurement 95% NIS target
	double e_radar_95_;

	// laser measurement dimension (px, py)
	int n_z_laser_;

	// laser predicted mean
	VectorXd z_pred_laser_;

	// laser predicted covariance
	MatrixXd S_laser_;

	// laser measurement noise covariance matrix
	MatrixXd R_laser_;

	// sigma points in laser measurement space
	MatrixXd Zsig_laser_;

	// number of laser measurements
	long num_laser_measurement_;

	// number of laser measurements above 95%
	long num_laser_measurement_95_;

	// laser measurement 95% NIS target
	double e_laser_95_;



	/**
	 * Constructor
	 */
	UKF();

	/**
	 * Destructor
	 */
	virtual ~UKF();

	/**
	 * ProcessMeasurement
	 * @param measurement_pack The latest measurement data of either radar or laser
	 */
	void ProcessMeasurement(const MeasurementPackage& measurement_pack);

	/**
	 * Prediction Predicts sigma points, the state, and the state covariance
	 * matrix
	 * @param delta_t Time between k and k+1 in s
	 */
	void Prediction(double delta_t);

	/**
	 * Updates the state and the state covariance matrix using a laser measurement
	 * @param measurement_pack The measurement at k+1
	 */
	void UpdateLidar(const MeasurementPackage& measurement_pack);

	/**
	 * Updates the state and the state covariance matrix using a radar measurement
	 * @param measurement_pack The measurement at k+1
	 */
	void UpdateRadar(const MeasurementPackage& measurement_pack);

private:
	///* initially set to false, set to true in first call of ProcessMeasurement
	bool is_initialized_;

	// previous timestamp
	long long previous_timestamp_;

	// generate augmented sigma points
	void GenerateAugmentedSigmaPoints();

	// predict sigma points
	void PredictSigmaPoints(const double& delta_t);

	// predict mean and covariance
	void PredictMeanAndCovariance();

	// predict radar measurement mean and covariance
	void PredictRadarMeasurement();

	// update radar state based on measurement z, return NIS
	double UpdateRadarState(const VectorXd& z);

	// predict laser measurement mean and covariance
	void PredictLaserMeasurement();

	// update laser state based on measurement z, return NIS
	double UpdateLaserState(const VectorXd& z);
};

#endif /* UKF_H */
