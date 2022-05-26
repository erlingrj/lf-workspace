#include <Eigen/Dense>

#pragma once

class KalmanFilterCpp {
public:
	KalmanFilterCpp(int n, int m);
	/**
	 * A = system matrix nxn x_k+1 = A*x_k
	 * C = Measuremennt matrix: y = Cx_k
	 * Q = Process noise covariance. Gets added to the covariance at each predictionS
	 * R = Measuremnt noise covariance
	 * P = Covariance matrix nxn. Shows the uncertainty in the measurements
	 */

	void init(
						double dt,
						Eigen::MatrixXd A,
						Eigen::MatrixXd C,
						Eigen::MatrixXd Q,
						Eigen::MatrixXd R,
						Eigen::MatrixXd P);
	void update(Eigen::VectorXd y);

	Eigen::VectorXd get_state() { return x_hat;}
	Eigen::MatrixXd get_covariance() { return P;}
	double get_time() {return t;}


private:
	Eigen::MatrixXd A,C,Q,R,P,L,P0, K;
	int m,n;
	double t;
	double dt;

	bool initialized;

	Eigen::MatrixXd I;
	Eigen::VectorXd x_hat, x_hat_new;

};

typedef struct kf_states_s {
            Eigen::VectorXd x_hat;
            Eigen::MatrixXd P;
        } kf_states_t;

