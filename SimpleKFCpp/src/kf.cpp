#include "kf.hpp"

#include <Eigen/Dense>

#include <stdexcept>
#include <iostream>


KalmanFilterCpp::KalmanFilterCpp(int n, int m):
			  A(n,n), C(m,n), Q(n,n), R(m,m), P0(n,n), P(n,n), K(n,n),
			  initialized(false), I(n,n), x_hat(n), x_hat_new(n)
			  {}


void KalmanFilterCpp::init(double dt,
		Eigen::MatrixXd A,
		Eigen::MatrixXd C,
		Eigen::MatrixXd Q,
		Eigen::MatrixXd R,
		Eigen::MatrixXd P) {
	this->dt = dt;
	this->A = A;
	this->C = C;
	this->Q = Q;
	this->R = R;
	this->P = P;
	this->P0 = P;

	I.setIdentity();
	x_hat.setZero();
	t = 0;
	initialized = true;
}


void KalmanFilterCpp::update(Eigen::VectorXd y) {
	if(!initialized) {
		throw std::runtime_error("Filter is not initialized!");
	}


	x_hat_new = A * x_hat;
	P = A*P*A.transpose() + Q;
	K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
	x_hat_new += K * (y - C*x_hat_new);
	P = (I - K*C)*P;
	x_hat = x_hat_new;
}
