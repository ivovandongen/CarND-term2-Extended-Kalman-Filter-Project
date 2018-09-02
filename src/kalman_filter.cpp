#include <kalman_filter.hpp>
#include <coordinates.hpp>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter(int noise_ax_, int noise_ay_)
        : noise_ax(noise_ax_), noise_ay(noise_ay_), F_(4, 4), Q_(4, 4), P_(4, 4) {
    // state covariance matrix P
    P_.row(0) << 1, 0, 0, 0;
    P_.row(1) << 0, 1, 0, 0;
    P_.row(2) << 0, 0, 1000, 0;
    P_.row(3) << 0, 0, 0, 1000;
};

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::init(VectorXd x) {
    x_ = std::move(x);
}

void KalmanFilter::predict(double delta_time) {
    // Modify the F matrix so that the time is integrated
    F_.row(0) << 1, 0, delta_time, 0;
    F_.row(1) << 0, 1, 0, delta_time;
    F_.row(2) << 0, 0, 1, 0;
    F_.row(3) << 0, 0, 0, 1;

    // 2. Set the process covariance matrix Q
    Q_.row(0) << pow(delta_time, 4) / 4 * noise_ax, 0, pow(delta_time, 3) / 2 * noise_ax, 0;
    Q_.row(1) << 0, pow(delta_time, 4) / 4 * noise_ay, 0, pow(delta_time, 3) / 2 * noise_ay;
    Q_.row(2) << pow(delta_time, 3) / 2 * noise_ax, 0, pow(delta_time, 2) * noise_ax, 0;
    Q_.row(3) << 0, pow(delta_time, 3) / 2 * noise_ay, 0, pow(delta_time, 2) * noise_ay;

    // Predict the new state
    x_ = F_ * x_;
    Eigen::MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::update(const VectorXd &z, const MatrixXd &H, const MatrixXd &R) {
    MatrixXd z_pred = H * x_;
    MatrixXd y = z - z_pred;
    MatrixXd Ht = H.transpose();
    MatrixXd S = H * P_ * Ht + R;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    // new estimate
    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K * H) * P_;
}

void KalmanFilter::updateEKF(const VectorXd &z, const Eigen::MatrixXd &Hj, const Eigen::MatrixXd &R) {

    // Convert coordinates
    auto polar = PolarCoordinate::fromCartesian({x_(0), x_(1), x_(2), x_(3)});
    VectorXd hx = VectorXd(3);
    hx << polar.rho, polar.phi, polar.rho_dot;

    // Subtract and normalise phi
    VectorXd y = z - hx;
    PolarCoordinate::normalisePhi(y(1));

    // EKF update
    MatrixXd Ht = Hj.transpose();
    MatrixXd S = Hj * P_ * Ht + R;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

    x_ = x_ + K * y;
    P_ = (I - K * Hj) * P_;
}
