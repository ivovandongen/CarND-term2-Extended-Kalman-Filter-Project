#pragma once

#include <Eigen/Dense>

class KalmanFilter {
public:
    /**
     * Constructor
     */
    KalmanFilter(int noise_ax, int noise_ay);

    /**
     * Destructor
     */
    virtual ~KalmanFilter();

    /**
     * Init Initializes Kalman filter
     *
     * @param x Initial state
     */
    void init(Eigen::VectorXd x);

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     *
     * @param delta_T Time between k and k+1 in s
     */
    void predict(double delta_time);

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     * @param H Measurement matrix
     * @param R Measurement covariance matrix
     */
    void update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     */
    void updateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &Hj, const Eigen::MatrixXd &R);

    const Eigen::VectorXd &getX() const { return x_; }

    const Eigen::MatrixXd &getP() const { return P_; }

private:
    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // Process noise
    int noise_ax;
    int noise_ay;
};
