#include <FusionEKF.hpp>

#include <Eigen/Dense>
#include <iostream>
#include <tools.hpp>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
    : R_laser_(MatrixXd(2, 2)), R_radar_(MatrixXd(3, 3)), H_laser_(MatrixXd(2, 4)), Hj_(MatrixXd(3, 4)), ekf_(9, 9) {
    // measurement covariance matrix - laser
    R_laser_ << 0.0225, 0, 0, 0.0225;

    // measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

    // measurement matrix
    H_laser_.row(0) << 1, 0, 0, 0;
    H_laser_.row(1) << 0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() = default;

void FusionEKF::initialize(const MeasurementPackage &measurement_pack) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */
        std::cout << "Radar not supported yet" << std::endl;
        return;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        /**
        Initialize state.
        */
        VectorXd x(4);
        x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
        ekf_.init(std::move(x));
    }

    //    ekf_.Init(x, P, F, H, R, Q);

    // done initializing, no need to predict or update
    is_initialized_ = true;
}

void FusionEKF::processMeasurement(const MeasurementPackage &measurement_pack) {
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        initialize(measurement_pack);
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
     TODO:
       * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
       * Update the process noise covariance matrix.
       * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */

    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.predict(dt);

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        // TODO UpdateEKF
    } else {
        // Laser updates
        ekf_.update(measurement_pack.raw_measurements_, H_laser_, R_laser_);
    }

    // print the output
    cout << "x_ = " << ekf_.getX() << endl;
    cout << "P_ = " << ekf_.getP() << endl;
}
