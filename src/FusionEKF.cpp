#include <FusionEKF.hpp>

#include <Eigen/Dense>
#include <tools.hpp>
#include <coordinates.hpp>

#include <algorithm>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
        : R_laser_(MatrixXd(2, 2)),
          R_radar_(MatrixXd(3, 3)),
          H_laser_(MatrixXd(2, 4)),
          Hj_(MatrixXd::Zero(3, 4)),
          ekf_(9, 9) {
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
    // First measurement
    cout << "Initialize" << endl;

    VectorXd x(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Convert radar from polar to cartesian coordinates and initialize state.
        cout << "Initial Measurement: Radar" << endl;

        double rho = measurement_pack.raw_measurements_[0];
        double phi = measurement_pack.raw_measurements_[1];
        double rho_dot = measurement_pack.raw_measurements_[2];
        auto point = CartesianCoordinate::fromPolar({rho, phi, rho_dot});

        x << point.x, point.y, point.vx, point.vy;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        // Initialize state.
        cout << "Initial Measurement: Laser." << endl;
        x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // Initialise EKF
    ekf_.init(std::move(x));
    std::cout << "x: " << ekf_.getX() << std::endl;

    // Timestamp of first measurement
    previous_timestamp_ = measurement_pack.timestamp_;

    // Done
    is_initialized_ = true;
}

void FusionEKF::processMeasurement(const MeasurementPackage &measurement_pack) {
    // Initialization
    if (!is_initialized_) {
        initialize(measurement_pack);
        return;
    }

    // Prediction
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.predict(dt);

    // Update
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        try {
            auto Hj = tools.CalculateJacobian(ekf_.getX());
            cout << "H = " << Hj << endl;
            ekf_.updateEKF(measurement_pack.raw_measurements_, Hj, R_radar_);
        } catch (...) {
            return;
        }
    } else {
        // Laser updates
        ekf_.update(measurement_pack.raw_measurements_, H_laser_, R_laser_);
    }
}
