#include <test.hpp>

#include <kalman_filter.hpp>
#include <measurement_package.hpp>

#include <vector>

TEST(KalmanFilter, Simple) {
    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    std::vector<MeasurementPackage> measurements = {
        {MeasurementPackage::SensorType::LASER, 1477010443000000, {4.632272e-01, 6.074152e-01}},
        {MeasurementPackage::SensorType::LASER, 1477010443100000, {9.685213e-01, 4.054501e-01}},
        {MeasurementPackage::SensorType::LASER, 1477010443200000, {9.477518e-01, 6.368242e-01}}};

    auto x = VectorXd(4);

    // state covariance matrix P
    auto P = MatrixXd(4, 4);
    P.row(0) << 1, 0, 0, 0;
    P.row(1) << 0, 1, 0, 0;
    P.row(2) << 0, 0, 1000, 0;
    P.row(3) << 0, 0, 0, 1000;

    // measurement covariance
    auto R = MatrixXd(2, 2);
    R.row(0) << 0.0225, 0;
    R.row(1) << 0, 0.0225;

    // measurement matrix
    auto H = MatrixXd(2, 4);
    H.row(0) << 1, 0, 0, 0;
    H.row(1) << 0, 1, 0, 0;

    // the initial transition matrix F
    auto F = MatrixXd(4, 4);
    F.row(0) << 1, 0, 1, 0;
    F.row(1) << 0, 1, 0, 1;
    F.row(2) << 0, 0, 1, 0;
    F.row(3) << 0, 0, 0, 1;

    // set the acceleration noise components
    int noise_ax = 5;
    int noise_ay = 5;

    // Process Covariance matrix
    auto Q = MatrixXd(4, 4);

    // Initialise Kalman Filter
    KalmanFilter kalmanFilter;
    kalmanFilter.Init(x, P, F, H, R, Q);

    // Set first measurement (initial location and zero velocity)
    kalmanFilter.x_ << measurements[0].raw_measurements_[0], measurements[0].raw_measurements_[1], 0, 0;
    MeasurementPackage::Timestamp previous_timestamp = measurements[0].timestamp_;

    auto processMeasurement = [&](MeasurementPackage &measurement_pack) {
        // compute the time elapsed between the current and previous measurements
        double dt = (measurement_pack.timestamp_ - previous_timestamp) / 1000000.0;
        previous_timestamp = measurement_pack.timestamp_;

        // 1. Modify the F matrix so that the time is integrated
        kalmanFilter.F_.row(0) << 1, 0, dt, 0;
        kalmanFilter.F_.row(1) << 0, 1, 0, dt;
        kalmanFilter.F_.row(2) << 0, 0, 1, 0;
        kalmanFilter.F_.row(3) << 0, 0, 0, 1;

        // 2. Set the process covariance matrix Q
        kalmanFilter.Q_.row(0) << pow(dt, 4) / 4 * noise_ax, 0, pow(dt, 3) / 2 * noise_ax, 0;
        kalmanFilter.Q_.row(1) << 0, pow(dt, 4) / 4 * noise_ay, 0, pow(dt, 3) / 2 * noise_ay;
        kalmanFilter.Q_.row(2) << pow(dt, 3) / 2 * noise_ax, 0, pow(dt, 2) * noise_ax, 0;
        kalmanFilter.Q_.row(3) << 0, pow(dt, 3) / 2 * noise_ay, 0, pow(dt, 2) * noise_ay;

        // 3. Call the Kalman Filter predict() function
        kalmanFilter.Predict();

        // 4. Call the Kalman Filter update() function with the most recent raw measurements_
        kalmanFilter.Update(measurement_pack.raw_measurements_);
    };

    VectorXd x_expected(4);
    MatrixXd P_expected(4, 4);

    // Process the second and third
    processMeasurement(measurements[1]);

    x_expected << 0.96749, 0.405862, 4.58427, -1.83232;

    P_expected << 0.0224541, 0, 0.204131, 0, 0, 0.0224541, 0, 0.204131, 0.204131, 0, 92.7797, 0, 0, 0.204131, 0,
        92.7797;

    ASSERT_TRUE(x_expected.isApprox(kalmanFilter.x_, 0.0001));
    ASSERT_TRUE(P_expected.isApprox(kalmanFilter.P_, 0.0001));

    processMeasurement(measurements[2]);

    x_expected << 0.958365, 0.627631, 0.110368, 2.04304;
    P_expected << 0.0220006, 0, 0.210519, 0, 0, 0.0220006, 0, 0.210519, 0.210519, 0, 4.08801, 0, 0, 0.210519, 0,
        4.08801;

    ASSERT_TRUE(x_expected.isApprox(kalmanFilter.x_, 0.0001));
    ASSERT_TRUE(P_expected.isApprox(kalmanFilter.P_, 0.0001));
}
