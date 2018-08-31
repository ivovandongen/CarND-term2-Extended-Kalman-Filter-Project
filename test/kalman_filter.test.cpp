#include <test.hpp>

#include <kalman_filter.hpp>
#include <measurement_package.hpp>

#include <vector>

TEST(KalmanFilter, Laser) {
    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    std::vector<MeasurementPackage> measurements = {
            {MeasurementPackage::SensorType::LASER, 1477010443000000, {4.632272e-01, 6.074152e-01}},
            {MeasurementPackage::SensorType::LASER, 1477010443100000, {9.685213e-01, 4.054501e-01}},
            {MeasurementPackage::SensorType::LASER, 1477010443200000, {9.477518e-01, 6.368242e-01}}};

    // Initialise Kalman Filter
    KalmanFilter kalmanFilter{5, 5};

    // measurement matrix
    MatrixXd H(2, 4);
    H.row(0) << 1, 0, 0, 0;
    H.row(1) << 0, 1, 0, 0;

    // measurement covariance
    MatrixXd R(2, 2);
    R.row(0) << 0.0225, 0;
    R.row(1) << 0, 0.0225;

    // Set first measurement (initial location and zero velocity)
    VectorXd x(4);
    x << measurements[0].raw_measurements_[0], measurements[0].raw_measurements_[1], 0, 0;
    kalmanFilter.init(std::move(x));

    // Set initial timestamp
    MeasurementPackage::Timestamp previous_timestamp = measurements[0].timestamp_;

    auto processMeasurement = [&](MeasurementPackage &measurement_pack) {
        // compute the time elapsed between the current and previous measurements
        double dt = (measurement_pack.timestamp_ - previous_timestamp) / 1000000.0;
        previous_timestamp = measurement_pack.timestamp_;

        // 3. Call the Kalman Filter predict() function
        kalmanFilter.predict(dt);

        // 4. Call the Kalman Filter update() function with the most recent raw measurements_
        kalmanFilter.update(measurement_pack.raw_measurements_, H, R);
    };

    VectorXd x_expected(4);
    MatrixXd P_expected(4, 4);

    // Process the second and third
    processMeasurement(measurements[1]);

    x_expected << 0.96749, 0.405862, 4.58427, -1.83232;

    P_expected.row(0) << 0.0224541, 0, 0.204131, 0;
    P_expected.row(1) << 0, 0.0224541, 0, 0.204131;
    P_expected.row(2) << 0.204131, 0, 92.7797, 0;
    P_expected.row(3) << 0, 0.204131, 0, 92.7797;

    ASSERT_TRUE(x_expected.isApprox(kalmanFilter.getX(), 0.0001));
    ASSERT_TRUE(P_expected.isApprox(kalmanFilter.getP(), 0.0001));

    processMeasurement(measurements[2]);

    x_expected << 0.958365, 0.627631, 0.110368, 2.04304;
    P_expected.row(0) << 0.0220006, 0, 0.210519, 0;
    P_expected.row(1) << 0, 0.0220006, 0, 0.210519;
    P_expected.row(2) << 0.210519, 0, 4.08801, 0;
    P_expected.row(3) << 0, 0.210519, 0, 4.08801;

    ASSERT_TRUE(x_expected.isApprox(kalmanFilter.getX(), 0.0001));
    ASSERT_TRUE(P_expected.isApprox(kalmanFilter.getP(), 0.0001));
}
