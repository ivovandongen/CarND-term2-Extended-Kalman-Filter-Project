#include <tools.hpp>

#include <vector>
#include <iostream>

Tools::Tools() = default;

Tools::~Tools() = default;

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                     const std::vector<Eigen::VectorXd> &ground_truth) {
    Eigen::VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.empty() || (estimations.size() != ground_truth.size())) {
        std::cout << "Estimations don't match the ground truth" << std::endl;
        return rmse;
    }

    //accumulate squared residuals
    for (int i = 0; i < estimations.size(); ++i) {
        Eigen::VectorXd res = estimations[i] - ground_truth[i];
        res = res.array() * res.array();
        rmse += res;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd &x_state) {
    Eigen::MatrixXd Hj(3, 4);

    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //check division by zero
    if (px == 0 && py == 0) {
        std::cout << "Division by 0" << std::endl;
        return Hj;
    }

    // compute the Jacobian matrix
    double c1 = px * px + py * py;
    double c2 = sqrt(c1);
    double c3 = pow(c1, 3 / 2);
    double c4 = vx * py;
    double c5 = vy * px;

    Hj << px / c2, py / c2, 0, 0,
            -(py / c1), px / c1, 0, 0,
            py * (c4 - c5) / c3, px * (c5 - c4) / c3, px / c2, py / c2;

    return Hj;
}
