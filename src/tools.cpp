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
    /**
    TODO:
      * Calculate a Jacobian here.
    */
}
