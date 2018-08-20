#include <tools.hpp>

#include <vector>

Tools::Tools() = default;

Tools::~Tools() = default;

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                     const std::vector<Eigen::VectorXd> &ground_truth) {
    /**
    TODO:
      * Calculate the RMSE here.
    */
}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd &x_state) {
    /**
    TODO:
      * Calculate a Jacobian here.
    */
}
