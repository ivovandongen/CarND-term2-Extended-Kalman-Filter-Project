#include "coordinates.hpp"

#include <algorithm>
#include <cmath>

CartesianCoordinate CartesianCoordinate::fromPolar(PolarCoordinate polar) {
    static const double threshold = 0.0001;
    return {std::max<double>(polar.rho * cos(polar.phi), threshold),
            std::max<double>(polar.rho * sin(polar.phi), threshold),
            std::max<double>(polar.rho_dot * cos(polar.phi), threshold),
            std::max<double>(polar.rho_dot * sin(polar.phi), threshold)};
}

PolarCoordinate PolarCoordinate::fromCartesian(CartesianCoordinate ct) {
    double rho = sqrt(ct.x * ct.x + ct.y * ct.y);
    double phi = atan2(ct.y, ct.x);
    double rho_dot = (ct.x * ct.vx + ct.y * ct.vy) / rho;

    // Normalise phi
    normalisePhi(phi);

    return {rho, phi, rho_dot};
}

void PolarCoordinate::normalisePhi(double &phi) {
    static const double TWO_PI = 2 * M_PI;
    while (abs(phi) > M_PI) {
        phi = (phi < -M_PI) ? phi + TWO_PI : phi - TWO_PI;
    }
}