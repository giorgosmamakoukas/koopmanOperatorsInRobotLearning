#include "../headers/dynamics.hpp"
#include <cmath>

void invertedPendulumDynamics(const Eigen::VectorXd &x,
                              const Eigen::VectorXd &u,
                              Eigen::VectorXd &x_dot) {
  constexpr double g = 9.81; // Acceleration due to gravity (m/s^2)
  constexpr double l = 1.0;  // Length of the pendulum (m)

  // Continuous time dynamics
  x_dot(0) = x(1);
  x_dot(1) = -g / l * std::sin(x(0)) + u(0);
}