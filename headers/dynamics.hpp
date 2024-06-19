#pragma once
#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include <eigen3/Eigen/Dense>
#include <functional>

typedef std::function<void(const Eigen::VectorXd &, const Eigen::VectorXd &,
                           Eigen::VectorXd &)>
    DynamicsFunction;

/// @brief Compute the continuous-time dynamics of a 2D inverted pendulum system.
/// @param x The current state vector [theta, theta_dot].
/// @param u The control input vector [force].
/// @param x_dot The resulting state derivative vector.
void invertedPendulumDynamics(const Eigen::VectorXd &x,
                              const Eigen::VectorXd &u, Eigen::VectorXd &x_dot);

#endif // DYNAMICS_HPP