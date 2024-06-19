#pragma once
#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <eigen3/Eigen/Dense>

/// @brief Simulates inverted pendulum dynamics with LQR control
/// @param K LQR gains computed for the inverted pendulum dynamics
/// @param x0 Initial state
/// @param dt Simulate time step for each successive state measurement
/// @param num_steps Number of simulation steps to propagate closed-loop system
void simulateInvertedPendulumWithLQR(const Eigen::MatrixXd &K,
                                     const Eigen::VectorXd &x0, const double &dt,
                                     const int &num_steps,
                                     std::function<void(const Eigen::VectorXd &, Eigen::VectorXd &)> psi);

#endif // SIMULATION_HPP