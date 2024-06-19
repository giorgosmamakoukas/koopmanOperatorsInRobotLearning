#pragma once
#ifndef DATAGENERATION_HPP
#define DATAGENERATION_HPP

#include <eigen3/Eigen/Dense>
#include <functional>

typedef std::function<void(const Eigen::VectorXd &, const Eigen::VectorXd &,
                           Eigen::VectorXd &)>
    DynamicsFunction;

/// @brief Simulates dynamical system forward in time and returns trajectory data for each state
/// @param dynamics Dynamical equations of system states
/// @param X Matrix to populate with state measurements
/// @param U Matrix to populate applied control
/// @param num_samples Number of forward simulation steps to integrate dynamics
/// @param dt Time spacing between measurement
/// @param state_size State dimension
/// @param control_size Control dimension
void generateData(const DynamicsFunction &dynamics, Eigen::MatrixXd &X,
                  Eigen::MatrixXd &U, int num_samples,
                  double dt, int state_size, int control_size);

#endif // DATA_GENERATION_HPP