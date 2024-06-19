#pragma once
#ifndef PLOTTING_HPP
#define PLOTTING_HPP

#include <eigen3/Eigen/Dense>
#include <string>

/// @brief Plots columns of Eigen matrix
/// @param matrix Eigenmatrix with rows = measurements and cols = states
void plotColumnVectors(const Eigen::MatrixXd& states, const std::vector<std::string>& labels);

#endif // PLOTTING_HPP