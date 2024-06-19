#pragma once
#ifndef RICCATISOLVER_HPP
#define RICCATISOLVER_HPP

#include <eigen3/Eigen/Dense>

/// @brief Computes solution to the discrete-timme algebraic Riccati equation
/// @param Ad Discrete-time linear state matrix
/// @param Bd Discrete-time linear input matrix
/// @param Q Cost matrix for states
/// @param R Cost matrix for control
/// @param P Solution to the discrete-time algebraic Riccati equation
/// @param tolerance Numerical tolerance for solver convergance
/// @param iter_max Maximum number of solver iterations
/// @return Boolean indicates whether solver converged to solution
bool solveRiccatiIterationD(const Eigen::MatrixXd &Ad,
                            const Eigen::MatrixXd &Bd, const Eigen::MatrixXd &Q,
                            const Eigen::MatrixXd &R, Eigen::MatrixXd &P,
                            const double &tolerance = 1.E-4,
                            const uint iter_max = 100000);

#endif // RICCATI_SOLVER_HPP