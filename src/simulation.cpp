#include "../headers/simulation.hpp"
#include "../headers/dynamics.hpp"
#include <eigen3/Eigen/Dense>
#include "../headers/plotting.hpp"
#include <string>
#include <vector>


#include <iomanip>
#include <iostream>

// TODO Implement control saturation limits
void simulateInvertedPendulumWithLQR(const Eigen::MatrixXd &K,
                                     const Eigen::VectorXd &x0, const double &dt,
                                     const int &num_steps,
                                     std::function<void(const Eigen::VectorXd &, Eigen::VectorXd &)> psi)
{
  const int stateDim = x0.size();

  Eigen::MatrixXd controlled_states(num_steps+1, stateDim);
  controlled_states.row(0) = x0.transpose();

  Eigen::VectorXd x = x0;
  // Simulate dynamics
  for (int i = 0; i < num_steps; ++i)

  {
    // Compute control input u = -K * Psi(x)
    Eigen::VectorXd basisFunction(x.rows() + 2);
    psi(x, basisFunction);
    Eigen::VectorXd u = -K * basisFunction;

    // Inverted pendulum dynamics: x_dot = f(x, u)
    Eigen::VectorXd x_dot(stateDim);
    invertedPendulumDynamics(x, u, x_dot);

    // Update state: x_{k+1} = x_k + dt * f(x_k, u_k)
    x = x + dt * x_dot;
    controlled_states.row(i+1) = x.transpose();

    std::cout << "Time: " << std::fixed << std::setprecision(2) << i * dt
              << "\tState: " << std::fixed << std::setprecision(2) << x.transpose()
              << "\tControl Input: " << std::fixed << std::setprecision(2)  << u.transpose() << std::endl;
  } 
  std::vector<std::string> state_labels = {"Angle (rad)", "Ang Velocity (rad/s)"};
  plotColumnVectors(controlled_states, state_labels);
}