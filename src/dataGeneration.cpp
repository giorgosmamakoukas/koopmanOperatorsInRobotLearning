#include "../headers/dataGeneration.hpp"
#include <random>

void generateData(const DynamicsFunction &dynamics, Eigen::MatrixXd &X, Eigen::MatrixXd &U, int num_samples, double dt, int state_size, int control_size)
{
    X.resize(state_size, num_samples);
    U.resize(control_size, num_samples);

    Eigen::VectorXd x(state_size);
    x << 0.01, 0.0;

    Eigen::VectorXd x_dot(state_size); // Used to store dynamics fdot at given state
    Eigen::VectorXd u(control_size);   // Used to store control

    // Random number generator for control input
    std::random_device rd;
    std::mt19937 gen(rd());

    // TODO: Provide control inputs as function arguments or define in params file
    const double u_mean = 0.0;
    const double u_std = 0.1;
    std::normal_distribution<double> control_dist(u_mean, u_std);

    for (int i = 0; i < num_samples; ++i)
    {
        // Generate random control input
        for (int j = 0; j < control_size; ++j)
        {
            u(j) = control_dist(gen);
        }

        // Compute dynamics at state x and given control u
        dynamics(x, u, x_dot);

        X.col(i) = x;    // Store current state
        U.col(i) = u;    // Store applied control
        x += x_dot * dt; // Compute next state
    }
}