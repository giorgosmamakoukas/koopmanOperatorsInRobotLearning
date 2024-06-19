#include <eigen3/Eigen/Dense>
#include <iostream>
#include <random>
#include <time.h>
#include <vector>

#include "../headers/dataGeneration.hpp"
#include "../headers/dynamics.hpp"
#include "../headers/riccatiSolver.hpp"
#include "../headers/simulation.hpp"
#include "../headers/koopmanModel.hpp"

using namespace Eigen;
using namespace std;

int main()
{
    
  // State and control dimensions
  const int stateDim = 2;
  const int controlDim = 1; // Size of the control input

  // Generate training data
  const int num_samples_train = 10;
  const double dt = 0.1;
  MatrixXd Xtrain, Utrain;
  generateData(invertedPendulumDynamics, Xtrain, Utrain, num_samples_train, dt,
               stateDim, controlDim);

  // Train least-squares dynamics
  KoopmanModel koopman(stateDim, controlDim);
  LinearMatrices leastSquaresAandB = koopman.computeKoopmanViaLeastSquares(Xtrain, Utrain, dt);

  // Solve Riccati equation to calculate LQR gains
  MatrixXd P;
  const int psiDim = leastSquaresAandB.A.rows();
  const MatrixXd R = 1000 * MatrixXd::Identity(controlDim, 1);
  MatrixXd Qlifted = MatrixXd::Zero(psiDim, psiDim);
  const MatrixXd Q = MatrixXd::Identity(stateDim, stateDim);
  Qlifted.block(0, 0, stateDim, stateDim) << Q;

  if (!solveRiccatiIterationD(leastSquaresAandB.A, leastSquaresAandB.B, Qlifted, R, P))
  {
    std::cerr << "Riccati solution failed!" << std::endl;
    return -1;
  }

  // Compute LQR gains
  MatrixXd K;
  K = R.inverse() * leastSquaresAandB.B.transpose() * P;

  // Control system with LQR gains
  VectorXd x0(stateDim);
  x0 << 0.5, 0; // Initial state

  // Simulate inverted pendulum system with LQR control
  int control_steps = 100;
  std::cout << "-------------------------------------- CLOSED LOOP CONTROL -------------------------------------- " << std::endl;
  simulateInvertedPendulumWithLQR(K, x0, dt, control_steps, KoopmanModel::computeStateBasisFunction);

  return 0;
}