#pragma once

#include <eigen3/Eigen/Dense>

struct LinearMatrices
{
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
};

class KoopmanModel
{
public:
    KoopmanModel() = default;
    KoopmanModel(int stateDimension, int controlDimension) : mStateDim(stateDimension), mControlDim(controlDimension){};
    LinearMatrices &computeKoopmanViaLeastSquares(const Eigen::MatrixXd &M, const Eigen::MatrixXd &U,
                                                  double dt);

    /// @brief Computes basis functions Psi(x,u) of Koopman model
    /// @param state System state
    /// @param control Control input of system
    /// @param basisFunction Vector of basis function to be populated
    static void computeStateBasisFunction(const Eigen::VectorXd &state, Eigen::VectorXd &basisFunction);

private:
    int mStateDim;
    int mControlDim;
    LinearMatrices mLeastSquaresKoopman;
    /// @brief
    /// @param states
    /// @param liftedMeasurements
    /// @param psi
    void liftMeasurements(const Eigen::MatrixXd &measurements, Eigen::MatrixXd &liftedMeasurements, std::function<void(const Eigen::VectorXd &, Eigen::VectorXd &)> psi);
};