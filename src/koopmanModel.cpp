#include <eigen3/Eigen/Dense>
#include "../headers/koopmanModel.hpp"

void KoopmanModel::computeStateBasisFunction(const Eigen::VectorXd &state, Eigen::VectorXd &basisFunction)
{
    basisFunction << state,
        sin(state(0)),
        cos(state(0));
}

void KoopmanModel::liftMeasurements(const Eigen::MatrixXd &measurements, Eigen::MatrixXd &liftedMeasurements, std::function<void(const Eigen::VectorXd &, Eigen::VectorXd &)> psi)
{
    // Apply computeBasisFunction to each column of the measurements matrix
    for (int col = 0; col < measurements.cols(); ++col)
    {
        Eigen::VectorXd basisFunction(measurements.rows() + 2);
        psi(measurements.col(col), basisFunction);
        liftedMeasurements.col(col) = basisFunction;
    }
}

LinearMatrices &KoopmanModel::computeKoopmanViaLeastSquares(const Eigen::MatrixXd &M, const Eigen::MatrixXd &U,
                                                            const double dt)
{
    // Convert state measurements to basis function
    Eigen::MatrixXd psiM(M.rows() + 2, M.cols()); // Create a new matrix to store the transformed states
    liftMeasurements(M, psiM, computeStateBasisFunction);

    // Extract PsiX and PsiY from measurement matrix
    const Eigen::MatrixXd PsiX = psiM.leftCols(psiM.cols() - 1);
    const Eigen::MatrixXd PsiY = psiM.rightCols(psiM.cols() - 1);
    const Eigen::MatrixXd Ut = U.leftCols(U.cols() - 1);

    // Create [X;U]
    Eigen::MatrixXd XU(PsiX.rows() + U.rows(), PsiX.cols());
    XU << PsiX, Ut;

    // Compute the pseudo-inverse of [X; U]
    Eigen::MatrixXd XU_pseudo_inv =
        XU.completeOrthogonalDecomposition().pseudoInverse();

    // Compute matrix [A; B]
    Eigen::MatrixXd ABls = PsiY * XU_pseudo_inv;

    // Extract A and B
    const int state_dimension = PsiX.rows();
    const int input_dimension = U.rows();
    mLeastSquaresKoopman = {ABls.leftCols(ABls.cols() - input_dimension),
                            ABls.rightCols(ABls.cols() - state_dimension)};

    return mLeastSquaresKoopman;
}
