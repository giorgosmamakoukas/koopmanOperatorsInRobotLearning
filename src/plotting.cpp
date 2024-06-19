#include "../headers/plotting.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace Eigen;

void plotColumnVectors(const Eigen::MatrixXd& matrix, const std::vector<std::string>& labels){
    plt::figure();
    std::vector<double> state_vec;
    for (size_t i = 0; i < matrix.cols(); ++i){

        // Assign the column of the matrix to a vector
        VectorXd col = matrix.col(i);

        // Convert the Eigen vector to a standard vector
        state_vec.assign(col.data(), col.data() + col.size());

        // Plot state
        plt::plot(state_vec,  {{"label", labels.at(i)}});
        plt::legend();


    }
    plt::show();
}