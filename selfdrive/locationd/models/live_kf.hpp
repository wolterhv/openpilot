#include <string>

#include "constants.hpp"

// from rednose.helpers.ekf_sym import EKF_sym, gen_code
// from rednose.helpers.sympy_helpers import euler_rotate, quat_matrix_r, quat_rotate

#define EARTH_GM 3.986005e14  // m^3/s^2 (gravitational constant * mass of earth)


class LiveKalman
{
    LiveKalman(std::string generated_dir);

    // TODO find out what's private and what's public here

    // Functions

    static void generate_code(std::string generated_dir);

    // Attributes

    std::string name;

    Eigen::VectorXd initial_x(23);

    Eigen::VectorXd initial_P_diag(22);

    Eigen::DiagonalMatrix<double, 22, 22> Q;

    uint dim_state;
    uint dim_state_err;   

    std::map<char, Eigen::Matrix> obs_noise;

    EKFSym filter;
}

