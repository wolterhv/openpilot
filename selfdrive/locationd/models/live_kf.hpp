#include "constants.hpp"

from rednose.helpers.ekf_sym import EKF_sym, gen_code
from rednose.helpers.sympy_helpers import euler_rotate, quat_matrix_r, quat_rotate

#define EARTH_GM 3.986005e14  // m^3/s^2 (gravitational constant * mass of earth)

