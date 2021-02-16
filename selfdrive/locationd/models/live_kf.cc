#include "live_kf.hpp"

#define NOISEBUILDER(x) Eigen::DiagonalMatrix(Eigen::ArrayBase::pow(Eigen::VectorXd(x),2))

// slices are just what there is under an array[start:stop:step] operation,
// like so: array[slice(start, stop, step)].
// The following slice objects are used within the brackets of arrays in order
// to get the desired slices of the arrays.
// To find a way to do this in c++, first we have to know what kind of arrays
// we're slicing.
// According to the syntax used in this script. The matrix which we want to
// slice is one with lots of rows, and the indices in these slices are row
// indices.
// These slices can be simple C arrays and can be passed to a special function
// to get an Eigen::Block from a Matrix or Eigen object
const struct _States {
    char ECEF_POS[]             = { 0,  3};  // x, y and z in ECEF in meters
    char ECEF_ORIENTATION[]     = { 3,  7};  // quat for pose of phone in ecef
    char ECEF_VELOCITY[]        = { 7, 10};  // ecef velocity in m/s
    char ANGULAR_VELOCITY[]     = {10, 13};  // roll, pitch and yaw rates in device frame in rad/s
    char GYRO_BIAS[]            = {13, 16};  // roll, pitch and yaw biases
    char ODO_SCALE[]            = {16, 17};  // odometer scale
    char ACCELERATION[]         = {17, 20};  // Acceleration in device frame in m/s**2
    char IMU_OFFSET[]           = {20, 23};  // imu offset angles in radians
    
    // Error-state has different slices because it is an ESKF
    char ECEF_POS_ERR[]         = { 0,  3};
    char ECEF_ORIENTATION_ERR[] = { 3,  6};  // euler angles for orientation error
    char ECEF_VELOCITY_ERR[]    = { 6,  9};
    char ANGULAR_VELOCITY_ERR[] = { 9, 12};
    char GYRO_BIAS_ERR[]        = {12, 15};
    char ODO_SCALE_ERR[]        = {15, 16};
    char ACCELERATION_ERR[]     = {16, 19};
    char IMU_OFFSET_ERR[]       = {19, 22};
} States;
// TODO find a cleaner way to do this

// matrix.block(height, width, row0, col0)
// rowblock : get all columns, give only row0 and height
// get_matrix_block(Matrix m, block_t block)
//     return m.block(block.height, block.width, 
//                    block.row,    block.col)
// get_matrix_rowset(Matrix m, rowset_t rowset)
//     return m.block(rowset.height, m.width,
//                    rowset.start,  0)


LiveKalman(const std::string generated_dir)
{
    name = "live";

    initial_x << -2.7e6, 4.2e6, 3.8e6,
                      1,     0,     0, 0,
                      0,     0,     0,
                      0,     0,     0,
                      0,     0,     0,
                      1,
                      0,     0,     0,
                      0,     0,     0;

    // state covariance
    initial_P_diag <<   1e8,  1e8,  1e8,
                        1e3,  1e3,  1e3,
                        1e2,  1e2,  1e2,
                          1,    1,    1,
                       0.05, 0.05, 0.05,
                       0.02,
                          1,    1,    1,
                       0.01, 0.01, 0.01;
    initial_P_diag = ArrayBase::pow(initial_P_diag, 2);

    // process noise
    Q = 
        Eigen::DiagonalMatrix(
            ArrayBase::pow(
                ((MatrixXd<22,1>) <<       0.03,        0.03,        0.03,
                                          0.001,       0.001,       0.001,
                                           0.01,        0.01,        0.01,
                                            0.1,         0.1,         0.1,
                                      0.005/100,   0.005/100,   0.005/100,
                                       0.02/100,
                                              3,           3,           3,
                                        0.05/60,     0.05/60,     0.05/60), 
                2));
  
    dim_state     = initial_x.rows();
    dim_state_err = initial_P_diag.rows();

    obs_noise[ObservationKind.ODOMETRIC_SPEED]            = np.atleast_2d(std::pow(0.2,2)); // TODO translate
    // The np.at_least_2d(x) returns a 2d array like so: [[x]]
    // I'm not sure that Eigen supports this
    obs_noise[ObservationKind.PHONE_GYRO]                 = NOISEBUILDER(  0.025,   0.025,   0.025);
    obs_noise[ObservationKind.PHONE_ACCEL]                = NOISEBUILDER(    0.5,     0.5,     0.5);
    obs_noise[ObservationKind.CAMERA_ODO_ROTATION]        = NOISEBUILDER(   0.05,    0.05,    0.05);
    obs_noise[ObservationKind.IMU_FRAME]                  = NOISEBUILDER(   0.05,    0.05,    0.05);
    obs_noise[ObservationKind.NO_ROT]                     = NOISEBUILDER(0.00025, 0.00025, 0.00025);
    obs_noise[ObservationKind.ECEF_POS]                   = NOISEBUILDER(      5,       5,       5);
    obs_noise[ObservationKind.ECEF_VEL]                   = NOISEBUILDER(    0.5,     0.5,     0.5);
    obs_noise[ObservationKind.ECEF_ORIENTATION_FROM_GPS]  = NOISEBUILDER(    0.2,     0.2,     0.2,   0.2)};

    // init filter
    filter = EKF_sym(generated_dir, 
                     name, 
                     Q, 
                     initial_x, 
                     Eigen::DiagonalMatrix(initial_P_diag),
                     dim_state, 
                     dim_state_err, 
                     max_rewind_age=0.2); // TODO the parameter name probably
                                          // needs to be removed
}


static void
LiveKalman::generate_code(std::string generated_dir)
{
}
    name = LiveKalman.name
    dim_state = LiveKalman.initial_x.shape[0]
    dim_state_err = LiveKalman.initial_P_diag.shape[0]

    state_sym = sp.MatrixSymbol('state', dim_state, 1)
    state = sp.Matrix(state_sym)
    x, y, z = state[States.ECEF_POS, :]
    q = state[States.ECEF_ORIENTATION, :]
    v = state[States.ECEF_VELOCITY, :]
    vx, vy, vz = v
    omega = state[States.ANGULAR_VELOCITY, :]
    vroll, vpitch, vyaw = omega
    roll_bias, pitch_bias, yaw_bias = state[States.GYRO_BIAS, :]
    odo_scale = state[States.ODO_SCALE, :][0, :]
    acceleration = state[States.ACCELERATION, :]
    imu_angles = state[States.IMU_OFFSET, :]

    dt = sp.Symbol('dt')

    # calibration and attitude rotation matrices
    quat_rot = quat_rotate(*q)

    # Got the quat predict equations from here
    # A New Quaternion-Based Kalman Filter for
    # Real-Time Attitude Estimation Using the Two-Step
    # Geometrically-Intuitive Correction Algorithm
    A = 0.5 * sp.Matrix([[0, -vroll, -vpitch, -vyaw],
                         [vroll, 0, vyaw, -vpitch],
                         [vpitch, -vyaw, 0, vroll],
                         [vyaw, vpitch, -vroll, 0]])
    q_dot = A * q

    # Time derivative of the state as a function of state
    state_dot = sp.Matrix(np.zeros((dim_state, 1)))
    state_dot[States.ECEF_POS, :] = v
    state_dot[States.ECEF_ORIENTATION, :] = q_dot
    state_dot[States.ECEF_VELOCITY, 0] = quat_rot * acceleration

    # Basic descretization, 1st order intergrator
    # Can be pretty bad if dt is big
    f_sym = state + dt * state_dot

    state_err_sym = sp.MatrixSymbol('state_err', dim_state_err, 1)
    state_err = sp.Matrix(state_err_sym)
    quat_err = state_err[States.ECEF_ORIENTATION_ERR, :]
    v_err = state_err[States.ECEF_VELOCITY_ERR, :]
    omega_err = state_err[States.ANGULAR_VELOCITY_ERR, :]
    acceleration_err = state_err[States.ACCELERATION_ERR, :]

    # Time derivative of the state error as a function of state error and state
    quat_err_matrix = euler_rotate(quat_err[0], quat_err[1], quat_err[2])
    q_err_dot = quat_err_matrix * quat_rot * (omega + omega_err)
    state_err_dot = sp.Matrix(np.zeros((dim_state_err, 1)))
    state_err_dot[States.ECEF_POS_ERR, :] = v_err
    state_err_dot[States.ECEF_ORIENTATION_ERR, :] = q_err_dot
    state_err_dot[States.ECEF_VELOCITY_ERR, :] = quat_err_matrix * quat_rot * (acceleration + acceleration_err)
    f_err_sym = state_err + dt * state_err_dot

    # Observation matrix modifier
    H_mod_sym = sp.Matrix(np.zeros((dim_state, dim_state_err)))
    H_mod_sym[States.ECEF_POS, States.ECEF_POS_ERR] = np.eye(States.ECEF_POS.stop - States.ECEF_POS.start)
    H_mod_sym[States.ECEF_ORIENTATION, States.ECEF_ORIENTATION_ERR] = 0.5 * quat_matrix_r(state[3:7])[:, 1:]
    H_mod_sym[States.ECEF_ORIENTATION.stop:, States.ECEF_ORIENTATION_ERR.stop:] = np.eye(dim_state - States.ECEF_ORIENTATION.stop)

    # these error functions are defined so that say there
    # is a nominal x and true x:
    # true x = err_function(nominal x, delta x)
    # delta x = inv_err_function(nominal x, true x)
    nom_x = sp.MatrixSymbol('nom_x', dim_state, 1)
    true_x = sp.MatrixSymbol('true_x', dim_state, 1)
    delta_x = sp.MatrixSymbol('delta_x', dim_state_err, 1)

    err_function_sym = sp.Matrix(np.zeros((dim_state, 1)))
    delta_quat = sp.Matrix(np.ones((4)))
    delta_quat[1:, :] = sp.Matrix(0.5 * delta_x[States.ECEF_ORIENTATION_ERR, :])
    err_function_sym[States.ECEF_POS, :] = sp.Matrix(nom_x[States.ECEF_POS, :] + delta_x[States.ECEF_POS_ERR, :])
    err_function_sym[States.ECEF_ORIENTATION, 0] = quat_matrix_r(nom_x[States.ECEF_ORIENTATION, 0]) * delta_quat
    err_function_sym[States.ECEF_ORIENTATION.stop:, :] = sp.Matrix(nom_x[States.ECEF_ORIENTATION.stop:, :] + delta_x[States.ECEF_ORIENTATION_ERR.stop:, :])

    inv_err_function_sym = sp.Matrix(np.zeros((dim_state_err, 1)))
    inv_err_function_sym[States.ECEF_POS_ERR, 0] = sp.Matrix(-nom_x[States.ECEF_POS, 0] + true_x[States.ECEF_POS, 0])
    delta_quat = quat_matrix_r(nom_x[States.ECEF_ORIENTATION, 0]).T * true_x[States.ECEF_ORIENTATION, 0]
    inv_err_function_sym[States.ECEF_ORIENTATION_ERR, 0] = sp.Matrix(2 * delta_quat[1:])
    inv_err_function_sym[States.ECEF_ORIENTATION_ERR.stop:, 0] = sp.Matrix(-nom_x[States.ECEF_ORIENTATION.stop:, 0] + true_x[States.ECEF_ORIENTATION.stop:, 0])

    eskf_params = [[err_function_sym, nom_x, delta_x],
                   [inv_err_function_sym, nom_x, true_x],
                   H_mod_sym, f_err_sym, state_err_sym]
    #
    # Observation functions
    #
    #imu_rot = euler_rotate(*imu_angles)
    h_gyro_sym = sp.Matrix([vroll + roll_bias,
                                      vpitch + pitch_bias,
                                      vyaw + yaw_bias])

    pos = sp.Matrix([x, y, z])
    gravity = quat_rot.T * ((EARTH_GM / ((x**2 + y**2 + z**2)**(3.0 / 2.0))) * pos)
    h_acc_sym = (gravity + acceleration)
    h_phone_rot_sym = sp.Matrix([vroll, vpitch, vyaw])

    speed = sp.sqrt(vx**2 + vy**2 + vz**2 + 1e-6)
    h_speed_sym = sp.Matrix([speed * odo_scale])

    h_pos_sym = sp.Matrix([x, y, z])
    h_vel_sym = sp.Matrix([vx, vy, vz])
    h_orientation_sym = q
    h_imu_frame_sym = sp.Matrix(imu_angles)

    h_relative_motion = sp.Matrix(quat_rot.T * v)

    obs_eqs = [[h_speed_sym, ObservationKind.ODOMETRIC_SPEED, None],
               [h_gyro_sym, ObservationKind.PHONE_GYRO, None],
               [h_phone_rot_sym, ObservationKind.NO_ROT, None],
               [h_acc_sym, ObservationKind.PHONE_ACCEL, None],
               [h_pos_sym, ObservationKind.ECEF_POS, None],
               [h_vel_sym, ObservationKind.ECEF_VEL, None],
               [h_orientation_sym, ObservationKind.ECEF_ORIENTATION_FROM_GPS, None],
               [h_relative_motion, ObservationKind.CAMERA_ODO_TRANSLATION, None],
               [h_phone_rot_sym, ObservationKind.CAMERA_ODO_ROTATION, None],
               [h_imu_frame_sym, ObservationKind.IMU_FRAME, None]]

    gen_code(generated_dir, 
             name, 
             f_sym, 
             dt, 
             state_sym, 
             obs_eqs, 
             dim_state, 
             dim_state_err, 
             eskf_params)

TYPEOF_filter__state
LiveKalman::get_x()
{
    return filter.state();
}

TYPEOF_filter__filter_time
LiveKalman::get_t()
{
    return filter.filter_time;
}

TYPEOF_filter__covs
LiveKalman::get_P()
{
    return filter.covs();
}


TYPEOF_filter__rts_smooth
LiveKalman::rts_smooth(TYPEOF_estimates estimates)
{
    return self.filter.rts_smooth(estimates, 
                                  true); // norm_quats
}


TYPEOF_filter__predict_and_update_batch
LiveKalman::init_state(state,
                       covs_diag,   // TODO make optional
                       covs,        // TODO make optional
                       filter_time) // TODO make optional
{
    // TODO implement
  def init_state(self, state, covs_diag=None, covs=None, filter_time=None):
    if covs_diag is not None:
      P = np.diag(covs_diag)
    elif covs is not None:
      P = covs
    else:
      P = self.filter.covs()
    self.filter.init_state(state, P, filter_time)
}

  def predict_and_observe(self, t, kind, meas, R=None):
    if len(meas) > 0:
      meas = np.atleast_2d(meas)
    if kind == ObservationKind.CAMERA_ODO_TRANSLATION:
      r = self.predict_and_update_odo_trans(meas, t, kind)
    elif kind == ObservationKind.CAMERA_ODO_ROTATION:
      r = self.predict_and_update_odo_rot(meas, t, kind)
    elif kind == ObservationKind.ODOMETRIC_SPEED:
      r = self.predict_and_update_odo_speed(meas, t, kind)
    else:
      if R is None:
        R = self.get_R(kind, len(meas))
      elif len(R.shape) == 2:
        R = R[None]
      r = self.filter.predict_and_update_batch(t, kind, meas, R)

    # Normalize quats
    quat_norm = np.linalg.norm(self.filter.x[3:7, 0])
    self.filter.x[States.ECEF_ORIENTATION, 0] = self.filter.x[States.ECEF_ORIENTATION, 0] / quat_norm

    return r

  def get_R(self, kind, n):
    obs_noise = self.obs_noise[kind]
    dim = obs_noise.shape[0]
    R = np.zeros((n, dim, dim))
    for i in range(n):
      R[i, :, :] = obs_noise
    return R

  def predict_and_update_odo_speed(self, speed, t, kind):
    z = np.array(speed)
    R = np.zeros((len(speed), 1, 1))
    for i, _ in enumerate(z):
      R[i, :, :] = np.diag([0.2**2])
    return self.filter.predict_and_update_batch(t, kind, z, R)

  def predict_and_update_odo_trans(self, trans, t, kind):
    z = trans[:, :3]
    R = np.zeros((len(trans), 3, 3))
    for i, _ in enumerate(z):
        R[i, :, :] = np.diag(trans[i, 3:]**2)
    return self.filter.predict_and_update_batch(t, kind, z, R)

  def predict_and_update_odo_rot(self, rot, t, kind):
    z = rot[:, :3]
    R = np.zeros((len(rot), 3, 3))
    for i, _ in enumerate(z):
        R[i, :, :] = np.diag(rot[i, 3:]**2)
    return self.filter.predict_and_update_batch(t, kind, z, R)



int
main(int argc, char *argv[])
{
    // argv[2] is "generated_dir"
    LiveKalman.generate_code(argv[2]);
    return 0;
}
