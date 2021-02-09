#include "locationd.hpp"

#define DEG2RAD(x) ((x) * M_PI / 180.0
// TODO this should seriously be defined somewhere central. It's a macro used
// by a lot of source files

#define PI 3.1415 // TODO find a better and more centralized definition of pi

// Localizer class functions

Localizer::Localizer(std::vector<std::string> disabled_logs = {},
                     TYPE_DOG                 dog           = false)
{
    // Public attributes
    last_gps_fix = 0.0;

    // Private attributes
    kf                = LiveKalman(GENERATED_DIR);
    max_age           = 0.1; // seconds
    disabled_logs     = disabled_logs;
    calib             = Eigen::Vector3d::Zero();
    device_from_calib = Eigen::Matrix3d::Identity();
    calib_from_device = Eigen::Matrix3d::Identity();
    calibrated        = false;
    H                 = get_H();

    posenet_invalid_count = 0;
    posenet_speed = 0.0;
    car_speed = 0.0;
    posenet_stds = 10*Eigen::Matrix<TYPE_REAL,POSENET_STD_HIST,1>::Ones();

    converter = coord.LocalCoord.from_ecef(self.kf.x[States.ECEF_POS]); // python

    unix_timestamp_millis = 0.0;
    last_gps_fix          = 0.0;
    device_fell           = false;

    // Init actions
    reset_kalman();
}


// TODO implement
Localizer::~Localizer()
{
}


// TODO implement
static TYPE_FIX
Localizer::msg_from_state()
{
    // NOTE this function could likely use some helper functions to become more
    // readable
    predicted_std = std::sqrt(predicted_cov.diag()); // TODO check function compatibility

    ECEF fix_ecef        = predicted_state[States.ECEF_POS]; // TODO include type in decl./def.
    ECEF fix_ecef_std    = predicted_std[States.ECEF_POS_ERR]; // TODO include type in decl./def.
    ECEF vel_ecef_std    = predicted_std[States.ECEF_VELOCITY_ERR]; // TODO include type in decl./def.
    ECEF vel_ecef        = predicted_state[States.ECEF_VELOCITY]; // TODO include type in decl./def.
    Geodetic fix_pos_geo = ecef2geodetic(fix_ecef); // TODO implement, declare type
    //fix_pos_geo_std = np.abs(coord.ecef2geodetic(fix_ecef + fix_ecef_std) - fix_pos_geo)

    Eigen::Vector3d orientation_ecef            = quat2euler (predicted_state[State.ECEF_ORIENTATION]);
    Eigen::Vector3d orientation_ecef_std        = rot2euler  (predicted_state[States.ECEF_ORIENTATION_ERR]); // TODO implement, declare type
    Eigen::Matrix3d device_from_ecef            = quat2rot   (predicted_state[States.ECEF_ORIENTATION]).T();  // TODO implement, declare type
    Eigen::Vector3d calibrated_orientation_ecef = rot2euler  (calib_from_device.dot(device_from_ecef)); // TODO implement, declare type

    acc_calib         = calib_from_device.dot(predicted_state[States.ACCELERATION]); // TODO implement, type declare
    acc_calib_std     = std::sqrt(diagonal(dot(calib_from_device,dot(predicted_cov[States.ACCELERATION_ERR, States.ACCELERATION_ERR],transpose(calib_from_device))))); // TODO untangle, implement and type declare
    ang_vel_calib     = calib_from_device.dot(predicted_state[States.ANGULAR_VELOCITY]); // TODO implement, type declare
    ang_vel_calib_std = std::sqrt(diagonal(dot(calib_from_device, dot(predicted_cov[States.ANGULAR_VELOCITY_ERR, States.ANGULAR_VELOCITY_ERR],transpose(calib_from_device))))); // TODO untangle, implement and type declare

    LiveLocationKalman fix = messaging.log.LiveLocationKalman.new_message() // TODO implement
    return fix;
}


TYPE_FIX
Localizer::liveLocationMsg()
{
    TYPE_FIX fix = msg_from_state(converter,
                                  calib_from_device,
                                  H,  // TODO clue to get_H may lie here
                                  kf.x,
                                  kf.P); // TODO may be better to use a pointer instead
    TYPE_REAL old_mean = mean(posenet_stds[:POSENET_STD_HIST/2]); // TODO translate
    TYPE_REAL new_mean = mean(posenet_stds[POSENET_STD_HIST/2:]); // TODO translate
    bool std_spike = ((new_mean/old_mean > 4) && (new_mean > 7));

    fix.posenetOK = !(std_spike && (car_speed > 5));
    fix.deviceStable = !(device_fell);
    device_fell = false;

    // fix.gpsWeek = time.week
    // fix.gpsTimeOfWeek = time.tow
    fix.unixTimestampMillis = unix_timestamp_millis;

    if      ((fix.positionECEF.std.norm() < 50) && (calibrated))
        fix.status = "valid";
    else if  (fix.positionECEF.std.norm() < 50)
        fix.status = "uncalibrated";
    else
        fix.status = "uninitialized";

    return fix;
}

// TODO Overload this function to handle at least vectors and quaternions. Find
// out which other overloads it needs or make it support everything that
// k.predict_and_observe supports
void
Localizer::update_kalman(TYPE_TIME time,
                         TYPE_KIND kind,
                         TYPE_MEAS meas,
                         TYPE_R    R)
{
    try {
        kf.predict_and_observe(time, kind, meas, R); // TODO translate
    } catch () {
        cloudlog.error("Error in predict and observe, kalman reset"); // TODO translate
        reset_kalman();
    }
    return;
}

// TODO in prototype, TYPE_SOCK is used instead of TYPE_LOG, fix this
void
Localizer::handle_gps(TYPE_TIME  current_time,
                      TYPE_LOG  *log)
{
    if (log->flags % 2 == 0)
        return;

    last_gps_fix = current_time;

    converter = coord.LocalCoord.from_geodetic(log->latitude,
                                               log->longitude,
                                               log->altitude); // TODO implement

    Euler::Vector3d ecef_pos = converter.ned2ecef([0, 0, 0]); // TODO implement
    Euler::Vector3d ecef_vel = converter.ned2ecef(np.array(log->vNED)) - ecef_pos; // TODO translate
    Euler::Matrix3d ecef_pos_R = np.diag([(3*log->verticalAccuracy)**2]*3); // TODO translate
    Euler::Matrix3d ecef_vel_R = np.diag([(log->speedAccuracy)**2]*3); // TODO translate

    // self.time = GPSTime.from_datetime(datetime.utcfromtimestamp(log->timestamp*1e-3))
    unix_timestamp_millis = log->timestamp;
    TYPE_REAL gps_est_error = np.sqrt((self.kf.x[0] - ecef_pos[0])**2 +
                                      (self.kf.x[1] - ecef_pos[1])**2 +
                                      (self.kf.x[2] - ecef_pos[2])**2); // TODO translate

    Eigen::Vector3d orientation_ecef = quat2euler(self.kf.x[States.ECEF_ORIENTATION]);
    // common/transformations/orientation.py: euler_from_quat = quat2euler
    Eigen::Vector3d orientation_ned = ned_euler_from_ecef(ecef_pos, orientation_ecef);
    Eigen::Vector3d orientation_ned_gps (0, 0, DEG2RAD(log->bearing));
    TYPE_REAL orientation_error = std::fmod(orientation_ned - orientation_ned_gps - PI, 2*PI) - PI;
    // TODO the above may fail due to
    // - std::fmod being incapable of operating with Eigen objects
    // - Eigen objects not supporting having non-eigen scalars subtracted
    Euler::Quaterniond initial_pose_ecef_quat = euler2quat(ecef_euler_from_ned(ecef_pos, orientation_ned_gps));

    if ((ecef_vel.norm() > 5) && (orientation_error.norm() > 1)) {
      cloudlog.error("Locationd vs ubloxLocation orientation difference too large, kalman reset"); // TODO translate
      reset_kalman(ecef_pos, initial_pose_ecef_quat);
      update_kalman(current_time,
                    ObservationKind.ECEF_ORIENTATION_FROM_GPS,
                    initial_pose_ecef_quat); // TODO translate kind
    } else if (gps_est_error > 50) {
      cloudlog.error("Locationd vs ubloxLocation position difference too large, kalman reset") // TODO translate
      reset_kalman(ecef_pos, initial_pose_ecef_quat)
    }

    update_kalman(current_time, ObservationKind.ECEF_POS, ecef_pos, ecef_pos_R);
    update_kalman(current_time, ObservationKind.ECEF_VEL, ecef_vel, ecef_vel_R);

    return;
}


void
Localizer::handle_car_state(TYPE_TIME  current_time,
                            TYPE_LOG  *log)
{
    speed_counter += 1;
    if (speed_counter % SENSOR_DECIMATION == 0) {
        update_kalman(current_time, ObservationKind.ODOMETRIC_SPEED, log->vEgo);
        car_speed = std::abs(log->vEgo);
        if (log->vEgo == 0)
            update_kalman(current_time,
                          ObservationKind.NO_ROT,
                          Eigen::Vector3d::Zero());
    }
    return;
}


void
Localizer::handle_cam_odo(TYPE_TIME  current_time,
                          TYPE_LOG  *log)
{
    cam_counter += 1;

    if (cam_counter % VISION_DECIMATION == 0) {
        rot_device = device_from_calib.dot(log->rot);
        rot_device_std = device_from_calib.dot(log->rotStd);
        update_kalman(current_time,
                      ObservationKind.CAMERA_ODO_ROTATION,
                      np.concatenate([rot_device, 10*rot_device_std])); // TODO translate np.concatenate
        TYPE_UNK trans_device = device_from_calib.dot(log->trans);
        TYPE_UNK trans_device_std = device_from_calib.dot(log->transStd);
        // TODO TYPE_UNK could be a Matrix or a Vector
        posenet_speed = trans_device.norm();
        posenet_stds[:-1] = self.posenet_stds[1:];  // TODO translate
        posenet_stds[-1] = trans_device_std[0];     // TODO translate
        update_kalman(current_time,
                      ObservationKind.CAMERA_ODO_TRANSLATION,
                      np.concatenate([trans_device, 10*trans_device_std])); // TODO translate np.concatenate
    }
    return;
}


// TODO implement
void
Localizer::handle_sensors(TYPE_TIME  current_time,
                          TYPE_LOG  *log)
{
    // TODO does not yet account for double sensor readings in the log
    for    (TYPE_LOG::iterator sr; 
            sr = log->iterator.begin();
            sr != log->iterator.end() { // TODO verify iterator setup
        // TODO handle messages from IMUs at the same time  
        if (sr.source == SensorSource.lsm6ds3) {
            continue;
        // Gyro uncalibrated
        } else if ((sr.source == 5) && (sr.type == 16)) {
            gyro_counter += 1
        // Accelerometer
        } else if ((sr.source == 1) && (sr.type == 1)) {
            // Check if device fell, estimate 10 for g
            // 40 m/s² is a good filter for falling detection, no false
            // positives in 20 kmin of driving
            // ?? WHV: how does this make any sense physically? The acceleration
            // ?? due to gravity of a falling object is around 10 m/s². How do
            // ?? you go from that to 40 m/s²? 
            // ?? Also, how does the folowing calculation consider the likely
            // ?? change of orientation of the device as it falls? The vector
            // ?? substracted from the acceleration is of fixed orientation and
            // ?? magnitude. This part is very confusing and needs an
            // ?? explaining comment.
            Eigen::Vector3d acceldiff = sr.acceleration.v - Eigen::Vector3d(10, 0, 0);
            device_fell = (device_fell || (acceldiff.norm() > 40));

            acc_counter += 1;
            if (acc_counter % SENSOR_DECIMATION == 0) {
                Eigen::Vector3d v = sr.acceleration.v;
                update_kalman(current_time,
                              ObservationKind.PHONE_ACCEL, // TODO translate
                              Eigen::Vector3d(-v[2], -v[1], -v[0])); // TODO verify accesor works
            }

        }
    }
}

void
Localizer::handle_live_calib(TYPE_TIME   current_time,
                             TYPE_LOG   *log)
{
    if (log->rpyCalib.length() > 0) { // TODO translate
        calib = log->rpyCalib;
        device_from_calib = rot_from_euler(calib); // TODO implement
        calib_from_device = device_from_calib.T; // TODO translate
        calibrated = (log->calStatus == 1);
    }
}

void
Localizer::reset_kalman(TYPE_TIME           current_time,
                        Eigen::Quaterniond  init_orient,
                        Eigen::Vector3d     init_pos)
{
    filter_time = current_time;
    init_x = LiveKalman.initial_x.copy(); // TODO: translate

    if (init_orient is not None) { // TODO translate
        init_x[3:7] = init_orient;
    }
    if (init_pos is not None) { // TODO translate
        init_x[:3] = init_pos;
    }
    kf.init_state(init_x,
                  np.diag(LiveKalman.initial_P_diag),
                  current_time); // TODO translate

    observation_buffer = {};

    gyro_counter  = 0;
    acc_counter   = 0;
    speed_counter = 0;
    cam_counter   = 0;

    return;
}

void
Localizer::reset_kalman(Eigen::Quaterniond init_orient,
                        Eigen::Vector3d    init_pos)
{
    TYPE_TIME none_time = // TODO fix this
    Localizer::reset_kalman(none_time, init_orient, init_pos);
    return;
}

// Global functions
// to_float
// Receives an array of ??
// Returns a 3-member array of floats
//
// Usage locationd.py:141
// fix.positionGeodetic, fix_pos_geo,
// field                 value
// argument: value
// output: field.value
// field is a LiveLocationKalman.positionGeodetic (cereal log.capnp)
// LiveLocationKalman.positionGeodetic is a
// LiveLocationKalman.Measurement
// field.value is a LiveLocationKalman.Measurement.value
// LiveLocationKalman.Measurement.value is a list of float64
// Still not sure how to unpack this
std::vector<float>
to_float(float_list)
{
    std::vector<float> float_list;
    return float_list;
}


// TODO implement
TYPE_FUNCTION
get_H()
{
    // this returns a function to eval the jacobian
    // of the observation function of the local vel
    return H_f;
}

// function: location
void
locationd_thread(TYPE_MESSAGING_MASTER    *sm             = NULL,
                 TYPE_MESSAGING_MASTER    *pm             = NULL,
                 std::vector<std::string>  disabled_logs  = {})
{
    // Initialize sm and pm to none
    if (sm == NULL) {
        std::vector<std::string> socks = {"gpsLocationExternal",
                                          "sensorEvents",
                                          "cameraOdometry",
                                          "liveCalibration",
                                          "carState"};
        *sm = messaging.SubMaster(socks);
    }
    if (pm == NULL) {
        std::vector<std::string> socks = {"liveLocationKalman"}a;
        *pm = messaging.PubMaster(socks);
    }

    TYPE_PARAMS params = Params();
    Localizer localizer(disabled_logs);

    while true {
        sm->update();

        TYPE_SOCK sock;
        bool updated;
        for     (map<TYPE_SOCK, bool>::iterator ii;
                 ii = sm->updated.begin();
                 ii != sm->updated.end();
                 ++ii) {
            sock = ii->first;
            updated = ii->second;

            if (updated && sm->valid[sock]) {
                t = sm->logMonoTime[sock] * 1e-9;
                switch(sock) {
                case "sensorEvents":
                    localizer.handle_sensors    (t, sm[sock]);
                    break;
                case "gpsLocationExternal":
                    localizer.handle_gps        (t, sm[sock]);
                    break;
                case "carState":
                    localizer.handle_car_state  (t, sm[sock]);
                    break;
                case "cameraOdometry":
                    localizer.handle_cam_odo    (t, sm[sock]);
                    break;
                case "liveCalibration":
                    localizer.handle_live_calib (t, sm[sock]);
                    break;
                }
            }

        }

        if (sm->updated["cameraOdometry"]) {
            TYPE_TIME t = sm->logMonoTime["cameraOdometry"];
            msg = messaging::new_message("liveLocationKalman");
            msg.logMonoTime = t;

            msg.liveLocationKalman           = localizer.liveLocationMsg();
            msg.liveLocationKalman.inputsOK  = sm->all_alive_and_valid();
            msg.liveLocationKalman.sensorsOK = (   sm->alive["sensorEvents"]
                                                && sm->valid["sensorEvents"]);

            TYPE_REAL gps_age = (t / 1e9) - localizer.last_gps_fix;
            msg.liveLocationKalman.gpsOK = (gps_age < 1.0);
            pm->send("liveLocationKalman", msg);

            if      (   (sm->frame % 1200 == 0)
                     && (msg.liveLocationKalman.gpsOK)) {
                Json location = Json::object {
                    {"latitude",  msg.liveLocationKalman.positionGeodetic.value[0]},
                    {"longitude", msg.liveLocationKalman.positionGeodetic.value[1]},
                    {"altitude",  msg.liveLocationKalman.positionGeodetic.value[2]}
                };
                params.put("LastGPSPosition", location.dump());
            }
        }
    }

    return;
}

int
main(int argc, char *argv[])
{
    putenv("OMP_NUM_THREADS=1");
    locationd_thread();
    return 0;
}
