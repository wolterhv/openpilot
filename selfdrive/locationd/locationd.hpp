#define _USE_MATH_DEFINES

#include <stdlib.h> // putenv
#include <math.h> // std::abs
#include <vector>
#include <string>
#include "json.hpp"

#include <eigen3/Eigen/Dense>
#include "common/transformations/orientation.hpp"
#include "common/transformations/coordinates.hpp"
// TODO include the capnp library
#include "models/constants.hpp"
#include "models/live_kf.hpp"
#include "cereal/messaging.hpp" // cereal messaging
#include "cereal/services.h" // service list
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/params.h"

// TBD types
// #define TYPE_DISABLED_LOGS vector<string>
// #define TYPE_DOG bool
// #define TYPE_REAL float
// #define TYPE_INTEGER int
// #define TYPE_ULONG unsigned long long
// #define NP_ARRAY vector<TYPE_REAL>

#define VISION_DECIMATION  2
#define SENSOR_DECIMATION  10
#define POSENET_STD_HIST   40

class Localizer
{
    Localizer();
    Localizer(TYPE_DISABLED_LOGS disabled_logs,
              TYPE_DOG           dog);

public:
    // Functions
    void handle_sensors    (TYPE_TIME t, TYPE_SOCK sock);
    void handle_gps        (TYPE_TIME t, TYPE_SOCK sock);
    void handle_car_state  (TYPE_TIME t, TYPE_SOCK sock);
    void handle_cam_odo    (TYPE_TIME t, TYPE_SOCK sock);
    void handle_live_calib (TYPE_TIME t, TYPE_SOCK sock);

    TYPE_MSG liveLocationMsg();

private:
    // Functions
    void reset_kalman();
    void reset_kalman(init_pos,
                      init_orient);
    void update_kalman(TYPE_TIME time,
                       kind,
                       meas,
                       R);
    static TYPE_FIX msg_from_state(converter,
                                   calib_from_device,
                                   H,
                                   predicted_state,
                                   predicted_cov):

    // Attributes
    LiveKalman           kf;
    TYPE_REAL            max_age;
    TYPE_DISABLED_LOGS   disabled_logs;
    Eigen::Vector3d      calib;
    Eigen::Matrix3d      device_from_calib;
    Eigen::Matrix3d      calib_from_device;
    bool                 calibrated;
    TYPE_FUNCTION        H;

    TYPE_INTEGER         posenet_invalid_count;
    TYPE_REAL            posenet_speed;
    TYPE_REAL            car_speed;
    Eigen::Matrix<TYPE_REAL,POSENET_STD_HIST,1> posenet_stds;

    LocalCoord           converter;

    int64                unix_timestamp_millis;
    double               last_gps_fix;
    bool                 device_fell;

    // Kalman
    TYPE_OBSBUF  observation_buffer;
    TYPE_TIME    filter_time;
    TYPE_COUNTER gyro_counter;
    TYPE_COUNTER acc_counter;
    TYPE_COUNTER speed_counter;
    TYPE_COUNTER cam_counter;
}
