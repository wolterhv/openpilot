#include "locationd.hpp"

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

    unix_timestamp_millis = 0;
    last_gps_fix          = 0.0;
    device_fell           = false;

    // Init actions
    reset_kalman();
}

Localizer::~Localizer()
{
}

// Localizer::update_kalman
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

void
Localizer::reset_kalman(TYPE_TIME current_time,
                        TYPE_ORIENT init_orient,
                        TYPE_POS init_pos) 
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

TYPE_FUNCTION
get_H()
{
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
