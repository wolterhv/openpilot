// GENERATED_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), 'generated'))
#define GENERATED_DIR "./generated"
// How to do this in C++?

enum _ObservationKind {
  UNKNOWN                   =  0;
  NO_OBSERVATION            =  1;
  GPS_NED                   =  2;
  ODOMETRIC_SPEED           =  3;
  PHONE_GYRO                =  4;
  GPS_VEL                   =  5;
  PSEUDORANGE_GPS           =  6;
  PSEUDORANGE_RATE_GPS      =  7;
  SPEED                     =  8;
  NO_ROT                    =  9;
  PHONE_ACCEL               = 10;
  ORB_POINT                 = 11;
  ECEF_POS                  = 12;
  CAMERA_ODO_TRANSLATION    = 13;
  CAMERA_ODO_ROTATION       = 14;
  ORB_FEATURES              = 15;
  MSCKF_TEST                = 16;
  FEATURE_TRACK_TEST        = 17;
  LANE_PT                   = 18;
  IMU_FRAME                 = 19;
  PSEUDORANGE_GLONASS       = 20;
  PSEUDORANGE_RATE_GLONASS  = 21;
  PSEUDORANGE               = 22;
  PSEUDORANGE_RATE          = 23;
  ECEF_VEL                  = 31;
  ECEF_ORIENTATION_FROM_GPS = 32;
                                 
  ROAD_FRAME_XY_SPEED       = 24;  // (x, y) [m/s]
  ROAD_FRAME_YAW_RATE       = 25;  // [rad/s]
  STEER_ANGLE               = 26;  // [rad]
  ANGLE_OFFSET_FAST         = 27;  // [rad]
  STIFFNESS                 = 28;  // [-]
  STEER_RATIO               = 29;  // [-]
  ROAD_FRAME_X_SPEED        = 30;  // (x) [m/s]
} ObservationKind;

// There is a list of names for the previous constants, a function that is
// supposed to associate the names to the constants and a subset of the
// constants under the name SAT_OBS. None of that is used and it seems
// outdated, because the names and the constants are not correctly mapped.
// Because of the former, those elements have been omitted in this translation.
// If they are to be incorporated, base it on the python version of this file.
