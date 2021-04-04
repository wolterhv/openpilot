#!/usr/bin/env python3

"""
INTRO

The goal of this script is to provide testing for a C++ rewrite of the functions underlying the locationd Localizer update_kalman method. This method calls the predict_and_observe method from the LiveKalman class

A very rudimentary call graph is shown below:

    Call graph:
        selfdrive/locationd/models/live_kf.py
            predict_and_observe
                np.atleast_2d
                predict_and_update_odo_trans
                    np.zeros
                    np.diag
                    self.filter.predict_and_update_batch (EKF)
                predict_and_update_odo_rot
                    np.zeros
                    np.diag
                    self.filter.predict_and_update_batch (EKF)
                predict_and_update_odo_speed
                    np.array
                    np.zeros
                    np.diag
                    self.filter.predict_and_update_batch (EKF)
                self.filter.predict_and_update_batch (EKF)
                self.get_R
                    np.zeros
                    for loop for R[i,:,:] assignment
                np.linalg.norm
                self.filter.x assignment (EKF)

    predict_and_update_batch is "the main Kalman filter function"
    Call graph of _predict_and_update_batch
        self._predict (points to _predict_blas, a python-wrapped C++ function)
        self._update  (points to _update_blas, a python-wrapped C++ function)
        self.augment    (array-modifying python function)
        self.checkpoint (array-modifying python function)

TODO

x Determine algorithms to test
o Generate test input data
o Record performance of current model
o Develop tests
o Test new algorithms when ready

NOTES

* The speed, trans and rot parameters are all different measurements. Valid measurements, according to calls to the update_kalman function in locationd.py can be:
  * initial_pose_ecef_quat
  * ecef_pos
  * ecef_vel
  * vEgo
  * (see all possible measurements in the ObservationKind class)
  One check in the predict_and_observe function passes the measurement through the np.atleast_2d function, which reshapes data to have at least 2 dimensions)

The get_R function just fills a tensor with values from obs_noise, which is a dictionary with noise for several kinds of ObservationKind. For example, an initialized LiveKalman object lk satisfies

    lk.obs_noise[ObservationKind.ODOMETRIC_SPEED] = np.atleast_2d(0.2**2)

and

    lk.obs_noise[ObservationKind.PHONE_GYRO] = np.diag([0.025**2, 0.025**2, 0.025**2])

"""


import random

import unittest

from selfdrive.locationd.models.live_kf import ObservationKind
from selfdrive.locationd.models.live_kf    import LiveKalman

TIME_BASE_VAL = 3600
GENERATED_DIR = "somewhere" # path to where generated code is spit out

def get_random_time_value(self):
  # TODO find a solution to the issue described below
  # There may be an issue with the time value because the
  # predict_and_update_batch function actually runs some checks in the time,
  # and compares it with the
  # if time < self.filter_time
  # then fast forward is used
  # if time < self.rewind_t[0]
  # or time < self.rewind_t[-1] - self.max_rewind_age
  # then error
  return 3600*random.random()

class TestLiveKF(unittest.TestCase):

  def __init__(self):
    self.kf = LiveKalman(GENERATED_DIR)

  # NOTE: The predict_and_update_batch function is already partially written in
  # CC (loaded via EKF) and is part of the rednose  project, so I won't touch
  # it unless there is explicit interest for that

  def test_predict_and_observe(self):

    meas = None
    # TODO find something reasonable to use as "meas"
    t = get_random_time_value()
    kind = None
    # TODO find something reasonable to use as "kind". Is it necessary to loop
    # through all kinds, or just to use a kind which does not trigger
    # "predict_and_observe" calling a sibling function?

    self.kf._use_cc_rewrite = False
    r_py = self.kf.predict_and_observe(meas, t, kind)
    self.kf._use_cc_rewrite = True
    r_cc = self.kf.predict_and_observe(meas, t, kind)

    self.assertEqual(r_py, r_cc)

  def test_predict_and_update_odo_trans(self):
    # trans, t, kind
    # trans is np.concatenate([trans_device, 10*trans_device_std])
    # trans_device is self.device_from_calib.dot(log.trans)
    # log comes from cereal
    trans     = 10*np.random.random(3) # list of float32
    trans_std =    np.random.random(3) # list of float32

    device_from_calib = np.eye(3)
    trans_device     = device_from_calib.dot(trans)
    trans_device_std = device_from_calib.dot(trans_std)

    # meas defined as in locationd.py
    meas  = np.concatenate([trans_device, 10*trans_device_std])
    t    = random.random()*TIME_BASE_VAL
    kind = ObservationKind.CAMERA_ODO_ROTATION

    self.kf._use_cc_rewrite = False
    r_py = self.kf.predict_and_update_odo_trans(t, kind, meas, R=None)
    self.kf._use_cc_rewrite = True
    r_cc = self.kf.predict_and_update_odo_trans(t, kind, meas, R=None)

    self.assertEqual(r_py, r_cc)

  def test_predict_and_update_odo_rot(self):
    # meas, t, kind
    # These rot and rot std are taken from the log.capnp structure
    # rot is a list of float32 values, which have the rad/s unit
    # rotStd has the same form, corresponds to the standard deviation possibly
    # the @(number) is just a field reference
    device_from_calib = np.eye(3)
    rot               = np.random.random(3)
    rot_std           = np.random.random(3)

    # (the definition of meas is copied from locationd.py)
    rot_device     = device_from_calib.dot(rot)
    rot_device_std = device_from_calib.dot(rot_std)
    meas  = np.concatenate([rot_device, 10*rot_device_std])
    t    = random.random()*TIME_BASE_VAL
    kind = ObservationKind.CAMERA_ODO_ROTATION

    self.kf._use_cc_rewrite = False
    r_py = self.kf.predict_and_update_odo_rot(t, kind, meas, R=None)
    self.kf._use_cc_rewrite = True
    r_cc = self.kf.predict_and_update_odo_rot(t, kind, meas, R=None)

    self.assertEqual(r_py, r_cc)

  def test_predict_and_update_odo_speed(self):
    # meas, t, kind
    # In locationd.py, the provided meas parameter is "log.vEgo". From the
    # log.capnp source file, we can see that this is also a list of float32
    # values. Probably, each value corresponds to a component of a velocity
    # vector, in which case the list would be 3-items long. It would be a good
    # idea to be able to randomise realistic velocity vectors, but if only the
    # mathematical side of the function is of interest, we may as well just
    # feed random numbers in
    meas  = 30*np.random.random(3)
    t     = TIME_BASE_VAL*random.random()
    kind  = ObservationKind.ODOMETRIC_SPEED

    self.kf._use_cc_rewrite = False
    r_py = self.kf.predict_and_update_odo_speed(t, kind, meas, R=None)
    self.kf._use_cc_rewrite = True
    r_cc = self.kf.predict_and_update_odo_speed(t, kind, meas, R=None)

    self.assertEqual(r_py, r_cc)

  def test_get_R():
    # kind, n
    # R is a 3D tensor. Two of R's dimensions are equal, so R is kind of like a
    # prism with a square base. n is the length of this prism, the side of the
    # square base is taken from the obs_noise member of self.
    # In C++, R can be just a simple 3D array

    n = 5 # TODO answer: how significant can this parameter be?
    for kind in self.kf.obs_noise.keys():

      self.kf._use_cc_rewrite = False
      R_py = self.kf.get_R(kind, n)
      self.kf._use_cc_rewrite = True
      R_cc = self.kf.get_R(kind, n)

      self.assertEqual(R_py, R_cc)


if __name__ == "__main__":
  unittest.main()

