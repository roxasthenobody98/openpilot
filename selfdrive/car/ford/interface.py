#!/usr/bin/env python3
from cereal import car
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
#from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.car.ford.values import MAX_ANGLE, CAR
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]): # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "ford"
    ret.communityFeature = True                              
    ret.safetyModel = car.CarParams.SafetyModel.ford
    ret.dashcamOnly = False
    ret.steerLimitAlert = False
    ret.enableCamera = True
    ret.steerRateCost = 0.5
    ret.steerActuatorDelay = 0.1
    ret.lateralTuning.pid.kf = 0.00006
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.0], [0.0]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.01], [0.005]]
    ret.steerMaxBP = [0.]  # m/s
    ret.steerMaxV = [1.]
    
    if candidate in [CAR.F150, CAR.F150SG]:
      ret.wheelbase = 3.68
      ret.steerRatio = 18.0
      ret.mass = 4770. * CV.LB_TO_KG + STD_CARGO_KG
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.5328
    if candidate == CAR.EDGE:
      ret.wheelbase = 3.00
      ret.steerRatio = 16.0
      ret.mass = 4100. * CV.LB_TO_KG + STD_CARGO_KG
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.5328
    elif candidate in [CAR.FUSION, CAR.FUSIONSG, CAR.MONDEO]:
      ret.wheelbase = 2.85
      ret.steerRatio = 14.8
      ret.mass = 3045. * CV.LB_TO_KG + STD_CARGO_KG
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.5328
    
    #INDI tuning TODO: Tune
    #ret.lateralTuning.init('indi')
    #ret.lateralTuning.indi.innerLoopGain = 1.0
    #ret.lateralTuning.indi.outerLoopGain = 1.0
    #ret.lateralTuning.indi.timeConstant = 1.0
    #ret.lateralTuning.indi.actuatorEffectiveness = 1.0
    #ret.steerActuatorDelay = 0.5


    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.steerControlType = car.CarParams.SteerControlType.angle

    ret.enableCamera = True
    cloudlog.warning("ECU Camera Simulated: %r", ret.enableCamera)

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)

    #ret = car.CarState.new_message()               
    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid
    ret.engineRPM = self.CS.engineRPM
    
    # events
    events = self.create_common_events(ret)
    
    if self.CS.lkas_state not in [2, 3] and ret.vEgo > 13.* CV.MPH_TO_MS and ret.cruiseState.enabled:
      events.add(car.CarEvent.EventName.steerTempUnavailableMute)
      print("PSCM LOCKOUT") 
      
    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):

    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators,
                               c.hudControl.visualAlert, c.cruiseControl.cancel)

    self.frame += 1
    return can_sends
