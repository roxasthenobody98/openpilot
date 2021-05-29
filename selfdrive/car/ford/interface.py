#!/usr/bin/env python3
from cereal import car
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
#from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.car.ford.values import MAX_ANGLE, CAR
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from common.op_params import opParams

op_params = opParams()
apaAcknowledge = op_params.get('apaAcknowledge')

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
    
    if candidate in [CAR.F150, CAR.F150SG]:
      ret.wheelbase = 3.68
      ret.steerRatio = 18.0
      ret.mass = 4770. * CV.LB_TO_KG + STD_CARGO_KG
      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.innerLoopGainBP = [0.]
      ret.lateralTuning.indi.innerLoopGainV = [4.0]
      ret.lateralTuning.indi.outerLoopGainBP = [0.]
      ret.lateralTuning.indi.outerLoopGainV = [3.5]
      ret.lateralTuning.indi.timeConstantBP = [0.]
      ret.lateralTuning.indi.timeConstantV = [2.0]
      ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
      ret.lateralTuning.indi.actuatorEffectivenessV = [1.0]
      ret.steerActuatorDelay = 0.3
      ret.steerLimitTimer = 0.8
      ret.steerRateCost = 1.0
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.5328
    elif candidate == CAR.TRANSIT:
      ret.wheelbase = 3.04
      ret.steerRatio = 14.8
      ret.mass = 3900. * CV.LB_TO_KG + STD_CARGO_KG
      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.innerLoopGainBP = [0.]
      ret.lateralTuning.indi.innerLoopGainV = [4.0]
      ret.lateralTuning.indi.outerLoopGainBP = [0.]
      ret.lateralTuning.indi.outerLoopGainV = [3.5]
      ret.lateralTuning.indi.timeConstantBP = [0.]
      ret.lateralTuning.indi.timeConstantV = [2.0]
      ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
      ret.lateralTuning.indi.actuatorEffectivenessV = [1.0]
      ret.steerActuatorDelay = 0.3
      ret.steerLimitTimer = 0.8
      ret.steerRateCost = 1.0
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.5328
    elif candidate in [CAR.FUSION, CAR.FUSIONSG, CAR.MONDEO]:
      ret.wheelbase = 2.85
      ret.steerRatio = 14.8
      ret.mass = 3045. * CV.LB_TO_KG + STD_CARGO_KG
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.01], [0.005]]     # TODO: tune this
      ret.lateralTuning.pid.kf = 1. / MAX_ANGLE   # MAX Steer angle to normalize FF
      ret.steerActuatorDelay = 0.1  # Default delay, not measured yet
      ret.steerLimitTimer = 0.8
      ret.steerRateCost = 1.0
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
    if not apaAcknowledge:
      events.add(car.CarEvent.EventName.apaNotAcknowledged)
    if self.CC.enabled_last:
      #if self.CS.sappHandshake != 2 and self.CC.sappConfig_last != 16:
      #  events.add(car.CarEvent.EventName.pscmHandshaking)
      if self.CS.sappHandshake == 2 and self.CC.sappConfig_last == 224:
        events.add(car.CarEvent.EventName.pscmHandshaked)
      if self.CS.sappHandshake == 3 and self.CC.sappConfig_last in [16, 224]:
        events.add(car.CarEvent.EventName.pscmLostHandshake)
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
