from cereal import car
import numpy as np
from selfdrive.config import Conversions as CV
from common.numpy_fast import interp, clip
from selfdrive.car.ford.fordcan import create_steer_command, create_lkas_ui, spam_cancel_button
from opendbc.can.packer import CANPacker


MAX_STEER_DELTA = 1
TOGGLE_DEBUG = False
COUNTER_MAX = 7

ANGLE_MAX_BP = [0., 36.]
ANGLE_MAX_V = [40., 15.]

ANGLE_DELTA_BP = [0., 5., 15.]
ANGLE_DELTA_V = [5., .8, .15]     #windup
ANGLE_DELTA_VU = [5., 3.5, 0.4] #unwind

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.packer = CANPacker(dbc_name)
    self.enable_camera = CP.enableCamera
    self.enabled_last = False
    self.main_on_last = False
    self.vehicle_model = VM
    self.generic_toggle_last = 0
    self.steer_alert_last = False
    self.lkas_action = 0
    self.lkasCounter = 0

  def update(self, enabled, CS, frame, actuators, visual_alert, pcm_cancel):

    can_sends = []
    steer_alert = visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired
    
    apply_steer = actuators.steerAngle
    if (frame % 100) == 0:
      if enabled:
        self.lkasCounter +=1 
      if not enabled:
        if self.enabled_last == True:
          self.lkasCounter = 0
    if self.enable_camera:
      if pcm_cancel:
       #print("CANCELING!!!!")
       can_sends.append(spam_cancel_button(self.packer))
      if (frame % 3) == 0:
      #Stock IPMA Message is 33Hz. PSCM accepts commands at 100hz. 
        curvature = self.vehicle_model.calc_curvature(actuators.steerAngle*np.pi/180., CS.out.vEgo)
        #if self.lkasCounter >= 7:
        #  self.lkas_action = 0
        #  self.lkasCounter = 0
        #else:
        self.lkas_action = 2   # 0-7 accepted. 2 and 4 have action. 
        angle_lim = interp(CS.out.vEgo, ANGLE_MAX_BP, ANGLE_MAX_V)
        apply_steer = clip(apply_steer, -angle_lim, angle_lim)
        print("LKAS State:", CS.lkas_state, "Current Angle:", CS.out.steeringAngle, "Desired Angle:", apply_steer)
        if enabled:
          if self.lastAngle * apply_steer > 0. and abs(apply_steer) > abs(self.lastAngle):
            angle_rate_lim = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_V)
          else:
            angle_rate_lim = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_VU)
          
          apply_steer = clip(apply_steer, self.lastAngle - angle_rate_lim, self.lastAngle + angle_rate_lim) * CV.DEG_TO_MRAD
        else:
          apply_steer = CS.out.steeringAngle
        self.lastAngle = apply_steer
        can_sends.append(create_steer_command(self.packer, apply_steer, enabled, CS.lkas_state, CS.out.steeringAngle, curvature, self.lkas_action))
        self.generic_toggle_last = CS.out.genericToggle
      if (frame % 1) == 0 or (self.enabled_last != enabled) or (self.main_on_last != CS.out.cruiseState.available) or (self.steer_alert_last != steer_alert):
        if steer_alert:
          steer_chime = 2
        else:
          steer_chime = 0
        can_sends.append(create_lkas_ui(self.packer, CS.out.cruiseState.available, enabled, steer_chime, CS.ipmaHeater, CS.ahbcCommanded, CS.ahbcRamping, CS.ipmaConfig, CS.ipmaNo, CS.ipmaStats, CS.persipma, CS.dasdsply, CS.x30))
        self.enabled_last = enabled  
        self.main_on_last = CS.out.cruiseState.available
      self.steer_alert_last = steer_alert

    return can_sends
