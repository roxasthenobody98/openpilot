from cereal import car
import numpy as np
from common.numpy_fast import interp, clip
from selfdrive.car import make_can_msg
from selfdrive.car.ford.fordcan import create_steer_command, create_lkas_ui, spam_cancel_button
from opendbc.can.packer import CANPacker


MAX_STEER_DELTA = 1
TOGGLE_DEBUG = False
COUNTER_MAX = 7

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
    #self.lkasToggle = 1
    self.lastAngle = 0
    self.angleReq = 0
    self.sappConfig = 0
    self.sappChime = 0
    self.chimeCounter = 0
    self.sappConfig_last = 0
    self.angleReq_last = 0
    self.apaCounter = 0
    self.sappAction = 0
    self.eightysix = 0

  def update(self, enabled, CS, frame, actuators, visual_alert, pcm_cancel):

    can_sends = []
    steer_alert = visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired
    #if (frame % 50) == 0:
    #  if CS.out.genericToggle == 1:
    #    self.lkasToggle += 1
    #  if self.lkasToggle > 6:
    #    self.lkasToggle = 1
    apply_steer = actuators.steerAngle
    if self.enable_camera:
      if pcm_cancel:
       #print("CANCELING!!!!")
       can_sends.append(spam_cancel_button(self.packer))
      if (frame % 1) == 0:
        self.main_on_last = CS.out.cruiseState.available
      #SAPP Config Value Handshake
      if (frame % 2) == 0:
        if CS.out.vEgo >= 0:
          self.apaCounter += 1
          if self.sappConfig_last != 16:
            self.sappConfig = 70 #168 perpendicular
          #if self.apaCounter == 6:
          #  self.sappConfig = 168
          if CS.sappHandshake == 1 and self.apaCounter > 8:
            self.sappConfig = 86 #200 perpendicular
            self.eightysix += 1
            print("config 86")
          if CS.sappHandshake == 1 and self.apaCounter > 13 and self.sappConfig_last == 86:
            self.angleReq = 1
            print("angle 1")
          if self.sappConfig_last == 86 and self.eightysix == 20:
            self.apaCounter = 0
            self.eightysix = 0
          if CS.sappHandshake == 2 and self.sappConfig_last != 16: # and self.apaCounter in range (15,16):
            self.sappConfig = 224
            self.sappAction += 1
            print("config 224 angle 1")
          if CS.sappHandshake == 2 and self.sappAction >= 3 and self.sappConfig_last == 224:
            self.sappConfig = 16
            print("config 16 angle 1")
          if CS.sappHandshake == 3:
            self.sappConfig = 0
            self.apaCounter = 0
        self.sappConfig_last = self.sappConfig
        self.angleReq_last = self.angleReq
        #if CS.out.vEgo >= 1:
        #  self.apaCounter = 0 
        #  if CS.sappHandshake == 2 and self.sappConfig_last == 16:
        #    self.sappConfig = 16
        #    self.angleReq = 1
        print("Handshake:", CS.sappHandshake, "Config:", self.sappConfig_last, "Counter:", self.apaCounter, "AngleRequest:", self.angleReq, "fwdAction:", self.sappAction)
      #Stock IPMA Message is 33Hz. PSCM accepts commands at max 44Hz. 
        curvature = self.vehicle_model.calc_curvature(actuators.steerAngle*np.pi/180., CS.out.vEgo)
        self.lkas_action = 0 #6 Finished 5 NotAccessible 4 ApaCancelled 2 On 1 Off  
        if enabled:
          if self.lastAngle * apply_steer > 0.:
            angle_rate_lim = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_V)
          else:
            angle_rate_lim = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_VU)
          
          apply_steer = clip(apply_steer, self.lastAngle - angle_rate_lim, self.lastAngle + angle_rate_lim) 
        else:
          apply_steer = CS.out.steeringAngle
        self.lastAngle = apply_steer
        #print("action:", self.lkas_action, "toggle:", self.lkasToggle)
        #if self.lkasCounter < COUNTER_MAX:
        #  can_sends.append(create_steer_command(self.packer, apply_steer, enabled, CS.lkas_state, CS.out.steeringAngle, curvature, self.lkas_action))
        #else:
        #  self.lkasCounter = 0
        #  print("CAN Message successfully blocked for 1 message")
        #  pass
        can_sends.append(create_steer_command(self.packer, apply_steer, enabled, CS.out.steeringAngle, self.lkas_action, self.angleReq_last, self.sappConfig_last, self.sappChime))
        self.generic_toggle_last = CS.out.genericToggle
      if (frame % 1) == 0 or (self.enabled_last != enabled) or (self.main_on_last != CS.out.cruiseState.available) or (self.steer_alert_last != steer_alert):
        if steer_alert:
          steer_chime = 2
        else:
          steer_chime = 0
        can_sends.append(create_lkas_ui(self.packer, CS.out.cruiseState.available, enabled, steer_chime, CS.ipmaHeater, CS.ahbcCommanded, CS.ahbcRamping, CS.ipmaConfig, CS.ipmaNo, CS.ipmaStats))
        self.enabled_last = enabled                         
      self.steer_alert_last = steer_alert

    return can_sends
