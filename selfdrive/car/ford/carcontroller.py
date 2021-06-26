from cereal import car
from common.numpy_fast import interp, clip
from selfdrive.car import make_can_msg
from selfdrive.car.ford.fordcan import create_steer_command, create_speed_command, create_speed_command2, create_ds_118, create_lkas_ui, create_accdata, create_accdata2, create_accdata3, spam_cancel_button
from selfdrive.car.ford.values import CAR, CarControllerParams
from opendbc.can.packer import CANPacker

MAX_STEER_DELTA = 0.2
TOGGLE_DEBUG = False
COUNTER_MAX = 7

def actuator_hystereses(brake, braking, brake_steady, v_ego, car_fingerprint):
  # hyst params
  brake_hyst_on = 0.02     # to activate brakes exceed this value
  brake_hyst_off = 0.005                     # to deactivate brakes below this value
  brake_hyst_gap = 0.01                      # don't change brake command for small oscillations within this value

  #*** hysteresis logic to avoid brake blinking. go above 0.1 to trigger
  if (brake < brake_hyst_on and not braking) or brake < brake_hyst_off:
    brake = 0.
  braking = brake > 0.

  # for small brake oscillations within brake_hyst_gap, don't change the brake command
  if brake == 0.:
    brake_steady = 0.
  elif brake > brake_steady + brake_hyst_gap:
    brake_steady = brake - brake_hyst_gap
  elif brake < brake_steady - brake_hyst_gap:
    brake_steady = brake + brake_hyst_gap
  brake = brake_steady

  return brake, braking, brake_steady

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
    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
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
    self.acc_decel_command = 0
    self.desiredSpeed = 20
    self.stopStat = 0
    self.alwaysTrue = True   
    
  def update(self, enabled, CS, frame, actuators, visual_alert, pcm_cancel, left_line, right_line, lead, left_lane_depart, right_lane_depart):
  
    frame_step = CarControllerParams.FRAME_STEP
    
    can_sends = []
    steer_alert = visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired
    apply_steer = actuators.steeringAngleDeg
    if self.enable_camera:
      if CS.epsAssistLimited:
        print("PSCM Assist Limited")
      if (frame % 2) == 0:
        if CS.CP.openpilotLongitudinalControl:
          brake, self.braking, self.brake_steady = actuator_hystereses(actuators.brake, self.braking, self.brake_steady, CS.out.vEgo, CS.CP.carFingerprint)
          apply_gas = clip(actuators.gas, 0., 1.)
          apply_brake = int(clip(self.brake_last * CarControllerParams.BRAKE_MAX, 0, CarControllerParams.BRAKE_MAX - 1))
          if apply_brake <= 0.04:
            self.acc_decel_command = 1
          else:
            self.acc_decel_command = 0
          can_sends.append(create_accdata(self.packer, enabled, apply_gas, apply_brake, self.acc_decel_command, self.desiredSpeed, self.stopStat))
          can_sends.append(create_accdata2(self.packer, enabled, frame, 0, 0, 0, 0, 0))
          can_sends.append(create_accdata3(self.packer, enabled, 1, 3, lead, 2))
          self.apply_brake_last = apply_brake
        if self.alwaysTrue == True:
          self.actlnocs = 0
          self.actlbrknocs = 0
          self.speed = 0
          self.drvstate = 6
          can_sends.append(create_ds_118(self.packer,  CS.filler1, CS.filler2, CS.filler3, CS.brakectr, CS.awdlckmax, CS.awdlckmn, self.drvstate, CS.drvtq, CS.emergbrk, CS.stoplmp, CS.angle))
          can_sends.append(create_speed_command(self.packer, enabled, frame, self.speed, CS.trlraid, self.actlnocs, CS.actlnocnt, CS.actlqf, CS.epsgear, frame_step))
          can_sends.append(create_speed_command2(self.packer, enabled, frame, self.speed, CS.lsmcdecel, self.actlbrknocs, CS.actlbrknocnt, CS.actlbrkqf, CS.brkfld, CS.stab_stat, frame_step))
        else:
          can_sends.append(create_ds_118(self.packer,  CS.filler1, CS.filler2, CS.filler3, CS.brakectr, CS.awdlckmax, CS.awdlckmn, CS.drvstate, CS.drvtq, CS.emergbrk, CS.stoplmp, CS.angle))
          can_sends.append(create_speed_command(self.packer, enabled, frame, CS.vehSpeed, CS.trlraid, CS.actlnocs, CS.actlnocnt, CS.actlqf, CS.epsgear, frame_step))
          can_sends.append(create_speed_command2(self.packer, enabled, frame, CS.vehSpeed2, CS.lsmcdecel, CS.actlbrknocs, CS.actlbrknocnt, CS.actlbrkqf, CS.brkfld, CS.stab_stat, frame_step))
      if pcm_cancel:
       #print("CANCELING!!!!")
        can_sends.append(spam_cancel_button(self.packer))
      if (frame % 1) == 0:
        self.main_on_last = CS.out.cruiseState.available
      #SAPP Config Value Handshake
      if (frame % 2) == 0:
        if not enabled:
          self.apaCounter = 0
          self.eightysix = 0
          self.angleReq = 0
          self.sappAction = 0
        if enabled:
          self.apaCounter += 1 #Increment counter 
          #Sets config to base value when init handshake
          if CS.sappHandshake == 0 and self.sappConfig_last not in [16, 86, 224] :
            self.sappConfig = 70
          #waits for the pscm to respond, and waits 8 frames as well. sets config to response
          if CS.sappHandshake == 1 and self.apaCounter > 8:
            self.sappConfig = 86
            self.eightysix += 1
          #waits 5 frames then sends the angle request
          if CS.sappHandshake == 1 and self.apaCounter > 13 and self.sappConfig_last == 86:
            self.angleReq = 1
          #when 20 frames have passed at response config, values are cleared and angle request is held
          if self.sappConfig_last == 86 and self.eightysix == 20:
            self.apaCounter = 0
            self.eightysix = 0
            self.angleReq = 1
          #pscm responds to handshake. config is set to parallel action. 
          if CS.sappHandshake == 2 and self.sappConfig_last != 16: # and self.apaCounter in range (15,16):
            self.sappConfig = 224
            self.angleReq = 1
            self.sappAction += 1
          #once action is held for 3 frames, final response is sent. pscm is handshaken
          if CS.sappHandshake == 2 and self.sappAction >= 3 and self.sappConfig_last == 224:
            self.sappConfig = 16
            self.angleReq = 1
          #if pscm faults, values reset to retry. 
          if CS.sappHandshake == 3:
            self.sappConfig = 0
            self.apaCounter = 0
            self.angleReq = 0
        self.sappConfig_last = self.sappConfig
        self.angleReq_last = self.angleReq
        print("Handshake:", CS.sappHandshake, "Config:", self.sappConfig_last, "Desired Angle:", apply_steer, "Curr Angle:", CS.out.steeringAngleDeg) # "Counter:", self.apaCounter, "AngleRequest:", self.angleReq, "fwdAction:", self.sappAction)
        self.lkas_action = 0 #6 Finished 5 NotAccessible 4 ApaCancelled 2 On 1 Off  
        angle_lim = interp(CS.out.vEgo, CarControllerParams.ANGLE_MAX_BP, CarControllerParams.ANGLE_MAX_V)
        apply_steer = clip(apply_steer, -angle_lim, angle_lim)
        if enabled:
          if self.lastAngle * apply_steer > 0. and abs(apply_steer) > abs(self.lastAngle):
            angle_rate_lim = interp(CS.out.vEgo, CarControllerParams.ANGLE_DELTA_BP, CarControllerParams.ANGLE_DELTA_V)
          else:
            angle_rate_lim = interp(CS.out.vEgo, CarControllerParams.ANGLE_DELTA_BP, CarControllerParams.ANGLE_DELTA_VU)
          
          apply_steer = clip(apply_steer, self.lastAngle - angle_rate_lim, self.lastAngle + angle_rate_lim) 
        else:
          apply_steer = CS.out.steeringAngleDeg
        self.lastAngle = apply_steer
        can_sends.append(create_steer_command(self.packer, apply_steer, enabled, CS.out.steeringAngleDeg, self.lkas_action, self.angleReq_last, self.sappConfig_last, self.sappChime))
        self.generic_toggle_last = CS.out.genericToggle
      if (frame % 1) == 0 or (self.enabled_last != enabled) or (self.main_on_last != CS.out.cruiseState.available) or (self.steer_alert_last != steer_alert):
        lines = 0
        if left_line and right_line:
          if left_lane_depart:
            lines = 9
          elif right_lane_depart:
            lines = 21
          else:
            lines = 6
        elif left_line and not right_line:
          if left_lane_depart:
            lines = 14
          else:
            lines = 11
        elif right_line and not left_line:
          if right_lane_depart:
            lines = 22
          else:
            lines = 7
        else:
          lines = 12  
                
        if steer_alert:
          self.steer_chime = 1
          self.daschime = 2
        else:
          self.steer_chime = 0
          self.daschime = 0
        can_sends.append(create_lkas_ui(self.packer, CS.out.cruiseState.available, enabled, self.steer_chime, CS.ipmaHeater, CS.ahbcCommanded, CS.ahbcRamping, CS.ipmaConfig, CS.ipmaNo, CS.ipmaStats, CS.persipma, CS.dasdsply, CS.x30, self.daschime, lines))
        self.enabled_last = enabled                         
      self.steer_alert_last = steer_alert

    return can_sends
