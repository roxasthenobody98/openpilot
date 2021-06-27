from cereal import car
from common.numpy_fast import interp, clip
from selfdrive.car import make_can_msg
from selfdrive.car.ford.fordcan import create_steer_command, create_speed_command, create_speed_command2, create_lkas_ui, create_accdata, create_accdata2, create_accdata3, spam_cancel_button
from selfdrive.car.ford.values import CAR, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV

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
    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
    self.lastAngle = 0
    self.angleReq = 0
    self.sappState = 0
    self.acc_decel_command = 0
    self.desiredSpeed = 20
    self.stopStat = 0
    self.steerAllowed = False
    self.apaCntr = 0
    
  def update(self, enabled, CS, frame, actuators, visual_alert, pcm_cancel, left_line, right_line, lead, left_lane_depart, right_lane_depart):
  
    frame_step = CarControllerParams.FRAME_STEP
    
    can_sends = []
    steer_alert = visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired
    apply_steer = actuators.steeringAngleDeg
    if self.enable_camera:
      if not self.steerAllowed:
        self.apaCntr = 0
      if enabled:
        self.steerAllowed = True
      else:
        self.steerAllowed = False
      if CS.epsAssistLimited:
        print("PSCM Assist Limited")
      #op Long (Buggy)
      if (frame % 2) == 0:
        if CS.CP.openpilotLongitudinalControl:
          brake, self.braking, self.brake_steady = actuator_hystereses(actuators.brake, self.braking, self.brake_steady, CS.out.vEgo, CS.CP.carFingerprint)
          self.brake_last = brake
          apply_gas = actuators.gas * 5
          apply_brake = self.brake_last * -20
          if apply_brake <= -0.08:
            self.acc_decel_command = 1
          else:
            self.acc_decel_command = 0
          print("Brake Actuator:", actuators.brake, "Gas Actuator:", actuators.gas, "Clipped Brake:", apply_brake, "Clipped Gas:", apply_gas)
          can_sends.append(create_accdata(self.packer, enabled, apply_gas, apply_brake, self.acc_decel_command, self.desiredSpeed, self.stopStat))
          can_sends.append(create_accdata2(self.packer, enabled, frame, 0, 0, 0, 0, 0))
          can_sends.append(create_accdata3(self.packer, enabled, 1, 3, lead, 2))
          self.apply_brake_last = apply_brake
      if pcm_cancel:
       #print("CANCELING!!!!")
        can_sends.append(spam_cancel_button(self.packer))
      if (frame % 1) == 0:
        if self.steerAllowed:
          self.apaCntr += 1
        self.main_on_last = CS.out.cruiseState.available
        #SAPP Handshake
      if (frame % 2) == 0:
        if CS.sappHandshake in [1,2]:
          if self.steerAllowed:
            self.sappState = 2
            self.angleReq = 1
          else:
            self.sappState = 1
            self.angleReq = 0
        else:
          self.sappState = 1
          self.angleReq = 0
        #Speed spoofy bois
        if self.steerAllowed:
          speed = 0
        else:
          speed = CS.vehSpeed
        can_sends.append(create_speed_command(self.packer, enabled, frame, speed, CS.out.gearShifter, frame_step))
        can_sends.append(create_speed_command2(self.packer, enabled, frame, speed, frame_step))
      #Angle Limits
      if (frame % 2) == 0:
        angle_lim = interp(CS.out.vEgo, CarControllerParams.ANGLE_MAX_BP, CarControllerParams.ANGLE_MAX_V)
        apply_steer = clip(apply_steer, -angle_lim, angle_lim)
        if self.steerAllowed:
          if self.lastAngle * apply_steer > 0. and abs(apply_steer) > abs(self.lastAngle):
            angle_rate_lim = interp(CS.out.vEgo, CarControllerParams.ANGLE_DELTA_BP, CarControllerParams.ANGLE_DELTA_V)
          else:
            angle_rate_lim = interp(CS.out.vEgo, CarControllerParams.ANGLE_DELTA_BP, CarControllerParams.ANGLE_DELTA_VU)
          
          apply_steer = clip(apply_steer, self.lastAngle - angle_rate_lim, self.lastAngle + angle_rate_lim) 
        else:
          apply_steer = CS.out.steeringAngleDeg
        self.lastAngle = apply_steer
        can_sends.append(create_steer_command(self.packer, apply_steer, enabled, self.sappState, self.angleReq))
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
          self.daschime = 0
        else:
          self.steer_chime = 0
          self.daschime = 0
        can_sends.append(create_lkas_ui(self.packer, CS.out.cruiseState.available, enabled, self.steer_chime, CS.ipmaHeater, CS.ahbcCommanded, CS.ahbcRamping, CS.ipmaConfig, CS.ipmaNo, CS.ipmaStats, CS.persipma, CS.dasdsply, CS.x30, self.daschime, lines))
        self.enabled_last = enabled                         
      self.steer_alert_last = steer_alert

    return can_sends
