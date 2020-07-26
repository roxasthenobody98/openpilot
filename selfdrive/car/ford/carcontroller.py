from cereal import car
from selfdrive.car import make_can_msg
from selfdrive.car.ford.fordcan import create_steer_command, create_lkas_ui, spam_cancel_button
from opendbc.can.packer import CANPacker
from selfdrive.car.ford.values import SteerLimitParams
from selfdrive.car import apply_std_steer_torque_limits

MAX_STEER_DELTA = 1
TOGGLE_DEBUG = False

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.packer = CANPacker(dbc_name)
    self.enable_camera = CP.enableCamera
    self.enabled_last = False
    self.main_on_last = False
    self.vehicle_model = VM
    self.generic_toggle_last = 0
    self.steer_alert_last = False
    self.apply_steer_last = 0
    #self.lkas_action = 0

  def update(self, enabled, CS, frame, actuators, visual_alert, pcm_cancel):

    can_sends = []


    ahbc = CS.ahbcCommanded
    defog = CS.ipmaHeater
    ahbcramping = CS.ahbcRamping
    config = CS.ipmaConfig
    noipma = CS.ipmaNo
    ladeny = CS.laDenyStat
    stats = CS.ipmaStats
    if self.enable_camera:

      if pcm_cancel:
       print("CANCELING!!!!")
       can_sends.append(spam_cancel_button(self.packer))

      if (frame % 3) == 0:
        lkas_enabled = enabled
        if lkas_enabled:
          new_steer = actuators.steer * SteerLimitParams.STEER_MAX
          steer_alert = visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired

          apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, SteerLimitParams)
          self.steer_rate_limited = new_steer != apply_steer #actuators.steer
          curvature = self.vehicle_model.calc_curvature(actuators.steerAngle*3.1415/180., CS.out.vEgo)
        else:
          apply_steer = 0

        # The use of the toggle below is handy for trying out the various LKAS modes
        #if TOGGLE_DEBUG:
        #  self.lkas_action += int(CS.out.genericToggle and not self.generic_toggle_last)
        #  self.lkas_action &= 0xf
        #else:
        #  self.lkas_action = 5   # 4 and 5 seem the best. 8 and 9 seem to aggressive and laggy

        can_sends.append(create_steer_command(self.packer, enabled, apply_steer, curvature)) #self.lkas_action , CS.lkas_state,, CS.out.steeringAngle))
        self.generic_toggle_last = CS.out.genericToggle

      if (frame % 100) == 0 or (self.enabled_last != enabled) or (self.main_on_last != CS.out.cruiseState.available) or \
         (self.steer_alert_last != steer_alert):
        can_sends.append(create_lkas_ui(self.packer, CS.out.cruiseState.available, enabled, steer_alert, defog, ahbc, ahbcramping, config, noipma, ladeny, stats))
        self.enabled_last = enabled                         
        self.main_on_last = CS.out.cruiseState.available
      self.steer_alert_last = steer_alert

    return can_sends
