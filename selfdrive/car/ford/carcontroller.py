from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.ford.fordcan import make_can_msg, create_steer_command, create_lkas_ui, \
                                       spam_cancel_button
from selfdrive.can.packer import CANPacker


MAX_STEER_DELTA = 1
TOGGLE_DEBUG = False

class CarController():
  def __init__(self, dbc_name, enable_camera, vehicle_model):
    self.packer = CANPacker(dbc_name)
    self.enable_camera = enable_camera
    self.enabled_last = False
    self.main_on_last = False
    self.vehicle_model = vehicle_model
    self.generic_toggle_last = 0
    self.steer_alert_last = False
    self.lkas_action = 0

  def update(self, enabled, CS, frame, actuators, visual_alert, pcm_cancel):

    can_sends = []
    steer_alert = visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired

    apply_steer = actuators.steer

    if self.enable_camera:

      if pcm_cancel:
        print("CANCELING!!!!")
        can_sends.append(spam_cancel_button(self.packer))

      if (frame % 3) == 0:

        curvature = self.vehicle_model.calc_curvature(actuators.steerAngle*CV.DEG_TO_RAD, CS.v_ego)

        # The use of the toggle below is handy for trying out the various LKAS modes
        if TOGGLE_DEBUG:
          self.lkas_action += int(CS.generic_toggle and not self.generic_toggle_last)
          self.lkas_action &= 0xf
        else:
          self.lkas_action = 5   # 4 and 5 seem the best. 8 and 9 seem to aggressive and laggy

        can_sends.append(create_steer_command(self.packer, apply_steer, enabled,
                                              CS.lkas_state, CS.angle_steers, curvature, self.lkas_action))
        self.generic_toggle_last = CS.generic_toggle

      if (frame % 100) == 0 or (self.enabled_last != enabled) or (self.main_on_last != CS.main_on) or \
         (self.steer_alert_last != steer_alert):
        can_sends.append(create_lkas_ui(self.packer, CS.main_on, enabled, steer_alert))

      self.enabled_last = enabled
      self.main_on_last = CS.main_on
      self.steer_alert_last = steer_alert

    return can_sends
