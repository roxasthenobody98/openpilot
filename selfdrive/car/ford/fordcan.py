from common.numpy_fast import clip
from selfdrive.car.ford.values import MAX_ANGLE


def create_steer_command(packer, enabled, apply_steer, curvature): # lkas_action, angle_cmd, angle_steers):
  """Creates a CAN message for the Ford Steer Command."""

  #if enabled and lkas available:
  #if enabled and lkas_state in [2,3]: #and (frame % 500) >= 3:
  #  action = lkas_action
  #else:
  #  action = 0xf
  #  angle_cmd = angle_steers/MAX_ANGLE

  #angle_cmd = clip(angle_cmd * MAX_ANGLE, - MAX_ANGLE, MAX_ANGLE)

  values = {
    "Steer_Angle_Req": apply_steer,
    #"Lkas_Action": action,
    "Lkas_Alert": 0xe,             # no alerts
    "Lane_Curvature": clip(curvature, -0.03, 0.03),   # is it just for debug?
    #"Lane_Curvature": 0,   # is it just for debug?
    #"Steer_Angle_Req": angle_cmd
  }
  return packer.make_can_msg("Lane_Keep_Assist_Control", 0, values)


def create_lkas_ui(packer, main_on, enabled, steer_alert, defog, ahbc, ahbcramping, config, noipma, ladeny, stats):
  """Creates a CAN message for the Ford Steer Ui."""

  if not main_on:
    lines = 0xf
  elif enabled:
    lines = 0x3
  else:
    lines = 0x6

  values = {
    "PersIndexIpma_D_Actl": 0x80,
    "DasStats_D_Dsply": 0x45,
    "Set_Me_X30": 0x30,
    "Lines_Hud": lines,
    "Hands_Warning_W_Chime": steer_alert,
    "CamraDefog_B_Req": defog,
    "AhbHiBeam_D_Rq": ahbc,
    "AhbcRampingV_D_Rq": ahbcramping,
    "FeatConfigIpmaActl": config,
    "FeatNoIpmaActl": noipma,
    "LaDenyStats_B_Dsply": ladeny,
    "CamraStats_D_Dsply": stats,
  }
  return packer.make_can_msg("Lane_Keep_Assist_Ui", 0, values)

def spam_cancel_button(packer):
  values = {
    "Cancel": 1
  }
  return packer.make_can_msg("Steering_Buttons", 0, values)
