from common.numpy_fast import clip
from selfdrive.car.ford.values import MAX_ANGLE

def create_steer_command(packer, angle_cmd, enabled, angle_steers, lkas_action, angleReq, sappConfig, sappChime):
  """Creates a CAN message for the Ford Steer Command."""

  #if enabled and lkas available:
  if enabled: # and (frame % 500) >= 3:
    action = lkas_action
    angle_cmd = angle_steers/MAX_ANGLE
  else:
    action = 0xf
    angle_cmd = angle_steers/MAX_ANGLE

  angle_cmd = clip(angle_cmd * MAX_ANGLE, - MAX_ANGLE, MAX_ANGLE)

  values = {
    "ApaSys_D_Stat": action,
    "EPASExtAngleStatReq": angleReq,
    "ExtSteeringAngleReq2": angle_cmd,
    "SAPPStatusCoding": sappConfig,
    "ApaChime_D_Rq": sappChime,
  }
  return packer.make_can_msg("ParkAid_Data", 0, values)

#def create_lkas_status(packer, enabled, lkasState, steer_pressed, steer_error):
#  """Creates a CAN message for the Ford LKAS State"""
#  values = {
#    "LaActAvail_D_Actl": lkasState,
#    "LaHandsOff_B_Actl": steer_pressed,
#    "LaActDeny_B_Actl": steer_error,
#  }
#  return packer.make_can_msg("Lane_Keep_Assist_Status", 0, values)

def create_lkas_ui(packer, main_on, enabled, steer_alert, defog, ahbc, ahbcramping, config, noipma, stats):
  """Creates a CAN message for the Ford Steer Ui."""
  if enabled:
    lines = 0x6
  else:
    lines = 0xc

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
    "CamraStats_D_Dsply": stats,
  }
  return packer.make_can_msg("Lane_Keep_Assist_Ui", 0, values)

def spam_cancel_button(packer):
  values = {
    "Cancel": 1
  }
  return packer.make_can_msg("Steering_Buttons", 0, values)
