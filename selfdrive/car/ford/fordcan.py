from common.numpy_fast import clip
from selfdrive.car.ford.values import MAX_ANGLE
from cereal import car

def fordchecksum(cnt, speed):
  # Checksum is 256 - cnt - speed - 4 with bitwise shifting and rounding on the speed.
  speed = int(round(speed / 0.01, 2))
  top = speed >> 8
  bottom = speed & 0xff
  cs = 256 - cnt - top - bottom - 4
  if cs < 0:
    cs = cs + 256
  return cs

def create_steer_command(packer, angle_cmd, enabled, action, angleReq):
  """Creates a CAN message for the Ford Steer Command."""
  
  values = {
    "ApaSys_D_Stat": action,
    "EPASExtAngleStatReq": angleReq,
    "ExtSteeringAngleReq2": angle_cmd,
  }
  return packer.make_can_msg("ParkAid_Data", 2, values)

def create_speed_command(packer, enabled, frame, speed, gear, frame_step):
  """Creates a CAN message for the Ford Speed Command."""
    
  if gear == 1:
    reverse = 3
    trailer = 1
  else:
    reverse = 1
    trailer = 0
    
  cnt = frame % frame_step
  cs = fordchecksum(cnt, speed)
  
  values = {
    "VehVTrlrAid_B_Avail": trailer,
    "VehVActlEng_No_Cs": cs,
    "VehVActlEng_No_Cnt": cnt,
    "VehVActlEng_D_Qf": 3,
    "GearRvrse_D_Actl": reverse,
    "Veh_V_ActlEng": speed,
  }
  return packer.make_can_msg("EngVehicleSpThrottle2", 2, values)

def create_speed_command2(packer, enabled, frame, speed, frame_step):
  """Creates a CAN message for the Ford Speed Command."""
  cnt = frame % frame_step
  cs = fordchecksum(cnt, speed)
  
  values = {
    "Veh_V_ActlBrk": speed,
    "LsmcBrkDecel_D_Stat": 4,
    "VehVActlBrk_No_Cs": cs,
    "VehVActlBrk_No_Cnt": cnt,
    "VehVActlBrk_D_Qf": 3,
  }
  return packer.make_can_msg("BrakeSysFeatures", 2, values)

def create_lkas_ui(packer, main_on, enabled, steer_alert, defog, ahbc, ahbcramping, config, noipma, stats, persipma, dasdsply, x30, daschime, lines):
  """Creates a CAN message for the Ford Steer Ui."""

  values = {
    "PersIndexIpma_D_Actl": persipma,
    "DasStats_D_Dsply": dasdsply,
    "Set_Me_X30": x30,
    "Lines_Hud": lines,
    "Hands_Warning_W_Chime": steer_alert,
    "CamraDefog_B_Req": defog,
    "AhbHiBeam_D_Rq": ahbc,
    "AhbcRampingV_D_Rq": ahbcramping,
    "FeatConfigIpmaActl": config,
    "FeatNoIpmaActl": noipma,
    "CamraStats_D_Dsply": stats,
    "DasWarn_D_Dsply": daschime,
  }
  return packer.make_can_msg("Lane_Keep_Assist_Ui", 0, values)

def create_accdata(packer, enabled, acc_gas, acc_brk, acc_decel, acc_spd, stopstat):
  """Creates a CAN message for ACCDATA"""
  
  values = {
    "AccBrkTot_A_Rq": acc_brk,
    "AccPrpl_A_Pred": acc_gas, 
    "AccPrpl_A_Rq": acc_gas, 
    "AccVeh_V_Trg": acc_spd, 
    "AccBrkPrchg_B_Rq": acc_decel, 
    "AccBrkDecel_B_Rq": acc_decel,
    "Cmbb_B_Enbl": 1,
    "AccStopStat_B_Rq": stopstat,
  }
  return packer.make_can_msg("ACCDATA", 0, values)

def create_accdata2(packer, enabled, frame_step, fcwhud_1, fcwhud_2, fcwhud_3, hud_intensity, flash_rate):
  """Creates a CAN message for ACCDATA_2"""
  csum = 0
  values = {
    "CmbbBrkDecel_No_Cs": csum, 
    "CmbbBrkDecel_No_Cnt": frame_step, 
    "HudBlk1_B_Rq": fcwhud_1,
    "HudBlk2_B_Rq": fcwhud_2,
    "HudBlk3_B_Rq": fcwhud_3,
    "HudDsplyIntns_No_Actl": hud_intensity, 
    "HudFlashRate_D_Actl": flash_rate,
  }
  return packer.make_can_msg("ACCDATA_2", 0, values)

def create_accdata3(packer, enabled, fcw_status, fcw_sensitivity, chevrons, gap):
  """Creates a CAN message for ACCDATA_3"""
  
  values = {
    "FdaMem_B_Stat": 1, 
    "AccMemEnbl_B_RqDrv": 1,
    "FcwMemStat_B_Actl": fcw_status,
    "FcwMemSens_D_Actl": fcw_sensitivity, 
    "AccFllwMde_B_Dsply": chevrons, 
    "AccTGap_B_Dsply": gap
  }
  return packer.make_can_msg("ACCDATA_3", 0, values)

def spam_cancel_button(packer):
  values = {
    "Cancel": 1
  }
  return packer.make_can_msg("Steering_Buttons", 0, values)
