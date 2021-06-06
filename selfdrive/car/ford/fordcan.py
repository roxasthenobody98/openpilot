from common.numpy_fast import clip
from selfdrive.car.ford.values import MAX_ANGLE

def create_steer_command(packer, angle_cmd, enabled, angle_steers, action, angleReq, sappConfig, sappChime):
  """Creates a CAN message for the Ford Steer Command."""

  values = {
    "ApaSys_D_Stat": action,
    "EPASExtAngleStatReq": angleReq,
    "ExtSteeringAngleReq2": angle_cmd,
    "SAPPStatusCoding": sappConfig,
    "ApaChime_D_Rq": sappChime,
  }
  return packer.make_can_msg("ParkAid_Data", 2, values)

def create_ds_118(packer, filler1, filler2, filler3, brakectr, awdlckmax, awdlckmn, drvstate, drvtq, emergbrk, stoplmp, angle):
  """Creates a CAN message for the Ford 118 message."""

  values = {
    "BrkCtrFnd_B_Stat":  brakectr,
    "AwdLck_Tq_RqMx": awdlckmax,
    "AwdLck_Tq_RqMn": awdlckmn,
    "DrvSte_D_Stat": drvstate,
    "DrvSte_Tq_Rq": drvtq,
    "EmgcyBrkLamp_D_Rq": emergbrk,
    "StopLamp_B_RqBrk": stoplmp,
    "SteWhlRelInit_An_Sns": angle,
    "DS_Filler_1": filler1,
    "DS_Filler_2": filler2,
    "DS_Filler_3": filler3,
  }
  return packer.make_can_msg("BrakeSnData_5", 2, values)

def create_speed_command(packer, enabled, frame, speed, trlraid, actlnocs, actlnocnt, actlqf, gear, frame_step):
  """Creates a CAN message for the Ford Speed Command."""
  #Checksum is similar to mazda. Start with 249, then subtract the messages per line, then add 10. Bitwise shifting will be needed in order to specify a speed, otherwise if it is 0, then no action is needed. 
  cnt = frame/frame_step % 16 #message only sends odd numbered counters for some weird reason. this might work. not in sync with 1045. 
  csum_a = 249 - gear - trlraid - cnt - actlqf -  speed
  csum_a = csum_a + 10
  cksum_a = csum_a
  #if enabled:
  #  cksum_a = csum_a
  #  cnt = frame/frame_step % 16
  #else:
  #  cksum_a = actlnocs
  #  cnt = actlnocnt
     
  values = {
    "VehVTrlrAid_B_Avail": trlraid,
    "VehVActlEng_No_Cs": cksum_a,
    "VehVActlEng_No_Cnt": cnt,
    "VehVActlEng_D_Qf": actlqf,
    "GearRvrse_D_Actl": gear,
    "Veh_V_ActlEng": speed,
  }
  return packer.make_can_msg("EngVehicleSpThrottle2", 2, values)

def create_speed_command2(packer, enabled, frame, speed2, lsmcdecel, actlbrknocs, actlbrknocnt, actlbrkqf, brklvl, vehstab, frame_step):
  """Creates a CAN message for the Ford Speed Command."""
  #Checksum is similar to mazda. Start with 249, then subtract the messages per line, then add 10. Bitwise shifting will be needed in order to specify a speed, otherwise if it is 0, then no action is needed. 
  cnt2 = frame/frame_step % 16
  csum_b = 249 - speed2 - actlbrkqf - cnt2 - brklvl - lsmcdecel - vehstab
  csum_b = csum_b + 10
  cksum_b = csum_b
  print("cnt2:", cnt2)
  #if enabled:
  #  cksum_a = csum_a
  #  cnt2 = frame/frame_step % 16
  #else:
  #  cksum_a = actlnocs
  #  cnt2 = actlnocnt
  values = {
    "Veh_V_ActlBrk": speed2,
    "LsmcBrkDecel_D_Stat": lsmcdecel,
    "VehVActlBrk_No_Cs": cksum_b,
    "VehVActlBrk_No_Cnt": cnt2,
    "VehVActlBrk_D_Qf": actlbrkqf,
    "BrkFluidLvl_D_Stat": brklvl,
    "VehStab_D_Stat": vehstab,
  }
  return packer.make_can_msg("BrakeSysFeatures", 2, values)

def create_lkas_ui(packer, main_on, enabled, steer_alert, defog, ahbc, ahbcramping, config, noipma, stats, persipma, dasdsply, x30, daschime):
  """Creates a CAN message for the Ford Steer Ui."""
  if enabled:
    lines = 0x6
  else:
    lines = 0xc

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

def create_accdata(packer, enabled, acc_gas, acc_brk, acc_decel, acc_spd, stopstat)
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

def create_accdata2(packer, enabled, frame_step, fcwhud_1, fcwhud_2, fcwhud_3, hud_intensity, flash_rate)
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

def create_accdata3(packer, enabled, fcw_status, fcw_sensitivity, chevrons, gap)
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
