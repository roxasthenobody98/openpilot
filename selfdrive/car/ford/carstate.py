from cereal import car
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.ford.values import DBC, SPEED_FACTOR

 #WHEEL_RADIUS = 0.33
GearShifter = car.CarState.GearShifter
  
class CarState(CarStateBase):
  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()
    speed_factor = SPEED_FACTOR[self.CP.carFingerprint]
    #ret.wheelSpeeds.rr = cp.vl["WheelSpeed"]['WhlRr_W_Meas'] * CV.MPH_TO_MS
    #ret.wheelSpeeds.rl = cp.vl["WheelSpeed"]['WhlRl_W_Meas'] * CV.MPH_TO_MS
    #ret.wheelSpeeds.fr = cp.vl["WheelSpeed"]['WhlFr_W_Meas'] * CV.MPH_TO_MS
    #ret.wheelSpeeds.fl = cp.vl["WheelSpeed"]['WhlFl_W_Meas'] * CV.MPH_TO_MS
    ret.vEgoRaw = cp.vl["BrakeSysFeatures"]['Veh_V_ActlBrk'] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0.001
    ret.steeringAngleDeg = cp.vl["BrakeSnData_5"]['SteWhlRelInit_An_Sns']
    ret.steeringPressed = cp_cam.vl["Lane_Keep_Assist_Status"]['LaHandsOff_B_Actl'] == 0
    ret.steerError = cp_cam.vl["Lane_Keep_Assist_Status"]['LaActDeny_B_Actl'] == 1
    ret.cruiseState.speed = cp.vl["Cruise_Status"]['Set_Speed'] * CV.MPH_TO_MS
    ret.cruiseState.enabled = not (cp.vl["Cruise_Status"]['Cruise_State'] in [0, 3])
    ret.cruiseState.available = cp.vl["Cruise_Status"]['Cruise_State'] != 0
    ret.gas = cp.vl["EngineData_14"]['ApedPosScal_Pc_Actl'] / 100.
    ret.gasPressed = ret.gas > 1e-6
    ret.brakePressed = cp.vl["Cruise_Status"]['Brake_Drv_Appl'] == 2
    ret.brakeLights = bool(cp.vl["BCM_to_HS_Body"]['Brake_Lights'])
    ret.genericToggle = bool(cp.vl["Steering_Buttons"]['Dist_Incr'])
    self.latLimit = cp_cam.vl["Lane_Keep_Assist_Status"]['LatCtlLim_D_Stat']
    self.lkas_state = cp_cam.vl["Lane_Keep_Assist_Status"]['LaActAvail_D_Actl']
    self.laneCurvature = cp_cam.vl["Lane_Keep_Assist_Control"]['Lane_Curvature']
    self.left_blinker_on = bool(cp.vl["Steering_Buttons"]['Left_Turn_Light'])
    ret.leftBlinker = self.left_blinker_on > 0
    self.right_blinker_on = bool(cp.vl["Steering_Buttons"]['Right_Turn_Light'])    
    ret.rightBlinker = self.right_blinker_on > 0
    ret.doorOpen = any([cp.vl["Doors"]['Door_FL_Open'],cp.vl["Doors"]['Door_FR_Open'],
                        cp.vl["Doors"]['Door_RL_Open'], cp.vl["Doors"]['Door_RR_Open']]) 
    ret.steeringTorqueEps = cp_cam.vl["EPAS_INFO"]['SteeringColumnTorque']
    ret.steeringTorque = cp_cam.vl["EPAS_INFO"]['SteeringColumnTorque']
    self.epsAssistLimited = cp_cam.vl["EPAS_INFO"]['SteMdule_D_Stat'] == 1
    ret.seatbeltUnlatched = cp.vl["RCMStatusMessage2_FD1"]['FirstRowBuckleDriver'] == 2
    self.cruise_mode = cp.vl["ACCDATA_3"]['AccMemEnbl_B_RqDrv']
    ret.stockFcw = cp.vl["ACCDATA_3"]['FcwVisblWarn_B_Rq'] !=0
    ret.stockAeb = self.cruise_mode !=0 and ret.cruiseState.enabled and ret.stockFcw
    self.engineRPM = cp.vl["EngineData_14"]['EngAout_N_Actl'] 
    #Gear Shifter
    gear = cp.vl["TransGearData"]['GearLvrPos_D_Actl']
    if gear == 0:
      ret.gearShifter = GearShifter.park
    elif gear == 1:
      ret.gearShifter = GearShifter.reverse
    elif gear == 2:
      ret.gearShifter = GearShifter.neutral
    elif gear in (3, 4, 5, 6, 7, 8, 9, 10, 11): #6R80 only. Not counting 10R80
      ret.gearShifter = GearShifter.drive
    else:
      ret.gearShifter = GearShifter.unknown
    #SODL/SODR BLIS
    ret.leftBlindspot = cp.vl["Side_Detect_L_Stat"]['SodDetctLeft_D_Stat'] !=0
    ret.rightBlindspot = cp.vl["Side_Detect_R_Stat"]['SodDetctRight_D_Stat'] !=0
    #fordcan params
    self.ahbcCommanded = cp_cam.vl["Lane_Keep_Assist_Ui"]['AhbHiBeam_D_Rq']
    self.ipmaHeater = cp_cam.vl["Lane_Keep_Assist_Ui"]['CamraDefog_B_Req']
    self.ahbcRamping = cp_cam.vl["Lane_Keep_Assist_Ui"]['AhbcRampingV_D_Rq']
    self.ipmaConfig = cp_cam.vl["Lane_Keep_Assist_Ui"]['FeatConfigIpmaActl']
    self.ipmaNo = cp_cam.vl["Lane_Keep_Assist_Ui"]['FeatNoIpmaActl']
    self.laDenyStat = cp_cam.vl["Lane_Keep_Assist_Ui"]['LaDenyStats_B_Dsply']
    self.ipmaStats = cp_cam.vl["Lane_Keep_Assist_Ui"]['CamraStats_D_Dsply']
    self.persipma = cp_cam.vl["Lane_Keep_Assist_Ui"]['PersIndexIpma_D_Actl']
    self.dasdsply = cp_cam.vl["Lane_Keep_Assist_Ui"]['DasStats_D_Dsply']
    self.x30 = cp_cam.vl["Lane_Keep_Assist_Ui"]['Set_Me_X30']
    #APA Stuff
    self.sappHandshake = cp_cam.vl["EPAS_INFO"]['SAPPAngleControlStat1']
    #Speed
    self.vehSpeed = cp.vl["EngVehicleSpThrottle2"]['Veh_V_ActlEng']
    self.vehSpeed2 = cp.vl["BrakeSysFeatures"]['Veh_V_ActlBrk']
    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
    # sig_name, sig_address, default
      ("WhlRr_W_Meas", "WheelSpeed", 0.),
      ("WhlRl_W_Meas", "WheelSpeed", 0.),
      ("WhlFr_W_Meas", "WheelSpeed", 0.),
      ("WhlFl_W_Meas", "WheelSpeed", 0.),
      ("SteWhlRelInit_An_Sns", "BrakeSnData_5", 0.),
      ("Cruise_State", "Cruise_Status", 0.),
      ("Set_Speed", "Cruise_Status", 0.),
      ("ApedPosScal_Pc_Actl", "EngineData_14", 0.),
      ("Dist_Incr", "Steering_Buttons", 0.),
      ("Lane_Keep_Toggle", "Steering_Buttons", 0.),
      ("Brake_Drv_Appl", "Cruise_Status", 0.),
      ("Brake_Lights", "BCM_to_HS_Body", 0.),
      ("Left_Turn_Light", "Steering_Buttons", 0.),
      ("Right_Turn_Light", "Steering_Buttons", 0.),
      ("Door_FL_Open", "Doors", 0.),
      ("Door_FR_Open", "Doors", 0.),
      ("Door_RL_Open", "Doors", 0.),
      ("Door_RR_Open", "Doors", 0.),
      ("GearLvrPos_D_Actl", "TransGearData", 0.),
      ("FirstRowBuckleDriver", "RCMStatusMessage2_FD1", 0.),
      ("SodDetctLeft_D_Stat", "Side_Detect_L_Stat", 0.),
      ("SodDetctRight_D_Stat", "Side_Detect_R_Stat", 0.),
      ("FcwVisblWarn_B_Rq", "ACCDATA_3", 0.),
      ("Veh_V_ActlEng", "EngVehicleSpThrottle2", 0.),
      ("Veh_V_ActlBrk", "BrakeSysFeatures", 0.),
      ("AccMemEnbl_B_RqDrv", "ACCDATA_3", 0.),
      ("EngAout_N_Actl", "EngineData_14", 0.),
    ]
    
    checks = []
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)
  
  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
    # sig_name, sig_address, default
      ("SAPPAngleControlStat1", "EPAS_INFO", 0.),
      ("SteeringColumnTorque", "EPAS_INFO", 0.),
      ("SteMdule_D_Stat", "EPAS_INFO", 0.),
      ("FeatConfigIpmaActl", "Lane_Keep_Assist_Ui", 0.),
      ("FeatNoIpmaActl", "Lane_Keep_Assist_Ui", 0.),
      ("LaDenyStats_B_Dsply", "Lane_Keep_Assist_Ui", 0.),
      ("CamraStats_D_Dsply", "Lane_Keep_Assist_Ui", 0.),
      ("Lane_Curvature", "Lane_Keep_Assist_Control", 0.),
      ("AhbHiBeam_D_Rq", "Lane_Keep_Assist_Ui", 0.),
      ("CamraDefog_B_Req", "Lane_Keep_Assist_Ui", 0.),
      ("AhbcRampingV_D_Rq", "Lane_Keep_Assist_Ui", 0.),
      ("LatCtlLim_D_Stat", "Lane_Keep_Assist_Status", 0.),
      ("LaActAvail_D_Actl", "Lane_Keep_Assist_Status", 0.),
      ("LaHandsOff_B_Actl", "Lane_Keep_Assist_Status", 0.),
      ("LaActDeny_B_Actl", "Lane_Keep_Assist_Status", 0.),
      ("PersIndexIpma_D_Actl", "Lane_Keep_Assist_Ui", 0.),
      ("DasStats_D_Dsply", "Lane_Keep_Assist_Ui", 0.),
      ("Set_Me_X30", "Lane_Keep_Assist_Ui", 0.),
    ]

    checks = [] 
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
