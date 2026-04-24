import copy
from cereal import car
from opendbc.can import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.volvo.values import CarControllerParams, DBC, CANBUS
from opendbc.car import Bus, structs


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.cruiseState_enabled_prev = False
    self.eps_torque_timer = 0
    self.frame = 0

  def update(self, can_parsers) -> structs.CarState:
    pt_cp = can_parsers[Bus.pt]
    cam_cp = can_parsers[Bus.cam]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # car speed
    ret.vEgoRaw = pt_cp.vl["VehicleSpeed1"]["VehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    # gas pedal
    ret.gasPressed = pt_cp.vl["AccPedal"]["AccPedal"] >= 10

    # brake pedal
    ret.brake = pt_cp.vl["BrakePedal"]["BrakePressure"]
    ret.brakePressed = pt_cp.vl["Brake_Info"]["BrakePedal"] == 2

    # steering
    ret.steeringAngleDeg = pt_cp.vl["PSCM1"]["SteeringAngleServo"]
    ret.steeringRateDeg = pt_cp.vl["SAS0"]["SteeringRateOfChange"]
    self.steeringDirection = pt_cp.vl["SAS0"]["SteeringDirection"]
    ret.steeringTorque = pt_cp.vl["PSCM1"]["EPSTorque"]
    if self.steeringDirection:
      ret.steeringTorque = -abs(ret.steeringTorque)
    ret.steeringTorqueEps = pt_cp.vl["PSCM1"]["LKATorque"]
    ret.steeringPressed = False

    # cruise state
    ret.cruiseState.speed = pt_cp.vl["ACC_Speed"]["ACC_Speed"] * CV.KPH_TO_MS
    ret.cruiseState.available = bool(cam_cp.vl["FSM0"]["ACC_Available"])
    ret.cruiseState.enabled = bool(cam_cp.vl["FSM0"]["ACC_Enabled"])
    # ACC_Standstill bit = 1 when Volvo's ACC is holding the car at 0 km/h
    # with brake applied (standstill hold). OP's SNG block reads this to know
    # when to blast Resume button so stock ACC properly releases and follows
    # the lead resuming. Without this, stock ACC sees OP commanding accel
    # from standstill without a Resume press and hard-cancels (observed in
    # drive 38).
    ret.cruiseState.standstill = bool(cam_cp.vl["FSM3"]["ACC_Standstill"])
    ret.cruiseState.nonAdaptive = False
    ret.accFaulted = False
    self.acc_distance = cam_cp.vl["FSM1"]["ACC_Distance"]

    # Check if servo stops responding when ACC is active
    if ret.cruiseState.enabled and ret.vEgo > self.CP.minSteerSpeed:
      if not self.cruiseState_enabled_prev:
        self.eps_torque_timer = 0

      if ret.steeringTorqueEps == 0:
        self.eps_torque_timer += 1
      else:
        self.eps_torque_timer = 0

      ret.steerFaultTemporary = self.eps_torque_timer >= CarControllerParams.STEER_TIMEOUT
    else:
      ret.steerFaultTemporary = False

    self.cruiseState_enabled_prev = ret.cruiseState.enabled

    # gear
    ret.gearShifter = car.CarState.GearShifter.drive

    # safety
    ret.stockFcw = False
    ret.stockAeb = False

    # button presses
    ret.leftBlinker = pt_cp.vl["MiscCarInfo"]["TurnSignal"] == 1
    ret.rightBlinker = pt_cp.vl["MiscCarInfo"]["TurnSignal"] == 3

    # lock info
    ret.doorOpen = not all([pt_cp.vl["Doors"]["DriverDoorClosed"], pt_cp.vl["Doors"]["PassengerDoorClosed"]])
    ret.seatbeltUnlatched = False

    # Store info from servo message PSCM1
    self.pscm_stock_values = pt_cp.vl["PSCM1"]

    # Messages forwarded for oplong and radar spoofing
    self.stock_FSM1 = copy.copy(cam_cp.vl["FSM1"])
    self.stock_FSM3 = copy.copy(cam_cp.vl["FSM3"])
    self.ACC_Check = cam_cp.vl["FSM3"]["ACC_Check"]

    self.frame += 1
    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
      ("VehicleSpeed1", 50),
      ("AccPedal", 100),
      ("BrakePedal", 50),
      ("Brake_Info", 50),
      ("PSCM1", 50),
      ("ACC_Speed", 50),
      ("MiscCarInfo", 25),
      ("Doors", 20),
      ("SAS0", 100)
    ]

    cam_messages = [
      ("FSM0", 100),
      ("FSM1", 50),
      ("FSM3", 50),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CANBUS.cam),
    }
