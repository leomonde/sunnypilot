from cereal import car
from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.radar_interface import RadarInterface
from opendbc.car.volvo.values import DBC


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface  # required — default stub returns empty RadarData

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "volvo"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volvo)]

    # Delphi ESR 2.5 on aux bus → planner sees leads via RadarInterface (64 tracks @ 20Hz).
    ret.radarUnavailable = Bus.radar not in DBC[candidate]

    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.8

    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = alpha_long
    ret.pcmCruise = True  # OP requer ACC nativo ativo

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
 car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    ret.intelligentCruiseButtonManagementAvailable = True
    return ret
