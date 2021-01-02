#!/usr/bin/env python3 
import capnp
import time
import os
from cereal import car, log
from common.numpy_fast import clip
from common.realtime import sec_since_boot, config_realtime_process, Priority, Ratekeeper, DT_CTRL
from common.profiler import Profiler
from common.params import Params, put_nonblocking
import cereal.messaging as messaging
from selfdrive.services import service_list

from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.laterald import Lateral
from selfdrive.controls.lib.alertmanager import AlertManager
#### From openpilot
from selfdrive.config import Conversions as CV
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.car_helpers import get_car, get_startup_event, get_one_can
from selfdrive.controls.lib.lane_planner import CAMERA_OFFSET
from selfdrive.controls.lib.drive_helpers import update_v_cruise, initialize_v_cruise
from selfdrive.controls.lib.longcontrol import LongControl, STARTING_TARGET_SPEED
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from selfdrive.controls.lib.events import Events, ET
from selfdrive.controls.lib.alertmanager import AlertManager
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.controls.lib.planner import LON_MPC_STEP
from selfdrive.locationd.calibrationd import Calibration
from selfdrive.hardware import HARDWARE

from setproctitle import setproctitle


LDW_MIN_SPEED = 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1
STEER_ANGLE_SATURATION_TIMEOUT = 1.0 / DT_CTRL
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees

SIMULATION = "SIMULATION" in os.environ
NOSENSOR = "NOSENSOR" in os.environ

ThermalStatus = log.ThermalData.ThermalStatus
State = log.ControlsState.OpenpilotState
HwType = log.HealthData.HwType
LongitudinalPlanSource = log.Plan.LongitudinalPlanSource
Desire = log.PathPlan.Desire
LaneChangeState = log.PathPlan.LaneChangeState
LaneChangeDirection = log.PathPlan.LaneChangeDirection
EventName = car.CarEvent.EventName



class Controls:
  def __init__(self, sm=None, pm=None, can_sock=None):
    config_realtime_process(3, Priority.CTRL_HIGH)
    # Setup sockets
    self.pm = pm
    if self.pm is None:
      self.pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState',
                                     'carControl', 'carEvents', 'carParams'])

    self.sm = sm
    if self.sm is None:
      self.sm = messaging.SubMaster(['thermal', 'health', 'model', 'liveCalibration', 'frontFrame',
                                     'dMonitoringState', 'plan', 'pathPlan', 'liveLocationKalman'])

    self.can_sock = can_sock
    if can_sock is None:
      can_timeout = None if os.environ.get('NO_CAN_TIMEOUT', False) else 100
      self.can_sock = messaging.sub_sock('can', timeout=can_timeout)

    # wait for one health and one CAN packet
    print("Waiting for CAN messages...")
    get_one_can(self.can_sock)

    self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'])

    # read params
    params = Params()
    self.is_metric = params.get("IsMetric", encoding='utf8') == "1"
    self.is_ldw_enabled = params.get("IsLdwEnabled", encoding='utf8') == "1"
    community_feature_toggle = params.get("CommunityFeaturesToggle", encoding='utf8') == "1"
    openpilot_enabled_toggle = params.get("OpenpilotEnabledToggle", encoding='utf8') == "1"
    passive = params.get("Passive", encoding='utf8') == "1" or not openpilot_enabled_toggle

    # detect sound card presence and ensure successful init
    sounds_available = HARDWARE.get_sound_card_online()

    car_recognized = self.CP.carName != 'mock'
    # If stock camera is disconnected, we loaded car controls and it's not dashcam mode
    controller_available = self.CP.enableCamera and self.CI.CC is not None and not passive and not self.CP.dashcamOnly
    community_feature_disallowed = self.CP.communityFeature and not community_feature_toggle
    self.read_only = not car_recognized or not controller_available or \
                       self.CP.dashcamOnly or community_feature_disallowed
    if self.read_only:
      self.CP.safetyModel = car.CarParams.SafetyModel.noOutput

    # Write CarParams for radard and boardd safety mode
    cp_bytes = self.CP.to_bytes()
    params.put("CarParams", cp_bytes)
    put_nonblocking("CarParamsCache", cp_bytes)

    self.CC = car.CarControl.new_message()
    self.AM = AlertManager()
    self.events = Events()

    self.LoC = LongControl(self.CP, self.CI.compute_gb)
    self.VM = VehicleModel(self.CP)

    if self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP)
    elif self.CP.lateralTuning.which() == 'indi':
      self.LaC = LatControlINDI(self.CP)
    elif self.CP.lateralTuning.which() == 'lqr':
      self.LaC = LatControlLQR(self.CP)

    self.state = State.disabled
    self.enabled = False
    self.active = False
    self.can_rcv_error = False
    self.soft_disable_timer = 0
    self.v_cruise_kph = 255
    self.v_cruise_kph_last = 0
    self.mismatch_counter = 0
    self.can_error_counter = 0
    self.last_blinker_frame = 0
    self.saturated_count = 0
    self.distance_traveled = 0
    self.last_functional_fan_frame = 0
    self.events_prev = []
    self.current_alert_types = [ET.PERMANENT]

    self.sm['liveCalibration'].calStatus = Calibration.CALIBRATED
    self.sm['thermal'].freeSpace = 1.
    self.sm['dMonitoringState'].events = []
    self.sm['dMonitoringState'].awarenessStatus = 1.
    self.sm['dMonitoringState'].faceDetected = False

    self.startup_event = get_startup_event(car_recognized, controller_available)

    if not sounds_available:
      self.events.add(EventName.soundsUnavailable, static=True)
    if community_feature_disallowed:
      self.events.add(EventName.communityFeatureDisallowed, static=True)
    if not car_recognized:
      self.events.add(EventName.carUnrecognized, static=True)

    # controlsd is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)
    self.prof = Profiler(False)  # off by default

  def data_sample(self):
    """Receive data from sockets and update carState"""

    # Update carState from CAN
    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    CS = self.CI.update(self.CC, can_strs)

    self.sm.update(0)

    # Check for CAN timeout
    if not can_strs:
      self.can_error_counter += 1
      self.can_rcv_error = True
    else:
      self.can_rcv_error = False

    # When the panda and controlsd do not agree on controls_allowed
    # we want to disengage openpilot. However the status from the panda goes through
    # another socket other than the CAN messages and one can arrive earlier than the other.
    # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    if not self.enabled:
      self.mismatch_counter = 0

    if not self.sm['health'].controlsAllowed and self.enabled:
      self.mismatch_counter += 1

    self.distance_traveled += CS.vEgo * DT_CTRL

    return CS

  def state_transition(self, CS):
    """Compute conditional state transitions and execute actions on state transitions"""

    self.v_cruise_kph_last = self.v_cruise_kph

    # if stock cruise is completely disabled, then we can use our own set speed logic
    if not self.CP.enableCruise:
      self.v_cruise_kph = update_v_cruise(self.v_cruise_kph, CS.buttonEvents, self.enabled)
    elif self.CP.enableCruise and CS.cruiseState.enabled:
      self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH

    # decrease the soft disable timer at every step, as it's reset on
    # entrance in SOFT_DISABLING state
    self.soft_disable_timer = max(0, self.soft_disable_timer - 1)

    self.current_alert_types = [ET.PERMANENT]

    # ENABLED, PRE ENABLING, SOFT DISABLING
    if self.state != State.disabled:
      # user and immediate disable always have priority in a non-disabled state
      if self.events.any(ET.USER_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.USER_DISABLE)

      elif self.events.any(ET.IMMEDIATE_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.IMMEDIATE_DISABLE)

      else:
        # ENABLED
        if self.state == State.enabled:
          if self.events.any(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            self.soft_disable_timer = 300   # 3s
            self.current_alert_types.append(ET.SOFT_DISABLE)

        # SOFT DISABLING
        elif self.state == State.softDisabling:
          if not self.events.any(ET.SOFT_DISABLE):
            # no more soft disabling condition, so go back to ENABLED
            self.state = State.enabled

          elif self.events.any(ET.SOFT_DISABLE) and self.soft_disable_timer > 0:
            self.current_alert_types.append(ET.SOFT_DISABLE)

          elif self.soft_disable_timer <= 0:
            self.state = State.disabled

        # PRE ENABLING
        elif self.state == State.preEnabled:
          if not self.events.any(ET.PRE_ENABLE):
            self.state = State.enabled
          else:
            self.current_alert_types.append(ET.PRE_ENABLE)

    # DISABLED
    elif self.state == State.disabled:
      if self.events.any(ET.ENABLE):
        if self.events.any(ET.NO_ENTRY):
          self.current_alert_types.append(ET.NO_ENTRY)

        else:
          if self.events.any(ET.PRE_ENABLE):
            self.state = State.preEnabled
          else:
            self.state = State.enabled
          self.current_alert_types.append(ET.ENABLE)
          self.v_cruise_kph = initialize_v_cruise(CS.vEgo, CS.buttonEvents, self.v_cruise_kph_last)

    # Check if actuators are enabled
    self.active = self.state == State.enabled or self.state == State.softDisabling
    if self.active:
      self.current_alert_types.append(ET.WARNING)

    # Check if openpilot is engaged
    self.enabled = self.active or self.state == State.preEnabled

  def state_control(self, CS):
    """Given the state, this function returns an actuators packet"""

    plan = self.sm['plan']
    path_plan = self.sm['pathPlan']

    actuators = car.CarControl.Actuators.new_message()

    if CS.leftBlinker or CS.rightBlinker:
      self.last_blinker_frame = self.sm.frame

    # State specific actions

    if not self.active:
      self.LaC.reset()
      self.LoC.reset(v_pid=CS.vEgo)

    plan_age = DT_CTRL * (self.sm.frame - self.sm.rcv_frame['plan'])
    # no greater than dt mpc + dt, to prevent too high extraps
    dt = min(plan_age, LON_MPC_STEP + DT_CTRL) + DT_CTRL

    a_acc_sol = plan.aStart + (dt / LON_MPC_STEP) * (plan.aTarget - plan.aStart)
    v_acc_sol = plan.vStart + dt * (a_acc_sol + plan.aStart) / 2.0

    # Gas/Brake PID loop
    actuators.gas, actuators.brake = self.LoC.update(self.active, CS, v_acc_sol, plan.vTargetFuture, a_acc_sol, self.CP)
    # Steering PID loop and lateral MPC
    actuators.steer, actuators.steerAngle, lac_log = self.LaC.update(self.active, CS, self.CP, path_plan)

    # Check for difference between desired angle and angle for angle based control
    angle_control_saturated = self.CP.steerControlType == car.CarParams.SteerControlType.angle and \
      abs(actuators.steerAngle - CS.steeringAngle) > STEER_ANGLE_SATURATION_THRESHOLD

    if angle_control_saturated and not CS.steeringPressed and self.active:
      self.saturated_count += 1
    else:
      self.saturated_count = 0

    # Send a "steering required alert" if saturation count has reached the limit
    if (lac_log.saturated and not CS.steeringPressed) or \
       (self.saturated_count > STEER_ANGLE_SATURATION_TIMEOUT):
      # Check if we deviated from the path
      left_deviation = actuators.steer > 0 and path_plan.dPoly[3] > 0.1
      right_deviation = actuators.steer < 0 and path_plan.dPoly[3] < -0.1

      if left_deviation or right_deviation:
        self.events.add(EventName.steerSaturated)

    return actuators, v_acc_sol, a_acc_sol, lac_log

  def publish_logs(self, CS, start_time, actuators, v_acc, a_acc, lac_log):
    """Send actuators and hud commands to the car, send controlsstate and MPC logging"""

    CC = car.CarControl.new_message()
    CC.enabled = self.enabled
    CC.actuators = actuators

    CC.cruiseControl.override = True
    CC.cruiseControl.cancel = not self.CP.enableCruise or (not self.enabled and CS.cruiseState.enabled)

    # Some override values for Honda
    # brake discount removes a sharp nonlinearity
    brake_discount = (1.0 - clip(actuators.brake * 3., 0.0, 1.0))
    speed_override = max(0.0, (self.LoC.v_pid + CS.cruiseState.speedOffset) * brake_discount)
    CC.cruiseControl.speedOverride = float(speed_override if self.CP.enableCruise else 0.0)
    CC.cruiseControl.accelOverride = self.CI.calc_accel_override(CS.aEgo, self.sm['plan'].aTarget, CS.vEgo, self.sm['plan'].vTarget)

    CC.hudControl.setSpeed = float(self.v_cruise_kph * CV.KPH_TO_MS)
    CC.hudControl.speedVisible = self.enabled
    CC.hudControl.lanesVisible = self.enabled
    CC.hudControl.leadVisible = self.sm['plan'].hasLead

    right_lane_visible = self.sm['pathPlan'].rProb > 0.5
    left_lane_visible = self.sm['pathPlan'].lProb > 0.5
    CC.hudControl.rightLaneVisible = bool(right_lane_visible)
    CC.hudControl.leftLaneVisible = bool(left_lane_visible)

    recent_blinker = (self.sm.frame - self.last_blinker_frame) * DT_CTRL < 5.0  # 5s blinker cooldown
    ldw_allowed = self.is_ldw_enabled and CS.vEgo > LDW_MIN_SPEED and not recent_blinker \
                    and not self.active and self.sm['liveCalibration'].calStatus == Calibration.CALIBRATED

    meta = self.sm['model'].meta
    if len(meta.desirePrediction) and ldw_allowed:
      l_lane_change_prob = meta.desirePrediction[Desire.laneChangeLeft - 1]
      r_lane_change_prob = meta.desirePrediction[Desire.laneChangeRight - 1]
      l_lane_close = left_lane_visible and (self.sm['pathPlan'].lPoly[3] < (1.08 - CAMERA_OFFSET))
      r_lane_close = right_lane_visible and (self.sm['pathPlan'].rPoly[3] > -(1.08 + CAMERA_OFFSET))

      CC.hudControl.leftLaneDepart = bool(l_lane_change_prob > LANE_DEPARTURE_THRESHOLD and l_lane_close)
      CC.hudControl.rightLaneDepart = bool(r_lane_change_prob > LANE_DEPARTURE_THRESHOLD and r_lane_close)

    if CC.hudControl.rightLaneDepart or CC.hudControl.leftLaneDepart:
      self.events.add(EventName.ldw)

    clear_event = ET.WARNING if ET.WARNING not in self.current_alert_types else None
    alerts = self.events.create_alerts(self.current_alert_types, [self.CP, self.sm, self.is_metric])
    self.AM.add_many(self.sm.frame, alerts, self.enabled)
    self.AM.process_alerts(self.sm.frame, clear_event)
    CC.hudControl.visualAlert = self.AM.visual_alert

    if not self.read_only:
      # send car controls over can
      can_sends = self.CI.apply(CC)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))

    force_decel = (self.sm['dMonitoringState'].awarenessStatus < 0.) or \
                  (self.state == State.softDisabling)

    steer_angle_rad = (CS.steeringAngle - self.sm['pathPlan'].angleOffset) * CV.DEG_TO_RAD

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    controlsState = dat.controlsState
    controlsState.alertText1 = self.AM.alert_text_1
    controlsState.alertText2 = self.AM.alert_text_2
    controlsState.alertSize = self.AM.alert_size
    controlsState.alertStatus = self.AM.alert_status
    controlsState.alertBlinkingRate = self.AM.alert_rate
    controlsState.alertType = self.AM.alert_type
    controlsState.alertSound = self.AM.audible_alert
    controlsState.driverMonitoringOn = self.sm['dMonitoringState'].faceDetected
    controlsState.canMonoTimes = list(CS.canMonoTimes)
    controlsState.planMonoTime = self.sm.logMonoTime['plan']
    controlsState.pathPlanMonoTime = self.sm.logMonoTime['pathPlan']
    controlsState.enabled = self.enabled
    controlsState.active = self.active
    controlsState.vEgo = CS.vEgo
    controlsState.vEgoRaw = CS.vEgoRaw
    controlsState.angleSteers = CS.steeringAngle
    controlsState.curvature = self.VM.calc_curvature(steer_angle_rad, CS.vEgo)
    controlsState.steerOverride = CS.steeringPressed
    controlsState.state = self.state
    controlsState.engageable = not self.events.any(ET.NO_ENTRY)
    controlsState.longControlState = self.LoC.long_control_state
    controlsState.vPid = float(self.LoC.v_pid)
    controlsState.vCruise = float(self.v_cruise_kph)
    controlsState.upAccelCmd = float(self.LoC.pid.p)
    controlsState.uiAccelCmd = float(self.LoC.pid.i)
    controlsState.ufAccelCmd = float(self.LoC.pid.f)
    controlsState.angleSteersDes = float(self.LaC.angle_steers_des)
    controlsState.vTargetLead = float(v_acc)
    controlsState.aTarget = float(a_acc)
    controlsState.jerkFactor = float(self.sm['plan'].jerkFactor)
    controlsState.gpsPlannerActive = self.sm['plan'].gpsPlannerActive
    controlsState.vCurvature = self.sm['plan'].vCurvature
    controlsState.decelForModel = self.sm['plan'].longitudinalPlanSource == LongitudinalPlanSource.model
    controlsState.cumLagMs = -self.rk.remaining * 1000.
    controlsState.startMonoTime = int(start_time * 1e9)
    controlsState.mapValid = self.sm['plan'].mapValid
    controlsState.forceDecel = bool(force_decel)
    controlsState.canErrorCounter = self.can_error_counter

    if self.CP.lateralTuning.which() == 'pid':
      controlsState.lateralControlState.pidState = lac_log
    elif self.CP.lateralTuning.which() == 'lqr':
      controlsState.lateralControlState.lqrState = lac_log
    elif self.CP.lateralTuning.which() == 'indi':
      controlsState.lateralControlState.indiState = lac_log
    self.pm.send('controlsState', dat)

    # carState
    car_events = self.events.to_msg()
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.events = car_events
    self.pm.send('carState', cs_send)

    # carEvents - logged every second or on change
    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
      ce_send = messaging.new_message('carEvents', len(self.events))
      ce_send.carEvents = car_events
      self.pm.send('carEvents', ce_send)
    self.events_prev = self.events.names.copy()

    # carParams - logged every 50 seconds (> 1 per segment)
    if (self.sm.frame % int(50. / DT_CTRL) == 0):
      cp_send = messaging.new_message('carParams')
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

    # copy CarControl to pass to CarInterface on the next iteration
    self.CC = CC

  def step(self):
    start_time = sec_since_boot()
    self.prof.checkpoint("Ratekeeper", ignore=True)

    # Sample data from sockets and get a carState
    CS = self.data_sample()
    self.prof.checkpoint("Sample")

    self.update_events(CS)

    if not self.read_only:
      # Update control state
      self.state_transition(CS)
      self.prof.checkpoint("State transition")

    # Compute actuators (runs PID loops and lateral MPC)
    actuators, v_acc, a_acc, lac_log = self.state_control(CS)

    self.prof.checkpoint("State Control")

    # Publish data
    self.publish_logs(CS, start_time, actuators, v_acc, a_acc, lac_log)
    self.prof.checkpoint("Sent")

  def controlsd_thread(self):
    while True:
      self.step()
      self.rk.monitor_time()
      self.prof.display()


def isActive(state):
  """Check if the actuators are enabled"""
  return state in [State.enabled, State.softDisabling]


def isEnabled(state):
  """Check if openpilot is engaged"""
  return (isActive(state) or state == State.preEnabled)

def events_to_bytes(events):
  # optimization when comparing capnp structs: str() or tree traverse are much slower
  ret = []
  for e in events:
    if isinstance(e, capnp.lib.capnp._DynamicStructReader):
      e = e.as_builder()
    ret.append(e.to_bytes())
  return ret

def wait_for_can(logcan):
  print("Waiting for CAN messages...")
  while len(messaging.recv_sock(logcan, wait=True).can) == 0:
    pass

def data_sample(CI, CC, can_sock, carstate, lac_log, lateral, sm, profiler):
  """Receive data from sockets and create events for battery, temperature and disk space"""
 
  can_strs = [can_sock.recv()]
  profiler.checkpoint('can_recv', True)
  CS = CI.update(CC, can_strs, lac_log, profiler)
  profiler.checkpoint('carstate')
  lateral.update(CS, sm, 0, 1)
  profiler.checkpoint('lateral')
  if CS.canTime + 20 < CS.sysTime: 
    can_strs = messaging.drain_sock_raw(can_sock, wait_for_one=False)
    profiler.checkpoint('drain_can')
    if len(can_strs) > 0: 
      print("  Controls lagged by %d CAN packets at %d at %0.2f m/s!" % (len(can_strs), int(time.time()*1000), CS.vEgo), [len(x) for x in can_strs])
      for i in range(len(can_strs[-40:])):
        #time.sleep(0.00001)
        CS = CI.update(CC, [can_strs[i]], lac_log, profiler)
        profiler.checkpoint('drain_carstate')
        lateral.update(CS, sm, i, len(can_strs[-40:]))
        profiler.checkpoint('drain_lateral')
    else:
      #print("  CAN lagged!")
      CI.canTime += 20
  #else:
  #  time.sleep(0.00001)

  events = list(CS.events)

  return CS, events

def state_transition(frame, CS, CP, state, events, soft_disable_timer, v_cruise_kph, AM):
  """Compute conditional state transitions and execute actions on state transitions"""
  enabled = isEnabled(state)

  v_cruise_kph_last = v_cruise_kph

  # if stock cruise is completely disabled, then we can use our own set speed logic
  if not CP.enableCruise:
    v_cruise_kph = update_v_cruise(v_cruise_kph, CS.buttonEvents, enabled)
  elif CP.enableCruise and CS.cruiseState.enabled:
    v_cruise_kph = CS.cruiseState.speed #* CV.MS_TO_KPH

  # decrease the soft disable timer at every step, as it's reset on
  # entrance in SOFT_DISABLING state
  soft_disable_timer = max(0, soft_disable_timer - 1)

  # DISABLED
  if state == State.disabled:
    if get_events(events, [ET.ENABLE]):
      if get_events(events, [ET.NO_ENTRY]):
        for e in get_events(events, [ET.NO_ENTRY]):
          AM.add(frame, str(e) + "NoEntry", enabled)

      else:
        if get_events(events, [ET.PRE_ENABLE]):
          state = State.preEnabled
        else:
          state = State.enabled
        AM.add(frame, "enable", enabled)
        v_cruise_kph = initialize_v_cruise(CS.vEgo, CS.buttonEvents, v_cruise_kph_last)

  # ENABLED
  elif state == State.enabled:
    if get_events(events, [ET.USER_DISABLE]):
      state = State.disabled
      AM.add(frame, "disable", enabled)

    elif get_events(events, [ET.IMMEDIATE_DISABLE]):
      state = State.disabled
      for e in get_events(events, [ET.IMMEDIATE_DISABLE]):
        AM.add(frame, e, enabled)

    elif get_events(events, [ET.SOFT_DISABLE]):
      state = State.softDisabling
      soft_disable_timer = 600   # 6s
      for e in get_events(events, [ET.SOFT_DISABLE]):
        AM.add(frame, e, enabled)

  # SOFT DISABLING
  elif state == State.softDisabling:
    if get_events(events, [ET.USER_DISABLE]):
      state = State.disabled
      AM.add(frame, "disable", enabled)

    elif get_events(events, [ET.IMMEDIATE_DISABLE]):
      state = State.disabled
      for e in get_events(events, [ET.IMMEDIATE_DISABLE]):
        AM.add(frame, e, enabled)

    elif not get_events(events, [ET.SOFT_DISABLE]):
      # no more soft disabling condition, so go back to ENABLED
      state = State.enabled

    elif get_events(events, [ET.SOFT_DISABLE]) and soft_disable_timer > 0:
      for e in get_events(events, [ET.SOFT_DISABLE]):
        AM.add(frame, e, enabled)

    elif soft_disable_timer <= 0:
      state = State.disabled

  # PRE ENABLING
  elif state == State.preEnabled:
    if get_events(events, [ET.USER_DISABLE]):
      state = State.disabled
      AM.add(frame, "disable", enabled)

    elif get_events(events, [ET.IMMEDIATE_DISABLE, ET.SOFT_DISABLE]):
      state = State.disabled
      for e in get_events(events, [ET.IMMEDIATE_DISABLE, ET.SOFT_DISABLE]):
        AM.add(frame, e, enabled)

    elif not get_events(events, [ET.PRE_ENABLE]):
      state = State.enabled

  return state, soft_disable_timer, v_cruise_kph, v_cruise_kph_last


def state_control(frame, lkasMode, path_plan, CS, CP, state, events, AM, LaC, lac_log, profiler):
  """Given the state, this function returns an actuators packet"""

  actuators = car.CarControl.Actuators.new_message()

  enabled = isEnabled(state)
  active = isActive(state)

  # Steering PID loop and lateral MPC
  actuators.steer, actuators.steerAngle, lac_log = LaC.update(path_plan.paramsValid and CS.lkMode and (active or lkasMode), CS.brakePressed, CS.vEgo, CS.steeringAngle, CS.steeringRate, CS.steeringPressed, CP, path_plan, CS.canTime, CS.blinkers)
  profiler.checkpoint('lac_update')
    # parse warnings from car specific interface
  for e in get_events(events, [ET.WARNING]):
    extra_text = ""
    AM.add(frame, e, enabled, extra_text_2=extra_text)

  # Parse permanent warnings to display constantly
  for e in get_events(events, [ET.PERMANENT]):
    extra_text_1, extra_text_2 = "", ""
    AM.add(frame, str(e) + "Permanent", enabled, extra_text_1=extra_text_1, extra_text_2=extra_text_2)

  AM.process_alerts(frame)

  return actuators, lac_log

def data_send(sm, CS, CI, CP, state, events, actuators, carstate, carcontrol, carevents, carparams, controlsstate, sendcan, AM, LaC, start_time, lac_log, events_prev, profiler):
  """Send actuators and hud commands to the car, send controlsstate and MPC logging"""

  CC = car.CarControl.new_message()
  CC.enabled = isEnabled(state)
  CC.actuators = actuators

  CC.cruiseControl.override = True
  CC.cruiseControl.cancel = not CP.enableCruise or (not isEnabled(state) and CS.cruiseState.enabled)

  #CC.hudControl.setSpeed = float(v_cruise_kph * CV.KPH_TO_MS)
  CC.hudControl.speedVisible = isEnabled(state)
  CC.hudControl.lanesVisible = isEnabled(state)

  right_lane_visible = sm['pathPlan'].rProb > 0.5
  left_lane_visible = sm['pathPlan'].lProb > 0.5

  CC.hudControl.rightLaneVisible = bool(right_lane_visible)
  CC.hudControl.leftLaneVisible = bool(left_lane_visible)

  CC.hudControl.visualAlert = AM.visual_alert
  CC.hudControl.audibleAlert = AM.audible_alert
  profiler.checkpoint('data_send')
  can_sends = CI.apply(CC)
  profiler.checkpoint('ci_apply')
  sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
  events_bytes = None

  return CC, events_bytes


def controlsd_thread(gctx=None):
  setproctitle('controlsd')
  params = Params()
  print(params)
  # Pub Sockets
  profiler = Profiler(True)

  pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState',
                                     'carControl', 'carEvents', 'carParams'])
  can_timeout = None if os.environ.get('NO_CAN_TIMEOUT', False) else 100
  can_sock = messaging.sub_sock('can', timeout=can_timeout)

  sendcan = messaging.pub_sock('sendcan')
  controlsstate = messaging.pub_sock('controlsState')
  carstate = messaging.pub_sock('carState')
  carcontrol = messaging.pub_sock('carControl')
  carevents = messaging.pub_sock('carEvents')
  carparams = messaging.pub_sock('carParams')

  sm = messaging.SubMaster(['pathPlan','health','gpsLocationExternal'])

  hw_type = messaging.recv_one(sm.sock['health']).health.hwType
  is_panda_black = hw_type == log.HealthData.HwType.blackPanda  
  print("panda black: ", is_panda_black)
  wait_for_can(can_sock)
  CI, CP = get_car(can_sock, pm.sock['sendcan'])
  #logcan.close()

  # TODO: Use the logcan socket from above, but that will currenly break the tests
  #can_timeout = None #if os.environ.get('NO_CAN_TIMEOUT', False) else 100
  #can_sock = messaging.sub_sock(service_list['can'].port, timeout=can_timeout)

  # Write CarParams for radard and boardd safety mode
  params.put("CarParams", CP.to_bytes())
  params.put("LongitudinalControl", "1" if CP.openpilotLongitudinalControl else "0")

  CC = car.CarControl.new_message()
  AM = AlertManager()

  startup_alert = get_startup_event(True, True)
  AM.add(sm.frame, startup_alert, False)    

  LaC = LatControlPID(CP)
  lateral = Lateral(CP)
  lkasMode = int(float(LaC.kegman.conf['lkasMode']))
  #CI.CS.lkasMode = (lkasMode == 0)
  lac_log = None #car.CarState.lateralControlState.pidState.new_message()

  state = State.disabled
  soft_disable_timer = 0
  v_cruise_kph = 255
  events_prev = []
  frame = 0

  sm['pathPlan'].sensorValid = True
  sm['pathPlan'].posenetValid = True

  while True:

    start_time = 0 # time.time()  #sec_since_boot()

    # Sample data and compute car events
    CS, events = data_sample(CI, CC, can_sock, carstate, lac_log, lateral, sm, profiler)
    profiler.checkpoint('data_sample')

    state, soft_disable_timer, v_cruise_kph, v_cruise_kph_last = \
        state_transition(sm.frame, CS, CP, state, events, soft_disable_timer, v_cruise_kph, AM)
    profiler.checkpoint('state_transition')
    # Compute actuators (runs PID loops and lateral MPC)
    sm.update(0)
    profiler.checkpoint('sm_update')

    actuators, lac_log = state_control(sm.frame, lkasMode, sm['pathPlan'], CS, CP, state, events, AM, LaC, lac_log, profiler)
    profiler.checkpoint('state_control')

    # Publish data
    CC, events_prev = data_send(sm, CS, CI, CP, state, events, actuators, carstate, carcontrol, carevents, carparams,
                    controlsstate, sendcan, AM, LaC, start_time, lac_log, events_prev, profiler)
    profiler.checkpoint('data_send')
    frame += 1
    if frame % 10000 == 0 and profiler.enabled:
      profiler.display()
      profiler.reset(True)

    
# def main(gctx=None):
#   controlsd_thread(gctx)

# if __name__ == "__main__":
#   main()


def main(sm=None, pm=None, logcan=None):
  controls = Controls(sm, pm, logcan)
  controls.controlsd_thread()


if __name__ == "__main__":
  main()