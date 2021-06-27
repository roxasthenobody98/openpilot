#!/usr/bin/env python3
import os
import json
from common.colors import opParams_error as error
from common.colors import opParams_warning as warning
try:
  from common.realtime import sec_since_boot
except ImportError:
  import time
  sec_since_boot = time.time
  warning("Using python time.time() instead of faster sec_since_boot")

travis = False  # replace with travis_checker if you use travis or GitHub Actions


class ValueTypes:
  number = [float, int]
  none_or_number = [type(None), float, int]


class Param:
  def __init__(self, default=None, allowed_types=[], description=None, live=False, hidden=False, depends_on=False):
    self.default = default
    if not isinstance(allowed_types, list):
      allowed_types = [allowed_types]
    self.allowed_types = allowed_types
    self.description = description
    self.hidden = hidden
    self.live = live
    self.depends_on = depends_on
    self.children = []
    self._create_attrs()

  def is_valid(self, value):
    if not self.has_allowed_types:  # always valid if no allowed types, otherwise checks to make sure
      return True
    if self.is_list and isinstance(value, list):
      for v in value:
        if type(v) not in self.allowed_types:
          return False
      return True
    else:
      return type(value) in self.allowed_types or value in self.allowed_types

  def _create_attrs(self):  # Create attributes and check Param is valid
    self.has_allowed_types = isinstance(self.allowed_types, list) and len(self.allowed_types) > 0
    self.has_description = self.description is not None
    self.is_list = list in self.allowed_types
    self.is_bool = bool in self.allowed_types
    if self.has_allowed_types:
      assert type(self.default) in self.allowed_types or self.default in self.allowed_types, 'Default value type must be in specified allowed_types!'

      if self.is_list and self.default:
        for v in self.default:
          assert type(v) in self.allowed_types, 'Default value type must be in specified allowed_types!'


class opParams:
  def __init__(self):
    """
      To add your own parameter to opParams in your fork, simply add a new entry in self.fork_params, instancing a new Param class with at minimum a default value.
      The allowed_types and description args are not required but highly recommended to help users edit their parameters with opEdit safely.
        - The description value will be shown to users when they use opEdit to change the value of the parameter.
        - The allowed_types arg is used to restrict what kinds of values can be entered with opEdit so that users can't crash openpilot with unintended behavior.
              (setting a param intended to be a number with a boolean, or viceversa for example)
          Limiting the range of floats or integers is still recommended when `.get`ting the parameter.
          When a None value is allowed, use `type(None)` instead of None, as opEdit checks the type against the values in the arg with `isinstance()`.
        - Finally, the live arg tells both opParams and opEdit that it's a live parameter that will change. Therefore, you must place the `op_params.get()` call in the update function so that it can update.

      Here's an example of a good fork_param entry:
      self.fork_params = {'camera_offset': Param(default=0.06, allowed_types=VT.number), live=True}  # VT.number allows both floats and ints
    """

    VT = ValueTypes()
    self.fork_params = {'awareness_factor': Param(10., VT.number, 'Multiplier for the awareness times'),
                        'alca_min_speed': Param(20, VT.number, 'Speed limit to start ALC in MPH'),
                        'alca_nudge_required': Param(False, bool, "Require nudge to start ALC"),
                        'apaAcknowledge': Param(False, bool, "acknowledge the safety constraints and limitations"),
                        'athenaAllowed': Param(True, bool, "Enable Comma Athena. Not recommended. Comma get access to everything on your device."),
                        'autoUpdate': Param(True, bool, 'Whether to auto-update'),
                        'camera_offset': Param(0.06, VT.number, 'Your camera offset to use in lane_planner.py', live=True),
                        'cloak': Param(False, bool, "make comma believe you are on their fork"),
                        'supercloak': Param(False, bool, "give comma a fake dongle ID, sparing your old one. Still carries a risk of your device being banned."),
                        'supercloak_reregister': Param(False, bool, "dump your supercloak Dongle ID if it gets banned"),
                        'uploadsAllowed': Param(True, bool, "Allow uploads to Comma. Not recommended. If you are not cloaked and supercloaked, you risk your device being banned."),
                        #'use_car_caching': Param(True, bool, 'Whether to use fingerprint caching'),

                        ENABLE_COASTING: Param(False, bool, 'When true the car will try to coast down hills instead of braking.', live=True),
                        COAST_SPEED: Param(10.0, VT.number, 'The amount of speed to coast by before applying the brakes. Unit: MPH',
                                          live=True, depends_on=ENABLE_COASTING),
                        SETPOINT_OFFSET: Param(0, int, 'The difference between the car\'s set cruise speed and OP\'s. Unit: MPH', live=True),
                        DOWNHILL_INCLINE: Param(-1, VT.number, 'If the angle between the current road and the future predicted road is less than this value, '
                                                              'the car will try to coast downhill. Unit: degrees', live=True, depends_on=ENABLE_COASTING),
                        ALWAYS_EVAL_COAST: Param(False, bool, live=True, depends_on=ENABLE_COASTING),
                        EVAL_COAST_LONG: Param(False, bool, live=True, depends_on=ENABLE_COASTING),
                        ENABLE_LONG_PARAMS: Param(False, bool, live=True, description='When true the long controller will used the params in opParam '
                                                  'instead of the car\' params'),
                        ENABLE_GAS_PARAMS: Param(True, bool, live=True, depends_on=ENABLE_LONG_PARAMS),
                        GAS_MAX_BP: Param([0., 20, 33], [list, float, int], live=True, depends_on=ENABLE_GAS_PARAMS),
                        GAS_MAX_V: Param([0.3, 0.2, 0.075], [list, float], live=True, depends_on=ENABLE_GAS_PARAMS),
                        ENABLE_BRAKE_PARAMS: Param(False, bool, live=True, depends_on=ENABLE_LONG_PARAMS),
                        BRAKE_MAX_BP: Param([0., 20, 33], [list, float, int], live=True, depends_on=ENABLE_BRAKE_PARAMS),
                        BRAKE_MAX_V: Param([0.5, 0.5, 0.5], [list, float], live=True, depends_on=ENABLE_BRAKE_PARAMS),
                        ENABLE_LONG_PID_PARAMS: Param(False, bool, live=True, depends_on=ENABLE_LONG_PARAMS),
                        LONG_PID_KP_BP: Param([0., 5., 35.], [list, float, int], live=True, depends_on=ENABLE_LONG_PID_PARAMS),
                        LONG_PID_KP_V: Param([3.6, 2.4, 1.5], [list, float, int], live=True, depends_on=ENABLE_LONG_PID_PARAMS),
                        LONG_PID_KI_BP: Param([0., 35.], [list, float, int], live=True, depends_on=ENABLE_LONG_PID_PARAMS),
                        LONG_PID_KI_V: Param([0.54, 0.36], [list, float, int], live=True, depends_on=ENABLE_LONG_PID_PARAMS),
                        LONG_PID_KF: Param(1., VT.number, live=True, depends_on=ENABLE_LONG_PID_PARAMS),
                        LONG_PID_SAT_LIMIT: Param(0.8, VT.number, live=True, depends_on=ENABLE_LONG_PID_PARAMS),
                        ENABLE_LONG_DEADZONE_PARAMS: Param(False, bool, live=True, depends_on=ENABLE_LONG_PARAMS),
                        LONG_DEADZONE_BP: Param([0., 9.], [list, float, int], live=True, depends_on=ENABLE_LONG_DEADZONE_PARAMS),
                        LONG_DEADZONE_V: Param([0., .15], [list, float, int], live=True, depends_on=ENABLE_LONG_DEADZONE_PARAMS),
                        }

    self._params_file = '/data/op_params.json'
    self._backup_file = '/data/op_params_corrupt.json'
    self._last_read_time = sec_since_boot()
    self.read_frequency = 3  # max frequency to read with self.get(...) (sec)
    self._to_delete = []  # a list of unused params you want to delete
    self._run_init()  # restores, reads, and updates params

  def _run_init(self):  # does first time initializing of default params
    # Two required parameters for opEdit
    self.fork_params['username'] = Param(None, [type(None), str, bool], 'Your identifier provided with any crash logs sent to Sentry.\nHelps the developer reach out to you if anything goes wrong')
    self.fork_params['op_edit_live_mode'] = Param(False, bool, 'This parameter controls which mode opEdit starts in', hidden=True)
    self.fork_params["uniqueID"] = Param(None, [type(None), str], 'User\'s unique ID', hidden=True)
    self.params = self._get_all_params(default=True)  # in case file is corrupted
    
    for k, p in self.fork_params.items():
      d = p.depends_on
      while d:
        fp = self.fork_params[d]
        fp.children.append(k)
        d = fp.depends_on

    if travis:
      return

    if os.path.isfile(self._params_file):
      if self._read():
        to_write = self._add_default_params()  # if new default data has been added
        to_write |= self._delete_old()  # or if old params have been deleted
      else:  # backup and re-create params file
        error("Can't read op_params.json file, backing up to /data/op_params_corrupt.json and re-creating file!")
        to_write = True
        if os.path.isfile(self._backup_file):
          os.remove(self._backup_file)
        os.rename(self._params_file, self._backup_file)
    else:
      to_write = True  # user's first time running a fork with op_params, write default params

    if to_write:
      self._write()
      os.chmod(self._params_file, 0o764)
      
  def create_id(self):  # creates unique identifier to send with sentry errors. please update uniqueID with op_edit.py to your username!
    need_id = False
    if "uniqueID" not in self.fork_params:
      need_id = True
    if "uniqueID" in self.fork_params and self.get('uniqueID') is None:
      need_id = True
    if need_id:
      random_id = ''.join([random.choice(string.ascii_lowercase + string.digits) for i in range(15)])
      self.put('uniqueID', random_id)
      
  def get(self, key=None, force_live=False):  # key=None is dict of all params
    if key is None:
      return self._get_all_params(to_update=force_live)

    self._check_key_exists(key, 'get')
    param_info = self.fork_params[key]
    self._update_params(param_info.live or force_live)
    value = self.params[key]
    if param_info.is_valid(value):
      return value  # all good, returning user's value

    warning('User\'s value type is not valid! Returning default')  # somehow... it should always be valid
    return param_info.default  # return default value because user's value of key is not in allowed_types to avoid crashing openpilot

  def put(self, key, value):
    self._check_key_exists(key, 'put')
    if not self.fork_params[key].is_valid(value):
      raise Exception('opParams: Tried to put a value of invalid type!')
    self.params.update({key: value})
    self._write()

  def delete(self, key):  # todo: might be obsolete. remove?
    if key in self.params:
      del self.params[key]
      self._write()

  def _check_key_exists(self, key, met):
    if key not in self.fork_params:
      raise Exception('opParams: Tried to {} an unknown parameter! Key not in fork_params: {}'.format(met, key))

  def _add_default_params(self):
    added = False
    for key, param in self.fork_params.items():
      if key not in self.params:
        self.params[key] = param.default
        added = True
      elif not param.is_valid(self.params[key]):
        warning('Value type of user\'s {} param not in allowed types, replacing with default!'.format(key))
        self.params[key] = param.default
        added = True
    return added

  def _delete_old(self):
    deleted = False
    for param in self._to_delete:
      if param in self.params:
        del self.params[param]
        deleted = True
    return deleted

  def _get_all_params(self, default=False, return_hidden=False, to_update=False):
    self._update_params(to_update)
    if default:
      return {k: p.default for k, p in self.fork_params.items()}
    return {k: self.params[k] for k, p in self.fork_params.items() if k in self.params and (not p.hidden or return_hidden)}

  def _update_params(self, to_update):
    if not travis and sec_since_boot() - self._last_read_time >= self.read_frequency and to_update:  # make sure we aren't reading file too often
      if self._read():
        self._last_read_time = sec_since_boot()

  def __getitem__(self, s):  # can also do op_params['param_name']
    return self.get(s)

  def _read(self):
    try:
      with open(self._params_file, "r") as f:
        self.params = json.loads(f.read())
      return True
    except Exception as e:
      error(e)
      return False

  def _write(self):
    if not travis or os.path.isdir("/data/"):
      try:
        with open(self._params_file, "w") as f:
          f.write(json.dumps(self.params, indent=2))  # can further speed it up by remove indentation but makes file hard to read
        return True
      except Exception as e:
        print("Unable to write file: " + str(e))
        return False


ENABLE_COASTING = 'enable_coasting'
COAST_SPEED = 'coast_speed'
SETPOINT_OFFSET = 'setpoint_offset'
DOWNHILL_INCLINE = 'downhill_incline'
ALWAYS_EVAL_COAST = 'always_eval_coast_plan'
EVAL_COAST_LONG = 'eval_coast_long_controller'

ENABLE_LONG_PARAMS = 'enable_long_params'
ENABLE_GAS_PARAMS = 'enable_gas_params'
GAS_MAX_BP = 'gas_max_bp'
GAS_MAX_V = 'gas_max_v'
ENABLE_BRAKE_PARAMS = 'enable_brake_params'
BRAKE_MAX_BP = 'brake_max_bp'
BRAKE_MAX_V = 'brake_max_v'
ENABLE_LONG_PID_PARAMS = 'enable_long_pid_params'
LONG_PID_KP_BP = 'long_pid_kp_bp'
LONG_PID_KP_V = 'long_pid_kp_v'
LONG_PID_KI_BP = 'long_pid_ki_bp'
LONG_PID_KI_V = 'long_pid_ki_v'
LONG_PID_KF = 'long_pid_kf'
LONG_PID_SAT_LIMIT = 'long_pid_sat_limit'
ENABLE_LONG_DEADZONE_PARAMS = 'enable_long_deadzone_params'
LONG_DEADZONE_BP = 'long_deadzone_bp'
LONG_DEADZONE_V = 'long_deadzone_v'

