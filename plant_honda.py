import os
import sys
openpilot_path, _ = os.path.split(os.getcwd())
sys.path.append(openpilot_path)

import numpy as np
from panda import Panda
from opendbc import DBC_PATH
from common.realtime import Ratekeeper
from selfdrive.config import CruiseButtons
from selfdrive.car.honda.hondacan import fix
from selfdrive.car.honda.carstate import get_can_signals
from selfdrive.car.honda.interface import CarInterface
from common.dbc import dbc
from selfdrive.test.plant.plant import car_plant, get_car_can_parser, to_3_byte, to_3s_byte

honda = dbc(os.path.join(DBC_PATH, "honda_civic_touring_2016_can.dbc"))

# Trick: set 0x201 (interceptor) in fingerprints for gas is controlled like if there was an interceptor
CP = CarInterface.get_params("HONDA CIVIC 2016 TOURING", {0x201})

#serial = u'1f0032000651363038363036'  # recv
serial = u'520039000651363038363036'    # send


class PlantHonda(object):
  can_bus_initialized = False

  def __init__(self, lead_relevancy=False, rate=100, distance_lead=2.0):
    self.rate = rate
    self.civic = True
    self.brake_only = False

    if not PlantHonda.can_bus_initialized:
      if serial in Panda.list():
        self.p = Panda(serial)
        self.p.set_safety_mode(self.p.SAFETY_ALLOUTPUT)
        #self.p.can_clear(0)
        print 'Connect Panda [RECV]'
        PlantHonda.can_bus_initialized = True
      else:
        print 'Not Panda connect'
        exit()

    self.angle_steer = 0.
    self.gear_choice = 0
    self.speed, self.speed_prev = 0., 0.

    self.esp_disabled = 0
    self.main_on = 1
    self.user_gas = 0
    self.computer_brake, self.user_brake = 0, 0
    self.brake_pressed = 0
    self.angle_steer_rate = 0
    self.distance, self.distance_prev = 0., 0.
    self.speed, self.speed_prev = 0., 0.
    self.steer_error, self.brake_error, self.steer_not_allowed = 0, 0, 0
    self.gear_shifter = 8   # D gear
    self.pedal_gas = 0
    self.cruise_setting = 0

    self.seatbelt, self.door_all_closed = True, True
    self.steer_torque, self.v_cruise, self.acc_status = 0, 0, 0  # v_cruise is reported from can, not the one used for controls

    self.lead_relevancy = lead_relevancy    # True or False

    # lead car
    self.distance_lead, self.distance_lead_prev = distance_lead, distance_lead

    self.rk = Ratekeeper(rate, print_delay_threshold=100)
    self.ts = 1./rate

    self.cp = get_car_can_parser()

  def close(self):
    self.p.close()

  def speed_sensor(self, speed):
    if speed < 0.3:
      return 0
    else:
      return speed

  def current_time(self):
    return float(self.rk.frame) / self.rate

  def run(self, v_lead=0.0, cruise_buttons=None, grade=0.0, publish_model=True):
    gen_dbc, gen_signals, gen_checks = get_can_signals(CP)
    sig_name = [s[0] for s in gen_signals]
    sig_address = [s[1] for s in gen_signals]
    cks_address = set(check[0] for check in gen_checks)
    cks_address.add(0x18F)
    cks_address.add(0x30C)

    # ******** get messages sent to the car ********
    can_msgs = self.p.can_recv()
    can_msgs_bytes = []
    for address, _, dat, src in can_msgs:
      can_msgs_bytes.append((address, 0, bytes(dat), src))
    self.cp.update_can(can_msgs_bytes)
    if self.cp.vl[0x1fa]['COMPUTER_BRAKE_REQUEST']:
      brake = self.cp.vl[0x1fa]['COMPUTER_BRAKE']
    else:
      brake = 0.0

    if self.cp.vl[0x200]['GAS_COMMAND'] > 0:
      gas = self.cp.vl[0x200]['GAS_COMMAND'] / 256.0
    else:
      gas = 0.0

    if self.cp.vl[228]['STEER_TORQUE_REQUEST']:
      steer_torque = self.cp.vl[0xe4]['STEER_TORQUE'] * 1.0 / 0xf00
    else:
      steer_torque = 0.0

    distance_lead = self.distance_lead_prev + v_lead * self.ts

    # ***************** run the car ****************
    speed, acceleration = car_plant(self.distance_prev, self.speed_prev, grade, gas, brake)
    standstill = (speed == 0)
    distance = self.distance_prev + speed * self.ts
    speed = self.speed_prev + self.ts * acceleration
    if speed <= 0:
      speed = 0
      acceleration = 0

    # ****************** lateral *******************
    self.angle_steer -= (steer_torque/10.0) * self.ts

    # **************** radar model *****************
    if self.lead_relevancy:
      d_rel = np.maximum(0., distance_lead - distance)
      v_rel = v_lead - speed
    else:
      d_rel = 200.
      v_rel = 0.
      a_rel = 0
    lateral_pos_rel = 0.

    # print at 5hz
    if (self.rk.frame%(self.rate/5)) == 0:
      print "%6.2f m  %6.2f m/s  %6.2f m/s2   %.2f ang   gas: %.2f  brake: %.2f  steer: %5.2f    lead_rel: %6.2f m  %6.2f m/s" \
            % (distance, speed, acceleration, self.angle_steer, gas, brake, steer_torque, d_rel, v_rel)
      print "distance_lead_prev %f  distance_lead %f  d_rel %f" \
            % (self.distance_lead_prev, distance_lead, d_rel)
      health = self.p.health()
      print 'voltage: ', health['voltage'], 'current: ', health['current'], 'started: ', health['started'], \
            'controls_allowed', health['controls_allowed']

    # *************** publish the car **************
    vls = [self.speed_sensor(speed), self.speed_sensor(speed), self.speed_sensor(speed), self.speed_sensor(speed), self.speed_sensor(speed),
           self.angle_steer, self.angle_steer_rate, 0, self.gear_choice, speed!=0,
           0, 0, 0, 0,
           self.v_cruise, not self.seatbelt, self.seatbelt, self.brake_pressed, 0.,
           self.user_gas, cruise_buttons, self.esp_disabled, 0,
           self.user_brake, self.steer_error, self.brake_error,
           self.brake_error, self.gear_shifter, self.main_on, self.acc_status,
           self.pedal_gas, self.cruise_setting,
           # append one more zero for gas interceptor
           0, 0, 0, 0, 0, 0]

    # TODO: publish each message at proper frequency
    can_msgs = []
    for msg in set(sig_address):
      msg_struct = {}
      indxs = [i for i, x in enumerate(sig_address) if msg == sig_address[i]]
      for i in indxs:
        msg_struct[sig_name[i]] = vls[i]

      if "COUNTER" in honda.get_signals(msg):
        msg_struct["COUNTER"] = self.rk.frame % 4

      msg_data = honda.encode(msg, msg_struct)

      if "CHECKSUM" in honda.get_signals(msg):
        msg_data = fix(msg_data, msg)

      can_msgs.append([msg, 0, msg_data, 0])

    # add the radar message
    # TODO: use the DBC
    if self.rk.frame % 5 == 0:
      radar_state_msg = '\x79\x00\x00\x00\x00\x00\x00\x00'
      radar_msg = to_3_byte(d_rel*16.0) + \
                  to_3_byte(int(lateral_pos_rel*16.0)&0x3ff) + \
                  to_3s_byte(int(v_rel*32.0)) + \
                  "0f00000"
      can_msgs.append([0x400, 0, radar_state_msg, 1])
      can_msgs.append([0x445, 0, radar_msg.decode("hex"), 1])
    self.p.can_send_many(can_msgs)

    # **************** update prevs ****************
    self.speed = speed
    self.distance = distance
    self.distance_lead = distance_lead

    self.speed_prev = speed
    self.distance_prev = distance
    self.distance_lead_prev = distance_lead

    self.rk.keep_time()


# simple engage in standalone mode
def plant_thread(rate=100):
  plant = PlantHonda(rate=rate)
  while 1:
    if 20 <= plant.rk.frame % 100 <= 25:
      cruise_buttons = CruiseButtons.RES_ACCEL
    else:
      cruise_buttons = 0
    plant.run(cruise_buttons=cruise_buttons)


if __name__ == "__main__":
  plant_thread()
