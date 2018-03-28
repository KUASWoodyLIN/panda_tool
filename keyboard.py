# coding:utf-8
import os
import zmq
import selfdrive.messaging as messaging
from selfdrive.car.honda.carcontroller import CarController
from selfdrive.services import service_list
from selfdrive.car.honda import hondacan
from selfdrive.boardd.boardd import can_list_to_can_capnp
from common.numpy_fast import clip
from common.realtime import set_realtime_priority, Ratekeeper
from dashboard.lib.keyboardCatcher import KeyboardCatcher

def data_send(kb, frame, sendcan, accord, crv, GAS_MAX, BRAKE_MAX, STEER_MAX, GAS_OFFSET):
    #apply_gas, apply_brake, apply_steer= kb.get_data()
    # steer torque is converted back to CAN reference (positive when steering right)
    apply_gas = int(clip(kb.gas_press * GAS_MAX, 0, GAS_MAX - 1))
    apply_brake = int(clip(kb.brake_press * BRAKE_MAX, 0, BRAKE_MAX - 1))
    apply_steer = int(clip(-kb.steering_angle * STEER_MAX, -STEER_MAX, STEER_MAX))

    can_sends = []
    if accord:
      idx = frame % 2
      can_sends.append(hondacan.create_accord_steering_control(apply_steer, idx))
    else:
      idx = frame % 4
      can_sends.extend(hondacan.create_steering_control(apply_steer, crv, idx))
    if (frame % 2) == 0:
        idx = (frame / 2) % 4
        can_sends.append(hondacan.create_brake_command(apply_brake, pcm_override=True, pcm_cancel_cmd=True, chime=0, idx=idx))
        gas_amount = (apply_gas + GAS_OFFSET) * (apply_gas > 0)
        can_sends.append(hondacan.create_gas_command(gas_amount, idx))

    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())

def main():
    # lib
    kb = KeyboardCatcher()

    # loop rate 0.01s
    rate = 100
    frame = 0
    accord = False
    crv = False
    civic = False

    GAS_MAX = 1004
    BRAKE_MAX = 1024/4
    if civic:
      is_fw_modified = os.getenv("DONGLE_ID") in ['b0f5a01cf604185c']
      STEER_MAX = 0x1FFF if is_fw_modified else 0x1000
    elif crv:
      STEER_MAX = 0x300  # CR-V only uses 12-bits and requires a lower value
    else:
      STEER_MAX = 0xF00
    GAS_OFFSET = 328

    # start the loop
    set_realtime_priority(2)

    context = zmq.Context()

    # pub
    sendcan = messaging.pub_sock(context, service_list['sendcan'].port)

    rk = Ratekeeper(rate, print_delay_threshold=2. / 1000)

    while True:
        try:
            # catch lib input
            kb.catch_input()
            # publish data
            frame = frame + 1
            data_send(kb, frame, sendcan, accord, crv, GAS_MAX, BRAKE_MAX, STEER_MAX, GAS_OFFSET)
            # run loop at fixed rate
            rk.keep_time()
        except KeyboardInterrupt:
            kb.finish()
            exit()
        except Exception as e:
            print "exception", e


if __name__ == "__main__":
    main()
