#!/usr/bin/env python

import sys
import os
import struct
import binascii

path, panda_path = os.path.split(os.getcwd())
print'Openpilot path: ', path

from collections import defaultdict
from panda import Panda
from common.realtime import Ratekeeper
from selfdrive.test.plant.plant import get_car_can_parser, car_plant

serial = u'1f0032000651363038363036'  # recv
# serial = u'520039000651363038363036'    # send


def check_send_alive(rk, rate, panda):
    """ Panda to Panda, need to use this function before sending message"""
    send_alive = False
    while not send_alive:
        panda.can_send(0x1aa, 'Start', 0)
        can_msgs = panda.can_recv()
        for address, _, dat, src in can_msgs:
            send_alive = True if address == 426 and dat == 'Start' else False
        if (rk.frame % rate) == 1:
            print can_msgs
        rk.keep_time()


def main(rate=100):
    panda_list = Panda.list()
    print panda_list
    rk = Ratekeeper(rate)

    cp = get_car_can_parser()

    # init
    distance, distance_prev = 0., 0.
    speed, speed_prev = 0., 0.
    angle_steer = 0.
    grade = 0.0
    ts = 1./rate

    if serial in panda_list:
        panda = Panda(serial)
        panda.set_safety_mode(panda.SAFETY_ALLOUTPUT)
        panda.can_clear(0)
        print 'Connect Panda [Recv]'
        check_send_alive(rk, rate, panda)

        while True:

            # ******** get messages sent to the car *********
            can_msgs = panda.can_recv()
            can_msgs_bytes = []
            for address, _, dat, src in can_msgs:
                can_msgs_bytes.append((address, 0, bytes(dat), src))
            cp.update_can(can_msgs_bytes)
            if cp.vl[0x1fa]['COMPUTER_BRAKE_REQUEST']:
                brake = cp.vl[0x1fa]['COMPUTER_BRAKE']
            else:
                brake = 0.0

            if cp.vl[0x200]['GAS_COMMAND'] > 0:
                gas = cp.vl[0x200]['GAS_COMMAND'] / 256.0
            else:
                gas = 0.0

            if cp.vl[228]['STEER_TORQUE_REQUEST']:
                steer_torque = cp.vl[0xe4]['STEER_TORQUE'] * 1.0 / 0xf00
            else:
                steer_torque = 0.0

            # ***************** run the car *****************
            speed, acceleration = car_plant(distance_prev, speed_prev, grade, gas, brake)
            distance = distance_prev + speed * ts
            speed = speed_prev + acceleration * ts
            if speed <= 0:
                speed = 0
                acceleration = 0

            # ******************* lateral *******************
            angle_steer -= (steer_torque / 10.0) * ts

            # ***************** update prev *****************
            speed_prev = speed
            distance_prev = distance

            # **************** print message ****************
            if (rk.frame % rate) == 1:
                # if can_msgs:
                #     print can_msgs

                # print 'gas: ', gas, \
                #     'brake: ', brake, \
                #     'steer: ', steer_torque

                print "gas: %.2f  brake: %.2f  steer: %5.2f  %6.2f m  %6.2f m/s  %6.2f m/s2  %.2f ang" \
                      % (gas, brake, steer_torque, distance, speed, acceleration, angle_steer)

                health = panda.health()
                print 'voltage: ', health['voltage'], 'current: ', health['current'], 'started: ', health['started']

            rk.keep_time()
    else:
        print 'Panda connent fald'


if __name__ == '__main__':
    main()
