import os
import sys
path, panda_path = os.path.split(os.path.dirname(__file__))
sys.path.append(path)

import zmq
import json

from cereal import car, log
from common.realtime import sec_since_boot, set_realtime_priority, Ratekeeper
from selfdrive.services import service_list
from selfdrive.car import get_car
import selfdrive.messaging as messaging


def show_logcan(rate=100):
  context = zmq.Context()

  # pub
  sendcan = messaging.pub_sock(context, service_list['sendcan'].port)

  # sub
  logcan = messaging.sub_sock(context, service_list['can'].port)

  CC = car.CarControl.new_message()

  CI, CP = get_car(logcan, sendcan, None)

  print CP.carFingerprint

  print

  if CI is None:
    raise Exception("unsupported car")

  rk = Ratekeeper(rate)

  while True:
    CS = CI.update(CC)
    if(rk.frame % rate) == 1:
      print "Speed: %.2f  ACC: %.2f" % (CS.vEgo, CS.aEgo)
      print "WheelSpeeds: %.2f,%.2f,%.2f,%.2f" \
            % (CS.wheelSpeeds.fl, CS.wheelSpeeds.fr, CS.wheelSpeeds.rl, CS.wheelSpeeds.rr)
      print "EnableCruise", CP.enableCruise
      print "ButtonEvents", CS.buttonEvents

      print "Event", CS.events
    rk.keep_time()
    if CS.buttonEvents:
      print CS.buttonEvents
      #print CS.buttonEvents.pressed




def main(gctx=None):
  show_logcan()


if __name__ == "__main__":
  main()