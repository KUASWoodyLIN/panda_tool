import os
import sys
import time
path = os.path.dirname(os.path.abspath(__file__))
openpilot_path, _ = os.path.split(path)
sys.path.append(openpilot_path)
os.environ['OLD_CAN'] = '1'
os.environ['NOCRASH'] = '1'
print sys.path

import selfdrive.manager as manager

def main():
  try:
    manager.gctx = {}
    manager.prepare_managed_process('radard')
    manager.prepare_managed_process('controlsd')
    manager.prepare_managed_process('panda_boardd')
    #manager.prepare_managed_process('pandad')

    manager.start_managed_process('radard')
    manager.start_managed_process('controlsd')
    manager.start_managed_process('panda_boardd')
    #manager.start_managed_process('pandad')
  except KeyboardInterrupt:
    manager.kill_managed_process('radard')
    manager.kill_managed_process('controlsd')
    manager.start_managed_process('panda_boardd')
    #manager.kill_managed_process('pandad')
    time.sleep(5)





if __name__ == "__main__":
  main()
