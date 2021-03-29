#!/usr/bin/env python

import argparse
import os

parser = argparse.ArgumentParser()
# parser.add_argument("-a", type=float, default=0.0)
parser.add_argument("-jump_a", type=float, default=0.0)
parser.add_argument("-jump_b", type=float, default=0.0)
parser.add_argument("-jump_c", type=float, default=0.0)
parser.add_argument("-jump_d", type=float, default=0.0)
parser.add_argument("-opt_vel", type=float, default=0.0)
args = parser.parse_args()


def main():
  # Read parameters
  # a = args.a
  # b = args.b#
  # c = args.c  #
  param_dic = {
    # "restart_torque":args.a,
               "jump_a":args.jump_a,
               "jump_b":args.jump_b,
               "jump_c":args.jump_c,
               "jump_d":args.jump_c,
               "opt_vel":args.opt_vel}

  ## Compute or learning

  # write args to txt
  with open("../controllers/my_controller_python/args.txt",'w') as f1:
    f1.write(str(param_dic))

  # run webots
  os.system("sshpass -p 082003 ssh -p 22 SSH_Server@10.17.86.40")
  os.system("webots --mode=fast --no-rendering --stdout --stderr --minimize")

  # read metrics from txt
  with open("../controllers/my_controller_python/metrics.txt",'r') as f2:
    metrics_dic = eval(f2.read())
    # y = metrics_dic["restart_metrics"]
    y = metrics_dic["jump_metrics"]


  # Output the metrics
  print(y)


if __name__ == "__main__":
  main()
