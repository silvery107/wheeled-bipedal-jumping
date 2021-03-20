#!/usr/bin/env python

import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("-x", type=float, default=0.0)
args = parser.parse_args()


def main():
  # Read parameters
  x = args.x

  ## Compute or learning

  # write args to txt
  with open("../controllers/my_controller_python/args.txt",'w') as f1:
    f1.write(str(x))

  # run webots
  os.system('webots --mode=fast --no-rendering')

  # read metrics from txt
  with open("../controllers/my_controller_python/metrics.txt",'r') as f2:
    y = f2.read()

  # Output the metrics
  print(y)


if __name__ == "__main__":
  main()
