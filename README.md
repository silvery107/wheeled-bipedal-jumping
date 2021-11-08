# Introduction

<img src=".utils/../utils/ARM2021.jpg" width=400>

We establish a wheeled-bipedal jumping dynamical (W-JBD) model to optimize the height control, and the Bayesian optimization for torque planning (BOTP) method based on a joint optimization framework for torque planning to achieve accurate height control and minimal energy cost.

# Citation

Y. Zhuang et al., "*Height Control and Optimal Torque Planning for Jumping With Wheeled-Bipedal Robots*," 2021 6th IEEE International Conference on Advanced Robotics and Mechatronics (ICARM), 2021, pp. 477-482, doi: 10.1109/ICARM52023.2021.9536196.

# Dependency

- **Webots R2021a**
- **Python 3.7**

# Structure

```
├── contrllers
│   ├── my_controller_python
│   ├── ground
│   └── linear
├── advisor_config
│   ├── config.json
│   ├── min_function.py
│   ├── quick_start.md
│   └── sdk.md
├── worlds
│   └── ...
├── utils
│   └── ...
├── README.md
├── requirements.txt
├── robot_specifications.txt
└── ...
```

# Qucik Start

## Simulation Setup
1. Open `*.wbt` file in \<worlds\>
2. Control the robot using keyboard:

	|Motion|Command|
	|-|-|
	|Jump|Space|
	|Forward|W|
	|Backward|S|
	|Left Turn|A|
	|Right Turn|D|
	|Halt Turn|F|
	|Squat Down|↓|
	|Stand Up|↑|

## Optimization Setup
\<TODO\>

# TODO
- [ ] Add optimization setup instructions.
- [ ] Format data collection methods.
# Contributors
SilverSoul, Sure, Penson, Amanda, YZ, Xin Yang