================介绍================

轮式双足机器人的建模和控制
   
================软件架构说明================
    -worlds：包含各种wbt文件
    -contrllers：包含控制文件
	-linear：场景中运动杆的控制器
	-my_controller_python：机器人控制器
		-my_controller_python.py：主文件
		-panel.py：仪表盘，接收、计算、传递各项状态参数
		-motion.py：动作集，实现各种功能
		-PID_controller.py：PID
		-parameter.txt：保存运动参数
		-requriements.txt

================安装教程================

        安装webots
        需要python3.7
        安装相应依赖，若有自信可跳过此步：找到requriements.txt，在此目录下执行：pip install -r requriements.txt

================使用说明================

       双击任意worlds下的wbt文件，打开仿真。
       机器人控制方法：
	前进后退：“W”，“S”
                左右：按一下“A”或“D”即可转向
	停止转向：“F”
	蹲起：方向键↓、方向键↑
	跳跃：空格

================参与贡献================
Penson, SilverSoul, Sure, YZ, Amanda