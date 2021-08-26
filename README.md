# Cassie_Controller_AngularMomentum
This repository is the implementation of the controller described in [One-Step Ahead Prediction of Angular Momentum about the Contact Point for Control of Bipedal Locomotion: Validation in a LIP-inspired Controller](https://arxiv.org/abs/2008.10763). It is based on Agility Robotics's [Cassie repository](https://github.com/agilityrobotics/cassie-doc)
1. Please use MATLAB 2017b to run the code. As far as we known, MATLAB 2018b requires additional settings in simulink models.
2. The simulink models are located in Cassie_Controller_AngularMomentum\CustomCodes\SimulinkModel. There are three simulink models in it.
	 - Ideal_Simulator is a simulator of Cassie assuming the spring is rigid. All states can be measured perfectly. A simple Input-Output Linearization method could be used to track reference trajectories for control variables like foot position, torso angle and etc. With this simulator we could focus on generating reference trajectories without worrying about imperfection from state estimation or trajectory tracking. After simulation is finished, we could run CustomCodes\IdealSimulator\StartAnimation.m to visualize the result.
	 - FG_Simulator is modified from Agility Robotics' original simulator. The spring is compliant in this simulator and thus more complicated reference tracking methods are used, including iterative inverse kinematics, spring compensation and Passivity-based Controller.
	 - FG_RealTime is used for Cassie experiment, it used the same controller block as FG_Simulator.

3. The simulations can be run directly with no need for modification. But if you have a Cassie robot and want to implement it in experiment, you would need to identify your Cassie's spring coefficients and change the spring compensation.
4. In this version of controller a simple state estimator is used to estimate the robot torso velocity so that the code could be run in Windows alone. For a better state estimation, please use InEKF described in this [paper](https://journals.sagepub.com/doi/full/10.1177/0278364919894385). An instruction could be found in this [repository](https://github.com/UMich-BipedLab/cassie_ros). A Linux operating system is required to run the InEKF.
5. The base of the robot is located at IMU instead of the default torso origin set by Agility Robotics (AR). All kinematics and dynamics functions use this convention and so is Ideal_Simulator. But FG_Simulator still use AR's convention.
6. Please install Microsoft Visual 2015 and make it the default compiler for MATLAB. Otherwise all MATLAB System blocks need to be run in interpretation mode instead of code generation mode. 
7. Run startup.m before running simulink models to add all folders to path and create a Build folder for compiled files.

Other notes:
1. There is no standing controller.
2. To test the controller in experiment, please start with the robot hung in the air, with torso 1 ~ 1.2m above the ground and two feet touching the ground right beneath the torso, 30 cm apart from each other. Slightly higher initial torso position is fine but a too low initial torso position would cause the legs jerk when motor power is turned on. Before turning on the STO button (SA button on the remote), on the remote please set the S1 to 0, S2 button to 0, LS button to 1, RS button to 0, SB to -1, SC to -1, SD to -1, SG to -1. 
