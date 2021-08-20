function IRC = Construct_IRC()
%CONSTRUCT_IRC Summary of this function goes here
%   Detailed explanation goes here
IRC.turn_rps = 0; % turning velocity rad/s
IRC.motor_power = 0; % related to STO button SA
IRC.step_time = 0; 
IRC.ds = 0; 
IRC.Vx_tgd = 0;
IRC.Vy_tgd_avg = 0;
IRC.H = 0;
IRC.desired_com2stToe_lateral = 0;
IRC.direct_up_down = 0;
IRC.reset_IMU_KF = 0;
IRC.reset_bias = 0;
IRC.CL = 0;
end

