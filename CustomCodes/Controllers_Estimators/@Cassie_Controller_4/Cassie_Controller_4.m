%Yukai controller.

classdef Cassie_Controller_4 <matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime %#codegen
    % PROTECTED PROPERTIES ====================================================
    properties
        Kp_pitch;
        Kd_pitch;
        Kp_roll;
        Kd_roll;
        Kp_yaw;
        Kd_yaw;
        
        Kp_abduction;
        Kp_rotation;
        Kp_thigh;
        Kp_knee;
        Kp_toe;
        
        Kd_abduction;
        Kd_rotation;
        Kd_thigh;
        Kd_knee;
        Kd_toe;
        
        u_abduction_swing_cp;
        u_roll_cp;
        u_pitch_cp
        u_knee_cp;
        
        CovPos_abduction;
        CovPos_rotation;
        CovPos_thigh;
        CovPos_knee;
        CovPos_toe;
        CovPos_qj1;
        CovPos_qj2;
        CovPos_qj3;
        
        CovVel_abduction;
        CovVel_rotation;
        CovVel_thigh;
        CovVel_knee;
        CovVel_toe;
        CovVel_qj1;
        CovVel_qj2;
        CovVel_qj3;
        
        Cov_Euler;
        Cov_Euler_rates;
        
        Cov_Lx_LRToe_stTD0;
        Cov_Ly_LRToe_stTD0;   
        Cov_rpx_LRToe_stTD0;
        Cov_rpy_LRToe_stTD0;

        
        torque_transition_time;
        knee_cp_time;
        com_x_offset;
        Vx_tgd_offset;
        Vy_tgd_offset;
        
        stance_thre_lb;
        stance_thre_ub;
        
        IK_max_iter_num;
        
        rp_LRToe_fil_param;
        use_dq_AR_for_PD;
        
        sample_time;
    end
    properties (Access = private, Constant)
        TorqueLimits = repmat([112.5;112.5;195.2;195.2;45],[2,1]);
        ActuatorLimits = [-0.2618, 0.3927;    -0.3927, 0.3927;    -0.8727, 1.3963;    -2.8623, -0.7330;   -2.4435, -0.5236; ...
            -0.3927, 0.2618;    -0.3927, 0.3927;    -0.8727, 1.3963;    -2.8623, -0.7330;   -2.4435, -0.5236];
        ActuatorLimits_ARsafety = [-0.1618, 0.2927;    -0.2927, 0.2927;    -0.7727, 1.2963;    -2.7623, -0.8330;   -2.3435, -0.6236; ...
            -0.2927, 0.1618;    -0.2927, 0.2927;    -0.7727, 1.2963;    -2.7623, -0.8330;   -2.3435, -0.6236];
        Ks1 = 1500;
        Ks2 = 1250;
        
        total_mass = 32;
        
    end
    properties (Access = protected)
        
    end
    % PRIVATE PROPERTIES ====================================================
    properties(Access = private) % change when swing leg change
        rp_swToe_ini = zeros(3,1);
        rv_swToe_ini = zeros(3,1);
        rp_stToe_z_ini = 0;
        stToe_pos = zeros(3,1);
        swToe_pos = zeros(3,1);
        
        stTD0 = 0; % direction of stance toe at the beginning of a step
        tg_direction = 0;
        st_rotation_goal = 0;
        sw_rotation_goal = 0;
        st_rotation_goal_last = 0;
        sw_rotation_goal_last = 0;
        
        u_last = zeros(10,1);
    end
    properties(Access = private) % changes iteration
        initialized = 0;
        % for filters
        sigma_Lx_stToe_stTD0 = 100;
        Lx_stToe_stTD0_kf = 0;
        sigma_Ly_stToe_stTD0 = 100;
        Ly_stToe_stTD0_kf = 0;
        % IK solver
        qd_control = zeros(8,1);
        dqd_control = zeros(8,1);
        % previous iteration's variables 
        u_prev = zeros(10,1);
        Ki_knee = 2000;
        iqe_control_knee = 0;
        % filtered rp_LRToe
        rp_LToe_fil = zeros(3,1);
        rp_RToe_fil = zeros(3,1);
    end
    properties(Access = private) % low level controls
        hr = zeros(8,1);
        dhr = zeros(8,1);
        ddhr = zeros(8,1);
        h0 = zeros(8,1);
        dh0 = zeros(8,1);
    end
    properties(Access = private) % parameters
        
    end
    
    
    % PROTECTED METHODS =====================================================
    methods (Access = protected)
        
        function [userInputs, Data] = stepImpl(obj,EstStates,t_total,isSim,IRC)
            %STEPIMPL System output and state update equations.
            
            %% Initialize --------------------------------------------------------
            
            Data = PreFunctions.Construct_Data;
            % Reset the desired motor torques to zero in case they aren't defined
            userInputs = CassieModule.getUserInStruct;
            u = zeros(10,1);
            
            if t_total>0.001 && EstStates.isCalibrated
                % Let the output be torso angle, com height and delta x,delta z of swing
                % feet and com. delta = p_com - p_swfeet.
                
                g = 9.81;
%                 H = 0.8; % desired com height
                CL = 0.1; % foot clearance
                if isSim == 2
                    spring_height_compensation = 0;
                else
                    spring_height_compensation = 0.00;
                end
                g=9.81;
                Kd = 50;
                Kp = 600;
                %% Read Command
                Vx_tgd_no_offset = IRC.Vx_tgd; % Desired velocity in target yaw direction at the end of a step
                H = IRC.H; % desired com height
                desired_com2stToe_lateral = IRC.desired_com2stToe_lateral; % at the begining of left stance (also at the end of left stance)
                Vy_tgd_no_offset = -EstStates.stanceLeg*(-desired_com2stToe_lateral*sqrt(g/H)*sinh(sqrt(g/H)*IRC.step_time)/(1+cosh(sqrt(g/H)*IRC.step_time))); % initial ( and negative end) velocity required to achieve rp_com2stToe_desired
                Vy_tgd_no_offset = Vy_tgd_no_offset + IRC.Vy_tgd_avg;
                
                    Vx_tgd = Vx_tgd_no_offset + obj.Vx_tgd_offset;
                    Vy_tgd = Vy_tgd_no_offset + obj.Vy_tgd_offset;
                
                direct_up_down = IRC.direct_up_down;
                
                q = EstStates.q;
                dq = EstStates.dq;
                q_ss = EstStates.q; % second set of q, used for low level PD control
                if obj.use_dq_AR_for_PD
                    dq_ss = EstStates.dq_AR;
                else
                    dq_ss = EstStates.dq;
                end
                s = EstStates.s;
                t = EstStates.t;
                stanceLeg = EstStates.stanceLeg;
                LegSwitch = EstStates.LegSwitch;
                GRF_z = EstStates.GRF_z;
                
                ds = IRC.ds;
                
                if obj.initialized == 0 || IRC.motor_power == 0
                    obj.tg_direction = q(4);
                end
                   
                s_sat = min([1,s]);
                
                
                %% Observations
                p_com = p_COM(q);
                p_com = p_com + YToolkits.Angles.Rz(q(4))*[obj.com_x_offset;0;0];
                Jp_com = Jp_COM(q);
                dJp_com = dJp_COM(q,dq);
                v_com = Jp_com*dq;
                
                p_LToe = p_LeftToeJoint(q);
                Jp_LToe = Jp_LeftToeJoint(q);
                dJp_LToe = dJp_LeftToeJoint(q,dq);
                v_LToe = Jp_LToe*dq;
                
                p_RToe = p_RightToeJoint(q);
                Jp_RToe = Jp_RightToeJoint(q);
                dJp_RToe = dJp_RightToeJoint(q,dq);
                v_RToe = Jp_RToe*dq;
                
                
                % com position RELATIVE to toes
                
                rp_LToe = p_com - p_LToe;
                Jrp_LToe = Jp_com - Jp_LToe;
                dJrp_LToe = dJp_com - dJp_LToe;
                rv_LToe = v_com - v_LToe;
                
                rp_RToe = p_com - p_RToe;
                Jrp_RToe = Jp_com - Jp_RToe;
                dJrp_RToe = dJp_com - dJp_RToe;
                rv_RToe = v_com - v_RToe;
                
                
                LG = getCassieAngularMomentum(p_com,[q;dq]);
                JL_dq_LToe = Jdq_AMworld_about_pA(q,dq,p_LToe,zeros(3,20));
                JL_dq_RToe = Jdq_AMworld_about_pA(q,dq,p_RToe,zeros(3,20));
                L_LToe = JL_dq_LToe*dq;
                L_RToe = JL_dq_RToe*dq;
                
                L_LToe_obs = L_LToe;
                L_RToe_obs = L_RToe;
                
                L_LToe_stTD0_obs = YToolkits.Angles.Rz( - obj.stTD0) * L_LToe_obs;
                L_RToe_stTD0_obs = YToolkits.Angles.Rz( - obj.stTD0) * L_RToe_obs;
                
                % add filters for some raw measurement
                obj.rp_LToe_fil = YToolkits.first_order_filter(obj.rp_LToe_fil, rp_LToe , obj.rp_LRToe_fil_param);
                obj.rp_RToe_fil = YToolkits.first_order_filter(obj.rp_RToe_fil, rp_RToe , obj.rp_LRToe_fil_param);
                % order the index of stance leg and swing leg
                if stanceLeg == -1 % right stanceleg
                    st_abduction = 7;
                    st_rotation = 8;
                    st_thigh = 9;
                    st_knee = 10;
                    st_shin = 11;
                    st_ankle = 12;
                    st_toe = 13;
                    sw_abduction = 14;
                    sw_rotation = 15;
                    sw_thigh = 16;
                    sw_knee = 17;
                    sw_shin = 18;
                    sw_ankle = 19;
                    sw_toe =20;
                    
                    st_motor_abduction = 1;
                    st_motor_rotation = 2;
                    st_motor_thigh = 3;
                    st_motor_knee = 4;
                    st_motor_toe = 5;
                    sw_motor_abduction = 6;
                    sw_motor_rotation = 7;
                    sw_motor_thigh = 8;
                    sw_motor_knee = 9;
                    sw_motor_toe = 10;
                    
                    abduction_direction = 1; % when the hip abduct outside, the sign is positive
                else
                    sw_abduction = 7;
                    sw_rotation = 8;
                    sw_thigh = 9;
                    sw_knee = 10;
                    sw_shin = 11;
                    sw_ankle = 12;
                    sw_toe = 13;
                    st_abduction = 14;
                    st_rotation = 15;
                    st_thigh = 16;
                    st_knee = 17;
                    st_shin = 18;
                    st_ankle = 19;
                    st_toe =20;
                    
                    sw_motor_abduction = 1;
                    sw_motor_rotation = 2;
                    sw_motor_thigh = 3;
                    sw_motor_knee = 4;
                    sw_motor_toe = 5;
                    st_motor_abduction = 6;
                    st_motor_rotation = 7;
                    st_motor_thigh = 8;
                    st_motor_knee = 9;
                    st_motor_toe = 10;
                    
                    abduction_direction = -1; % when the hip abduct outside, the sign is negative
                end
                
                if LegSwitch == 1
                    obj.u_last = obj.u_prev;
                    
                    obj.tg_direction = YToolkits.wrap_to_Pi(obj.tg_direction + IRC.turn_rps*IRC.step_time);
                    obj.stTD0 = YToolkits.wrap_to_Pi((q(4) + q(st_rotation))); % stTD0 means stToe_direction_0 

                    
                    obj.st_rotation_goal_last = obj.hr(4);
                    obj.sw_rotation_goal_last = obj.hr(3);
                    obj.st_rotation_goal = - 0.5* YToolkits.wrap_to_Pi( obj.tg_direction - obj.stTD0);
                    obj.sw_rotation_goal = 0.5* YToolkits.wrap_to_Pi( obj.tg_direction - obj.stTD0);
                    % update L_*Toe_stTD0_obs if stTD0 is changed in this
                    % iteration
                    L_LToe_stTD0_obs = YToolkits.Angles.Rz( - obj.stTD0) * L_LToe_obs;
                    L_RToe_stTD0_obs = YToolkits.Angles.Rz( - obj.stTD0) * L_RToe_obs;
                    
                    obj.iqe_control_knee = 0;
                    if stanceLeg == -1
                        obj.rp_swToe_ini = rp_RToe;
                        obj.rv_swToe_ini = rv_RToe;
                        obj.Lx_stToe_stTD0_kf = L_LToe_stTD0_obs(1);
                        obj.Ly_stToe_stTD0_kf = L_LToe_stTD0_obs(2);
                        obj.sigma_Lx_stToe_stTD0 = obj.Cov_Lx_LRToe_stTD0;
                        obj.sigma_Ly_stToe_stTD0 = obj.Cov_Ly_LRToe_stTD0;
                        
                        obj.rp_stToe_z_ini = rp_LToe(3);
                    else
                        obj.rp_swToe_ini = rp_LToe;
                        obj.rv_swToe_ini = rv_LToe;
                        obj.Lx_stToe_stTD0_kf = L_RToe_stTD0_obs(1);
                        obj.Ly_stToe_stTD0_kf = L_RToe_stTD0_obs(2);
                        obj.sigma_Lx_stToe_stTD0 = obj.Cov_Lx_LRToe_stTD0;
                        obj.sigma_Ly_stToe_stTD0 = obj.Cov_Ly_LRToe_stTD0;
                        
                        obj.rp_stToe_z_ini = rp_RToe(3);
                    end
                end
                
                if stanceLeg == -1
                    
                    p_stToe = p_LToe;
                    Jp_stToe = Jp_LToe;
                    dJp_stToe = dJp_LToe;
                    v_stToe = v_LToe;
                    
                    p_swToe = p_RToe;
                    Jp_swToe = Jp_RToe;
                    dJp_swToe = dJp_RToe;
                    v_swToe = v_RToe;
                    
                    rp_stToe = rp_LToe;
                    Jrp_stToe = Jrp_LToe;
                    dJrp_stToe = dJrp_LToe;
                    rv_stToe = rv_LToe;
                    
                    rp_swToe = rp_RToe;
                    Jrp_swToe = Jrp_RToe;
                    dJrp_swToe = dJrp_RToe;
                    rv_swToe = rv_RToe;
                    
                    rp_stToe_fil = obj.rp_LToe_fil;
                    rp_swToe_fil = obj.rp_RToe_fil;
                    
                    L_stToe = L_LToe;
                    L_swToe = L_RToe;
                    
                    L_stToe_obs = L_LToe_obs;
                    L_swToe_obs = L_RToe_obs;
                    L_stToe_stTD0_obs = L_LToe_stTD0_obs;
                    L_swToe_stTD0_obs = L_RToe_stTD0_obs;
                    
                    Cov_Lx_stToe_stTD0 = obj.Cov_Lx_LRToe_stTD0;
                    Cov_Lx_swToe_stTD0 = obj.Cov_Lx_LRToe_stTD0;
                    Cov_Ly_stToe_stTD0 = obj.Cov_Ly_LRToe_stTD0;
                    Cov_Ly_swToe_stTD0 = obj.Cov_Ly_LRToe_stTD0;
                    
                    Cov_rpx_stToe_stTD0 = obj.Cov_rpx_LRToe_stTD0;
                    Cov_rpx_swToe_stTD0 = obj.Cov_rpx_LRToe_stTD0;
                    Cov_rpy_stToe_stTD0 = obj.Cov_rpy_LRToe_stTD0;
                    Cov_rpy_swToe_stTD0 = obj.Cov_rpy_LRToe_stTD0;
                    
                else
                    p_stToe = p_RToe;
                    Jp_stToe = Jp_RToe;
                    dJp_stToe = dJp_RToe;
                    v_stToe = v_RToe;
                    
                    p_swToe = p_LToe;
                    Jp_swToe = Jp_LToe;
                    dJp_swToe = dJp_LToe;
                    v_swToe = v_LToe;
                    
                    rp_stToe = rp_RToe;
                    Jrp_stToe = Jrp_RToe;
                    dJrp_stToe = dJrp_RToe;
                    rv_stToe = rv_RToe;
                    
                    rp_swToe = rp_LToe;
                    Jrp_swToe = Jrp_LToe;
                    dJrp_swToe = dJrp_LToe;
                    rv_swToe = rv_LToe;
                    
                    rp_stToe_fil = obj.rp_RToe_fil;
                    rp_swToe_fil = obj.rp_LToe_fil;
                    
                    L_stToe = L_RToe;
                    L_swToe = L_LToe;
                    
                    L_stToe_obs = L_RToe_obs;
                    L_swToe_obs = L_LToe_obs;
                    L_stToe_stTD0_obs = L_RToe_stTD0_obs;
                    L_swToe_stTD0_obs = L_LToe_stTD0_obs;
                    
                    Cov_Lx_stToe_stTD0 = obj.Cov_Lx_LRToe_stTD0;
                    Cov_Lx_swToe_stTD0 = obj.Cov_Lx_LRToe_stTD0;
                    Cov_Ly_stToe_stTD0 = obj.Cov_Ly_LRToe_stTD0;
                    Cov_Ly_swToe_stTD0 = obj.Cov_Ly_LRToe_stTD0;
                    
                    Cov_rpx_stToe_stTD0 = obj.Cov_rpx_LRToe_stTD0;
                    Cov_rpx_swToe_stTD0 = obj.Cov_rpx_LRToe_stTD0;
                    Cov_rpy_stToe_stTD0 = obj.Cov_rpy_LRToe_stTD0;
                    Cov_rpy_swToe_stTD0 = obj.Cov_rpy_LRToe_stTD0;
                    
                end
                
                
                % Transform to stTD0 frame, stTD0 frame is 
                rp_stToe_stTD0 = YToolkits.Angles.Rz( - obj.stTD0) * rp_stToe_fil;
                %% KF Angular Momemtum
                % Ly
                At = 1;
                Ct = 1;
                Bt = 1;
                
                ut = obj.sample_time*obj.total_mass*g*rp_stToe_stTD0(1);
                Rt = (obj.sample_time*obj.total_mass*g)^2*Cov_rpx_stToe_stTD0;
                Qt = Cov_Ly_stToe_stTD0;
                
                Ly_stToe_stTD0_bar = obj.Ly_stToe_stTD0_kf + ut;
                sigma_Ly_stToe_stTD0_bar = At*obj.sigma_Ly_stToe_stTD0*At' + Rt;
                Kt = sigma_Ly_stToe_stTD0_bar*Ct'*(Ct*sigma_Ly_stToe_stTD0_bar*Ct'+Qt)^-1;
                obj.Ly_stToe_stTD0_kf = Ly_stToe_stTD0_bar + Kt*(L_stToe_stTD0_obs(2)-Ct*Ly_stToe_stTD0_bar);
                obj.sigma_Ly_stToe_stTD0 = (1-Kt*Ct)*sigma_Ly_stToe_stTD0_bar;
                
                % Lx
                At = 1;
                Ct = 1;
                Bt = 1;
                
                ut = obj.sample_time*obj.total_mass*g*(-rp_stToe_stTD0(2));
                Rt = (obj.sample_time*obj.total_mass*g)^2*Cov_rpy_stToe_stTD0;
                Qt = Cov_Lx_stToe_stTD0;
                
                Lx_stToe_stTD0_bar = obj.Lx_stToe_stTD0_kf + ut;
                sigma_Lx_stToe_stTD0_bar = At*obj.sigma_Lx_stToe_stTD0*At' + Rt;
                Kt = sigma_Lx_stToe_stTD0_bar*Ct'*(Ct*sigma_Lx_stToe_stTD0_bar*Ct'+Qt)^-1;
                obj.Lx_stToe_stTD0_kf = Lx_stToe_stTD0_bar + Kt*(L_stToe_stTD0_obs(1)-Ct*Lx_stToe_stTD0_bar);
                obj.sigma_Lx_stToe_stTD0 = (1-Kt*Ct)*sigma_Lx_stToe_stTD0_bar;
                
                
                
                T_left = max([0, 1-s])/ds;
                l = sqrt(g/rp_stToe_stTD0(3));
                one_step_max_vel_gain = IRC.step_time*l*0.2;
                
                pseudo_com_vx_stTD0 = L_stToe_stTD0_obs(2)/(32*rp_stToe_stTD0(3));
%                 pseudo_com_vx_stTD0 = obj.Ly_stToe_stTD0_kf/(32*rp_stToe(3));
                dx0_next_stTD0 = rp_stToe_stTD0(1)*l*sinh(l*T_left) + pseudo_com_vx_stTD0*cosh(l*T_left);
                % x0_next is the desired relative position of COM to stance foot swing foot in the beginning of next step,(at this step it is still swing foot) so that COM velocity can be V at time IRC.step_time
                vx_com0_next_stTd0 = rp_stToe_stTD0(1)*l*sinh(l*T_left) + v_com(1)*cosh(l*T_left);
                
%                 pseudo_com_vy_stTD0 = -L_stToe_stTD0_obs(1)/(32*rp_stToe_stTD0(3));
                pseudo_com_vy_stTD0 = -obj.Lx_stToe_stTD0_kf/(32*rp_stToe(3));
                dy0_next_stTD0 = rp_stToe_stTD0(2)*l*sinh(l*T_left) + pseudo_com_vy_stTD0*cosh(l*T_left);
                
                dxyz0_next_tgd = YToolkits.Angles.Rz(obj.stTD0 - obj.tg_direction) * [dx0_next_stTD0; dy0_next_stTD0; 0];
                
                dx0_next_tgd = dxyz0_next_tgd(1);
                dy0_next_tgd = dxyz0_next_tgd(2);

%                 dxf_next_tgd_goal = median([dx0_next_tgd + one_step_max_vel_gain, dx0_next_tgd - one_step_max_vel_gain, Vx_tgd]);
                dxf_next_tgd_goal = Vx_tgd;
                dyf_next_tgd_goal = Vy_tgd;
                

                x0_next_tgd_goal = (dxf_next_tgd_goal - dx0_next_tgd*cosh(l*IRC.step_time))/(l*sinh(l*IRC.step_time));
                y0_next_tgd_goal = (dyf_next_tgd_goal - dy0_next_tgd*cosh(l*IRC.step_time))/(l*sinh(l*IRC.step_time));
                
                if direct_up_down
                    x0_next_tgd_goal = 0;
                    y0_next_tgd_goal = - abduction_direction * desired_com2stToe_lateral; % Notice: why negative sign? A: During left stance, y0_next_tgd_goal is for right toe.
                end
                
                xyz0_next_goal = YToolkits.Angles.Rz(obj.tg_direction) * [x0_next_tgd_goal; y0_next_tgd_goal; 0];
                
                x0_next_goal = xyz0_next_goal(1);
                y0_next_goal = xyz0_next_goal(2);
                

                %% obtain tg yaw frame value
                v_com_tgd = YToolkits.Angles.Rz(obj.tg_direction)' * v_com;
                v_com_stTD0 = YToolkits.Angles.Rz(obj.stTD0)' * v_com;
                %% generate reference signal
                w = pi/IRC.step_time;
                
                
                ref_rp_swToe_x = 1/2*(obj.rp_swToe_ini(1) - x0_next_goal)*cos(w*t) + 1/2*(obj.rp_swToe_ini(1) + x0_next_goal);
                ref_rv_swToe_x = 1/2*(obj.rp_swToe_ini(1) - x0_next_goal)*(-w*sin(w*t));
                ref_ra_swToe_x = 1/2*(obj.rp_swToe_ini(1) - x0_next_goal)*(-w^2*cos(w*t));
                
                ref_rp_swToe_y = 1/2*(obj.rp_swToe_ini(2) - y0_next_goal)*cos(w*t) + 1/2*(obj.rp_swToe_ini(2) + y0_next_goal);
                ref_rv_swToe_y = 1/2*(obj.rp_swToe_ini(2) - y0_next_goal)*(-w*sin(w*t));
                ref_ra_swToe_y = 1/2*(obj.rp_swToe_ini(2) - y0_next_goal)*(-w^2*cos(w*t));
                
                %             ref_rp_swToe_z = 1/2*CL*cos(2*w*t)+(H-1/2*CL);
                %             ref_rv_swToe_z = 1/2*CL*(-2*w*sin(2*w*t));
                %             ref_ra_swToe_z = 1/2*CL*(-4*w^2*cos(2*w*t));
                ref_rp_swToe_z= 4*CL*(s-0.5)^2+(H-CL);
                ref_rv_swToe_z = 8*CL*(s-0.5)*ds;
                ref_ra_swToe_z = 8*CL*ds^2;
                
                ref_st_rotation = (1-s_sat) * obj.st_rotation_goal_last + s_sat*obj.st_rotation_goal;
                ref_sw_rotation = (1-s_sat) * obj.sw_rotation_goal_last + s_sat*obj.sw_rotation_goal;
                
                obj.hr= [0;0;ref_st_rotation; ref_sw_rotation; (H+spring_height_compensation); ref_rp_swToe_x; ref_rp_swToe_y; ref_rp_swToe_z];
                obj.dhr = [0;0;0;0;0;ref_rv_swToe_x;ref_rv_swToe_y;ref_rv_swToe_z];
                obj.ddhr = [0;0;0;0;0;ref_ra_swToe_x;ref_ra_swToe_y;ref_ra_swToe_z];
                
                %% track reference signal


                % Jh is jacobian for output
                Jh = zeros(8,20);
                dJh = zeros(8,20);
                
                Jh(1,5) = 1; % torso pitch
                Jh(2,6) = 1; % torso roll
                Jh(3,st_rotation) = 1; % stance rotation
                Jh(4,sw_rotation) = 1; % swing rotation.
                Jh(5,:) = Jrp_stToe(3,:); % com to stance toe height
                Jh([6,7,8],:) = Jrp_swToe([1,2,3],:); % com to swing toe x y z relativ pos
                
                
                obj.h0 = [q(5);q(6);q(st_rotation);q(sw_rotation);rp_stToe(3);rp_swToe([1,2,3])];
                obj.dh0 = Jh*dq;
                
                
                y = obj.h0 - obj.hr;
                dy = obj.dh0 - obj.dhr;

                %% caculate torque (IOL)
                motor_index = [1,2,3,4,6,7,8,9];
                
                M = InertiaMatrix(q);
                C = CoriolisTerm(q,dq);
                G = GravityVector(q);
                B=zeros(20,10); B(7:10,1:4) = eye(4); B(13:17,5:9) = eye(5); B(20,10) = 1;
                B_cut = B(:,motor_index);
                
                % Jacobian for spring
                JsL = zeros(2,20); JsL(1,11) = 1; JsL(2,[10 11 12]) = J_HeelSpringDeflectionEst(q(10),q(11),q(12));
                JsR = zeros(2,20); JsR(1,18) = 1; JsR(2,[17 18 19]) = J_HeelSpringDeflectionEst(q(17),q(18),q(19));
                Js = [JsL; JsR];
                
                [Jsd1,Jsd2] = size(Js);
                [JsLd1,JsLd2] = size(JsL);
                [JsRd1,JsRd2] = size(JsR);
                
                % body Jacobian on foot
                Jb_L = Jb_LeftToeBottomBack(q);
                Jb_R = Jb_RightToeBottomBack(q);
                dJb_L = dJb_LeftToeBottomBack(q,dq);
                dJb_R = dJb_RightToeBottomBack(q,dq);
                % cartesian Jacobian on foot
                Jc_L = Jp_LeftToeBottomBack(q);
                Jc_R = Jp_RightToeBottomBack(q);
                dJc_L = dJp_LeftToeBottomBack(q,dq);
                dJc_R = dJp_RightToeBottomBack(q,dq);
                % Jacobian for ground constraint
                Jg_L = zeros(5,20); Jg_L(1:3,:)= Jc_L; Jg_L([4,5],:)= Jb_L([5,6],:);
                Jg_R = zeros(5,20); Jg_R(1:3,:)= Jc_R; Jg_R([4,5],:)= Jb_R([5,6],:);
                dJg_L = zeros(5,20); dJg_L(1:3,:)= dJc_L; dJg_L([4,5],:)= dJb_L([5,6],:);
                dJg_R = zeros(5,20); dJg_R(1:3,:)= dJc_R; dJg_R([4,5],:)= dJb_R([5,6],:);
                if stanceLeg == -1
                    Jg = Jg_L;
                    dJg = dJg_L;
                else
                    Jg = Jg_R;
                    dJg = dJg_R;
                end
                
                [Jgd1,Jgd2] = size(Jg);
                
                % Jt is Jacobian for torso constraint
                fixed_torso_freedom =[2,4,6];
                Jt = zeros(length(fixed_torso_freedom),20);% Torso constraint Jacobian
                for i = 1: length(fixed_torso_freedom)
                    Jt(i,fixed_torso_freedom(i)) = 1;
                end
                [Jtd1,Jtd2] = size(Jt);
                
                if 0 % 1 is 2D torso constraint, 0 is 3D free torso
                    Me = [M, -Jt', -Js', -Jg';
                        Jt, zeros(Jtd1,Jtd1), zeros(Jtd1,Jsd1), zeros(Jtd1,Jgd1);
                        Js, zeros(Jsd1,Jtd1), zeros(Jsd1,Jsd1), zeros(Jsd1,Jgd1);
                        Jg, zeros(Jgd1,Jtd1), zeros(Jgd1,Jsd1), zeros(Jgd1,Jgd1)];
                    He = [C+G;zeros(Jtd1+Jsd1+Jgd1,1)];
                    Be = [B_cut;zeros(Jtd1+Jsd1+Jgd1,length(motor_index))];
                    S = [eye(20),zeros(20,Jtd1+Jsd1+Jgd1)]; % S is used to seperate ddq with Ft Fs Fg;
                    u_select = (Jh*S*Me^-1*Be)^-1*(-Kd*dy-Kp*y+obj.ddhr+Jh*S*Me^-1*He);
                else
                    Me = [M,  -Js', -Jg';
                        Js, zeros(Jsd1,Jsd1), zeros(Jsd1,Jgd1);
                        Jg, zeros(Jgd1,Jsd1), zeros(Jgd1,Jgd1)];
                    He = [G;zeros(Jsd1+Jgd1,1)];
                    Be = [B_cut;zeros(Jsd1+Jgd1,length(motor_index))];
                    S = [eye(20),zeros(20,Jsd1+Jgd1)]; % S is used to seperate ddq with Ft Fs Fg;
                    u_select = (Jh*S*Me^-1*Be)^-1*(-Kd*dy-Kp*y+obj.ddhr+Jh*S*Me^-1*He);
                end
                
                u(motor_index) = u_select;
                % flat swing toe
                [Angle_LT, Angle_RT, dAngle_LT, dAngle_RT] = getToeAbsoluteAngle(q,dq);

                if stanceLeg == -1
                    u(10) = -50 * Angle_RT - 5 * dAngle_RT;
                else
                    u(5) = -50 * Angle_LT - 5 * dAngle_LT;
                end
%                 u = zeros(10,1);
                userInputs.torque = u;
                
                %% log variables for next iteration
                obj.u_prev = u;
                %% Return
                userInputs.telemetry(1) = 0;
                userInputs.telemetry(2) = 0;
                userInputs.telemetry(3) = 0;
                userInputs.telemetry(4) = 0;
                userInputs.telemetry(5) = EstStates.Voltage;
                userInputs.telemetry(6) = 0;
                userInputs.telemetry(8) = GRF_z(1);
                userInputs.telemetry(9) = GRF_z(2);
                %% log Data
                Data.stanceLeg = stanceLeg;
                Data.Lx_LToe = L_LToe(1);
                Data.Lx_RToe = L_RToe(1);
                
                Data.Ly_LToe = L_LToe(2);
                Data.Ly_RToe = L_RToe(2);
                
                Data.Lx_stToe = L_stToe(1);
                Data.Ly_stToe = L_stToe(2);
                Data.Lx_swToe = L_swToe(1);
                Data.Ly_swToe = L_swToe(2);
                
                Data.Ly_G = LG(2);
%                 Data.Lx_stToe_kf = obj.Lx_stToe_kf;
%                 Data.Ly_stToe_kf = obj.Ly_stToe_kf;
                
%                 Data.dx0_next = dx0_next;
                Data.x0_next_goal = x0_next_goal;
%                 Data.dxf_next_goal = dxf_next_goal;

                Data.dx0_next_stTD0 = dx0_next_stTD0;
                Data.x0_next_tgd_goal = x0_next_tgd_goal;
                Data.dxf_next_tgd_goal = dxf_next_tgd_goal;
                
                Data.tg_direction = obj.tg_direction;
                
                Data.vx_com0_next_stTd0 = vx_com0_next_stTd0;
                
                Data.hr = obj.hr;
                Data.dhr = obj.dhr;
                Data.h0 = obj.h0;
                Data.dh0 = obj.dh0;
                
                
                Data.p_stToe = p_stToe;
                Data.p_swToe = p_swToe;
                Data.p_LToe = p_LToe;
                Data.p_RToe = p_RToe;
                
                Data.rp_stToe = rp_stToe;
                Data.rv_stToe = rv_stToe;
                
                Data.rp_LToe = rp_LToe;
                Data.rv_LToe = rv_LToe;
                Data.rp_RToe = rp_RToe;
                Data.rv_RToe = rv_RToe;
                
                Data.rp_LToe_fil = obj.rp_LToe_fil;
                Data.rp_RToe_fil = obj.rp_RToe_fil;
                
                Data.v_stToe = v_stToe;
                Data.v_swToe = v_swToe;
                Data.v_LToe = v_LToe;
                Data.v_RToe = v_RToe;
                
                Data.p_com = p_com;
                Data.v_com = v_com;
                Data.vx_com = v_com(1);
                Data.vy_com = v_com(2);
                Data.vz_com = v_com(3);
                Data.px_com = p_com(1);
                Data.py_com = p_com(2);
                Data.pz_com = p_com(3);
                
                Data.vx_com_tgd = v_com_tgd(1);
                Data.vy_com_tgd = v_com_tgd(2);
                Data.Lx_stToe_stTD0_obs = L_stToe_stTD0_obs(1);
                Data.Lx_stToe_stTD0_kf = obj.Lx_stToe_stTD0_kf;
                Data.Ly_stToe_stTD0_obs = L_stToe_stTD0_obs(2);
                Data.Ly_stToe_stTD0_kf = obj.Ly_stToe_stTD0_kf;
                
                
                Data.pseudo_com_vx_stTD0 = pseudo_com_vx_stTD0;
                Data.q = q;
                Data.dq = dq;
                Data.dq_ss = dq_ss;
                Data.u = u;
                Data.s = s;
                Data.t = t;
                
                
                Data.qsL_1 = EstStates.qsL_1;
                Data.qsL_2 = EstStates.qsL_2;
                Data.qsR_1 = EstStates.qsR_1;
                Data.qsR_2 = EstStates.qsR_2;
                
                Data.dqsL_1 = EstStates.dqsL_1;
                Data.dqsL_2 = EstStates.dqsL_2;
                Data.dqsR_1 = EstStates.dqsR_1;
                Data.dqsR_2 = EstStates.dqsR_2;
                
                Data.q1 = q(1);
                Data.q2 = q(2);
                Data.q3 = q(3);
                Data.q4 = q(4);
                Data.q5 = q(5);
                Data.q6 = q(6);
                Data.q7 = q(7);
                Data.q8 = q(8);
                Data.q9 = q(9);
                Data.q10 = q(10);
                Data.q11 = q(11);
                Data.q12 = q(12);
                Data.q13 = q(13);
                Data.q14 = q(14);
                Data.q15 = q(15);
                Data.q16 = q(16);
                Data.q17 = q(17);
                Data.q18 = q(18);
                Data.q19 = q(19);
                Data.q20 = q(20);
                
                Data.dq1 = dq(1);
                Data.dq2 = dq(2);
                Data.dq3 = dq(3);
                Data.dq4 = dq(4);
                Data.dq5 = dq(5);
                Data.dq6 = dq(6);
                Data.dq7 = dq(7);
                Data.dq8 = dq(8);
                Data.dq9 = dq(9);
                Data.dq10 = dq(10);
                Data.dq11 = dq(11);
                Data.dq12 = dq(12);
                Data.dq13 = dq(13);
                Data.dq14 = dq(14);
                Data.dq15 = dq(15);
                Data.dq16 = dq(16);
                Data.dq17 = dq(17);
                Data.dq18 = dq(18);
                Data.dq19 = dq(19);
                Data.dq20 = dq(20);
            end
        end % stepImpl
        
        %% Default functions
        function setupImpl(obj)
            %SETUPIMPL Initialize System object.
        end % setupImpl
        
        function resetImpl(~)
            %RESETIMPL Reset System object states.
        end % resetImpl
        
%         function [name_1, name_2]  = getInputNamesImpl(~)
%             %GETINPUTNAMESIMPL Return input port names for System block
%             name_1 = 'EstStates';
%             name_2 = 't_total';
%         end % getInputNamesImpl
%         
%         function [name_1, name_2] = getOutputNamesImpl(~)
%             %GETOUTPUTNAMESIMPL Return output port names for System block
%             name_1 = 'userInputs';
%             name_2 = 'Data';
%         end % getOutputNamesImpl
        
        % PROPAGATES CLASS METHODS ============================================
        function [out, Data] = getOutputSizeImpl(~)
            %GETOUTPUTSIZEIMPL Get sizes of output ports.
            out = [1, 1];
            Data = [1, 1];
        end % getOutputSizeImpl
        
        function [out, Data] = getOutputDataTypeImpl(~)
            %GETOUTPUTDATATYPEIMPL Get data types of output ports.
            out = 'CassieUserInBus';
            Data = 'cassieDataBus';
        end % getOutputDataTypeImpl
        
        function [out, Data] = isOutputComplexImpl(~)
            %ISOUTPUTCOMPLEXIMPL Complexity of output ports.
            out = false;
            Data = false;
        end % isOutputComplexImpl
        
        function [out, Data] = isOutputFixedSizeImpl(~)
            %ISOUTPUTFIXEDSIZEIMPL Fixed-size or variable-size output ports.
            out = true;
            Data = true;
        end % isOutputFixedSizeImpl
    end % methods
end % classdef