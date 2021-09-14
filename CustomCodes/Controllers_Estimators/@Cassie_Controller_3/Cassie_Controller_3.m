%Yukai controller.

classdef Cassie_Controller_3 <matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime %#codegen
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
        
        Kp_st_rotation;
        Kd_st_rotation;
        
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
        st_rotation_torque_ramp_time;
        com_x_offset;
        Vx_tgd_offset;
        Vy_tgd_offset;
        
        stance_thre_lb;
        stance_thre_ub;
        stance_thre_lb_2;
        stance_thre_ub_2; 
        
        IK_max_iter_num;
        yaw_error_fil_param;
        rp_LRToe_fil_param;
        adjusted_desired_roll_angle_fil_param;
        use_dq_AR_for_PD;
        lateral_correction_gain;
        one_step_max_vel_gain;
        max_st_rotation_motor_torque;
        
        KsL_1;
        KsL_2;
        KsR_1;
        KsR_2;
        
        toe_leave_ground_lb;
        toe_leave_ground_ub;
        
        sample_time;
    end
    properties (Access = private, Constant)
        TorqueLimits = repmat([112.5;112.5;195.2;195.2;45],[2,1]);
        ActuatorLimits = [          -0.2618,0.3491;     -0.3840,0.3840;     -0.8727,1.3963;     -2.7227,-0.7330;    -2.4435,-0.6109;
                                    -0.3491,0.2618;     -0.3840,0.3840;     -0.8727,1.3963;     -2.7227,-0.7330;    -2.4435,-0.6109];
        ActuatorLimits_ARsafety = [ -0.1618,0.2491;     -0.2840,0.2840;     -0.7727,1.2963;     -2.6227,-0.8330;    -2.3435,-0.7109;
                                    -0.2491,0.1618;     -0.2840,0.2840;     -0.7727,1.2963;     -2.6227,-0.8330;    -2.3435,-0.7109];
        Ks1 = 1500;
        Ks2 = 1250;
        
        total_mass = 31.8840;
        

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
        st_rotation_pos_last = 0;
        st_rotation_vel_last = 0;
        sw_rotation_pos_last = 0;
        sw_rotation_vel_last = 0;
        
        u_last = zeros(10,1);
        
        swing_leg_leave_ground = 0;
        ini_toe_height_diff = 0;
        
        s_ini = 0;
        
        H_next = 0;
        H_this = 0;
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
        ddqd_control = zeros(8,1);
        % previous iteration's variables 
        u_prev = zeros(10,1);
        Ki_knee = 2000;
        iqe_control_knee = 0;
        % filtered rp_LRToe
        rp_LToe_fil = zeros(3,1);
        rp_RToe_fil = zeros(3,1);
        yaw_error_fil = 0;
        % store values in last iteration
        comp_knee_st_prev = 0;
        adjusted_desired_roll_angle_fil = 0;
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
        
        function [userInputs, Data] = stepImpl(obj,EstStates,t_total,isSim,IRC,DynamicMatrixLibrary)
            %STEPIMPL System output and state update equations.
            
            %% Initialize --------------------------------------------------------
            
            Data = PreFunctions.Construct_Data;
            % Reset the desired motor torques to zero in case they aren't defined
            userInputs = CassieModule.getUserInStruct;
            u = zeros(10,1);
            Kp = repmat([obj.Kp_abduction;obj.Kp_rotation;obj.Kp_thigh;obj.Kp_knee; obj.Kp_toe],[2,1]);
            Kd = repmat([obj.Kd_abduction;obj.Kd_rotation;obj.Kd_thigh;obj.Kd_knee; obj.Kd_toe],[2,1]);
            
            if t_total>0.001 && EstStates.isCalibrated
                % Let the output be torso angle, com height and delta x,delta z of swing
               
                
                % feet and com. delta = p_com - p_swfeet.
                
                g = 9.81;
%                 H = 0.8; % desired com height
                CL = IRC.CL; % foot clearance
                if isSim == 2
                    spring_height_compensation = 0;
                else
                    spring_height_compensation = 0.00;
                end
                g=9.81;
                
                %% Read Command
                Vx_tgd_no_offset = IRC.Vx_tgd; % Desired velocity in target direction direction at the end of a step
                H = IRC.H; % desired com height
                desired_com2stToe_lateral = IRC.desired_com2stToe_lateral; % at the begining of left stance (also at the end of left stance)
                Vy_tgd_0_oscilate = -EstStates.stanceLeg*(-desired_com2stToe_lateral*sqrt(g/H)*sinh(sqrt(g/H)*IRC.step_time)/(1+cosh(sqrt(g/H)*IRC.step_time))); % initial ( and negative end) velocity required to achieve rp_com2stToe_desired
                Vy_tgd_no_offset = Vy_tgd_0_oscilate + IRC.Vy_tgd_avg;
                
                Vx_tgd = Vx_tgd_no_offset + obj.Vx_tgd_offset;
%                 Vy_tgd = Vy_tgd_no_offset + obj.Vy_tgd_offset - 0.15;
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
                a_world = EstStates.a_world;
                
                ds = IRC.ds;
                
                if obj.initialized == 0 || IRC.motor_power == 0
                    obj.tg_direction = q(4);
                end
                   
                s_sat = min([1,s]);
                
                %% Generate fucntions of GRF
                [f_GRF_L, f_GRF_R] = get_f_GRF(obj, GRF_z);
                [f_GRF_L_2, f_GRF_R_2] = get_f_GRF_2(obj, GRF_z);
                %% Generate functions of s and t
%                 fs_1 = YToolkits.bezier([0,0,ones(1,20)],s);
%                 dfs_1 = YToolkits.dbezier([0,0,ones(1,20)],s)*ds;%
                fs_2_bezier = [0,1,3,6,6,6]/6;
%                 fs_2_bezier = [6,6,6,6,6,6]/6;
                fs_2 = YToolkits.bezier(fs_2_bezier,s);
                dfs_2 = YToolkits.dbezier(fs_2_bezier,s)*ds;               
                ft_1 = min([1,1/obj.torque_transition_time*t]);
%                 ft_2 = min([1,1/obj.knee_cp_time*t]);
%                 [ft_3,dft_3] = YToolkits.sin2horizontal(t, 0.1);     
                ft_4 = min(1,t/obj.st_rotation_torque_ramp_time);
                %% Observations
                p_com = p_COM(q);
                p_com = p_com + YToolkits.Angles.Rz(q(4))*[obj.com_x_offset;0;0];
                Jp_com = Jp_COM(q);
                v_com = Jp_com*dq;
                
                p_LToe = p_LeftToeJoint(q);
                Jp_LToe = Jp_LeftToeJoint(q);
                v_LToe = Jp_LToe*dq;
                
                p_RToe = p_RightToeJoint(q);
                Jp_RToe = Jp_RightToeJoint(q);
                v_RToe = Jp_RToe*dq;
                
                
                % com position RELATIVE to toes
                
                rp_LToe = p_com - p_LToe;
%                 Jrp_LToe = Jp_com - Jp_LToe;
                rv_LToe = v_com - v_LToe;
                
                rp_RToe = p_com - p_RToe;
%                 Jrp_RToe = Jp_com - Jp_RToe;
                rv_RToe = v_com - v_RToe;

                JL_dq_LToe = Jdq_AMworld_about_pA(q,dq,p_LToe,zeros(3,20));
%                 JL_dq_RToe = Jdq_AMworld_about_pA(q,dq,p_RToe,zeros(3,20));
                L_LToe = JL_dq_LToe*dq;
%                 L_RToe = JL_dq_RToe*dq;
                L_RToe = L_LToe + obj.total_mass * cross(p_LToe - p_RToe, v_com); %% attention!!!! if urdf changes, total mass must change too!
                
                L_LToe_obs = L_LToe;
                L_RToe_obs = L_RToe;
                
                L_LToe_stTD0_obs = YToolkits.Angles.Rz( - obj.stTD0) * L_LToe_obs;
                L_RToe_stTD0_obs = YToolkits.Angles.Rz( - obj.stTD0) * L_RToe_obs;
                
                % if assume spring is rigid ( for low level controls)
                q_rigid = q;
                q_rigid(11) = 0; q_rigid(12) = -q_rigid(10)+deg2rad(13); q_rigid(18) = 0; q_rigid(19) = -q_rigid(17)+deg2rad(13);
                dq_rigid = dq;
                dq_rigid(11) = 0; dq_rigid(12) = -dq_rigid(10); dq_rigid(18) = 0; dq_rigid(19) = -dq_rigid(17);
                
                
                p_com_rigid =  p_COM(q_rigid);
                p_com_rigid = p_com_rigid + YToolkits.Angles.Rz(q(4))*[obj.com_x_offset;0;0];
                Jp_com_rigid =  Jp_COM(q_rigid);
                v_com_rigid =  Jp_com*dq_rigid;
                
                p_LToe_rigid =  p_LeftToeJoint(q_rigid);
                Jp_LToe_rigid =  Jp_LeftToeJoint(q_rigid);
                v_LToe_rigid =  Jp_LToe*dq_rigid;
                
                p_RToe_rigid =  p_RightToeJoint(q_rigid);
                Jp_RToe_rigid =  Jp_RightToeJoint(q_rigid);
                v_RToe_rigid =  Jp_RToe*dq_rigid;
                
                % com position RELATIVE to toes
                
                rp_LToe_rigid =  p_com_rigid - p_LToe_rigid;
                Jrp_LToe_rigid =  Jp_com_rigid - Jp_LToe_rigid;
                rv_LToe_rigid =  v_com_rigid - v_LToe_rigid;
                
                rp_RToe_rigid =  p_com_rigid - p_RToe_rigid;
                Jrp_RToe_rigid =  Jp_com_rigid - Jp_RToe_rigid;
                rv_RToe_rigid =  v_com_rigid - v_RToe_rigid;
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
                    obj.st_rotation_pos_last = obj.h0(4);
                    obj.st_rotation_vel_last = obj.dh0(4);
                    obj.sw_rotation_pos_last = obj.h0(3);
                    obj.sw_rotation_vel_last = obj.dh0(3);
                    obj.st_rotation_goal = - 0.5* YToolkits.wrap_to_Pi( obj.tg_direction - obj.stTD0);
                    obj.sw_rotation_goal = 0.5* YToolkits.wrap_to_Pi( obj.tg_direction - obj.stTD0);
                    obj.st_rotation_goal = clamp2limit(obj, obj.st_rotation_goal, st_motor_rotation);
                    obj.sw_rotation_goal = clamp2limit(obj, obj.sw_rotation_goal, sw_motor_rotation);
                    % update L_*Toe_stTD0_obs if stTD0 is changed in this
                    % iteration
                    L_LToe_stTD0_obs = YToolkits.Angles.Rz( - obj.stTD0) * L_LToe_obs;
                    L_RToe_stTD0_obs = YToolkits.Angles.Rz( - obj.stTD0) * L_RToe_obs;
                    
                    obj.iqe_control_knee = 0;
                    obj.swing_leg_leave_ground = 0;
                    
                    obj.s_ini = 0;
                    
                   
                    if stanceLeg == -1
                        obj.rp_swToe_ini = rp_RToe_rigid; % notice: this value could be overwrite in later code
                        obj.rv_swToe_ini = rv_RToe_rigid;
%                         obj.rp_stToe_ini = rp_LToe_rigid;
%                         obj.rv_stToe_ini = rv_LToe_rigid;
                        obj.Lx_stToe_stTD0_kf = L_LToe_stTD0_obs(1);
                        obj.Ly_stToe_stTD0_kf = L_LToe_stTD0_obs(2);
                        obj.sigma_Lx_stToe_stTD0 = obj.Cov_Lx_LRToe_stTD0;
                        obj.sigma_Ly_stToe_stTD0 = obj.Cov_Ly_LRToe_stTD0;
                        
                        obj.rp_stToe_z_ini = rp_LToe_rigid(3);
                        
%                         obj.H_next = min([H, H + (p_LToe(3) - p_RToe(3))]);
%                         obj.H_this = min([H, H - (p_LToe(3) - p_RToe(3))]);
                        obj.H_next = H;
                        obj.H_this = H;
                    else
                        obj.rp_swToe_ini = rp_LToe_rigid; % notice: this value could be overwrite in later code
                        obj.rv_swToe_ini = rv_LToe_rigid;
%                         obj.rp_stToe_ini = rp_RToe_rigid;
%                         obj.rv_stToe_ini = rv_RToe_rigid;
                        obj.Lx_stToe_stTD0_kf = L_RToe_stTD0_obs(1);
                        obj.Ly_stToe_stTD0_kf = L_RToe_stTD0_obs(2);
                        obj.sigma_Lx_stToe_stTD0 = obj.Cov_Lx_LRToe_stTD0;
                        obj.sigma_Ly_stToe_stTD0 = obj.Cov_Ly_LRToe_stTD0;
                        
                        obj.rp_stToe_z_ini = rp_RToe_rigid(3);
                        
%                         obj.H_next = min([H, H + (p_RToe(3) - p_LToe(3))]);
%                         obj.H_this = min([H, H - (p_RToe(3) - p_LToe(3))]);
                        obj.H_next = H;
                        obj.H_this = H;
                    end
                end
                
                if stanceLeg == -1
                    
                    p_stToe = p_LToe;
%                     Jp_stToe = Jp_LToe;
                    v_stToe = v_LToe;
                    
                    p_swToe = p_RToe;
%                     Jp_swToe = Jp_RToe;
                    v_swToe = v_RToe;
                    
                    rp_stToe = rp_LToe;
%                     Jrp_stToe = Jrp_LToe;
                    rv_stToe = rv_LToe;
                    
                    rp_swToe = rp_RToe;
%                     Jrp_swToe = Jrp_RToe;
%                     rv_swToe = rv_RToe;
                    
                    rp_stToe_fil = obj.rp_LToe_fil;
                    rp_swToe_fil = obj.rp_RToe_fil;
                    
                    L_stToe = L_LToe;
                    L_swToe = L_RToe;
                    
%                     L_stToe_obs = L_LToe_obs;
%                     L_swToe_obs = L_RToe_obs;
                    L_stToe_stTD0_obs = L_LToe_stTD0_obs;
%                     L_swToe_stTD0_obs = L_RToe_stTD0_obs;
                    
                    Cov_Lx_stToe_stTD0 = obj.Cov_Lx_LRToe_stTD0;
                    Cov_Lx_swToe_stTD0 = obj.Cov_Lx_LRToe_stTD0;
                    Cov_Ly_stToe_stTD0 = obj.Cov_Ly_LRToe_stTD0;
                    Cov_Ly_swToe_stTD0 = obj.Cov_Ly_LRToe_stTD0;
                    
                    Cov_rpx_stToe_stTD0 = obj.Cov_rpx_LRToe_stTD0;
                    Cov_rpx_swToe_stTD0 = obj.Cov_rpx_LRToe_stTD0;
                    Cov_rpy_stToe_stTD0 = obj.Cov_rpy_LRToe_stTD0;
                    Cov_rpy_swToe_stTD0 = obj.Cov_rpy_LRToe_stTD0;
                    
                    f_GRF_st = f_GRF_L;
                    f_GRF_sw = f_GRF_R;
                    
                    Ks_st_1 = obj.KsL_1;
                    Ks_st_2 = obj.KsL_2;
                    Ks_sw_1 = obj.KsR_1;
                    Ks_sw_2 = obj.KsR_2;
                    
                    qs_st_1 = EstStates.qsL_1;
                    qs_st_2 = EstStates.qsL_2;                    
                    qs_sw_1 = EstStates.qsR_1;
                    qs_sw_2 = EstStates.qsR_2;                 
                else
                    p_stToe = p_RToe;
                    Jp_stToe = Jp_RToe;
                    v_stToe = v_RToe;
                    
                    p_swToe = p_LToe;
%                     Jp_swToe = Jp_LToe;
                    v_swToe = v_LToe;
                    
                    rp_stToe = rp_RToe;
%                     Jrp_stToe = Jrp_RToe;
                    rv_stToe = rv_RToe;
                    
                    rp_swToe = rp_LToe;
%                     Jrp_swToe = Jrp_LToe;
%                     rv_swToe = rv_LToe;
                    
                    rp_stToe_fil = obj.rp_RToe_fil;
                    rp_swToe_fil = obj.rp_LToe_fil;
                    
                    L_stToe = L_RToe;
                    L_swToe = L_LToe;
                    
%                     L_stToe_obs = L_RToe_obs;
%                     L_swToe_obs = L_LToe_obs;
                    L_stToe_stTD0_obs = L_RToe_stTD0_obs;
%                     L_swToe_stTD0_obs = L_LToe_stTD0_obs;
                    
                    Cov_Lx_stToe_stTD0 = obj.Cov_Lx_LRToe_stTD0;
                    Cov_Lx_swToe_stTD0 = obj.Cov_Lx_LRToe_stTD0;
                    Cov_Ly_stToe_stTD0 = obj.Cov_Ly_LRToe_stTD0;
                    Cov_Ly_swToe_stTD0 = obj.Cov_Ly_LRToe_stTD0;
                    
                    Cov_rpx_stToe_stTD0 = obj.Cov_rpx_LRToe_stTD0;
                    Cov_rpx_swToe_stTD0 = obj.Cov_rpx_LRToe_stTD0;
                    Cov_rpy_stToe_stTD0 = obj.Cov_rpy_LRToe_stTD0;
                    Cov_rpy_swToe_stTD0 = obj.Cov_rpy_LRToe_stTD0;
                    
                    f_GRF_st = f_GRF_R;
                    f_GRF_sw = f_GRF_L;
                    
                    Ks_st_1 = obj.KsR_1;
                    Ks_st_2 = obj.KsR_2;
                    Ks_sw_1 = obj.KsL_1;
                    Ks_sw_2 = obj.KsL_2;
                    
                    qs_st_1 = EstStates.qsR_1;
                    qs_st_2 = EstStates.qsR_2;                    
                    qs_sw_1 = EstStates.qsL_1;
                    qs_sw_2 = EstStates.qsL_2;      
                end
                
                if stanceLeg == -1
%                     p_stToe_rigid = p_LToe_rigid;
%                     Jp_stToe_rigid = Jp_LToe_rigid;
%                     v_stToe_rigid = v_LToe_rigid;
                    
%                     p_swToe_rigid = p_RToe_rigid;
%                     Jp_swToe_rigid = Jp_RToe_rigid;
%                     v_swToe_rigid = v_RToe_rigid;
                    
                    rp_stToe_rigid = rp_LToe_rigid;
                    Jrp_stToe_rigid = Jrp_LToe_rigid;
%                     rv_stToe_rigid = rv_LToe_rigid;
                    
                    rp_swToe_rigid = rp_RToe_rigid;
                    Jrp_swToe_rigid = Jrp_RToe_rigid;
%                     rv_swToe_rigid = rv_RToe_rigid;
                    
                else
%                     p_stToe_rigid = p_RToe_rigid;
%                     Jp_stToe_rigid = Jp_RToe_rigid;
%                     v_stToe_rigid = v_RToe_rigid;
                    
%                     p_swToe_rigid = p_LToe_rigid;
%                     Jp_swToe_rigid = Jp_LToe_rigid;
%                     v_swToe_rigid = v_LToe_rigid;
                    
                    rp_stToe_rigid = rp_RToe_rigid;
                    Jrp_stToe_rigid = Jrp_RToe_rigid;
%                     rv_stToe_rigid = rv_RToe_rigid;
                    
                    rp_swToe_rigid = rp_LToe_rigid;
                    Jrp_swToe_rigid = Jrp_LToe_rigid;
%                     rv_swToe_rigid = rv_LToe_rigid;
                end
                
                % Transform to stTD0 frame, stTD0 frame is 
                rp_stToe_stTD0 = YToolkits.Angles.Rz( - obj.stTD0) * rp_stToe_fil;
                rp_swToe_stTD0 = YToolkits.Angles.Rz( - obj.stTD0) * rp_swToe;
                rp_stToe_tgd = YToolkits.Angles.Rz( - obj.tg_direction) * rp_stToe_fil;
                rp_swToe_tgd = YToolkits.Angles.Rz( - obj.tg_direction) * rp_swToe;
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
%                 one_step_max_vel_gain = IRC.step_time*l*0.2;
                
%                 pseudo_com_vx_stTD0 = L_stToe_stTD0_obs(2)/(32*rp_stToe_stTD0(3));
                pseudo_com_vx_stTD0 = obj.Ly_stToe_stTD0_kf/(32*rp_stToe(3));
%                 dx0_next_stTD0 = rp_stToe_stTD0(1)*l*sinh(l*T_left) + pseudo_com_vx_stTD0*cosh(l*T_left);
                dxf_this_stTD0 = rp_stToe_stTD0(1)*l*sinh(l*T_left) + pseudo_com_vx_stTD0*cosh(l*T_left);
                xf_this_stTD0 = rp_stToe_stTD0(1)*cosh(l*T_left) + pseudo_com_vx_stTD0*1/l*sinh(l*T_left);
                % x0_next is the desired relative position of COM to stance foot swing foot in the beginning of next step,(at this step it is still swing foot) so that COM velocity can be V at time IRC.step_time
                
%                 pseudo_com_vy_stTD0 = -L_stToe_stTD0_obs(1)/(32*rp_stToe_stTD0(3));
                pseudo_com_vy_stTD0 = -obj.Lx_stToe_stTD0_kf/(32*rp_stToe(3));
%                 dy0_next_stTD0 = rp_stToe_stTD0(2)*l*sinh(l*T_left) + pseudo_com_vy_stTD0*cosh(l*T_left);
                dyf_this_stTD0 = rp_stToe_stTD0(2)*l*sinh(l*T_left) + pseudo_com_vy_stTD0*cosh(l*T_left);
                yf_this_stTD0 = rp_stToe_stTD0(2)*cosh(l*T_left) + pseudo_com_vy_stTD0*1/l*sinh(l*T_left);
                
%                 dxyz0_next_tgd = YToolkits.Angles.Rz(obj.stTD0 - obj.tg_direction) * [dx0_next_stTD0; dy0_next_stTD0; 0];
                dxyzf_this_tgd = YToolkits.Angles.Rz(obj.stTD0 - obj.tg_direction) * [dxf_this_stTD0; dyf_this_stTD0; 0];
                xyzf_this_tgd = YToolkits.Angles.Rz(obj.stTD0 - obj.tg_direction) * [xf_this_stTD0; yf_this_stTD0; 0];
                
                dxf_this_tgd = dxyzf_this_tgd(1);
                dyf_this_tgd = dxyzf_this_tgd(2);
                
                xf_this_tgd = xyzf_this_tgd(1);
                yf_this_tgd = xyzf_this_tgd(2);
                
                dxf_next_tgd_goal = median([dxf_this_tgd + obj.one_step_max_vel_gain, dxf_this_tgd - obj.one_step_max_vel_gain, Vx_tgd]);
%                 dxf_next_tgd_goal = Vx_tgd;
                dyf_next_tgd_goal = Vy_tgd;
                % correct more in y direction.s
%                 dyf_next_tgd_goal = Vy_tgd + obj.lateral_correction_gain * ((-dyf_this_tgd*rp_stToe(3) - v_com(3)*(rp_swToe_tgd(2) - rp_stToe_tgd(2))) - (-(-Vy_tgd)*rp_stToe(3)))/rp_stToe(3); % -(-Vy_tgd)*rp_stToe(3) is reference AM at the begining of next step. (-dyf_this_tgd*rp_stToe(3) - v_com(3)*(rp_swToe_stTD0(2) - rp_stToe_stTD0(2))) is true AM at the begining of next step.
%                 dyf_next_tgd_goal = Vy_tgd + obj.lateral_correction_gain * ( (-Vy_tgd) - dyf_this_tgd); % Vg_tgd is target for next step, -Vy_tgd is target for this step.
                
%                 x0_next_tgd_goal = (dxf_next_tgd_goal - dxf_this_tgd*cosh(l*IRC.step_time))/(l*sinh(l*IRC.step_time));
%                 y0_next_tgd_goal = (dyf_next_tgd_goal - dyf_this_tgd*cosh(l*IRC.step_time))/(l*sinh(l*IRC.step_time));
%                 x0_next_tgd_goal = (dxf_next_tgd_goal - (dxf_this_tgd + v_com(3)* xf_this_tgd/rp_stToe(3))*cosh(l*IRC.step_time))/(l*sinh(l*IRC.step_time) - v_com(3)/rp_stToe(3)*cos(l*IRC.step_time));
%                 y0_next_tgd_goal = (dyf_next_tgd_goal - (dyf_this_tgd + v_com(3)* yf_this_tgd/rp_stToe(3))*cosh(l*IRC.step_time))/(l*sinh(l*IRC.step_time) - v_com(3)/rp_stToe(3)*cos(l*IRC.step_time)); 

                x0_next_tgd_goal = (dxf_next_tgd_goal - (dxf_this_tgd + v_com(3)* xf_this_tgd/obj.H_next)*cosh(l*IRC.step_time))/(l*sinh(l*IRC.step_time) - v_com(3)/obj.H_next*cos(l*IRC.step_time));
                y0_next_tgd_goal = (dyf_next_tgd_goal - (dyf_this_tgd + v_com(3)* yf_this_tgd/obj.H_next)*cosh(l*IRC.step_time))/(l*sinh(l*IRC.step_time) - v_com(3)/obj.H_next*cos(l*IRC.step_time));

                if direct_up_down
                    x0_next_tgd_goal = 0;
                    y0_next_tgd_goal = - abduction_direction * desired_com2stToe_lateral; % Notice: why negative sign? A: During left stance, y0_next_tgd_goal is for right toe.
                end
                
                xyz0_next_goal = YToolkits.Angles.Rz(obj.tg_direction) * [x0_next_tgd_goal; y0_next_tgd_goal; 0];
                
                x0_next_goal = xyz0_next_goal(1);
                y0_next_goal = xyz0_next_goal(2);
                %% obtain tg direction frame value
                v_com_tgd = YToolkits.Angles.Rz(obj.tg_direction)' * v_com;
                v_com_stTD0 = YToolkits.Angles.Rz(obj.stTD0)' * v_com;
                %% generate reference signal
                
%                 ref_rp_swToe_x = 1/2*(obj.rp_swToe_ini(1) - x0_next_goal)*cos(pi*s) + 1/2*(obj.rp_swToe_ini(1) + x0_next_goal);
%                 ref_rv_swToe_x = 1/2*(obj.rp_swToe_ini(1) - x0_next_goal)*(-pi*sin(pi*s)*ds);
%                 ref_ra_swToe_x = 1/2*(obj.rp_swToe_ini(1) - x0_next_goal)*(-pi^2*cos(pi*s)*ds^2);
%                 
%                 ref_rp_swToe_y = 1/2*(obj.rp_swToe_ini(2) - y0_next_goal)*cos(pi*s) + 1/2*(obj.rp_swToe_ini(2) + y0_next_goal);
%                 ref_rv_swToe_y = 1/2*(obj.rp_swToe_ini(2) - y0_next_goal)*(-pi*sin(pi*s)*ds);
%                 ref_ra_swToe_y = 1/2*(obj.rp_swToe_ini(2) - y0_next_goal)*(-pi^2*cos(pi*s)*ds^2);
                
                ref_rp_swToe_x = 1/2*(obj.rp_swToe_ini(1) - x0_next_goal)*cos(pi*(s-obj.s_ini)/(1-obj.s_ini)) + 1/2*(obj.rp_swToe_ini(1) + x0_next_goal);
                ref_rv_swToe_x = 1/2*(obj.rp_swToe_ini(1) - x0_next_goal)*(-pi/(1-obj.s_ini)*sin(pi*(s-obj.s_ini)/(1-obj.s_ini))*ds);
                ref_ra_swToe_x = 1/2*(obj.rp_swToe_ini(1) - x0_next_goal)*(-(pi/(1-obj.s_ini))^2*cos(pi*(s-obj.s_ini)/(1-obj.s_ini))*ds^2);
                
                ref_rp_swToe_y = 1/2*(obj.rp_swToe_ini(2) - y0_next_goal)*cos(pi*(s-obj.s_ini)/(1-obj.s_ini)) + 1/2*(obj.rp_swToe_ini(2) + y0_next_goal);
                ref_rv_swToe_y = 1/2*(obj.rp_swToe_ini(2) - y0_next_goal)*(-pi/(1-obj.s_ini)*sin(pi*(s-obj.s_ini)/(1-obj.s_ini))*ds);
                ref_ra_swToe_y = 1/2*(obj.rp_swToe_ini(2) - y0_next_goal)*(-(pi/(1-obj.s_ini))^2*cos(pi*(s-obj.s_ini)/(1-obj.s_ini))*ds^2);

%                 phase_overshoot = 0;
%                 overshoot_gain = 1+phase_overshoot/pi;
%                 ref_rp_swToe_x = 1/2*(obj.rp_swToe_ini(1) - x0_next_goal)*cos(overshoot_gain*pi*(s-obj.s_ini)/(1-obj.s_ini)) + 1/2*(obj.rp_swToe_ini(1) + x0_next_goal);
%                 ref_rv_swToe_x = 1/2*(obj.rp_swToe_ini(1) - x0_next_goal)*(-overshoot_gain*pi/(1-obj.s_ini)*sin(overshoot_gain*pi*(s-obj.s_ini)/(1-obj.s_ini))*ds);
%                 ref_ra_swToe_x = 1/2*(obj.rp_swToe_ini(1) - x0_next_goal)*(-(overshoot_gain*pi/(1-obj.s_ini))^2*cos(overshoot_gain*pi*(s-obj.s_ini)/(1-obj.s_ini))*ds^2);
                
%                 ref_rp_swToe_y = 1/2*(obj.rp_swToe_ini(2) - y0_next_goal)*cos(overshoot_gain*pi*(s-obj.s_ini)/(1-obj.s_ini)) + 1/2*(obj.rp_swToe_ini(2) + y0_next_goal);
%                 ref_rv_swToe_y = 1/2*(obj.rp_swToe_ini(2) - y0_next_goal)*(-overshoot_gain*pi/(1-obj.s_ini)*sin(overshoot_gain*pi*(s-obj.s_ini)/(1-obj.s_ini))*ds);
%                 ref_ra_swToe_y = 1/2*(obj.rp_swToe_ini(2) - y0_next_goal)*(-(overshoot_gain*pi/(1-obj.s_ini))^2*cos(overshoot_gain*pi*(s-obj.s_ini)/(1-obj.s_ini))*ds^2);
                

%                 coeff_1 = 1/(1-cos(overshoot_gain*pi));
%                 coeff_2 = -cos(overshoot_gain*pi)/(1-cos(overshoot_gain*pi));
%                 coeff_3 = -1/(1-cos(overshoot_gain*pi));
%                 coeff_4 = 1/(1-cos(overshoot_gain*pi));
%                 ref_rp_swToe_y = (coeff_1*obj.rp_swToe_ini(2) + coeff_3*y0_next_goal)*cos(overshoot_gain*pi*(s-obj.s_ini)/(1-obj.s_ini)) + (coeff_2*obj.rp_swToe_ini(2) + coeff_4*y0_next_goal);
%                 ref_rv_swToe_y = (coeff_1*obj.rp_swToe_ini(2) + coeff_3*y0_next_goal)*(-overshoot_gain*pi/(1-obj.s_ini)*sin(overshoot_gain*pi*(s-obj.s_ini)/(1-obj.s_ini))*ds);
%                 ref_ra_swToe_y = (coeff_1*obj.rp_swToe_ini(2) + coeff_3*y0_next_goal)*(-(overshoot_gain*pi/(1-obj.s_ini))^2*cos(overshoot_gain*pi*(s-obj.s_ini)/(1-obj.s_ini))*ds^2);

%                 ref_rp_swToe_z= 4*CL*(s-0.5)^2+(H-CL);
%                 ref_rv_swToe_z = 8*CL*(s-0.5)*ds;
%                 ref_ra_swToe_z = 8*CL*ds^2;
                para_a2 = -2 * obj.H_next + 2 * obj.rp_swToe_ini(3) + 4 * CL;
                para_a1 = 3 * obj.H_next - 3 * obj.rp_swToe_ini(3) - 4 * CL;
                para_a0 = obj.rp_swToe_ini(3);
                ref_rp_swToe_z= para_a2 * s^2 + para_a1 * s + para_a0;
                ref_rv_swToe_z = (2 * para_a2 * s + para_a1) * ds;
                ref_ra_swToe_z = (2 * para_a2) * ds^2;
                
                yaw_error = YToolkits.wrap_to_Pi(q(4) - obj.tg_direction);
                obj.yaw_error_fil = YToolkits.first_order_filter(obj.yaw_error_fil, yaw_error , obj.yaw_error_fil_param);
                
                ref_st_rotation = (1-s_sat) * obj.st_rotation_pos_last + s_sat*obj.st_rotation_goal;
                
                obj.sw_rotation_goal =  - yaw_error; % overwrite the sw_rotation_goal here
                obj.sw_rotation_goal = clamp2limit(obj, obj.sw_rotation_goal, sw_motor_rotation);
                ref_sw_rotation = (1-s_sat) * obj.sw_rotation_pos_last + s_sat*obj.sw_rotation_goal;
                
%                 ref_sw_rotation = YToolkits.bezier([obj.sw_rotation_pos_last, ...
%                                                     obj.sw_rotation_pos_last + 1/5*obj.sw_rotation_vel_last/ds, ...
%                                                     (obj.sw_rotation_pos_last + 1/5*obj.sw_rotation_vel_last/ds) + 1/3*(obj.sw_rotation_goal - (obj.sw_rotation_pos_last + 1/5*obj.sw_rotation_vel_last/ds)), ...
%                                                      (obj.sw_rotation_pos_last + 1/5*obj.sw_rotation_vel_last/ds) + 2/3*(obj.sw_rotation_goal - (obj.sw_rotation_pos_last + 1/5*obj.sw_rotation_vel_last/ds)), ...
%                                                     obj.sw_rotation_goal, obj.sw_rotation_goal],s_sat);
                                                
                ref_rp_stToe_z = (1-fs_2)*obj.rp_stToe_z_ini + fs_2 * (obj.H_this+spring_height_compensation);
                ref_rv_stToe_z = -dfs_2*obj.rp_stToe_z_ini + dfs_2 * (obj.H_this+spring_height_compensation);

%                 ref_rp_stToe_z = H;
%                 ref_rv_stToe_z = 0;

                
                obj.hr= [0;0;ref_st_rotation; ref_sw_rotation; ref_rp_stToe_z ; ref_rp_swToe_x; ref_rp_swToe_y; ref_rp_swToe_z];
                obj.dhr = [0;0;0;0;ref_rv_stToe_z;ref_rv_swToe_x;ref_rv_swToe_y;ref_rv_swToe_z];
                obj.ddhr = [0;0;0;0;0;ref_ra_swToe_x;ref_ra_swToe_y;ref_ra_swToe_z];
                
                if direct_up_down == 0
                    obj.dhr([6,7]) = obj.dhr([6,7]) + [eye(2), zeros(2,1)] * YToolkits.Angles.Rz(obj.tg_direction) * [-a_world(3)* xf_this_tgd/rp_stToe(3)*cosh(l*IRC.step_time)/(l*sinh(l*IRC.step_time) - v_com(3)/rp_stToe(3)*cos(l*IRC.step_time));...
                        -a_world(3)* yf_this_tgd/rp_stToe(3)*cosh(l*IRC.step_time)/(l*sinh(l*IRC.step_time) - v_com(3)/rp_stToe(3)*cos(l*IRC.step_time));...
                        0];
                end
                %% Inverse Kinematics
                
                
                % Jh is jacobian for output
                Jh = zeros(8,20);
                
                Jh(1,5) = 1; % torso pitch
                Jh(2,6) = 1; % torso roll
                Jh(3,st_rotation) = 1; % stance rotation
                Jh(4,sw_rotation) = 1; % swing rotation.
                Jh(5,:) = Jrp_stToe_rigid(3,:); % com to stance toe height
                Jh([6,7,8],:) = Jrp_swToe_rigid([1,2,3],:); % com to swing toe x y z relativ pos
                
                
                obj.h0 = [q_rigid(5);q_rigid(6);q_rigid(st_rotation);q_rigid(sw_rotation);rp_stToe_rigid(3);rp_swToe_rigid([1,2,3])];
                obj.dh0 = Jh*dq_rigid;
                
                
                %                 y = obj.h0 - obj.hr;
                %                 dy = obj.dh0 - obj.dhr;
                
                %             %% Inverse Kinematics
                obj.qd_control(1) = obj.hr(1); % torso pitch
                obj.qd_control(2) = obj.hr(2); % torso roll
                obj.qd_control(3) = obj.hr(3); % stance rotation
                obj.qd_control(4) = obj.hr(4); % swing rotation
                obj.dqd_control(1) = obj.dhr(1); % torso pitch
                obj.dqd_control(2) = obj.dhr(2); % torso roll
                obj.dqd_control(3) = obj.dhr(3); % stance rotation
                obj.dqd_control(4) = obj.dhr(4); % swing rotation
                
                % if leg switch, initially guess qd_control with actual value
                if LegSwitch == 1
                    obj.qd_control(5:8) = q_rigid([st_knee, sw_abduction, sw_thigh, sw_knee]);
                end
                q_iter = q_rigid;
                iter_num = 0;
                while 1
                    q_iter([st_knee, sw_abduction, sw_thigh, sw_knee, st_ankle, sw_ankle]) = [obj.qd_control(5:8); - obj.qd_control([5,8]) + deg2rad(13)]; % rigid constraint
                    q_iter(st_thigh) = q_rigid(st_thigh) - 1/2 * (q_iter(st_knee) - q_rigid(st_knee)); % assume the leg direction does not change after knee extend/retract
                    
                    p_com_iter =  p_COM(q_iter);
                    p_com_iter = p_com_iter + YToolkits.Angles.Rz(q(4))*[obj.com_x_offset;0;0];
                    p_LToe_iter =  p_LeftToeJoint(q_iter);
                    p_RToe_iter =  p_RightToeJoint(q_iter);
                    rp_LToe_iter =  p_com_iter - p_LToe_iter;
                    rp_RToe_iter =  p_com_iter - p_RToe_iter;
                    
                    Jp_com_iter =  Jp_COM(q_iter);
                    Jp_LToe_iter =  Jp_LeftToeJoint(q_iter);
                    Jp_RToe_iter =  Jp_RightToeJoint(q_iter);
                    Jrp_LToe_iter =  Jp_com_iter - Jp_LToe_iter;
                    Jrp_RToe_iter =  Jp_com_iter - Jp_RToe_iter;
                    
                    if stanceLeg == -1
                        rp_stToe_iter = rp_LToe_iter;
                        rp_swToe_iter = rp_RToe_iter;
                        Jrp_stToe_iter = Jrp_LToe_iter;
                        Jrp_swToe_iter = Jrp_RToe_iter;
                    else
                        rp_stToe_iter = rp_RToe_iter;
                        rp_swToe_iter = rp_LToe_iter;
                        Jrp_stToe_iter = Jrp_RToe_iter;
                        Jrp_swToe_iter = Jrp_LToe_iter;
                    end
                    J_allq = [Jrp_stToe_iter(3,:);Jrp_swToe_iter([1,2,3],:)];
                    J44 = [J_allq(:,st_knee) - J_allq(:,st_ankle) , J_allq(:,[sw_abduction, sw_thigh]), J_allq(:,sw_knee) - J_allq(:,sw_ankle)];
                    J44(:,1) = J44(:,1) - 1/2 * J_allq(:,st_thigh);
                    if max(abs(obj.hr(5:8) - [rp_stToe_iter(3);rp_swToe_iter([1,2,3])]))<0.001 || iter_num >= obj.IK_max_iter_num
                        break;
                    end
                    qd_control5_8_update = J44\(obj.hr(5:8) - [rp_stToe_iter(3);rp_swToe_iter([1,2,3])]);
                    if max(abs(qd_control5_8_update)) > 0.1
                        qd_control5_8_update = 0.1/max(abs(qd_control5_8_update)) * qd_control5_8_update;
                    end
                    obj.qd_control(5:8) = obj.qd_control(5:8) + qd_control5_8_update;
                    
                    obj.qd_control(5) = clamp2limit(obj, obj.qd_control(5), st_motor_knee);
                    obj.qd_control(6) = clamp2limit(obj, obj.qd_control(6), sw_motor_abduction);
                    obj.qd_control(7) = clamp2limit(obj, obj.qd_control(7), sw_motor_thigh);
                    obj.qd_control(8) = clamp2limit(obj, obj.qd_control(8), sw_motor_knee);
                    
                    iter_num = iter_num + 1;
                end
                obj.dqd_control(5:8) = J44\obj.dhr(5:8);
                %                 obj.ddhr(5) = 9.8*sqrt((rp_stToe(1)^2 + rp_stToe(2)^2))/rp_stToe(3) * sqrt((rp_stToe(1)^2 + rp_stToe(2)^2))/norm(rp_stToe);
                obj.ddqd_control(5:8) = J44\obj.ddhr(5:8);
                % special treatment for stance knee
                st_abd_thi_knee_fake_vel = (Jrp_stToe_rigid(:,[st_abduction, st_thigh, st_knee]) + [zeros(3,2),-Jrp_stToe_rigid(:,st_ankle)])\[v_com(1);v_com(2); 0];
%                 st_abd_thi_knee_fake_accel = (Jrp_stToe_rigid(:,[st_abduction, st_thigh, st_knee]) + [zeros(3,2),-Jrp_stToe_rigid(:,st_ankle)])\((32*9.8/32)*[rp_stToe(1)/rp_stToe(3); rp_stToe(2)/rp_stToe(3); 0]);
                obj.dqd_control(5) = st_abd_thi_knee_fake_vel(3);
%                 obj.ddqd_control(5) = st_abd_thi_knee_fake_accel(3);
                
                hr_recover = [obj.qd_control(1:4); rp_stToe_iter(3);rp_swToe_iter([1,2,3])];
                dhr_recover = [obj.dqd_control(1:4); J44*obj.dqd_control(5:8)];
                %% Compensate spring deflection
                ref_rp_swToe = obj.hr(6:8);
                %                 ref_rp_swToe_tgd = YToolkits.Angles.Rz(- obj.tg_direction)*ref_rp_swToe;
                gravity = 32*9.8;
                
%                 if stanceLeg == -1
%                     GRF_st_z = GRF_z(1);
%                 else
%                     GRF_st_z = GRF_z(2);
%                 end
                GRF_st_est = gravity * [ rp_stToe(1)/rp_stToe(3); rp_stToe(2)/rp_stToe(3); 1];% assume GRFz = gravity and GRF point to COM;
                GRF_sw_est = gravity * [ rp_swToe(1)/rp_swToe(3); rp_swToe(2)/rp_swToe(3); 1];% assume GRFz = gravity and GRF point to COM;
                % If leg touch the ground, the corresponding force on knee, knee spring and heel spring will be
                [tau_knee_st_est, tau_kneespring_st_est, tau_heelspring_st_est] = GRF_to_KneeAndTwoSprings(obj,GRF_st_est,q,stanceLeg);
                %                 [tau_knee_sw_est, tau_kneespring_sw_est, tau_heelspring_sw_est] = GRF_to_KneeAndTwoSprings(obj,GRF_sw_est,q,-stanceLeg);
                
                % compute the corresponding spring deflection.
                % stance leg
                deflection_kneespring_st_est = -tau_kneespring_st_est/Ks_st_1;
                deflection_heelspring_st_est = -tau_heelspring_st_est/Ks_st_2;
                J_ccct_st = ClosedChain_Coordinate_Transform_Jacobian(q([st_knee, st_shin, st_ankle])); %replace with rigid if vibration occurs
                if stanceLeg == -1
                    J_LL_st = J_LL_Left(q);
                    %                     J_LL_st = J_LL_Left(zeros(20,1));
                else
                    J_LL_st = J_LL_Right(q);
                    %                     J_LL_st = J_LL_Right(zeros(20,1));
                end
                J_LL_st_selected = J_LL_st([st_knee, st_shin, st_ankle]);
                J_LL_ccc = J_LL_st_selected*J_ccct_st^-1;
                LL_error = J_LL_ccc*[0;deflection_kneespring_st_est;deflection_heelspring_st_est];
                comp_knee_st = -LL_error/J_LL_ccc(1,1);
                dcomp_knee_st = (comp_knee_st - obj.comp_knee_st_prev)/obj.sample_time;
                %                 comp_knee_st_2 = (rp_stToe(3) - H)/J_LL(1,1);
                
                
                %                 Jsz1 = tau_kneespring_st_est/GRF_st_est(3);
                %                 Jsz2 = tau_heelspring_st_est/GRF_st_est(3);
                %                 k_equivalent = Ks_st_1 * Ks_st_2/(Ks_st_1*Jsz1^2 + Ks_st_2*Jsz2^2);
                
                a_comp_knee = -20;
                qd_control_adjusted = obj.qd_control;
                dqd_control_adjusted = obj.dqd_control;
                ddqd_control_adjusted = obj.ddqd_control;
                %                 qd_control_adjusted(5) = obj.qd_control(5) + comp_knee_st*(1 - exp(a_comp_knee*t));
                %                 dqd_control_adjusted(5) = obj.dqd_control(5) + comp_knee_st*(- a_comp_knee*exp(a_comp_knee*t));
                %                 ddqd_control_adjusted(5) = obj.ddqd_control(5) + comp_knee_st*(- a_comp_knee^2*exp(a_comp_knee*t));
                
                qd_control_adjusted(5) = obj.qd_control(5) + comp_knee_st;
                %                 dqd_control_adjusted(5) = obj.dqd_control(5) + dcomp_knee_st;
                %                 qd_control_adjusted(6) = obj.qd_control(6) + comp_sw(1);
                %                 qd_control_adjusted(7) = obj.qd_control(7) + comp_sw(2);
                %                 qd_control_adjusted(8) = obj.qd_control(8) + comp_sw(3);
                

                %                 if q(st_abduction) > obj.ActuatorLimits_ARsafety(st_motor_abduction,2) - roll_adjustment_thre
                %                    adjusted_desired_roll_angle = qd_control_adjusted(2) + q(sw_abduction) - (obj.ActuatorLimits_ARsafety(sw_motor_abduction,2)- roll_adjustment_thre);
                %                 end
                %                 if q(st_abduction) < obj.ActuatorLimits_ARsafety(st_motor_abduction,1) + roll_adjustment_thre
                %                    adjusted_desired_roll_angle = qd_control_adjusted(2) + q(sw_abduction) - (obj.ActuatorLimits_ARsafety(sw_motor_abduction,1)+ roll_adjustment_thre);
                %                 end
                %
                %                 if q(sw_abduction) > obj.ActuatorLimits_ARsafety(sw_motor_abduction,2) - roll_adjustment_thre
                %                    adjusted_desired_roll_angle = qd_control_adjusted(2) + q(st_abduction) - (obj.ActuatorLimits_ARsafety(sw_motor_abduction,2)- roll_adjustment_thre);
                %                 end
                %                 if q(sw_abduction) < obj.ActuatorLimits_ARsafety(sw_motor_abduction,1) + roll_adjustment_thre
                %                    adjusted_desired_roll_angle = qd_control_adjusted(2) + q(st_abduction) - (obj.ActuatorLimits_ARsafety(sw_motor_abduction,1)+ roll_adjustment_thre);
                %                 end
                if isSim
                    adjusted_desired_roll_angle = qd_control_adjusted(2);
                    roll_adjustment_thre = 0.1;
                    if qd_control_adjusted(6) > obj.ActuatorLimits_ARsafety(sw_motor_abduction,2) - roll_adjustment_thre
                        adjusted_desired_roll_angle = qd_control_adjusted(2) + qd_control_adjusted(6) - (obj.ActuatorLimits_ARsafety(sw_motor_abduction,2)- roll_adjustment_thre);
                    end
                    if qd_control_adjusted(6) < obj.ActuatorLimits_ARsafety(sw_motor_abduction,1) + roll_adjustment_thre
                        adjusted_desired_roll_angle = qd_control_adjusted(2) + qd_control_adjusted(6) - (obj.ActuatorLimits_ARsafety(sw_motor_abduction,1)+ roll_adjustment_thre);
                    end
%                     obj.adjusted_desired_roll_angle_fil = YToolkits.first_order_filter(obj.adjusted_desired_roll_angle_fil,adjusted_desired_roll_angle,obj.adjusted_desired_roll_angle_fil_param);
                    obj.adjusted_desired_roll_angle_fil = YToolkits.first_order_filter(obj.adjusted_desired_roll_angle_fil,adjusted_desired_roll_angle,1);
                    qd_control_adjusted(2) = obj.adjusted_desired_roll_angle_fil;
                end
                
                q0_control = zeros(8,1);
                q0_control(1) = q_ss(5);
                q0_control(2) = q_ss(6);
                q0_control(3) = q_ss(st_rotation);
                q0_control(4) = q_ss(sw_rotation);
                q0_control(5) = q_ss(st_knee);
                q0_control(6) = q_ss(sw_abduction);
                q0_control(7) = q_ss(sw_thigh);
                q0_control(8) = q_ss(sw_knee);
                dq0_control = zeros(8,1);
                dq0_control(1) = dq_ss(5);
                dq0_control(2) = dq_ss(6);
                dq0_control(3) = dq_ss(st_rotation);
                dq0_control(4) = dq_ss(sw_rotation);
                dq0_control(5) = dq_ss(st_knee);
                dq0_control(6) = dq_ss(sw_abduction);
                dq0_control(7) = dq_ss(sw_thigh);
                dq0_control(8) = dq_ss(sw_knee);
                
                %% Compute Torque
                qe_control = q0_control - qd_control_adjusted;
                dqe_control = dq0_control - dqd_control_adjusted;
                
                low_level_method = 3;
                switch low_level_method
                    case 1
                        %% compute torque (PD)
                        u_torso_pitch = -obj.Kp_pitch*q_ss(5) - obj.Kd_pitch*dq_ss(5);
                        u_torso_roll = obj.Kp_pitch*q_ss(6) + obj.Kd_pitch*dq_ss(6);
                        

                        obj.iqe_control_knee = obj.iqe_control_knee + obj.sample_time * dqe_control(5);
                        u(st_motor_thigh) = u_torso_pitch;
                        u(st_motor_abduction) = u_torso_roll;
                        u(st_motor_rotation) = -obj.Kp_st_rotation*qe_control(3) - obj.Kd_st_rotation*dqe_control(3);
                        u(sw_motor_rotation) = -obj.Kp_rotation*qe_control(4) - obj.Kd_rotation*dqe_control(4);
                        %                 u(st_motor_knee) = -obj.Kp_knee*qe_control(5) - obj.Kd_knee*dqe_control(5) - obj.Ki_knee*obj.iqe_control_knee;
                        u(st_motor_knee) = -obj.Kp_knee*qe_control(5) - obj.Kd_knee*dqe_control(5);
                        u(sw_motor_abduction) = -obj.Kp_abduction*qe_control(6) - obj.Kd_abduction*dqe_control(6);
                        u(sw_motor_thigh) = -obj.Kp_thigh*qe_control(7) - obj.Kd_thigh*dqe_control(7);
                        u(sw_motor_knee) = -obj.Kp_knee*qe_control(8) - obj.Kd_knee*dqe_control(8);
                        
                        % add compenstation for stance knee
                        u(st_motor_knee) = u(st_motor_knee) + tau_knee_st_est;
                        
                        % add compenstation for torso pitch and roll on stance
                        u(st_motor_thigh) = u(st_motor_thigh) +  obj.u_pitch_cp;
                        u(st_motor_abduction) = u(st_motor_abduction) + abduction_direction * obj.u_roll_cp;
                        
                        % control torso when leg is on the ground
                        u(st_motor_thigh) = f_GRF_st * u(st_motor_thigh) + (1 - f_GRF_st)*( - obj.Kd_thigh * dq_ss(st_thigh));
                        u(sw_motor_thigh) = f_GRF_sw * u_torso_pitch + (1 - f_GRF_sw)*u(sw_motor_thigh);
                        u(st_motor_abduction) = f_GRF_st * u(st_motor_abduction) + (1 - f_GRF_st)*( - obj.Kd_abduction * dq_ss(st_abduction));
                        u(sw_motor_abduction) = f_GRF_sw * u_torso_roll + (1 - f_GRF_sw)*u(sw_motor_abduction);
                    case 2
                        %% compute torque (PBC)
                        % get the indexes
                        qc_index = [5;6;st_rotation;sw_rotation;st_knee;sw_abduction;sw_thigh;sw_knee];
                        motor_index = [st_motor_thigh;st_motor_abduction;st_motor_rotation;sw_motor_rotation;st_motor_knee;sw_motor_abduction;sw_motor_thigh;sw_motor_knee];
                        Kp_PBC = [obj.Kp_pitch; obj.Kp_roll; Kp([st_motor_rotation;sw_motor_rotation;st_motor_knee;sw_motor_abduction;sw_motor_thigh;sw_motor_knee])];
                        Kd_PBC = [obj.Kd_pitch; obj.Kd_roll; Kd([st_motor_rotation;sw_motor_rotation;st_motor_knee;sw_motor_abduction;sw_motor_thigh;sw_motor_knee])];
                        
                        M = InertiaMatrix(q);
                        G = GravityVector(q);
                        B=zeros(20,10); B(7:10,1:4) = eye(4); B(13:17,5:9) = eye(5); B(20,10) = 1;
                        % body Jacobian on foot
                        Jb_L = Jb_LeftToeBottomBack(q);
                        Jb_R = Jb_RightToeBottomBack(q);
                        % cartesian Jacobian on foot
                        Jc_L = Jp_LeftToeBottomBack(q);
                        Jc_R = Jp_RightToeBottomBack(q);
                        % Homogeneous Transformation Matrix
                        T_L = T_LeftToeBottomBack(q);
                        T_R = T_RightToeBottomBack(q);
                        % Jacobian for ground constraint
                        Jg_L = zeros(5,20); Jg_L(1:3,:)= Jc_L; Jg_L([4,5],:)= Jb_L([5,6],:);
                        Jg_R = zeros(5,20); Jg_R(1:3,:)= Jc_R; Jg_R([4,5],:)= Jb_R([5,6],:);
                        if stanceLeg == -1
                            Jg = Jg_L;
                        else
                            Jg = Jg_R;
                        end
                        [Jgd1,Jgd2] = size(Jg);
                        
                        JsL = zeros(2,20); JsL(1,11) = 1; JsL(2,[10 11 12]) = J_HeelSpringDeflectionEst(q(10),q(11),q(12));
                        JsR = zeros(2,20); JsR(1,18) = 1; JsR(2,[17 18 19]) = J_HeelSpringDeflectionEst(q(17),q(18),q(19));
                        Js = [JsL; JsR];
                        
                        [Jsd1,Jsd2] = size(Js);
                        [JsLd1,JsLd2] = size(JsL);
                        [JsRd1,JsRd2] = size(JsR);
                        %Extended Matrix ( for walking)
                        Me = [M, -Jg', -Js';
                            Jg, zeros(Jgd1,Jgd1), zeros(Jgd1,Jsd1);
                            Js, zeros(Jsd1,Jgd1), zeros(Jsd1,Jsd1)];
                        He = [G;zeros(Jgd1,1);zeros(Jsd1,1)];
                        B = zeros(20,10); B(7:10,1:4) = eye(4); B(13:17,5:9) = eye(5); B(20,10) = 1;
                        B_cut = B(:,motor_index);
                        Be = [B_cut;zeros(Jgd1+Jsd1,length(motor_index))];
                        
                        Me_re = YToolkits.SquareMatrixReorder(Me,qc_index);
                        He_re = YToolkits.VectorReorder(He,qc_index);
                        Be_re = YToolkits.VectorReorder(Be,qc_index);
                        [M11, M12, M21, M22, H1, H2, B1, B2] = YToolkits.PartitionEOM(Me_re,He_re,Be_re,length(qc_index));
                        M22_inv = M22^-1;
                        M_bar = M11 - M12*M22_inv*M21;
                        H_bar = -M12*M22_inv*H2+H1;
                        B_bar = -M12*M22_inv*B2 + B1;
                        B_bar_inv = B_bar^-1;
                        M_bar_inv = M_bar^-1;
                        
                        u8 = B_bar_inv*(M_bar*ddqd_control_adjusted + H_bar -Kd_PBC.*dqe_control - Kp_PBC.*qe_control);
                        u_ff = B_bar_inv*(M_bar*ddqd_control_adjusted);
                        u_gv = B_bar_inv*H_bar;
                        u_fb = B_bar_inv*( -Kd_PBC.*dqe_control - Kp_PBC.*qe_control);
                        u(motor_index) = u8;
                    case 3
                        %% compute torque (PBC matrix + static gravity)
                        [u_gc_st_abduction, u_gc_st_thigh, u_gc_sw_abduction, u_gc_sw_thigh] = get_gravity_compensation(q, stanceLeg); % Gravity compensation for st_abduction, st_thigh, sw_abduction, sw_thigh
                        % get the indexes
                        qc_index = [5;6;st_rotation;sw_rotation;st_knee;sw_abduction;sw_thigh;sw_knee];
                        motor_index = [st_motor_thigh;st_motor_abduction;st_motor_rotation;sw_motor_rotation;st_motor_knee;sw_motor_abduction;sw_motor_thigh;sw_motor_knee];
                        Kp_PBC = [obj.Kp_pitch; obj.Kp_roll; obj.Kp_st_rotation; Kp([sw_motor_rotation;st_motor_knee;sw_motor_abduction;sw_motor_thigh;sw_motor_knee])];
                        Kd_PBC = [obj.Kd_pitch; obj.Kd_roll; obj.Kd_st_rotation; Kd([sw_motor_rotation;st_motor_knee;sw_motor_abduction;sw_motor_thigh;sw_motor_knee])];
                        
%                         [B_bar,B_bar_inv,H_bar,M_bar,M_bar_inv] = Select_DynamicMatrix(obj,DynamicMatrixLibrary,q(10),q(17),stanceLeg);
                        
                        [B_bar_inv,B_bar_inv_times_M_bar] = Select_DynamicMatrix(obj,DynamicMatrixLibrary,q(10),q(17),stanceLeg);
                        
%                         u8 = B_bar_inv*(M_bar*obj.ddqd_control + H_bar -Kd_PBC.*dqe_control - Kp_PBC.*qe_control);

                        u_ff = B_bar_inv_times_M_bar * ddqd_control_adjusted;
                        u_fb = B_bar_inv*( -Kd_PBC.*dqe_control - Kp_PBC.*qe_control);
                        u_gv = [u_gc_st_thigh; u_gc_st_abduction; 0; 0; tau_knee_st_est; u_gc_sw_abduction; u_gc_sw_thigh; 0];
                        u(motor_index) = u_ff + u_gv + u_fb;
                        
                end
                % flat swing toe
                [Angle_LToe, Angle_RToe, dAngle_LToe, dAngle_RToe] = getToeAbsoluteAngle_rigid(q,dq);
                if stanceLeg == -1
                    u(10) = - obj.Kp_toe * Angle_RToe - obj.Kd_toe * dAngle_RToe;
                else
                    u(5) = - obj.Kp_toe * Angle_LToe - obj.Kd_toe * dAngle_LToe;
                end

                
%                 % swing thigh & abduction don't apply torque until leg leave ground
%                 if stanceLeg == -1
%                     f_GRF_sw_2 = f_GRF_R_2;
%                 else
%                     f_GRF_sw_2 = f_GRF_L_2;
%                 end
% 
%                 if obj.swing_leg_leave_ground == 0
%                     obj.rp_swToe_ini(1:2) = rp_swToe_rigid(1:2);
%                     u(sw_motor_thigh) = (1-f_GRF_sw_2)*u(sw_motor_thigh);
%                     u(sw_motor_abduction) = (1-f_GRF_sw_2)*u(sw_motor_abduction);
%                 end
%                 if f_GRF_sw_2 <0.01
%                     obj.swing_leg_leave_ground = 1;
%                 end
                % limit stance rotation maximum torque
%                 u(st_motor_rotation) = median([-obj.max_st_rotation_motor_torque,obj.max_st_rotation_motor_torque,u(st_motor_rotation)]);
%                 u(sw_motor_rotation) = median([-obj.max_st_rotation_motor_torque,obj.max_st_rotation_motor_torque,u(sw_motor_rotation)]);
                %% smooth torque and assign
                u = ft_1*u + (1 - ft_1) * obj.u_last;
                % ramp st_motor_rotation torque up
                u(st_motor_rotation) = ft_4 * u(st_motor_rotation);
                
                % swing thigh & abduction don't apply torque until leg leave ground
                if LegSwitch == 1
                    obj.ini_toe_height_diff = p_swToe(3) - p_stToe(3);
                end
                f_thd = (((p_swToe(3) - p_stToe(3)) - obj.ini_toe_height_diff) - obj.toe_leave_ground_lb)/(obj.toe_leave_ground_ub - obj.toe_leave_ground_lb); % a ramp up function about toe height difference
                f_thd = median([f_thd,0,1]);
                if obj.swing_leg_leave_ground == 0
                    obj.rp_swToe_ini(1:2) = rp_swToe_rigid(1:2);
                    obj.s_ini = min([s, 0.6]); % s_ini will be the s when swing leg begin to swing.
                    u(sw_motor_thigh) = f_thd * u(sw_motor_thigh);
                    u(sw_motor_abduction) = f_thd * u(sw_motor_abduction);
                end
                if f_thd > 0.99
                    obj.swing_leg_leave_ground = 1;
                end
                
                % limit stance rotation maximum torque
                if stanceLeg == -1
                    GRF_st = EstStates.GRF_L;
                else
                    GRF_st = EstStates.GRF_R;
                end
                max_toe_yaw_torque = get_max_toe_yaw_torque(GRF_st(1), GRF_st(2), 0.6);
                u(st_motor_rotation) = median([- max_toe_yaw_torque, max_toe_yaw_torque, u(st_motor_rotation)]);
                
%                 u = zeros(10,1);
%                 u(4) = -1000*(q(10) - (-1.3965)) - 50*dq(10) + 120;
%                 u(9) = -1000*(q(17) - (-2)) - 20*dq(17);

                userInputs.torque = u;
                
                if obj.initialized == 0
                    obj.initialized = 1;
                end
                %% for plot purpose
                qmd = zeros(10,1); % reference trajecotry for motors
                dqmd = zeros(10,1);
                qm0 = [q_ss(7:10);q_ss(13);q_ss(14:17);q_ss(20)];
                dqm0 = [dq_ss(7:10);dq_ss(13);dq_ss(14:17);dq_ss(20)];
                if stanceLeg == -1
                    qmd(1) = q_ss(7);
                    qmd(2) = obj.qd_control(3);
                    qmd(3) = q_ss(9);
                    qmd(4) = obj.qd_control(5);
                    qmd(5) = 0;
                    qmd(6) = obj.qd_control(6);
                    qmd(7) = obj.qd_control(4);
                    qmd(8) = obj.qd_control(7);
                    qmd(9) = obj.qd_control(8);
                    qmd(10) = 0;
                    
                    dqmd(1) = dq_ss(7);
                    dqmd(2) = obj.qd_control(3);
                    dqmd(3) = dq_ss(9);
                    dqmd(4) = obj.qd_control(5);
                    dqmd(5) = 0;
                    dqmd(6) = obj.qd_control(6);
                    dqmd(7) = obj.qd_control(4);
                    dqmd(8) = obj.qd_control(7);
                    dqmd(9) = obj.qd_control(8);
                    dqmd(10) = 0;
                else
                    qmd(1) = obj.qd_control(6);
                    qmd(2) = obj.qd_control(4);
                    qmd(3) = obj.qd_control(7);
                    qmd(4) = obj.qd_control(8);
                    qmd(5) = 0;
                    qmd(6) = q_ss(14);
                    qmd(7) = obj.qd_control(3);
                    qmd(8) = q_ss(16);
                    qmd(9) = obj.qd_control(5);
                    qmd(10) = 0;
                    
                    dqmd(1) = obj.qd_control(6);
                    dqmd(2) = obj.qd_control(4);
                    dqmd(3) = obj.qd_control(7);
                    dqmd(4) = obj.qd_control(8);
                    dqmd(5) = 0;
                    dqmd(6) = dq_ss(14);
                    dqmd(7) = obj.qd_control(3);
                    dqmd(8) = dq_ss(16);
                    dqmd(9) = obj.qd_control(5);
                    dqmd(10) = 0;
                end
                %% log variables for next iteration
                obj.u_prev = u;
                obj.comp_knee_st_prev = comp_knee_st;
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
                
%                 Data.Lx_stToe_kf = obj.Lx_stToe_kf;
%                 Data.Ly_stToe_kf = obj.Ly_stToe_kf;
                
%                 Data.dx0_next = dx0_next;
                Data.x0_next_goal = x0_next_goal;
%                 Data.dxf_next_goal = dxf_next_goal;

                Data.dxf_this_stTD0 = dxf_this_stTD0;
                Data.x0_next_tgd_goal = x0_next_tgd_goal;
                Data.dxf_next_tgd_goal = dxf_next_tgd_goal;
                
                Data.dyf_this_stTD0 = dyf_this_stTD0;
                Data.dyf_this_tgd = dyf_this_tgd;
                Data.dyf_next_tgd_goal = dyf_next_tgd_goal;
                
                Data.tg_direction = obj.tg_direction;
                Data.stTD0 = obj.stTD0;
                
                Data.Vx_tgd = Vx_tgd;
                Data.Vy_tgd = Vy_tgd;
                
                Data.hr = obj.hr;
                Data.dhr = obj.dhr;
                Data.ddhr = obj.ddhr;
                Data.h0 = obj.h0;
                Data.dh0 = obj.dh0;
                
                Data.hr_recover = hr_recover;
                Data.dhr_recover = dhr_recover;
                Data.iter_num = iter_num;
                
                Data.qd_control = obj.qd_control;
                Data.dqd_control = obj.dqd_control;
                Data.q0_control = q0_control;
                Data.dq0_control = dq0_control;
                
                Data.qd_control_adjusted = qd_control_adjusted;
                Data.dqd_control_adjusted = dqd_control_adjusted;
                
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
                
                Data.u_ff = u_ff;
                Data.u_fb = u_fb;
                Data.u_gv = u_gv;
                
                Data.qmd = qmd;
                Data.dqmd = dqmd;
                Data.qm0 = qm0;
                Data.dqm0 = dqm0;
                
                Data.qsL_1 = EstStates.qsL_1;
                Data.qsL_2 = EstStates.qsL_2;
                Data.qsR_1 = EstStates.qsR_1;
                Data.qsR_2 = EstStates.qsR_2;
                
                Data.dqsL_1 = EstStates.dqsL_1;
                Data.dqsL_2 = EstStates.dqsL_2;
                Data.dqsR_1 = EstStates.dqsR_1;
                Data.dqsR_2 = EstStates.dqsR_2;
                
                Data.tau_knee_st_est = tau_knee_st_est;
                Data.tau_kneespring_st_est = tau_kneespring_st_est;
                Data.tau_heelspring_st_est = tau_heelspring_st_est;
                Data.comp_knee_st = comp_knee_st;
                Data.kneespring_st_stiffness = - tau_kneespring_st_est/qs_st_1;
                Data.heelspring_st_stiffness = - tau_heelspring_st_est/qs_st_2;
                
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