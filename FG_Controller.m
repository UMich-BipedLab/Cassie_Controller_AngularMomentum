%Yukai controller.

classdef FG_Controller <matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime %#codegen
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
        
        Kfs_p;
        Kfl_p;
        Kfs_d;
        Kfl_d;
        
        Kp_toe_stand;
        Kd_toe_stand;
        Kp_lateral_stand;
        Kd_lateral_stand;
        Kp_abduction_stand;
        Kd_abduction_stand;
        Kp_thigh_stand;
        Kd_thigh_stand;
        Kp_knee_stand;
        Kd_knee_stand;
        Kp_rotation_stand;
        Kd_rotation_stand;
        
        fil_para_x;
        fil_para_y;
        
        stance_thre_ub;
        stance_thre_lb;
        
        lateral_velocity_weight;
        init_lateral_velocity;
        
        abduction_inward_gain;
        
        joint_filter_choice;
        
        shift_time;
        shift_distance;
        standing_switch_time;
        
        toe_tilt_angle;
        
        final_sw_abduction;
        final_st_abduction;
        pre_final_sw_abduction;
        
        sagittal_offset_exp;
        lateral_offset_exp;
        turning_offset_exp;
        stand_offset_exp;
        
        u_abduction_cp;
        u_abduction_swing_cp;
        u_thigh_cp;
        u_knee_cp;
        
        fil_para_2;
        fil_para_3;
        fil_para_4;
        fil_para_5;
        
        force_step_end_s;
        
        Kp_toe_Segway;
        Kd_toe_Segway;
    end
    properties (Access = private, Constant)
        TorqueLimits = repmat([112.5;112.5;195.2;195.2;45],[2,1]);
        ActuatorLimits = [-0.2618, 0.3927;    -0.3927, 0.3927;    -0.8727, 1.3963;    -2.8623, -0.7330;   -2.4435, -0.5236; ...
            -0.3927, 0.2618;    -0.3927, 0.3927;    -0.8727, 1.3963;    -2.8623, -0.7330;   -2.4435, -0.5236];
        Ks1 = 1500;
        Ks2 = 1250;
    end
    properties (Access = protected)
        
        sagittal_offset = -0.01;
        lateral_offset = 0;
        turning_offset = 0;
        stand_offset = -0.01;
        
        Toe_thigh_offset = 1.0996;
        safe_TorqueLimits = repmat([112.5;112.5;195.2;195.2;45],[2,1]);
        
        standing_abduction_offset = 0.08;
        bezier_degree = 5;
    end
    % PRIVATE PROPERTIES ====================================================
    properties (Access = private)
        Kp = zeros(10,1);
        Kd = zeros(10,1);
        stanceLeg = 1;
        begin = 0;
        Switch = 0; % if switching stance leg
        walking_ini = 0;
        step_end = 0;
        task = 2; % 0 is do nothing; 1 is walking; 2 is standing up; 3 is test
        task_prev = 0;
        task_next = 0;
        t0 = 0; % the time when starting a step
        t_prev = 0;
        t1 = 0;
        s1 = 0; % transition time for stand to walk
        t2 = 0;
        s2 = 0; % transition time for walk to stand
        u_prev = zeros(10,1); % u in previous iteration
        u_last = zeros(10,1);% u in last step
        s_prev = 0;
        s_unsat_prev = 0;
        dqy_b_start = 0;
        gaitparams = struct( 'HAlpha',zeros(10,6),'ct',0);
        tau = 0;
        tau_prev = 0;
        GRF_vL_history = zeros(10,1);
        GRF_vR_history = zeros(10,1);
        
        foot_placement = 1;
        pitch_torso_control = 1;
        roll_torso_control = 1;
        stance_passive = 1;
        knee_com = 0;
        abduction_com = 0;
        thigh_compensation = 0;
        abduction_swing_com = 0; % was always 0
        reduce_impact = 0;
        keep_direction = 0;
        
        to_turn = 0;
        to_turn_prev = 1;
        tg_yaw = 0;
        
        tg_velocity_x = 0;
        
        uHip_gravity_2 = 1.2; % for swing leg
        dqx_b_fil = 0;
        dqy_b_fil = 0;
        dqz_b_fil = 0;
        dqx_fil = 0;
        dqy_fil = 0;
        dqz_fil = 0;
        com_vel_x_fil = 0;
        com_vel_y_fil = 0;
        com_vel_z_fil = 0;
        com_pos_x_fil = 0;
        com_pos_y_fil = 0;
        com_pos_z_fil = 0;
        
        
        pitch_des_fil = 0;
        tg_velocity_x_fil = 0;
        lateral_move_fil = 0;
        
        LL_des_fil = 0.9;
        roll_des_fil = 0;
        LL_des = 0.7;
        dLL_des_fil = 0;
        P_feedback_toe_fil = 0;
        
        Fs_fil = zeros(4,1);
        
        hd = zeros(10,1);
        dhd = zeros(10,1);
        hd_joint = zeros(10,1);
        dhd_joint = zeros(10,1);
        h0 = zeros(10,1);
        dh0 = zeros(10,1);
        h0_joint = zeros(10,1);
        dh0_joint = zeros(10,1);
        y = zeros(10,1);
        dy = zeros(10,1);
        y_joint = zeros(10,1);
        dy_joint = zeros(10,1);
        
        hd_prev = zeros(10,1);
        dhd_prev = zeros(10,1);
        h0_prev = zeros(10,1);
        dh0_prev = zeros(10,1);
        y_prev = zeros(10,1);
        dy_prev = zeros(10,1);
        hd_joint_prev = zeros(10,1);
        dhd_joint_prev = zeros(10,1);
        h0_joint_prev = zeros(10,1);
        dh0_joint_prev = zeros(10,1);
        y_joint_prev = zeros(10,1);
        dy_joint_prev = zeros(10,1);
        
        ddhr = zeros(10,1);
        ddhr_joint = zeros(10,1);
        
        
        hd_last = zeros(10,1);
        dhd_last = zeros(10,1);
        h0_last = zeros(10,1);
        dh0_last = zeros(10,1);
        
        v_final = zeros(2,1);
        v_final_avgy = 0;
        tp_last = 0; % how long does last step take
        
        sagittal_move = 0;
        lateral_move = 0;
        rotation_move = 0;
        
        to_stand_step_count = 0;
        
        
    end % properties
    
    % PROTECTED METHODS =====================================================
    methods (Access = protected)
        
        function [userInputs, Data] = stepImpl(obj, t, cassieOutputs, isSim, GaitLibrary, encoder_fil, DynamicMatrixLibrary)
            %STEPIMPL System output and state update equations.
            
            %% Initialize --------------------------------------------------------
            Data = PreFunctions.Construct_Data;
            % Reset the desired motor torques to zero in case they aren't defined
            userInputs = CassieModule.getUserInStruct;
            u = zeros(10,1);
            % Update the Cassie outputs data structure
            RadioButton = RadioChannelToButton(cassieOutputs.pelvis.radio.channel);
            obj.Kp = repmat([obj.Kp_abduction;obj.Kp_rotation;obj.Kp_thigh;obj.Kp_knee; obj.Kp_toe],[2,1]);
            obj.Kd = repmat([obj.Kd_abduction;obj.Kd_rotation;obj.Kd_thigh;obj.Kd_knee; obj.Kd_toe],[2,1]);
            if (isSim == 0 && t>10) || (isSim >0 && t>0.01)
                obj.begin = 1;
            end
            %% Read Commands
            % In experiment some settings are different from simulation
            if isSim == 0
                obj.pitch_torso_control = 0;
                obj.knee_com = 0;
                obj.abduction_com = 0;
                obj.foot_placement = 1;
                obj.lateral_offset = obj.lateral_offset_exp;
                obj.sagittal_offset = obj.sagittal_offset_exp - 0.02 * RadioButton.S1A;
                obj.turning_offset = obj.turning_offset_exp;
                obj.stand_offset = obj.stand_offset_exp + RadioButton.S2A*0.1;
            end
            % Receive command signal from RC radio
            if RadioButton.LVA < 0 % To prevent walking backward too fast
                p = 0.5;
            else
                p = 1;
            end
            obj.tg_velocity_x = p* RadioButton.LVA;
            obj.tg_velocity_x_fil = YToolkits.first_order_filter(obj.tg_velocity_x_fil, obj.tg_velocity_x, 0.0003);
            obj.lateral_move = 0.015*RadioButton.LHA;
            obj.lateral_move_fil = YToolkits.first_order_filter(obj.lateral_move_fil, obj.lateral_move, 0.0003);
            obj.rotation_move = -0.2*RadioButton.RHA;
            if abs(RadioButton.RHA)<0.1
                obj.to_turn = -1;
            else
                obj.to_turn = 1;
            end
            pitch_des = median([RadioButton.RVA,-0.5,0.5]);
            obj.pitch_des_fil = YToolkits.first_order_filter(obj.pitch_des_fil, pitch_des, 0.0005);
            LL_des = 0.68+RadioButton.LSA*0.2;
            obj.LL_des_fil = YToolkits.first_order_filter(obj.LL_des_fil,LL_des,obj.fil_para_4);
            roll_des = RadioButton.RHA*0.1;
            obj.roll_des_fil = YToolkits.first_order_filter(obj.roll_des_fil,roll_des,0.0003); % cutoff frequency 0.1 Hz;
            PBC = RadioButton.SDA;
            leg_up = RadioButton.SCA;
            %% state machine ( walk or stand)
            if RadioButton.SBA == 1
                obj.task_next = 1;
            else
                obj.task_next = 2;
            end
            if obj.task_next == 2
                obj.t1 = 0;
                obj.s1 = 0;
            end
            if obj.task_next ==1 && obj.s1 > 1
                obj.task = obj.task_next;
                obj.P_feedback_toe_fil = 0;
            end
            if obj.task_next == 2 && obj.step_end
                obj.task =2;
                obj.step_end = 0;
                obj.t2 = 0;
                obj.u_last = obj.u_prev;
            end
            %% begin calculation
            if  obj.begin ==1
                %% get values
                [qyaw, qpitch, qroll, dqyaw, dqpitch, dqroll] = IMU_to_Euler_v2(cassieOutputs.pelvis.vectorNav.orientation, cassieOutputs.pelvis.vectorNav.angularVelocity);
                qa = CassieModule.getDriveProperty(cassieOutputs,'position');
                dqa = CassieModule.getDriveProperty(cassieOutputs,'velocity');
                qj = CassieModule.getJointProperty(cassieOutputs,'position');
                dqj = CassieModule.getJointProperty(cassieOutputs,'velocity');
                qq = cassieOutputs.pelvis.vectorNav.orientation;
                qaL = qa(1:5);
                qaR = qa(6:10);
                qjL =  qj(1:2);
                qjR =  qj(4:5);
%                 qsL = getSpringDeflectionAngle(qaL(4),qjL(1),qjL(2));
%                 qsR = getSpringDeflectionAngle(qaR(4),qjR(1),qjR(2));
                qsL = getSpringDeflectionAngleV2(qaL(4),qjL(1),qjL(2));
                qsR = getSpringDeflectionAngleV2(qaR(4),qjR(1),qjR(2));                
                
                % Get current velocities
                dqaL =  dqa(1:5);
                dqaR =  dqa(6:10);
                dqjL =  dqj(1:2);
                dqjR =  dqj(4:5);
%                 dqsL = getSpringDeflectionRate(dqaL(4),dqjL(1),dqjL(2));
%                 dqsR = getSpringDeflectionRate(dqaR(4),dqjR(1),dqjR(2));
                dqsL = getSpringDeflectionRateV2(qaL(4),qjL(1),qjL(2),dqaL(4),dqjL(1),dqjL(2));
                dqsR = getSpringDeflectionRateV2(qaR(4),qjR(1),qjR(2),dqaR(4),dqjR(1),dqjR(2));
                
                % assign the value
                q_abduction_R = qaR(1);
                q_rotation_R = qaR(2);
                q_thigh_R = qaR(3);
                q_thigh_knee_R = qaR(4);
                q_knee_shin_R = qjR(1);
                q_thigh_shin_R = q_thigh_knee_R+q_knee_shin_R;
                q_shin_tarsus_R = qjR(2);
                q_toe_R = qaR(5);
                
                q_abduction_L = qaL(1);
                q_rotation_L = qaL(2);
                q_thigh_L = qaL(3);
                q_thigh_knee_L = qaL(4);
                q_knee_shin_L = qjL(1);
                q_thigh_shin_L = q_thigh_knee_L+q_knee_shin_L;
                q_shin_tarsus_L = qjL(2);
                q_toe_L = qaL(5);
                
                
                dq_abduction_R = dqaR(1);
                dq_rotation_R = dqaR(2);
                dq_thigh_R = dqaR(3);
                dq_thigh_knee_R = dqaR(4);
                dq_knee_shin_R = dqjR(1);
                dq_thigh_shin_R = dq_thigh_knee_R+dq_knee_shin_R;
                dq_shin_tarsus_R = dqjR(2);
                dq_toe_R = dqaR(5);
                
                dq_abduction_L = dqaL(1);
                dq_rotation_L = dqaL(2);
                dq_thigh_L = dqaL(3);
                dq_thigh_knee_L = dqaL(4);
                dq_knee_shin_L = dqjL(1);
                dq_thigh_shin_L = dq_thigh_knee_L+dq_knee_shin_L;
                dq_shin_tarsus_L = dqjL(2);
                dq_toe_L = dqaL(5);
                
                qall = [  0;  0;              0;              qyaw;           qpitch;              qroll;
                    q_abduction_L;	q_rotation_L;	q_thigh_L;      q_thigh_knee_L;     q_knee_shin_L;      q_shin_tarsus_L;    q_toe_L;
                    q_abduction_R;	q_rotation_R;	q_thigh_R;      q_thigh_knee_R;     q_knee_shin_R;      q_shin_tarsus_R;    q_toe_R];
                
                dqall = [ 0;  0;              0;              dqyaw;         dqpitch;            dqroll;
                    dq_abduction_L;	dq_rotation_L;	dq_thigh_L;     dq_thigh_knee_L;    dq_knee_shin_L;     dq_shin_tarsus_L;   dq_toe_L;
                    dq_abduction_R;	dq_rotation_R;	dq_thigh_R;     dq_thigh_knee_R;    dq_knee_shin_R;     dq_shin_tarsus_R;   dq_toe_R];
                
                %% process Data
                
                % get current h0_joint and h0
                obj.h0_joint=[qaL; qaR];
                obj.dh0_joint = [dqaL; dqaR];
                %                 if obj.joint_filter_choice == 1 % Use AR joint filter or custom filter.
                %                     obj.dh0_joint([3,4,5,8,9,10]) = encoder_fil([3,4,5,8,9,10]);
                %                 end
                [ obj.h0, obj.dh0] = get_FK(obj, obj.h0_joint,obj.dh0_joint);
                
                % get GRF and s
                if cassieOutputs.isCalibrated == 1
                    [ GRF_L, GRF_R ] = get_GRF(obj,qall,qsR,qsL,0);
                else
                    GRF_L = [0;0];
                    GRF_R = [0;0];
                end
                if isSim == 2
                    GRF_L(2) = dqj(3);
                    GRF_R(2) = dqj(6);
                end
                
                GRF_v = [GRF_L(2) GRF_R(2) ];
                obj.GRF_vL_history(2:end) = obj.GRF_vL_history(1:end-1);
                obj.GRF_vL_history(1) = GRF_L(2);
                obj.GRF_vR_history(2:end) = obj.GRF_vR_history(1:end-1);
                obj.GRF_vR_history(1) = GRF_R(2);
                [Fs1R, Fs2R, Fs1L, Fs2L] = get_spring_force(obj,qsR,qsL);
                [ s_L, s_R ] = obj.get_s_LR(GRF_v); % s is normalized between 0 and 1, 0 means the leg is in air and 1 means leg is on ground.
                s_LR = [s_L; s_R];
                
                if obj.stanceLeg == 1
                    swing_grf = GRF_L(2);
                    stance_grf = GRF_R(2);
                else
                    swing_grf = GRF_R(2);
                    stance_grf = GRF_L(2);
                end
                
                if obj.stanceLeg == 1
                    swing_grf_history = obj.GRF_vL_history;
                    stance_grf_history = obj.GRF_vR_history;
                else
                    swing_grf_history = obj.GRF_vR_history;
                    stance_grf_history = obj.GRF_vL_history;
                end
                
                % if not walking in previous moment, reset the current hd
                % to smooth the torque
                if obj.task == 1 && obj.task_prev~=1
                    obj.walking_ini = 0;
                    %                     obj.hd_joint =obj.h0_joint + obj.u_prev./obj.Kp;
                    obj.hd_joint =obj.h0_joint;
                    obj.dhd_joint = obj.dh0_joint;
                    [obj.hd, obj.dhd] = get_FK(obj, obj.hd_joint, obj.dhd_joint);
                end
                
                % If Cassie is standing or turning, swing leg is NOT commanded to maintain the direction of the robot.
                if (obj.to_turn ~=1 && obj.to_turn_prev == 1) || obj.task == 2
                    obj.tg_yaw = qyaw;
                end
                
                % what happens when a step end and a new step begin.
                if (obj.s_prev >=0.5 && swing_grf_history(1) > obj.stance_thre_ub) || obj.walking_ini == 0 || obj.s_unsat_prev > obj.force_step_end_s
%                 if (obj.s_prev >=0.5 && swing_grf > obj.stance_thre_ub) || obj.walking_ini == 0 || obj.s_unsat_prev > obj.force_step_end_s
                    obj.stanceLeg = -obj.stanceLeg;
                    if obj.walking_ini == 0
                        obj.stanceLeg = -1; % At the beginning of a step. stanceLeg is always left leg
                    end
                    obj.tp_last = t - obj.t0;
                    obj.t0=t;
                    obj.t_prev = obj.t0;
                    obj.s_prev = 0;
                    obj.s_unsat_prev = 0;
                    obj.Switch = 1;
                    obj.walking_ini =1;
                    obj.u_last = obj.u_prev; % u_prev is the torque command in previous iteration, u_last is the torque command at the end of last step
                    % save the velocity data at the end of a step.
                    obj.v_final = [obj.dqx_b_fil; obj.dqy_b_fil];
                    obj.v_final_avgy = (obj.lateral_velocity_weight*obj.dqy_b_fil+(1-obj.lateral_velocity_weight)*obj.dqy_b_start);
                    obj.dqy_b_start = obj.dqy_b_fil;
                    
                    obj.hd_last = obj.hd; % save the desired output at the end of a step. It is used to reset the first 2 bezier coefficient in next step to smooth the torque.
                    obj.dhd_last = obj.dhd;
                    obj.h0_last = obj.h0; % save the desired output at the end of a step. It is used to reset the first 2 bezier coefficient in next step to smooth the torque.
                    obj.dh0_last = obj.dh0;
                    %                     obj.hd_last = obj.h0; % save the desired output at the end of a step. It is used to reset the first 2 bezier coefficient in next step to smooth the torque.
                    %                     obj.dhd_last = obj.dh0;
                    % if command to stand, decide if should stand ( stand if the last step is left stance and the command is not made during this step or the previous step)
                    if obj.task_next == 2 && obj.stanceLeg == 1 && obj.to_stand_step_count >= 1.99
                        obj.step_end = 1;
                    end
                    % count the steps after command to stand
                    if obj.task_next == 2 && obj.task ~= 2
                        obj.to_stand_step_count = obj.to_stand_step_count + 1;
                    end
                    if obj.step_end == 1
                        obj.to_stand_step_count = 0;
                    end
                end
                
                % Get bezier coefficient for gait from Gaitlibrary
%                 obj.gaitparams = ControlPolicy( obj, GaitLibrary, obj.dqx_b_fil );
                obj.gaitparams = ControlPolicy( obj, GaitLibrary, 0 );
                obj.gaitparams.ct = 2.5*(1+RadioButton.RSA);
                if RadioButton.RSA < -0.98
                    obj.gaitparams.ct = 0;
                end
                s_unsat = obj.s_unsat_prev + (t - obj.t_prev)*obj.gaitparams.ct;
                s = min(s_unsat,1.05); % here s indicates the phase, 0 is the beginning of a step and 1 is the end of a step.
                
                % Generate some bezier curve that can be used later.
                sf_b = [0,0,ones(1,20)];
                s_fast = YToolkits.bezier(sf_b,s);
                ds_fast = YToolkits.dbezier(sf_b,s)*obj.gaitparams.ct;
                sl_b = [0,0,ones(1,4)];
                s_slow = YToolkits.bezier(sl_b,s);
                ds_slow = YToolkits.dbezier(sl_b,s)*obj.gaitparams.ct;
                sendl_b = [ones(1,4),0,0];
                send_slow = YToolkits.bezier(sendl_b,s);
                dsend_slow = YToolkits.dbezier(sendl_b,s)*obj.gaitparams.ct;
                sendf_b = [ones(1,20),0,0];
                send_fast = YToolkits.bezier(sendf_b,s);
                dsend_fast = YToolkits.dbezier(sendf_b,s)*obj.gaitparams.ct;
                
                
                
                % order the index of stance leg and swing leg
                if obj.stanceLeg == 1 % right stanceleg
                    st_abduction = 6;
                    st_rotation = 7;
                    st_thigh = 8;
                    st_knee = 9;
                    st_toe = 10;
                    sw_abduction = 1;
                    sw_rotation = 2;
                    sw_thigh = 3;
                    sw_knee = 4;
                    sw_toe =5;
                    GRF_st_index = 2;
                    GRF_sw_index = 1;
                    st_index = 2;
                    sw_index = 1;
                    abduction_direction = -1; % when the hip abduct outside, the sign is negative
                    st_LA = st_thigh;
                    st_LL = st_knee;
                    sw_LA = sw_thigh;
                    sw_LL = sw_knee;
                else
                    st_abduction = 1;
                    st_rotation = 2;
                    st_thigh = 3;
                    st_knee = 4;
                    st_toe = 5;
                    sw_abduction = 6;
                    sw_rotation = 7;
                    sw_thigh = 8;
                    sw_knee = 9;
                    sw_toe = 10;
                    GRF_st_index = 1;
                    GRF_sw_index = 2;
                    st_index = 1;
                    sw_index = 2;
                    abduction_direction = 1;
                    st_LA = st_thigh;
                    st_LL = st_knee;
                    sw_LA = sw_thigh;
                    sw_LL = sw_knee;
                end
                % estimating velocity
                [dqx,dqy,dqz] = get_velocity_v3(obj,qall,dqall); % v1 toe joint; v2 toe bottom; v3 toe back; v4 toe front
                % if the stance leg is off ground, set the velocity be 0 and let the velocity filter do the work
                dqx = s_LR(st_index)*dqx;
                dqy = s_LR(st_index)*dqy;
                dqz = s_LR(st_index)*dqz;
                
                % rotate the velocity to the torso frame ( YAW only!!!)
                Rz = YToolkits.Angles.Rz(qyaw);
                dq_b = Rz'*[dqx;dqy;dqz];
                dqx_b = dq_b(1);
                dqy_b = dq_b(2);
                dqz_b = dq_b(3);
                
                
                dqall_g = [ dqx;  dqy;              dqz;              dqyaw;         dqpitch;            dqroll;
                    dq_abduction_L;	dq_rotation_L;	dq_thigh_L;  dq_thigh_knee_L; dq_knee_shin_L;  dq_shin_tarsus_L; dq_toe_L;
                    dq_abduction_R;	dq_rotation_R;	dq_thigh_R;  dq_thigh_knee_R; dq_knee_shin_R;  dq_shin_tarsus_R; dq_toe_R];
                
                % These foot velocitiy and position has the assumption that torso xyz is
                % fixed at [0;0;0](but torso velocity is not 0). foot position is the toe joint position. IN THE BODY FRAME!
                [l_foot_v, r_foot_v] = get_feet_velocity(qall,dqall_g);
                [l_foot_p, r_foot_p] = get_feet_position(qall,dqall_g);
                r_foot_v = Rz'*r_foot_v;
                l_foot_v = Rz'*l_foot_v;
                r_foot_p = Rz'*r_foot_p;
                l_foot_p = Rz'*l_foot_p;
                foot_px = [l_foot_p(1); r_foot_p(1)];
                foot_py = [l_foot_p(2);r_foot_p(2)];
                foot_pz = [l_foot_p(3); r_foot_p(3)];
                
                % position and velocity of COM has the assumption that
                % torso position is at origion but velocity is not 0.
                com_pos = ComPosition(qall);
                com_pos = com_pos';
                com_vel = ComVelocity(qall,dqall_g);
                com_pos = Rz'*com_pos;
                com_vel = Rz'*com_vel;
                
                
                
                % filter the velocity
                obj.dqx_fil = YToolkits.first_order_filter(obj.dqx_fil,dqx,obj.fil_para_x);
                obj.dqy_fil = YToolkits.first_order_filter(obj.dqy_fil,dqy,obj.fil_para_y);
                obj.dqz_fil = YToolkits.first_order_filter(obj.dqz_fil,dqz,obj.fil_para_x);
                obj.dqx_b_fil = YToolkits.first_order_filter(obj.dqx_b_fil,dqx_b,obj.fil_para_x);
                obj.dqy_b_fil = YToolkits.first_order_filter(obj.dqy_b_fil,dqy_b,obj.fil_para_y);
                obj.dqz_b_fil = YToolkits.first_order_filter(obj.dqz_b_fil,dqz_b,obj.fil_para_x);
                obj.com_vel_x_fil = YToolkits.first_order_filter(obj.com_vel_x_fil,com_vel(1),obj.fil_para_2);
                obj.com_vel_y_fil = YToolkits.first_order_filter(obj.com_vel_y_fil,com_vel(2),obj.fil_para_2);
                obj.com_vel_z_fil = YToolkits.first_order_filter(obj.com_vel_z_fil,com_vel(3),obj.fil_para_2);
                obj.com_pos_x_fil = YToolkits.first_order_filter(obj.com_pos_x_fil,com_pos(1),obj.fil_para_3);
                obj.com_pos_y_fil = YToolkits.first_order_filter(obj.com_pos_y_fil,com_pos(2),obj.fil_para_3);
                obj.com_pos_z_fil = YToolkits.first_order_filter(obj.com_pos_z_fil,com_pos(3),obj.fil_para_3);
                %% walking
                if obj.task == 1 % walking
                    % Compute desired outputs ( here the outputs dose not
                    % include torso orientation. the outputs will be
                    % modified later
                    obj.hd = YToolkits.bezier(obj.gaitparams.HAlpha,s);
                    obj.dhd = YToolkits.dbezier(obj.gaitparams.HAlpha,s)*obj.gaitparams.ct;
                    obj.ddhr = YToolkits.d2bezier(obj.gaitparams.HAlpha,s)*obj.gaitparams.ct^2;
                    % swing leg foot placement
                    if obj.foot_placement ==1
                        % foot placement in saggital plane + add pitch angle in outputs
                        obj.hd(sw_LA) = obj.hd(sw_LA)   + (obj.Kfs_p*(obj.dqx_b_fil-obj.tg_velocity_x_fil) + obj.sagittal_offset + obj.Kfs_d*(obj.dqx_b_fil - obj.v_final(1)))*s_slow  + qpitch*s_slow;
                        obj.dhd(sw_LA) = obj.dhd(sw_LA) + (obj.Kfs_p*(obj.dqx_b_fil-obj.tg_velocity_x_fil) + obj.sagittal_offset + obj.Kfs_d*(obj.dqx_b_fil - obj.v_final(1)))*ds_slow + qpitch*ds_slow + dqpitch*s_slow;
                        
                        % foot placement in frontal plane + add roll angle in outputs
                        dqy_b_avg_1 = (obj.lateral_velocity_weight*obj.dqy_b_fil+(1-obj.lateral_velocity_weight)*obj.dqy_b_start);
                        lateral_ftpl = (obj.Kfl_p*dqy_b_avg_1 + obj.Kfl_d*(dqy_b_avg_1 - obj.v_final_avgy) + abduction_direction*obj.init_lateral_velocity*median([0,1,obj.dqx_b_fil]))*min(1.5*s,1);
                        if sign(lateral_ftpl) == abduction_direction
                            p = 1;
                        else
                            p = obj.abduction_inward_gain;
                        end
                        obj.hd(sw_abduction) = obj.hd(sw_abduction) +   p * lateral_ftpl * s_slow  + (obj.lateral_offset + obj.lateral_move)*s_slow  - qroll*s_slow ;
                        obj.dhd(sw_abduction) = obj.dhd(sw_abduction) + p * lateral_ftpl * ds_slow + (obj.lateral_offset + obj.lateral_move)*ds_slow - qroll*ds_slow - dqroll*s_slow;
                        
                        % use hip yaw motor on swing leg to maintain the direction ( or not).
                        if obj.to_turn ~=1 && obj.keep_direction
                            direction_keep_term = median([-0.2,0.2,YToolkits.wrap2Pi(obj.tg_yaw - qyaw)]);
                            obj.hd(sw_rotation) = obj.hd(sw_rotation) + (direction_keep_term+ obj.turning_offset)*s_slow;
                            obj.dhd(sw_rotation) = obj.dhd(sw_rotation) + (direction_keep_term + obj.turning_offset)*ds_slow;
                        else
                            obj.hd(sw_rotation) = obj.hd(sw_rotation) + (obj.rotation_move + obj.turning_offset)*s_slow;
                            obj.dhd(sw_rotation) = obj.dhd(sw_rotation) + (obj.rotation_move + obj.turning_offset)*ds_slow;
                        end
                        
                    end
                    % prevent sw abduction from hitting AR's safety bound
                    obj.hd(sw_abduction) = median([obj.ActuatorLimits(sw_abduction,1) + 0.1,obj.hd(sw_abduction),obj.ActuatorLimits(sw_abduction,2) - 0.1]);
                    
                    % flat the toe ( tilt a little bit)
                    %                     obj.hd(sw_toe) = - obj.h0_joint(sw_thigh) - deg2rad(13) -deg2rad(50); % 13 is the angle between tarsus and thihg, 50 is the angle of transforming frame on foot.
                    %                     obj.hd(sw_toe) = obj.hd(sw_toe) + obj.toe_tilt_angle*s_fast;
                    %                     obj.dhd(sw_toe) = 0;
                    %                     obj.hd(st_toe) = obj.h0(st_toe);
                    %                     obj.dhd(st_toe) = obj.dh0(st_toe);
                    
                    
                    % Preparation for standing
                    if obj.stanceLeg == -1 && obj.to_stand_step_count >=1.99
                        obj.hd(sw_abduction) = obj.hd(sw_abduction) - abduction_direction * obj.final_sw_abduction * s_slow;
                        obj.dhd(sw_abduction) = obj.dhd(sw_abduction) - abduction_direction * obj.final_sw_abduction * ds_slow;
                        obj.hd(st_abduction) = obj.hd(st_abduction) + abduction_direction * obj.final_st_abduction * s_slow;
                        obj.dhd(st_abduction) = obj.dhd(st_abduction) + abduction_direction * obj.final_st_abduction * ds_slow;
                    end
                    if obj.stanceLeg == 1 && obj.to_stand_step_count >=0.99
                        obj.hd(sw_abduction) = obj.hd(sw_abduction) - abduction_direction * obj.pre_final_sw_abduction * s_slow;
                        obj.dhd(sw_abduction) = obj.dhd(sw_abduction) - abduction_direction * obj.pre_final_sw_abduction * ds_slow;
                    end
                    
                    % compute torque
                    obj.y = obj.h0 - obj.hd;
                    obj.dy = obj.dh0 -obj.dhd;
                    
                    [ obj.hd_joint, obj.dhd_joint] = get_IK(obj,obj.hd,obj.dhd);
                    [ obj.h0_joint, obj.dh0_joint] = get_IK(obj,obj.h0,obj.dh0);
                    obj.ddhr_joint = obj.ddhr;
                    [JILL_L , JILA_L] = J_Inverse_Kinematics_p(obj.hd(3),obj.hd(4));
                    [JILL_R , JILA_R] = J_Inverse_Kinematics_p(obj.hd(8),obj.hd(9));
                    [dJILL_L , dJILA_L] = dJ_Inverse_Kinematics_p(obj.hd(3),obj.hd(4),obj.dhd(3),obj.dhd(4));
                    [dJILL_R , dJILA_R] = dJ_Inverse_Kinematics_p(obj.hd(8),obj.hd(9),obj.dhd(8),obj.dhd(9));
                    obj.ddhr_joint([3,4]) = [JILL_L ; JILA_L]*obj.ddhr([3,4])+ [dJILL_L ; dJILA_L]*obj.dhd([3,4]);
                    obj.ddhr_joint([8,9]) = [JILL_R ; JILA_R]*obj.ddhr([8,9])+ [dJILL_R ; dJILA_R]*obj.dhd([8,9]);
                    % make the stance leg passive
                    if obj.stance_passive == 1
                        obj.hd_joint(st_thigh) = obj.h0_joint(st_LA);
                        obj.dhd_joint(st_thigh) = 0;
                        obj.hd_joint(st_abduction) = obj.h0_joint(st_abduction);
                        obj.dhd_joint(st_abduction) = 0;
                    end
                    
                    % Save it for resetting bezier ( for the passive stance Leg)
                    [ obj.hd, obj.dhd] = get_FK(obj, obj.hd_joint,obj.dhd_joint);
                    
%                                         fix_h0_joint = [    0.0339
%                                             -0.0493
%                                             0.4965
%                                             -1.1119
%                                             -1.4896
%                                             0.0704
%                                             0.0408
%                                             0.7292
%                                             -1.5798
%                                             -1.5047];

%                     frequency = 2*(1+RadioButton.RSA);
%                     amp = 0.1;
%                     if RadioButton.RSA < -0.98
%                         frequency = 0;
%                     end
%                     coeff = 2*pi*frequency;
%                     obj.tau = obj.tau + coeff*(t-obj.t_prev);
%                     
%                     if leg_up ==-1
%                         fix_h0_joint = [    0.0339
%                             -0.0493
%                             0.4965
%                             -1.1119
%                             -1.4896
%                             0.0704
%                             0.0408
%                             1
%                             -2.3
%                             -1.5047];
%                         obj.stanceLeg = -1;
%                         obj.hd_joint = fix_h0_joint;
%                         obj.dhd_joint = zeros(10,1);
%                         obj.ddhr_joint = zeros(10,1);
%                         obj.hd_joint(4) = obj.hd_joint(4) + amp*sin(obj.tau);
%                         obj.dhd_joint(4) = amp*coeff*cos(obj.tau);
%                         obj.ddhr_joint(4) = -amp*coeff^2*sin(obj.tau);
%                     else
%                         fix_h0_joint = [0.0704
%                             0.0408
%                             1
%                             -2.3
%                             -1.5047
%                             0.0339
%                             -0.0493
%                             0.4965
%                             -1.1119
%                             -1.4896];
%                         obj.stanceLeg = 1;
%                         obj.hd_joint = fix_h0_joint;
%                         obj.dhd_joint = zeros(10,1);
%                         obj.ddhr_joint = zeros(10,1);
%                         obj.hd_joint(9) = obj.hd_joint(9) + 0.2*sin(obj.tau);
%                         obj.dhd_joint(9) = 0.2*coeff*cos(obj.tau);
%                         obj.ddhr_joint(9) = -0.2*coeff^2*sin(obj.tau);
%                     end
                    
                    %                     obj.hd_joint = fix_h0_joint+0.2*sin(2*pi*t)*ones(10,1);
                    %                     obj.dhd_joint = zeros(10,1)+2*pi*0.2*cos(2*pi*t)*ones(10,1);
                    %                     obj.ddhr_joint = zeros(10,1)+(2*pi)^2*0.2*(-sin(2*pi*t))*ones(10,1);
%                     obj.hd_joint = fix_h0_joint;
%                     obj.dhd_joint = zeros(10,1);
% %                     obj.ddhr_joint = zeros(10,1);
%                     obj.hd_joint = fix_h0_joint;
%                     obj.dhd_joint = zeros(10,1);
%                     obj.ddhr_joint = zeros(10,1);
                    
                    obj.y_joint= obj.h0_joint - obj.hd_joint;
                    obj.dy_joint = obj.dh0_joint - obj.dhd_joint;
                    u = - obj.Kp.*obj.y_joint - obj.Kd.*obj.dy_joint; %not final torque, some compensation for gravity will be added, the torque on stance hip roll and stance hip pitch will be replaced.
                    q_r = [0;0;0;0;0;0;obj.hd_joint(1:4);0;0;obj.hd_joint(5);obj.hd_joint(6:9);0;0;obj.hd_joint(10)];
                    dq_r =[0;0;0;0;0;0;obj.dhd_joint(1:4);0;0;obj.dhd_joint(5);obj.dhd_joint(6:9);0;0;obj.dhd_joint(10)];
                    ddq_r =[0;0;0;0;0;0;obj.ddhr_joint(1:4);0;0;obj.ddhr_joint(5);obj.ddhr_joint(6:9);0;0;obj.ddhr_joint(10)];
%                     q_0 = [0;0;0;qyaw;qpitch;qroll;obj.h0_joint(1:4);0;0;obj.h0_joint(5);obj.h0_joint(6:9);0;0;obj.h0_joint(10)];
%                     dq_0 = [0;0;0;dqyaw;dqpitch;dqroll;obj.dh0_joint(1:4);0;0;obj.dh0_joint(5);obj.dh0_joint(6:9);0;0;obj.dh0_joint(10)];
                    q_0 = qall;
                    dq_0 = dqall;
                    q_y = q_0 - q_r;
                    dq_y = dq_0 - dq_r;
                    
                    
                    % Torso Control
                    u_torso_pitch = - obj.Kp_pitch * qpitch - obj.Kd_pitch * dqpitch;
                    u_torso_roll = obj.Kp_roll * qroll + obj.Kd_roll * dqroll;
                    if obj.pitch_torso_control == 1
                        u(3) = (1 - s_L)*u(3) + s_L*u_torso_pitch;
                        u(8) = (1 - s_R)*u(8) + s_R*u_torso_pitch;
                    end
                    if obj.roll_torso_control == 1
                        u(1) = (1 - s_L)*u(1) + s_L*u_torso_roll;
                        u(6) = (1 - s_R)*u(6) + s_R*u_torso_roll;
                    end
                    
                    % abduction compensation
                    if obj.abduction_com == 1
                        u(st_abduction) = u(st_abduction) + abduction_direction*obj.u_abduction_cp*s_fast;
                        u(sw_abduction) = u(sw_abduction) - abduction_direction*obj.u_abduction_cp*(1-s_fast);
                    end
                    if obj.abduction_swing_com == 1
                        u(st_abduction) = u(st_abduction) + abduction_direction*obj.u_abduction_swing_cp*(1-s_fast);
                        u(sw_abduction) = u(sw_abduction) - abduction_direction*obj.u_abduction_swing_cp*s_fast;
                    end
                    u(sw_abduction) = u(sw_abduction) - abduction_direction*obj.uHip_gravity_2*s_fast;
                    % knee compensation
                    if obj.knee_com == 1
                        u(st_knee) = u(st_knee) + (obj.u_knee_cp)*s_fast*cos(obj.h0(st_thigh));
                        u(sw_knee) = u(sw_knee) + (obj.u_knee_cp)*(1-s_fast);
                    end
                    % thigh_compensation
                    if obj.thigh_compensation == 1
                        u(st_thigh) = u(st_thigh) + obj.u_thigh_cp*s_fast;
                        u(sw_thigh) = u(sw_thigh) + obj.u_thigh_cp*(1-s_fast);
                    end
                    u([5,10]) = 0;
                    
                    
                    %% Try Feedforward
                    if 1
                        [qc_index,motor_index,Kp_PBC,Kd_PBC,M_diag] = ControlTarget(obj);
                        M = EnertiaMatrix(qall);
                        C = CoriolisTerm(qall,dqall);
                        G = GravityVector(qall);
                        B=zeros(20,10); B(7:10,1:4) = eye(4); B(13:17,5:9) = eye(5); B(20,10) = 1;
                        
                        joint_damping = [ zeros(6,1); 1; 1; 1; 1; 0.1; 0.1; 1.1;1; 1; 1; 1; 0.1; 0.1; 1.1];
                        JD = zeros(20,1); JD(13) = 5; JD(20) = 5;
                        % Jacobian for fixing torso
%                         fixed_joint = [1,2,4,5,6]; %cartesian position and Euler Angle
                        fixed_joint = [1,2]; %cartesian position x y
%                         fixed_joint = [2]; %cartesian position y

                        %                     fixed_joint = [];
                        fix_z = 0;
                        Jt = zeros(length(fixed_joint),20);% Torso constraint Jacobian
                        for i = 1: length(fixed_joint)
                            Jt(i,fixed_joint(i)) = 1;
                        end
                        [Jtd1,Jtd2] = size(Jt);
                        % body Jacobian on foot
                        Jb_L = Jb_LeftToeBottomBack(qall);
                        Jb_R = Jb_RightToeBottomBack(qall);
                        dJb_L = dJb_LeftToeBottomBack(qall,dqall);
                        dJb_R = dJb_RightToeBottomBack(qall,dqall);
                        % cartesian Jacobian on foot
                        Jc_L = J_LeftToeBottomBack(qall);
                        Jc_R = J_RightToeBottomBack(qall);
                        dJc_L = dJ_LeftToeBottomBack(qall,dqall);
                        dJc_R = dJ_RightToeBottomBack(qall,dqall);
                        % Homogeneous Transformation Matrix
                        T_L = T_LeftToeBottomBack(qall);
                        T_R = T_RightToeBottomBack(qall);
                        % Jacobian for ground constraint
                        Jg_L = zeros(5,20); Jg_L(1:3,:)= Jc_L; Jg_L([4,5],:)= Jb_L([5,6],:);
                        Jg_R = zeros(5,20); Jg_R(1:3,:)= Jc_R; Jg_R([4,5],:)= Jb_R([5,6],:);
                        dJg_L = zeros(5,20); dJg_L(1:3,:)= dJc_L; dJg_L([4,5],:)= dJb_L([5,6],:);
                        dJg_R = zeros(5,20); dJg_R(1:3,:)= dJc_R; dJg_R([4,5],:)= dJb_R([5,6],:);
                        if obj.stanceLeg == -1
                            Jg = Jg_L;
                            dJg = dJg_L;
                        else
                            Jg = Jg_R;
                            dJg = dJg_R;
                        end
                        if fix_z == 1
                            Jg =[];
                            dJg = [];
                        end
                        [Jgd1,Jgd2] = size(Jg);
% %                         Jacobian for spring (simplified)
%                         JsL = zeros(2,20); JsL(1,11) = 1; JsL(2,[10 12]) = 1;
%                         JsR = zeros(2,20); JsR(1,18) = 1; JsR(2,[17 19]) = 1;
%                         Js = [JsL; JsR];
%                         [Jsd1,Jsd2] = size(Js);
%                         JsL = zeros(2,20); JsL(1,11) = 1; JsL(2,[10 11 12]) = 1;
%                         JsR = zeros(2,20); JsR(1,18) = 1; JsR(2,[17 18 19]) = 1;
%                         JsL = zeros(2,20); JsL(1,11) = 1; JsL(2,[10 11 12]) = [1.2174   1.7088   1.2122];
%                         JsR = zeros(2,20); JsR(1,18) = 1; JsR(2,[17 18 19]) = [1.2174   1.7088   1.2122];

%                       % Jacobian for spring 
                        JsL = zeros(2,20); JsL(1,11) = 1; JsL(2,[10 11 12]) = J_HeelSpringDeflectionEst(qall(10),qall(11),qall(12));
                        JsR = zeros(2,20); JsR(1,18) = 1; JsR(2,[17 18 19]) = J_HeelSpringDeflectionEst(qall(17),qall(18),qall(19));
                        Js = [JsL; JsR];

                        [Jsd1,Jsd2] = size(Js);
                        [JsLd1,JsLd2] = size(JsL);
                        [JsRd1,JsRd2] = size(JsR);
                        if obj.stanceLeg == -1
                            JsST = JsL;
                            FsST = [Fs1L; Fs2L];
                            JsSTd1 = JsLd1;
                            JsSTd2 = JsLd2;
                            JsSW = JsR;
                            FsSW = [Fs1R; Fs2R];
                            JsSWd1 = JsRd1;
                            JsSWd2 = JsRd2;
                        else
                            JsST = JsR;
                            FsST = [Fs1R; Fs2R];
                            JsSTd1 = JsRd1;
                            JsSTd2 = JsRd2;
                            JsSW = JsL;
                            FsSW = [Fs1L; Fs2L];
                            JsSWd1 = JsLd1;
                            JsSWd2 = JsLd2;
                        end
%                         dJsL = zeros(2,20); dJsL(2,[10 11 12]) = dJ_HeelSpringDeflectionEst(qall(10),qall(11),qall(12),dqall(10),dqall(11),dqall(12));
%                         dJsR = zeros(2,20); dJsR(2,[17 18 19]) = dJ_HeelSpringDeflectionEst(qall(17),qall(18),qall(19),dqall(17),dqall(18),dqall(19));
%                         dJs = [dJsL; dJsR];
%                         [dJsd1,dJsd2] = size(dJs);

                        deadzone_range = 20;
                        Fs1L_dz = YToolkits.deadzone_5rd_v1(Fs1L,deadzone_range);
                        Fs2L_dz = YToolkits.deadzone_5rd_v1(Fs2L,deadzone_range);
                        Fs1R_dz = YToolkits.deadzone_5rd_v1(Fs1R,deadzone_range);
                        Fs2R_dz = YToolkits.deadzone_5rd_v1(Fs2R,deadzone_range);
                        Fs = [Fs1L, Fs2L, Fs1R, Fs2R]';
                        Fs_dz = [Fs1L_dz,Fs2L_dz,Fs1R_dz,Fs2R_dz]';
                        obj.Fs_fil = YToolkits.first_order_filter(obj.Fs_fil,Fs,0.06);
                        Data.Fs = Fs;
                        Data.Fs_dz = Fs_dz;
                        Data.Fs_fil = obj.Fs_fil;
                        
%                         %Extended Matrix ( for walking)
%                                             Me = [M, -Jg', -Js'; Jg, zeros(Jgd1,Jgd1), zeros(Jgd1,Jsd1); Js, zeros(Jsd1,Jgd1), zeros(Jsd1,Jsd1)];
%                                             He = [-G;zeros(Jgd1,1);zeros(Jsd1,1)];
%                                             B = zeros(20,10); B(7:10,1:4) = eye(4); B(13:17,5:9) = eye(5); B(20,10) = 1;
%                                             B =B(:,[1:4,6:9]); % remove toe for now.
%                                             Be = [B;zeros(Jgd1+Jsd1,8)];
%                         
%                         %Extended Matrix ( fixing torso)
%                                             Me = [M, -Jt', -Js';
%                                                 Jt, zeros(Jtd1,Jtd1), zeros(Jtd1,Jsd1);
%                                                 Js, zeros(Jsd1,Jtd1), zeros(Jsd1,Jsd1)];
%                         %                     He = [-G-C+joint_damping.*dqall;zeros(Jtd1,1);zeros(Jsd1,1)];
%                                             He = [-G-C;zeros(Jtd1,1);zeros(Jsd1,1)];
%                                             B = B(:,h_index); % choose joint to control
%                                             Be = [B;zeros(Jtd1+Jsd1,length(h_index))];
%                                                 % Extended Matrix ( fixing torso(z free) and on ground)
%                         
%                         
% 
%                         % Extended Matrix ( fixing torso(z free) and on
%                         % ground) Spring force calculate directly.
%                         Me = [M, -Jt', -Jg';
%                             Jt, zeros(Jtd1,Jtd1), zeros(Jtd1,Jgd1);
%                             Jg, zeros(Jgd1,Jtd1), zeros(Jgd1,Jgd1)];
% 
% %                         He = [-G-C;zeros(Jtd1+Jsd1,1);dJg*dqall];
% %                         He =
% %                         [-G-C;zeros(Jtd1+Jsd1,1);dJg*dqall]+[JD.*dqall; zeros(Jsd1+J\td1+Jgd1,1)];
% 
% %                         He = [-G-C;zeros(Jtd1,1);dJg*dqall]-[Js'*obj.Fs_fil;zeros(Jtd1+Jgd1,1)];
%                         He = [-G-C;zeros(Jtd1,1);dJg*dqall]-[Js'*Fs_dz;zeros(Jtd1+Jgd1,1)];
% %                         He = [-G;zeros(Jtd1+Jsd1+Jgd1,1)];
%                         B_cut = B(:,motor_index); % remove toe for now.
%                         Be = [B_cut;zeros(Jtd1+Jgd1,length(motor_index))];
% 
% 
%                           % Extended Matrix ( fixing torso(z free) and on
%                           % ground) Spring considered on the stance Leg.
%                           Me = [M, -Jt', -JsSW', -Jg';
%                               Jt, zeros(Jtd1,Jtd1), zeros(Jtd1,JsSWd1), zeros(Jtd1,Jgd1);
%                               JsSW, zeros(JsSWd1,Jtd1), zeros(JsSWd1,JsSWd1), zeros(JsSWd1,Jgd1);
%                               Jg, zeros(Jgd1,Jtd1), zeros(Jgd1,JsSWd1), zeros(Jgd1,Jgd1)];
%                           
%                           He = [-G-C;zeros(Jtd1+JsSWd1,1);dJg*dqall]-[JsST'*FsST;zeros(Jtd1+JsSWd1+Jgd1,1)];
%                           B_cut = B(:,motor_index); % remove toe for now.
%                           Be = [B_cut;zeros(Jtd1+JsSWd1+Jgd1,length(motor_index))];
%                           
%                         % Extended Matrix ( fixing torso(z free) and on ground)
%                         Me = [M, -Jt', -Js', -Jg';
%                             Jt, zeros(Jtd1,Jtd1), zeros(Jtd1,Jsd1), zeros(Jtd1,Jgd1);
%                             Js, zeros(Jsd1,Jtd1), zeros(Jsd1,Jsd1), zeros(Jsd1,Jgd1);
%                             Jg, zeros(Jgd1,Jtd1), zeros(Jgd1,Jsd1), zeros(Jgd1,Jgd1)];
% %                         He = [-G-C;zeros(Jtd1+Jsd1,1);dJg*dqall]+[JD.*dqall; zeros(Jsd1+Jtd1+Jgd1,1)];
% %                         He = [-G-C;zeros(Jtd1+Jsd1,1);dJg*dqall];
%                         He = [-G;zeros(Jtd1+Jsd1+Jgd1,1)];
%                         B_cut = B(:,motor_index); % remove toe for now.
%                         Be = [B_cut;zeros(Jtd1+Jsd1+Jgd1,length(motor_index))];
                        
                        % Extended Matrix ( walking with no torso constraint)
                        Me = [M, -Js', -Jg';
                            Js, zeros(Jsd1,Jsd1), zeros(Jsd1,Jgd1);
                            Jg, zeros(Jgd1,Jsd1), zeros(Jgd1,Jgd1)];
                        He = [-G;zeros(Jsd1+Jgd1,1)];
                        B_cut = B(:,motor_index); % remove toe for now.
                        Be = [B_cut;zeros(Jsd1+Jgd1,length(motor_index))];
                        
                        %
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
                        
                        
%                        [M_bar,H_bar,B_bar,B_bar_inv,M_bar_inv] = load_stance_matrices(obj.stanceLeg);
% 
%                         % fixed x y on torso
%                         if obj.stanceLeg == -1
%                             B_bar = [1.10930087849978 -0.00486916933185339 0 0 0 0 0 0;0.00779497489766905 -1.0043876099142 0 0 0 0 0 0;-0.121054642116375 -0.000648219247761867 1 0 0 0 0 0;-0.500548074679357 0.0103990392420769 0 1 0 0 0 0;1.69345280753272e-18 1.32428649448922e-17 0 0 1 0 0 0;-1.40161300015776e-20 6.96528769369441e-19 0 0 0 1 0 0;-7.404883192428e-19 3.7526940982836e-19 0 0 0 0 1 0;1.63156702722289e-18 1.08807228306222e-19 0 0 0 0 0 1];
%                             B_bar_inv = [0.901499391523729 -0.00437037767746538 0 0 0 0 0 0;0.00699646735764858 -0.99566547527375 0 0 0 0 0 0;0.109135221453842 -0.00117446403106338 1 0 0 0 0 0;0.451171028213207 0.00816638021727595 0 1 0 0 0 0;-1.61929994787359e-18 1.319286444769e-17 0 0 1 0 0 0;7.76229186938326e-21 6.93448392414381e-19 0 0 0 1 0 0;6.64924209051625e-19 3.70406581671611e-19 0 0 0 0 1 0;-1.47161794849274e-18 1.15466164799797e-19 0 0 0 0 0 1];
%                             H_bar = [-13.3438721375029;-44.4616874529087;2.00682122470147;87.7116945593859;-2.20948011992632;0.0754348814844515;1.38965969747508;-2.9659329820444];
%                             M_bar = [1.31094459337744 0.198623954822274 0.200345191961099 -0.43583106877451 0.0034348759722042 -0.015765252426393 -0.741377846888177 -0.418541228203204;0.198623954822274 2.07681755695989 0.0371141376781436 -1.31909826671747 0.908372869789201 -0.0441531075453117 -0.033931753995836 0.0824932305439292;0.200345191961099 0.0371141376781436 0.724238000033056 0.0490298178111769 0.0912991755643171 -0.122561426404837 -0.166761828560633 -0.0828663589967611;-0.43583106877451 -1.31909826671747 0.0490298178111768 2.7139658357689 -0.0655162406458429 0.00218108665101457 0.0411556340167014 -0.0880426849519435;0.0034348759722042 0.908372869789201 0.0912991755643171 -0.0655162406458429 0.893578711549947 -0.0429887815465557 -0.00335966016471819 -0.00286926561021621;-0.015765252426393 -0.0441531075453117 -0.122561426404837 0.00218108665101456 -0.0429887815465557 0.141258949704546 -0.00316005470532751 -0.00195269118273896;-0.741377846888177 -0.033931753995836 -0.166761828560633 0.0411556340167014 -0.00335966016471819 -0.00316005470532751 0.725638740506349 0.368789297968237;-0.418541228203204 0.0824932305439292 -0.0828663589967611 -0.0880426849519435 -0.00286926561021621 -0.00195269118273896 0.368789297968237 0.400980531594718];
%                             M_bar_inv = [2.09805998449086 0.0102769096703212 -0.0882280906712877 0.335429146027453 0.0340858217975075 0.21453820910193 1.81215388394635 0.57786231620611;0.0102769096703205 1.78613870925826 0.106874781140308 0.806932297554511 -1.7628358482729 0.103629136456536 0.284075104660915 -0.430848296309696;-0.0882280906712875 0.106874781140308 1.7538640593969 -0.00866264468417528 -0.215499964030318 1.48686191377191 0.367511758231446 -0.0858376858696501;0.335429146027453 0.806932297554511 -0.00866264468417533 0.794955360137976 -0.758813343398907 0.0465453748961481 0.284137687161854 0.0903369312988101;0.0340858217975082 -1.7628358482729 -0.215499964030318 -0.758813343398908 2.88531485592641 0.155848765722036 -0.274186125234523 0.460677358457879;0.21453820910193 0.103629136456536 1.48686191377191 0.0465453748961483 0.155848765722035 8.48578400837463 0.591228902254749 0.0187829356130933;1.81215388394635 0.284075104660916 0.367511758231446 0.284137687161854 -0.274186125234524 0.591228902254748 4.33760940670734 -2.01705231112463;0.57786231620611 -0.430848296309696 -0.0858376858696503 0.09033693129881 0.460677358457879 0.0187829356130933 -2.01705231112463 5.04629863598503];
%                         else
%                             B_bar = [1.10699176441808 0.00269324076769047 0 0 0 0 0 0;-0.00662604935427409 -1.00314057874835 0 0 0 0 0 0;1.05944461358529e-18 -6.10864189352982e-20 1 0 0 0 0 0;1.60747514632746e-18 -7.73203294034275e-20 0 1 0 0 0 0;-9.58190072553443e-18 5.43292058344601e-19 0 0 1 0 0 0;-5.32567109847887e-18 2.75734560775012e-19 0 0 0 1 0 0;0.115010815958283 -0.00848521632118234 0 0 0 0 1 0;-0.495162712761175 -0.0121573505315499 0 0 0 0 0 1];
%                             B_bar_inv = [0.903363604580975 0.0024253586580467 0 0 0 0 0 0;-0.00596699202047738 -0.9968852738406 0 0 0 0 0 0;-9.57428207156648e-19 -6.34656846345363e-20 1 0 0 0 0 0;-1.45259591224928e-18 -8.09782015145214e-20 0 1 0 0 0 0;8.65918219753276e-18 5.648393982435e-19 0 0 1 0 0 0;4.81266274625851e-18 2.8779238563412e-19 0 0 0 1 0 0;-0.103947216487954 -0.00873772967419202 0 0 0 0 1 0;0.447239430240417 -0.0109185365412831 0 0 0 0 0 1];
%                             H_bar = [-12.9477898779167;44.2192966676058;1.97440590597593;-0.072344251871842;1.29017478387599;-3.01358038652475;-1.30010514670659;88.1579251083893];
%                             M_bar = [1.31266191169599 -0.193503678316237 -0.00457423143242484 0.0180053909509154 -0.742966872726405 -0.419480991190925 -0.203332863833314 -0.425796649266304;-0.193503678316237 2.06797323688354 0.904035835937814 -0.044906722542713 0.0315193963306659 -0.0836741511691165 0.0593929712646271 1.31858409330891;-0.00457423143242485 0.904035835937814 0.893489191027202 -0.0437352460112934 0.00252671411802609 0.00239816328006977 0.0993750181450043 0.0588305596039772;0.0180053909509154 -0.044906722542713 -0.0437352460112934 0.141393962674985 0.00315954008833179 0.00195116232219685 -0.122968418438505 -0.0021017131343938;-0.742966872726405 0.0315193963306659 0.00252671411802609 0.00315954008833179 0.725666429111715 0.368814662292639 0.158035859506804 0.0384007547766645;-0.419480991190925 -0.0836741511691165 0.00239816328006977 0.00195116232219685 0.368814662292639 0.400990223897485 0.0763490514645803 -0.0898974109795593;-0.203332863833314 0.0593929712646272 0.0993750181450042 -0.122968418438505 0.158035859506804 0.0763490514645803 0.717714587095491 -0.0283245483607014;-0.425796649266304 1.31858409330891 0.0588305596039772 -0.0021017131343938 0.0384007547766645 -0.0898974109795593 -0.0283245483607015 2.7400676964725];
%                             M_bar_inv = [2.09770786488186 -0.0118885289367203 -0.0289966784027067 -0.225515756339682 1.81482533054084 0.575828968294454 0.112681844723729 0.326769831633759;-0.0118885289367203 1.78581162534846 -1.76010038423473 0.0932970424075335 -0.280195842434109 0.430002357974601 0.0927499831351016 -0.804366328378053;-0.0289966784027071 -1.76010038423473 2.88223527165486 0.159527180507168 0.275463503306553 -0.457499611230902 -0.216313132439764 0.759628107396182;-0.225515756339681 0.0932970424075337 0.159527180507168 8.48768518382861 -0.586250240058409 -0.0150649411857512 1.48909446173182 -0.0537410673994926;1.81482533054084 -0.280195842434108 0.275463503306553 -0.586250240058409 4.33249922051647 -2.01806431407972 -0.329499778577392 0.280156433771758;0.575828968294456 0.430002357974601 -0.457499611230902 -0.015064941185752 -2.01806431407972 5.04550771732103 0.0993909892552688 0.0872108199387386;0.11268184472373 0.0927499831351013 -0.216313132439764 1.48909446173182 -0.329499778577391 0.0993909892552691 1.7648113512031 0.00478524264418728;0.326769831633759 -0.804366328378053 0.759628107396183 -0.0537410673994926 0.280156433771758 0.0872108199387382 0.00478524264418712 0.785446725000334];
%                         end
%                         
%                         % Free torso
% if obj.stanceLeg == -1
%     B_bar = [1.01964935414685 0.00208539309420858 0 0 0 0 0 0;0.00556668789748462 -0.941680467124516 0 0 0 0 0 0;-0.144640354048101 0.0217135904378402 1 0 0 0 0 0;-0.480808324791387 -0.00344626214278634 0 1 0 0 0 0;-0.000403392596387846 0.0660657497559364 0 0 1 0 0 0;0.00738286795108845 0.00379030117715461 0 0 0 1 0 0;0.0455324481155473 0.000128964930876547 0 0 0 0 1 0;0.021612658457414 0.000368474246926809 0 0 0 0 0 1];
%     B_bar_inv = [0.980717445625165 0.00217184221174481 0 0 0 0 0 0;0.00579745266680971 -1.06191850096007 0 0 0 0 0 0;0.141725435043583 0.0233721994346556 1 0 0 0 0 0;0.471557091666374 -0.00261540971304287 0 1 0 0 0 0;1.26010996062555e-05 0.0701573180506958 0 0 1 0 0 0;-0.00726248149004686 0.0040089565199713 0 0 0 1 0 0;-0.0446552138770223 3.80609532514157e-05 0 0 0 0 1 0;-0.0211980474075297 0.000344350335993073 0 0 0 0 0 1];
%     H_bar = [-11.666712091048;-43.4619558923897;1.04124963684956;86.3170944122267;-1.28812996326128;0.0985529806701464;1.37886077168215;-2.95193523782609];
%     M_bar = [1.1267550765918 0.173800851672618 0.160893935946696 -0.386077015141261 -6.40456182624417e-05 -0.00168398284853058 -0.653466851213953 -0.379104418038424;0.173800851672619 1.96383173329422 0.0126357267992334 -1.27267424084056 0.794071960173095 -0.0429553518341478 -0.0363149292688582 0.0803681226055891;0.160893935946696 0.0126357267992334 0.706281165342391 0.0196672690907689 0.0635526032160075 -0.11668552841233 -0.142599699311615 -0.0709583257222947;-0.386077015141261 -1.27267424084056 0.0196672690907689 2.64071641740005 -0.0382727883838238 0.00322635538443974 0.0429940980565413 -0.0851614104219459;-6.40456182624382e-05 0.794071960173095 0.0635526032160074 -0.0382727883838238 0.774539769923196 -0.0423912671906084 -0.00349371638495996 -0.00347015596372651;-0.00168398284853058 -0.0429553518341478 -0.11668552841233 0.00322635538443974 -0.0423912671906084 0.137961075607194 -0.0105604713738433 -0.00551215135903615;-0.653466851213953 -0.0363149292688582 -0.142599699311615 0.0429940980565413 -0.00349371638495996 -0.0105604713738433 0.681672532379665 0.347944386345805;-0.379104418038424 0.0803681226055891 -0.0709583257222947 -0.0851614104219459 -0.00347015596372651 -0.00551215135903615 0.347944386345805 0.390733004986243];
%     M_bar_inv = [2.291280392067 0.0109150339054604 -0.0771931013927788 0.332369888119584 0.0302475816801795 0.133682197265423 1.82901370570453 0.652700641265253;0.0109150339054599 1.79459753968612 0.13935158447862 0.820328688419348 -1.80542868698255 0.108247473085506 0.297239069679155 -0.433628847546848;-0.0771931013927791 0.13935158447862 1.76021356354405 0.0294241752497925 -0.202874976268448 1.49275028163057 0.363849417105399 -0.0822330425671671;0.332369888119584 0.820328688419348 0.0294241752497925 0.809330466886383 -0.799393116065976 0.0439200227940767 0.267801161625343 0.0905334189809749;0.0302475816801799 -1.80542868698256 -0.202874976268448 -0.799393116065976 3.13347936807864 0.24556570949804 -0.278850931448303 0.469232604845218;0.133682197265422 0.108247473085506 1.49275028163057 0.0439200227940768 0.24556570949804 8.66504090643724 0.58172066989964 -0.00549750332079556;1.82901370570453 0.297239069679155 0.363849417105399 0.267801161625344 -0.278850931448304 0.58172066989964 4.33007916226317 -2.01228140487374;0.652700641265254 -0.433628847546848 -0.0822330425671671 0.0905334189809748 0.469232604845219 -0.00549750332079537 -2.01228140487374 5.08256653074835];
%     
% else
%     B_bar = [1.01684285727325 -0.000116997064699382 0 0 0 0 0 0;0.00366571822468774 -0.941153820328764 0 0 0 0 0 0;0.000339508261096293 0.0661749112579458 1 0 0 0 0 0;-0.00742224122684714 0.00300616717394179 0 1 0 0 0 0;0.0457760649174647 0.000157988900145078 0 0 1 0 0 0;0.0207238077933235 -0.000251207606941497 0 0 0 1 0 0;0.13465150578555 -0.0120281954891833 0 0 0 0 1 0;-0.480644153900657 0.00345290525016544 0 0 0 0 0 1];
%     B_bar_inv = [0.98343656642075 -0.000122253333200172 0 0 0 0 0 0;0.00383040611054799 -1.06252604680173 0 0 0 0 0 0;-0.000587361623011425 0.0703126083623773 1 0 0 0 0 0;0.00728778858616469 0.00319322352962371 0 1 0 0 0 0;-0.0450184612683334 0.000173463598026651 0 0 1 0 0 0;-0.020379588152277 -0.000264381070950739 0 0 0 1 0 0;-0.132375141639624 -0.0127638094078776 0 0 0 0 1 0;0.472669810352899 0.00361004141554171 0 0 0 0 0 1];
%     H_bar = [-11.7978974047927;43.3994551646291;1.25345938103768;0.00994713351511025;0.56917044187531;-3.33585874114171;-1.71214107049092;86.3714458469];
%     M_bar = [1.12945484152292 -0.17412938839941 -0.000712116452687012 0.00149228025781935 -0.65039768425053 -0.377188394215959 -0.170246470332507 -0.391695233628732;-0.17412938839941 1.96136492147797 0.790780168839482 -0.0487746744646303 0.0228430574719591 -0.0868351110896418 0.0580368633803524 1.27540818178178;-0.000712116452687013 0.790780168839482 0.771110996904421 -0.0496200465990237 0.00394584044306648 0.00383415896273639 0.09828361101872 0.0375349863852481;0.00149228025781935 -0.0487746744646304 -0.0496200465990237 0.139984559478113 0.0105717172180184 0.00533358221701494 -0.121045109053967 -8.75888050681447e-05;-0.65039768425053 0.0228430574719591 0.00394584044306648 0.0105717172180184 0.679980397757203 0.348116065024003 0.148953889091941 0.0194612170306968;-0.377188394215959 -0.0868351110896418 0.00383415896273639 0.00533358221701495 0.348116065024003 0.391614824911896 0.0732158143320474 -0.0966929759208869;-0.170246470332507 0.0580368633803524 0.0982836110187199 -0.121045109053967 0.148953889091941 0.0732158143320474 0.718877764779408 -0.0386555743990978;-0.391695233628732 1.27540818178178 0.0375349863852481 -8.75888050681392e-05 0.0194612170306968 -0.0966929759208869 -0.0386555743990978 2.64967704694811];
%     M_bar_inv = [2.29544565233834 -0.013743012685192 -0.0344132995027523 -0.114905112856282 1.83232282477518 0.650040883578334 0.103472812602627 0.358202018491932;-0.0137430126851916 1.78854166807967 -1.80135110567533 0.0560798179092165 -0.277537772210953 0.430291044872012 0.0777398227137378 -0.818542293234395;-0.0344132995027526 -1.80135110567533 3.15261129003618 0.309981344836785 0.268086977564314 -0.471052957877557 -0.206361622872792 0.795165020692272;-0.114905112856282 0.0560798179092165 0.309981344836785 8.60885124477869 -0.58064204202033 0.0131262114783403 1.49325925470263 -0.0215579263840458;1.83232282477518 -0.277537772210953 0.268086977564314 -0.58064204202033 4.33893346023838 -2.01005528072346 -0.356796674753591 0.290216964908484;0.650040883578335 0.430291044872012 -0.471052957877557 0.01312621147834 -2.01005528072346 5.07319323882143 0.090825203609137 0.0968701023811099;0.103472812602627 0.0777398227137376 -0.206361622872792 1.49325925470263 -0.356796674753591 0.0908252036091371 1.7542796723253 0.0123769499543175;0.358202018491931 -0.818542293234395 0.795165020692272 -0.0215579263840459 0.290216964908484 0.0968701023811098 0.0123769499543174 0.814676750330513];
% end

%                         Free torso, Matrix Library
                        [B_bar,B_bar_inv,H_bar,M_bar,M_bar_inv] = Select_DynamicMatrix(obj,DynamicMatrixLibrary,obj.h0_joint(4),obj.h0_joint(9));
                         
                        
                        
                        
                        Kd_iol = 50;
                        Kp_iol = 500;
                        
                        
                        qc_y = q_y(qc_index);
                        dqc_y = dq_y(qc_index);
                        ddqc_r = ddq_r(qc_index);
                        ddqc_yd = -Kd_iol* dqc_y  - Kp_iol* qc_y;
                        ddqc_d = ddqc_yd + ddqc_r;
                        %                     %Decouple Matrix
                        %                     D = Me^-1*Be;
                        %                     D = D(qc_index,:);
                        %                     F = Me^-1*He;
                        %                     F = F(qc_index,:);
                        %                     u8 = D^-1*(ddqc_d+F);
                        %                     u8 = B_bar^-1*(M_bar*ddqc_r+H_bar+M_bar*(-Kd_iol* dqc_y  - Kp_iol* qc_y));
                        %                     u8 = B_bar^-1*(M_bar*ddqc_r+H_bar+M_bar*(-M_diag*Kd_PBC.* dqc_y  - M_diag*Kp_PBC.* qc_y));
                        %                     u8 = B_bar^-1*(M_bar*ddqc_r+H_bar -Kd_iol*dqc_y - Kp_iol*qc_y);
                        
                        %                     u8 = B_bar^-1*(M_bar*ddqc_r+H_bar -Kd_PBC.*dqc_y - Kp_PBC.*qc_y);
                        u8 = B_bar_inv*(M_bar*ddqc_r+H_bar -Kd_PBC.*dqc_y - Kp_PBC.*qc_y);
                        u_ff = B_bar_inv*(M_bar*ddqc_r);
                        u_gv = B_bar_inv*H_bar;
                        u_fb = B_bar_inv*( -Kd_PBC.*dqc_y - Kp_PBC.*qc_y);
%                         qs_offset = [-0.1;-0.08];
%                         Kps = 100*eye(2);
%                         Kds = 20*eye(2);
%                         u_damping = pinv(dqc_y'*B_bar)*(-dqsL'*Kps*qsL-dqsL'*Kds*dqsL);
%                         u_damping = (dqc_y'*B_bar)'*((dqc_y'*B_bar)*(dqc_y'*B_bar)')^-1*(-dqsL'*Kps*(qsL-qs_offset)-dqsL'*Kds*dqsL);
%                         u8 = u8+u_damping;
                        u = zeros(10,1);
                        %                     u([1:4,6:9]) = u8;
                        u(motor_index) = u8;
%                         Fe = Me^-1*(-He+Be*u8);
%                         u(4) = u(4) - 15*dqall(11);
                    end
                    
                    % Construct for feedforward
                    Data.q_r = q_r;
                    Data.dq_r = dq_r;
                    Data.ddq_r = ddq_r;
                    Data.q_0 = q_0;
                    Data.dq_0 = dq_0;
                    Data.q_y = q_y;
                    Data.dq_y = dq_y;
                    %                                         Data.controller_GRF = Fe(end-4:end);
%                     Data.Fe = Fe;
                    Data.u_ff = u_ff;
                    Data.u_fb = u_fb;
                    Data.u_gv = u_gv;
                    % Construct Data
                    Data.hd = obj.hd;
                    Data.dhd = obj.dhd;
                    Data.hd_joint = obj.hd_joint;
                    Data.dhd_joint = obj.dhd_joint;
                    Data.y_joint = obj.y_joint ;
                    Data.dy_joint   = obj.dy_joint ;

                    
                end
                %% stand up
                if obj.task == 2
                    % If next task is to walk, shift the center of mass to
                    % left
                    if obj.task_next == 1
                        obj.t1 = (obj.t1 + (t - obj.t_prev));
                        obj.s1 = obj.t1/obj.shift_time;
                        lateral_shift = obj.shift_distance * obj.s1;
                    else
                        lateral_shift = 0;
                    end
                    
                    % Based on the commanded roll angle, and lateral_shift,
                    % decide the Leg Length Difference in right and left
                    % leg.
                    left_tune = - obj.s2*max([s_L; s_R]) * (obj.Kp_lateral_stand*(qroll - obj.roll_des_fil - (-lateral_shift))  + obj.Kd_lateral_stand*dqroll);
                    right_tune = obj.s2*max([s_L; s_R]) * (obj.Kp_lateral_stand*(qroll - obj.roll_des_fil - (-lateral_shift)) + obj.Kd_lateral_stand*dqroll);
                    % Inverse kinematics
                    [qthigh_d_L, qknee_d_L ] = Inverse_Kinematics_p(obj.pitch_des_fil, min(obj.LL_des_fil + left_tune, 0.9));
                    [qthigh_d_R, qknee_d_R ] = Inverse_Kinematics_p(obj.pitch_des_fil, min(obj.LL_des_fil + right_tune, 0.9));
                    [dqthigh_d_L, dqknee_d_L] = Inverse_Kinematics_v(obj.pitch_des_fil, min(obj.LL_des_fil + left_tune, 0.9), 0, 0);
                    [dqthigh_d_R, dqknee_d_R] = Inverse_Kinematics_v(obj.pitch_des_fil, min(obj.LL_des_fil + right_tune, 0.9), 0, 0);
                    y_thigh = [obj.h0_joint(3)-qthigh_d_L; obj.h0_joint(8)-qthigh_d_R];
                    dy_thigh = [obj.dh0_joint(3)-dqthigh_d_L; obj.dh0_joint(8)-dqthigh_d_R];
                    y_knee = [obj.h0_joint(4)-qknee_d_L; obj.h0_joint(9)-qknee_d_R];
                    dy_knee = [obj.dh0_joint(4)-dqknee_d_L; obj.dh0_joint(9)-dqknee_d_R];
                    
                    % calculate the torque ( except toe).
                    u([1,6]) = - (obj.Kp_abduction_stand*[obj.h0_joint(1)-obj.standing_abduction_offset; obj.h0_joint(6) - ( - obj.standing_abduction_offset)] + obj.Kd_abduction_stand*[obj.dh0_joint(1); obj.dh0_joint(6)]);
                    u([2,7]) = - (obj.Kp_rotation_stand*[obj.h0_joint(2);obj.h0_joint(7)] + obj.Kd_rotation_stand*[obj.dh0_joint(2);obj.dh0_joint(7)]);
                    u([3,8]) = - (obj.Kp_thigh_stand.*y_thigh + obj.Kd_thigh_stand.*dy_thigh);
                    u([4,9]) = - (obj.Kp_knee_stand*y_knee + obj.Kd_knee_stand*dy_knee);
                    % calculate the toe torque
                    P_feedback_toe = ( com_pos(1) - obj.stand_offset- (r_foot_p(1)+l_foot_p(1))/2);
                    obj.P_feedback_toe_fil = YToolkits.first_order_filter(obj.P_feedback_toe_fil,P_feedback_toe,obj.fil_para_3);
                    u_toe = - (obj.Kp_toe_stand*obj.P_feedback_toe_fil + obj.Kd_toe_stand*( obj.com_vel_x_fil - 0));
                    u([5,10]) =min(s_L,s_R)*u_toe;
                    u([5,10]) = YToolkits.clamp(u([5,10]),-15,15);
                    
                    % from walking to stand ( to make torque smooth in transition)
                    obj.t2 = (obj.t2 + (t - obj.t_prev));
                    obj.s2 = min(obj.t2/obj.standing_switch_time,1);
                    s2_b = [0,0,ones(1,20)];
                    switch_weight  = YToolkits.bezier(s2_b,obj.s2);
                    u = switch_weight*u + (1-switch_weight)*obj.u_last;
                    
                    Data.LL_des_fil = obj.LL_des_fil;
                    Data.y_knee = y_knee;
                    Data.dy_knee = dy_knee;
                    Data.qknee_d_L = qknee_d_L;
                    Data.qknee_d_R = qknee_d_R;
                    Data.left_tune = left_tune;
                    Data.right_tune = right_tune;
                end
                
                
                %% Return
                userInputs.telemetry(1) = qsL(1)*1000;
                userInputs.telemetry(2) = qsL(2)*1000;
                userInputs.telemetry(3) = qsR(1)*1000;
                userInputs.telemetry(4) = qsR(2)*1000;
                userInputs.telemetry(5) = dqyaw*180/pi;
                userInputs.telemetry(6) = u(5);
                userInputs.telemetry(7) = u(10);
                userInputs.telemetry(8) = GRF_v(1);
                userInputs.telemetry(9) = GRF_v(2);
                
                u = YToolkits.vector_saturate(u,obj.safe_TorqueLimits,-obj.safe_TorqueLimits);
                %                 u = obj.u_prev+median([-10*ones(10,1),10*ones(10,1),u-obj.u_prev]')';
                %                 alpha = YToolkits.cutoff_frequency_to_alpha( 100,1/2000);
                %                 u = alpha*u + (1-alpha)*obj.u_prev;
                %                 u = zeros(10,1);
                userInputs.torque = u;
                %% log object properties
                obj.t_prev = t;
                obj.s_prev = s;
                obj.s_unsat_prev = s_unsat;
                obj.task_prev = obj.task;
                obj.u_prev = u;
                obj.hd_prev = obj.hd;
                obj.dhd_prev = obj.dhd;
                obj.h0_prev = obj.h0;
                obj.dh0_prev = obj.dh0;
                obj.y_prev = obj.y;
                obj.dy_prev = obj.dy;
                obj.hd_joint_prev = obj.hd_joint;
                obj.dhd_joint_prev = obj.dhd_joint;
                obj.h0_joint_prev = obj.h0_joint;
                obj.dh0_joint_prev = obj.dh0_joint;
                obj.y_joint_prev = obj.y_joint;
                obj.dy_joint_prev = obj.dy_joint;
                obj.to_turn_prev = obj.to_turn;
                %% log Data
                Data.s = s;
                Data.stanceLeg = obj.stanceLeg;
                Data.task = obj.task;
                Data.GRF = [GRF_L(2); GRF_R(2)];
                Data.s_LR = s_LR;
                Data.tp_last = obj.tp_last;
                
                Data.linearAcceleration = cassieOutputs.pelvis.vectorNav.linearAcceleration;
                Data.orientation = cassieOutputs.pelvis.vectorNav.orientation;
                Data.angularVelocity = cassieOutputs.pelvis.vectorNav.angularVelocity;

                Data.h0 = obj.h0;
                Data.dh0 = obj.dh0;
                Data.h0_joint = obj.h0_joint;
                Data.dh0_joint = obj.dh0_joint;
                Data.hd = obj.hd;
                Data.dhd = obj.dhd;
                Data.ddhr = obj.ddhr;
                Data.hd_joint = obj.hd_joint;
                Data.dhd_joint = obj.dhd_joint;
                Data.ddhr_joint = Data.ddhr_joint;
                
                Data.u = u;
                Data.torso_angle = [qyaw; qpitch; qroll];
                Data.d_torso_angle = [dqyaw; dqpitch; dqroll];
                Data.torso_vx = dqx;
                Data.torso_vy = dqy;
                Data.torso_vz = dqz;
                Data.torso_vx_b = dqx_b;
                Data.torso_vy_b = dqy_b;
                Data.torso_vz_b = dqz_b;
                Data.torso_vx_fil = obj.dqx_fil;
                Data.torso_vy_fil = obj.dqy_fil;
                Data.torso_vz_fil = obj.dqz_fil;
                Data.torso_vx_b_fil = obj.dqx_b_fil;
                Data.torso_vy_b_fil = obj.dqy_b_fil;
                Data.torso_vz_b_fil = obj.dqz_b_fil;
                Data.com_vel_x_fil = obj.com_vel_x_fil;
                Data.com_vel_y_fil = obj.com_vel_y_fil;
                Data.com_vel_z_fil = obj.com_vel_z_fil;
                Data.com_pos_fil = [obj.com_vel_x_fil;obj.com_vel_y_fil;obj.com_vel_z_fil];
                Data.tg_velocity_x = obj.tg_velocity_x;
                
                Data.r_foot_v = r_foot_v;
                Data.l_foot_v = l_foot_v;
                Data.r_foot_p = r_foot_p;
                Data.l_foot_p = l_foot_p;
                
                Data.com_pos = com_pos;
                Data.com_vel = com_vel;
                
                Data.encoder_fil = encoder_fil;
                
                Data.q_abduction_R = q_abduction_R;
                Data.q_rotation_R = q_rotation_R;
                Data.q_thigh_R = q_thigh_R;
                Data.q_thigh_knee_R = q_thigh_knee_R;
                Data.q_knee_shin_R = q_knee_shin_R;
                Data.q_thigh_shin_R = q_thigh_shin_R;
                Data.q_shin_tarsus_R = q_shin_tarsus_R;
                Data.q_toe_R = q_toe_R;
                
                Data.q_abduction_L = q_abduction_L ;
                Data.q_rotation_L = q_rotation_L;
                Data.q_thigh_L = q_thigh_L;
                Data.q_thigh_knee_L = q_thigh_knee_L;
                Data.q_knee_shin_L = q_knee_shin_L;
                Data.q_thigh_shin_L = q_thigh_shin_L;
                Data.q_shin_tarsus_L = q_shin_tarsus_L;
                Data.q_toe_L = q_toe_L;
                
                Data.dq_abduction_R = dq_abduction_R;
                Data.dq_rotation_R = dq_rotation_R;
                Data.dq_thigh_R = dq_thigh_R;
                Data.dq_thigh_knee_R = dq_thigh_knee_R;
                Data.dq_knee_shin_R = dq_knee_shin_R;
                Data.dq_thigh_shin_R = dq_thigh_shin_R;
                Data.dq_shin_tarsus_R = dq_shin_tarsus_R;
                Data.dq_toe_R = dq_toe_R;
                
                Data.dq_abduction_L = dq_abduction_L ;
                Data.dq_rotation_L = dq_rotation_L;
                Data.dq_thigh_L = dq_thigh_L;
                Data.dq_thigh_knee_L = dq_thigh_knee_L;
                Data.dq_knee_shin_L = dq_knee_shin_L;
                Data.dq_thigh_shin_L = dq_thigh_shin_L;
                Data.dq_shin_tarsus_L = dq_shin_tarsus_L;
                Data.dq_toe_L = dq_toe_L;
                
                Data.qall = qall;
                Data.dqall = dqall;
                
                Data.qq = qq;
                Data.qaR = qaR;
                Data.qjR = qjR;
                Data.qsR = qsR;
                Data.qaL = qaL;
                Data.qjL = qjL;
                Data.qsL = qsL;
                
                Data.dqaR = dqaR;
                Data.dqjR = dqjR;
                Data.dqsR = dqsR;
                Data.dqaL = dqaL;
                Data.dqjL = dqjL;
                Data.dqsL = dqsL;
            end
            % Return the updated Cassie inputs data structure
            
        end % stepImpl
        %% util functions
        
        function gaitparams = ControlPolicy( obj,GaitLibrary, phi)
            % Saturate interpolation value
            phi = clamp(phi, GaitLibrary.Velocity(1,1), GaitLibrary.Velocity(1,end));
            % Interpolate gaits
            HAlpha_R = interp1(GaitLibrary.Velocity(1,:),GaitLibrary.RightStance.HAlpha, phi);
            HAlpha_L = interp1(GaitLibrary.Velocity(1,:),GaitLibrary.LeftStance.HAlpha, phi);
            ct_R = interp1(GaitLibrary.Velocity(1,:), GaitLibrary.ct, phi);
            ct_L = interp1(GaitLibrary.Velocity(1,:), GaitLibrary.ct, phi);
            if obj.stanceLeg == 1
                gaitparams.HAlpha = reshape(HAlpha_R,10,6);
                gaitparams.HAlpha(:,1) = obj.hd_last;
                %                                 gaitparams.HAlpha(:,2) = obj.hd_last + obj.dhd_last/ct_R/obj.bezier_degree;
                gaitparams.ct = ct_R;
            else
                gaitparams.HAlpha = reshape(HAlpha_L,10,6);
                gaitparams.HAlpha(:,1) = obj.hd_last;
                %                                 gaitparams.HAlpha(:,2) = obj.hd_last + obj.dhd_last/ct_L/obj.bezier_degree;
                gaitparams.ct = ct_L;
            end
            %             if obj.stanceLeg == 1
            %                 gaitparams.HAlpha = reshape(HAlpha_R,10,6);
            %                 gaitparams.HAlpha(:,1) = obj.h0_last;
            %                 gaitparams.HAlpha(:,2) = obj.h0_last + obj.dh0_last/ct_R/obj.bezier_degree;
            %                 gaitparams.ct = ct_R;
            %             else
            %                 gaitparams.HAlpha = reshape(HAlpha_L,10,6);
            %                 gaitparams.HAlpha(:,1) = obj.h0_last;
            %                 gaitparams.HAlpha(:,2) = obj.h0_last + obj.dh0_last/ct_L/obj.bezier_degree;
            %                 gaitparams.ct = ct_L;
            %             end
        end
        function [dqx,dqy,dqz] = get_velocity_v3(obj,q,dq)
            if obj.stanceLeg == 1
                range = 14:20;
                J = J_RightToeBottomBack(q);
            else
                J = J_LeftToeBottomBack(q);
                range = 7:13;
            end
            v_torso = (-J(:,1:3)^-1)*(J(:,4:6)*dq(4:6)+J(:,range)*dq(range));
            dqx = v_torso(1);
            dqy = v_torso(2);
            dqz = v_torso(3);
        end
        
        function [ GRF_L, GRF_R  ] = get_GRF(obj,qall,qsR,qsL,u)
            qall(4) = 0;
            JR = J_RightToeJoint(qall);
            JL = J_LeftToeJoint(qall);
            [Fs1R, Fs2R, Fs1L, Fs2L] = get_spring_force(obj,qsR,qsL);
            JL_H = J_HeelSpringDeflectionEst(qall(10),qall(11),qall(12));
            JR_H = J_HeelSpringDeflectionEst(qall(17),qall(18),qall(19));
            JL_s = JL([1,3],[11,12]);
            JR_s = JR([1,3],[18,19]);
            GRF_L = (-JL_s')^-1*[Fs1L+JL_H(2)*Fs2L; JL_H(3)*Fs2L];
            GRF_R = (-JR_s')^-1*[Fs1R+JR_H(2)*Fs2R; JR_H(3)*Fs2R];
        end
        
%         function [ GRF_L, GRF_R  ] = get_GRF(obj,qall,qsR,qsL,u)
%             qall(4) = 0;
%             JR = J_RightToeJoint(qall);
%             JL = J_LeftToeJoint(qall);
%             [Fs1R, Fs2R, Fs1L, Fs2L] = get_spring_force(obj,qsR,qsL);
%             JR_s = JR([1,3],[18,19]);
%             JL_s = JL([1,3],[11,12]);
%             GRF_R = (-JR_s')^-1*[Fs1R+Fs2R; Fs2R];
%             GRF_L = (-JL_s')^-1*[Fs1L+Fs2L; Fs2L];
%         end
        
        function [Fs1R, Fs2R, Fs1L, Fs2L] = get_spring_force(obj,qsR,qsL)
            Fs1R =- obj.Ks1 * qsR(1);
            Fs2R =- obj.Ks2 * qsR(2);
            Fs1L =- obj.Ks1 * qsL(1);
            Fs2L =- obj.Ks2 * qsL(2);
        end
        
        function [s_L, s_R] = get_s_LR(obj, GRF_v)
            s_L = (GRF_v(1)-obj.stance_thre_lb)/(obj.stance_thre_ub-obj.stance_thre_lb);
            s_R = (GRF_v(2)-obj.stance_thre_lb)/(obj.stance_thre_ub-obj.stance_thre_lb);
            s_L = median([0,1,s_L ]);
            s_R = median([0,1,s_R]);
        end
        
        
        
        function [ hd_output, dhd_output] = get_FK(obj, hd_joint,dhd_joint)
            [ hd_output, dhd_output] = get_FK_v1(obj, hd_joint,dhd_joint);
        end
        function [ hd_joint, dhd_joint] = get_IK(obj, hd_output,dhd_output)
            [ hd_joint, dhd_joint] = get_IK_v1(obj, hd_output,dhd_output);
        end
        function [ hd_joint, dhd_joint] = get_IK_v1(obj, hd_output,dhd_output)
            hd_joint = hd_output;
            dhd_joint = dhd_output;
            hd_output(4) = min(hd_output(4),1.02);
            hd_output(9) = min(hd_output(9),1.02);
            [hd_joint(3), hd_joint(4)] = Inverse_Kinematics_p(hd_output(3), hd_output(4));
            [hd_joint(8), hd_joint(9)] = Inverse_Kinematics_p(hd_output(8), hd_output(9));
            [dhd_joint(3), dhd_joint(4)] = Inverse_Kinematics_v(hd_output(3), hd_output(4), dhd_output(3), dhd_output(4));
            [dhd_joint(8), dhd_joint(9)] = Inverse_Kinematics_v(hd_output(8), hd_output(9), dhd_output(8), dhd_output(9));
        end
        
        function [ hd_output, dhd_output] = get_FK_v1(obj, hd_joint,dhd_joint)
            hd_output = hd_joint;
            dhd_output = dhd_joint;
            [hd_output(3), hd_output(4)] = Forward_Kinematics_p(hd_joint(3), hd_joint(4));
            [hd_output(8), hd_output(9)] = Forward_Kinematics_p(hd_joint(8), hd_joint(9));
            [dhd_output(3), dhd_output(4)] = Forward_Kinematics_v(hd_joint(3), hd_joint(4), dhd_joint(3), dhd_joint(4));
            [dhd_output(8), dhd_output(9)] = Forward_Kinematics_v(hd_joint(8), hd_joint(9), dhd_joint(8), dhd_joint(9));
        end
        %% Default functions
        function setupImpl(obj)
            %SETUPIMPL Initialize System object.
        end % setupImpl
        
        function resetImpl(~)
            %RESETIMPL Reset System object states.
        end % resetImpl
        
        function [name_1, name_2, name_3, name_4, name_5, name_6]  = getInputNamesImpl(~)
            %GETINPUTNAMESIMPL Return input port names for System block
            name_1 = 't';
            name_2 = 'cassieOutputs';
            name_3 = 'isSim';
            name_4 = 'GaitLibrary';
            name_5 = 'encoder_fil';
            name_6 = 'DynamicMatrixLibrary';
        end % getInputNamesImpl
        
        function [name_1, name_2] = getOutputNamesImpl(~)
            %GETOUTPUTNAMESIMPL Return output port names for System block
            name_1 = 'userInputs';
            name_2 = 'Data';
        end % getOutputNamesImpl
        
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