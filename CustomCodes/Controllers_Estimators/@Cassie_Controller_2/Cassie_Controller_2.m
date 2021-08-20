%Yukai controller.

classdef Cassie_Controller_2 <matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime %#codegen
    % PROTECTED PROPERTIES ====================================================
    properties
        step_time;
        sample_time;
        cov_q_measured;
        cov_dq_estimated;
        
    end
    properties (Access = private, Constant)
        TorqueLimits = repmat([112.5;112.5;195.2;195.2;45],[2,1]);
        ActuatorLimits = [-0.2618, 0.3927;    -0.3927, 0.3927;    -0.8727, 1.3963;    -2.8623, -0.7330;   -2.4435, -0.5236; ...
            -0.3927, 0.2618;    -0.3927, 0.3927;    -0.8727, 1.3963;    -2.8623, -0.7330;   -2.4435, -0.5236];
        Ks1 = 1500;
        Ks2 = 1250;
        
        total_mass = 32;
        
    end
    properties (Access = protected)
        
    end
    % PRIVATE PROPERTIES ====================================================
    properties(Access = private) % change when swing leg change
        rp_swT_ini = zeros(3,1);
        rv_swT_ini = zeros(3,1);
        stToe_pos = zeros(3,1);
        swToe_pos = zeros(3,1);
    end
    properties(Access = private) 
        % for filters
        sigma_Lx = 100;
        Lx_stToe_kf = 100;
        sigma_Ly = 100;
        Ly_stToe_kf = 100;
        % IK solver
        qd_control = zeros(8,1);
        dqd_control = zeros(8,1);
    end
    
    
    % PROTECTED METHODS =====================================================
    methods (Access = protected)
        
        function [userInputs, Data] = stepImpl(obj,EstStates,t_total,isSim)
            %STEPIMPL System output and state update equations.
            
            %% Initialize --------------------------------------------------------
            
            Data = PreFunctions.Construct_Data;
            % Reset the desired motor torques to zero in case they aren't defined
            userInputs = CassieModule.getUserInStruct;
            u = zeros(10,1);
            
            if t_total>0.001
                % Let the output be torso angle, com height and delta x,delta z of swing
                % feet and com. delta = p_com - p_swfeet.
                
                g = 9.81;
                H = 0.8; % desired com height
                CL = 0.1; % foot clearance
                if isSim == 2
                    spring_height_compensation = 0;
                else
                    spring_height_compensation = 0.00;
                end
                Vx = 3; % Desired velocity at the end of a step
                %             Vy = 0.1; % left right should be different
                rp_com2stToe_desired = -0.15; % at the begining of left stance (also at the end of left stance)
                Vy = -rp_com2stToe_desired*sqrt(g/H)*sinh(sqrt(g/H)*obj.step_time)/(1+cosh(sqrt(g/H)*obj.step_time)); % initial ( and negative end) velocity required to achieve rp_com2stToe_desired
                Kd = 20;
                Kp = 500;
                
                
                
                
                
                
                
                q = EstStates.q;
                dq = EstStates.dq;
                s = EstStates.s;
                t = EstStates.t;
                stanceLeg = EstStates.stanceLeg;
                LegSwitch = EstStates.LegSwitch;
                
                
                
                g=9.81;
                ds = 1/obj.step_time;
                
                Cov_q_measured = eye(17) * obj.cov_q_measured;
                Cov_dq_estimated = eye(17) * obj.cov_dq_estimated;
                Cov_p_StanceToe = zeros(3,3);
                Cov_v_StanceToe = zeros(3,3);
                
                % construct full q and dq (x,y) with measured q based on stance
                % leg information
                Jp_LT_pre = Jp_LeftToeJoint(q);
                Jp_RT_pre = Jp_RightToeJoint(q);
                rp_Hip2LT = -p_LeftToeJoint(q); % relative position between hip and left toe
                rp_Hip2RT = -p_RightToeJoint(q);
                rv_Hip2LT = -Jp_LT_pre*dq;
                rv_Hip2RT = -Jp_RT_pre*dq;
                if stanceLeg == -1
                    Cov_q = [Jp_LT_pre(:,[4:20]);eye(17)] * Cov_q_measured * [Jp_LT_pre(:,[4:20]);eye(17)]' + [Cov_p_StanceToe,zeros(3,17);zeros(17,20)];
                    Cov_dq = [Jp_LT_pre(:,[4:20]);eye(17)] * Cov_dq_estimated * [Jp_LT_pre(:,[4:20]);eye(17)]' + [Cov_p_StanceToe,zeros(3,17);zeros(17,20)];
                else
                    Cov_q = [Jp_RT_pre(:,[4:20]);eye(17)] * Cov_q_measured * [Jp_RT_pre(:,[4:20]);eye(17)]' + [Cov_p_StanceToe,zeros(3,17);zeros(17,20)];
                    Cov_dq = [Jp_RT_pre(:,[4:20]);eye(17)] * Cov_dq_estimated * [Jp_RT_pre(:,[4:20]);eye(17)]'+ [Cov_p_StanceToe,zeros(3,17);zeros(17,20)];
                end
                
                p_com = p_COM(q);
                Jp_com = Jp_COM(q);
                dJp_com = dJp_COM(q,dq);
                v_com = Jp_com*dq;
                
                p_LT = p_LeftToeJoint(q);
                Jp_LT = Jp_LeftToeJoint(q);
                dJp_LT = dJp_LeftToeJoint(q,dq);
                v_LT = Jp_LT*dq;
                
                p_RT = p_RightToeJoint(q);
                Jp_RT = Jp_RightToeJoint(q);
                dJp_RT = dJp_RightToeJoint(q,dq);
                v_RT = Jp_RT*dq;
                
                % com position RELATIVE to toes
                
                rp_LT = p_com - p_LT;
                Jrp_LT = Jp_com - Jp_LT;
                dJrp_LT = dJp_com - dJp_LT;
                rv_LT = v_com - v_LT;
                
                rp_RT = p_com - p_RT;
                Jrp_RT = Jp_com - Jp_RT;
                dJrp_RT = dJp_com - dJp_RT;
                rv_RT = v_com - v_RT;
                
                LG = getCassieAngularMomentum(p_com,[q;dq]);
                L_LeftToe = getCassieAngularMomentum(p_LT,[q;dq]);
                L_RightToe = getCassieAngularMomentum(p_RT,[q;dq]);
                L_LeftToe_vg = obj.total_mass*cross(rp_LT,v_com);
                L_RightToe_vg = obj.total_mass*cross(rp_RT,v_com);
                
                L_LeftToe_obs = L_LeftToe;
                L_RightToe_obs = L_RightToe;
                
                % calculating covariance of
                JL_q_LT = Jq_AMworld_about_pA(q,dq,p_LT,Jp_LT);
                JL_dq_LT = Jdq_AMworld_about_pA(q,dq,p_LT,zeros(3,20));
                Cov_L_LT = JL_q_LT*Cov_q*JL_q_LT' + JL_dq_LT*Cov_dq*JL_dq_LT';
                Cov_Lx_LT = Cov_L_LT(1,1);
                Cov_Ly_LT = Cov_L_LT(2,2);
                
                JL_q_RT = Jq_AMworld_about_pA(q,dq,p_RT,Jp_RT);
                JL_dq_RT = Jdq_AMworld_about_pA(q,dq,p_RT,zeros(3,20));
                Cov_L_RT = JL_q_RT*Cov_q*JL_q_RT' + JL_dq_RT*Cov_dq*JL_dq_RT';
                Cov_Lx_RT = Cov_L_RT(1,1);
                Cov_Ly_RT = Cov_L_RT(2,2);
                
                Cov_rp_LT = Jrp_LT*Cov_q*Jrp_LT';
                Cov_rpx_LT = Cov_rp_LT(1,1);
                Cov_rpy_LT = Cov_rp_LT(2,2);
                
                Cov_rp_RT = Jrp_RT*Cov_q*Jrp_RT';
                Cov_rpx_RT = Cov_rp_RT(1,1);
                Cov_rpy_RT = Cov_rp_RT(2,2);
                
                
                
                if LegSwitch == 1
                    if stanceLeg == -1
                        obj.rp_swT_ini = rp_RT;
                        obj.rv_swT_ini = rv_RT;
                        obj.Lx_stToe_kf = L_LeftToe_obs(1);
                        obj.Ly_stToe_kf = L_LeftToe_obs(2);
                        
                        %                     obj.sigma = 2.28^2;
                        obj.sigma_Lx = Cov_Lx_LT;
                        obj.sigma_Ly = Cov_Ly_LT;
                    else
                        obj.rp_swT_ini = rp_LT;
                        obj.rv_swT_ini = rv_LT;
                        obj.Lx_stToe_kf = L_RightToe_obs(1);
                        obj.Ly_stToe_kf = L_RightToe_obs(2);
                        %                     obj.sigma = 2.28^2;
                        obj.sigma_Lx = Cov_Lx_RT;
                        obj.sigma_Ly = Cov_Ly_RT;
                        
                    end
                end
                
                if stanceLeg == -1
                    
                    p_stT = p_LT;
                    Jp_stT = Jp_LT;
                    dJp_stT = dJp_LT;
                    v_stT = v_LT;
                    
                    p_swT = p_RT;
                    Jp_swT = Jp_RT;
                    dJp_swT = dJp_RT;
                    v_swT = v_RT;
                    
                    rp_stT = rp_LT;
                    Jrp_stT = Jrp_LT;
                    dJrp_stT = dJrp_LT;
                    rv_stT = rv_LT;
                    
                    rp_swT = rp_RT;
                    Jrp_swT = Jrp_RT;
                    dJrp_swT = dJrp_RT;
                    rv_swT = rv_RT;
                    
                    L_stToe = L_LeftToe;
                    L_swToe = L_RightToe;
                    
                    L_stToe_obs = L_LeftToe_obs;
                    L_swToe_obs = L_RightToe_obs;
                    
                    Cov_Lx_stT = Cov_Lx_LT;
                    Cov_Lx_swT = Cov_Lx_RT;
                    Cov_Ly_stT = Cov_Ly_LT;
                    Cov_Ly_swT = Cov_Ly_RT;
                    
                    Cov_rpx_stT = Cov_rpx_LT;
                    Cov_rpx_swTx = Cov_rpx_RT;
                    Cov_rpy_stT = Cov_rpy_LT;
                    Cov_rpy_swT = Cov_rpy_RT;
                else
                    p_stT = p_RT;
                    Jp_stT = Jp_RT;
                    dJp_stT = dJp_RT;
                    v_stT = v_RT;
                    
                    p_swT = p_LT;
                    Jp_swT = Jp_LT;
                    dJp_swT = dJp_LT;
                    v_swT = v_LT;
                    
                    rp_stT = rp_RT;
                    Jrp_stT = Jrp_RT;
                    dJrp_stT = dJrp_RT;
                    rv_stT = rv_RT;
                    
                    rp_swT = rp_LT;
                    Jrp_swT = Jrp_LT;
                    dJrp_swT = dJrp_LT;
                    rv_swT = rv_LT;
                    
                    L_stToe = L_RightToe;
                    L_swToe = L_LeftToe;
                    
                    L_stToe_obs = L_RightToe_obs;
                    L_swToe_obs = L_LeftToe_obs;
                    
                    Cov_Lx_stT = Cov_Lx_RT;
                    Cov_Lx_swT = Cov_Lx_LT;
                    Cov_Ly_stT = Cov_Ly_RT;
                    Cov_Ly_swT = Cov_Ly_LT;
                    
                    Cov_rpx_stT = Cov_rpx_RT;
                    Cov_rpx_swT = Cov_rpx_LT;
                    Cov_rpy_stT = Cov_rpy_RT;
                    Cov_rpy_swT = Cov_rpy_LT;
                end
                
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
                end
                
                
                % Ly
                At = 1;
                Ct = 1;
                Bt = 1;
                
                ut = obj.sample_time*obj.total_mass*g*rp_stT(1);
                Rt = (obj.sample_time*obj.total_mass*g)^2*Cov_rpx_stT;
                Qt = Cov_Ly_stT;
                
                Ly_stToe_bar = obj.Ly_stToe_kf + ut;
                sigma_Ly_bar = At*obj.sigma_Ly*At' + Rt;
                Kt = sigma_Ly_bar*Ct'*(Ct*sigma_Ly_bar*Ct'+Qt)^-1;
                obj.Ly_stToe_kf = Ly_stToe_bar + Kt*(L_stToe_obs(2)-Ct*Ly_stToe_bar);
                obj.sigma_Ly = (1-Kt*Ct)*sigma_Ly_bar;
                
                % Lx
                At = 1;
                Ct = 1;
                Bt = 1;
                
                ut = obj.sample_time*obj.total_mass*g*(-rp_stT(2));
                Rt = (obj.sample_time*obj.total_mass*g)^2*Cov_rpy_stT;
                Qt = Cov_Lx_stT;
                
                Lx_stToe_bar = obj.Lx_stToe_kf + ut;
                sigma_Lx_bar = At*obj.sigma_Lx*At' + Rt;
                Kt = sigma_Lx_bar*Ct'*(Ct*sigma_Lx_bar*Ct'+Qt)^-1;
                obj.Lx_stToe_kf = Lx_stToe_bar + Kt*(L_stToe_obs(1)-Ct*Lx_stToe_bar);
                obj.sigma_Lx = (1-Kt*Ct)*sigma_Lx_bar;
                
                
                
                T_left = obj.step_time - t;
                l = sqrt(g/rp_stT(3));
                
                pseudo_com_vx = L_stToe(2)/(32*rp_stT(3));
                %             pseudo_com_vx = obj.Ly_stToe_kf/(32*rp_stT(3));
                one_step_max_vel_gain = obj.step_time*l*0.2;
                %             dx0_next = rp_stT(1)*l*sinh(l*T_left) + rv_stT(1)*cosh(l*T_left);
                dx0_next = rp_stT(1)*l*sinh(l*T_left) + pseudo_com_vx*cosh(l*T_left);
                dxf_next_goal = median([dx0_next + one_step_max_vel_gain, dx0_next - one_step_max_vel_gain, Vx]);
                x0_next = (dxf_next_goal - dx0_next*cosh(l*obj.step_time))/(l*sinh(l*obj.step_time));
                % x0_next is the desired relative position of COM to stance foot swing foot in the beginning of next step,(at this step it is still swing foot) so that COM velocity can be V at time obj.step_time
                
                pseudo_com_vy = -L_stToe(1)/(32*rp_stT(3));
                %             pseudo_com_vy = -obj.Lx_stToe_kf/(32*rp_stT(3));
                dy0_next = rp_stT(2)*l*sinh(l*T_left) + pseudo_com_vy*cosh(l*T_left);
                dyf_next_goal = -stanceLeg*Vy;
                y0_next = (dyf_next_goal - dy0_next*cosh(l*obj.step_time))/(l*sinh(l*obj.step_time));
                
                
                %% generate reference signal
                w = pi/obj.step_time;
                
                
                ref_rp_swT_x = 1/2*(obj.rp_swT_ini(1) - x0_next)*cos(w*t) + 1/2*(obj.rp_swT_ini(1) + x0_next);
                ref_rv_swT_x = 1/2*(obj.rp_swT_ini(1) - x0_next)*(-w*sin(w*t));
                ref_ra_swT_x = 1/2*(obj.rp_swT_ini(1) - x0_next)*(-w^2*cos(w*t));
                
                ref_rp_swT_y = 1/2*(obj.rp_swT_ini(2) - y0_next)*cos(w*t) + 1/2*(obj.rp_swT_ini(2) + y0_next);
                ref_rv_swT_y = 1/2*(obj.rp_swT_ini(2) - y0_next)*(-w*sin(w*t));
                ref_ra_swT_y = 1/2*(obj.rp_swT_ini(2) - y0_next)*(-w^2*cos(w*t));
                
                %             ref_rp_swT_z = 1/2*CL*cos(2*w*t)+(H-1/2*CL);
                %             ref_rv_swT_z = 1/2*CL*(-2*w*sin(2*w*t));
                %             ref_ra_swT_z = 1/2*CL*(-4*w^2*cos(2*w*t));
                ref_rp_swT_z= 4*CL*(s-0.5)^2+(H-CL);
                ref_rv_swT_z = 8*CL*(s-0.5)*ds;
                ref_ra_swT_z = 8*CL*ds^2;
                
                hr= [0;0;0;0;H+spring_height_compensation;ref_rp_swT_x;ref_rp_swT_y;ref_rp_swT_z];
                dhr = [0;0;0;0;0;ref_rv_swT_x;ref_rv_swT_y;ref_rv_swT_z];
                ddhr = [0;0;0;0;0;ref_ra_swT_x;ref_ra_swT_y;ref_ra_swT_z];
                
                %% track reference signal
                % Jh is jacobian for output
                Jh = zeros(8,20);
                dJh = zeros(8,20);
                
                Jh(1,5) = 1; % torso pitch
                Jh(2,6) = 1; % torso roll
                Jh(3,st_rotation) = 1; % stance rotation
                Jh(4,sw_rotation) = 1; % swing rotation.
                Jh(5,:) = Jrp_stT(3,:); % com to stance toe height
                Jh([6,7,8],:) = Jrp_swT([1,2,3],:); % com to swing toe x y z relativ pos
                
                
                dJh(5,:) = dJrp_stT(3,:);
                dJh([6,7,8],:) = dJrp_swT([1,2,3],:);
                
                h0 = [q(5);q(6);q(st_rotation);q(sw_rotation);rp_stT(3);rp_swT([1,2,3])];
                dh0 = Jh*dq;
                
                
                y = h0 - hr;
                dy = dh0 - dhr;
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
                    u_select = (Jh*S*Me^-1*Be)^-1*(-Kd*dy-Kp*y+ddhr+Jh*S*Me^-1*He);
                else
                    Me = [M,  -Js', -Jg';
                        Js, zeros(Jsd1,Jsd1), zeros(Jsd1,Jgd1);
                        Jg, zeros(Jgd1,Jsd1), zeros(Jgd1,Jgd1)];
                    He = [C+G;zeros(Jsd1+Jgd1,1)];
                    Be = [B_cut;zeros(Jsd1+Jgd1,length(motor_index))];
                    S = [eye(20),zeros(20,Jsd1+Jgd1)]; % S is used to seperate ddq with Ft Fs Fg;
                    u_select = (Jh*S*Me^-1*Be)^-1*(-Kd*dy-Kp*y+ddhr+Jh*S*Me^-1*He);
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
                
                %% log Data
                Data.stanceLeg = stanceLeg;
                Data.Lx_G = LG(1);
                Data.Lx_LeftToe = L_LeftToe(1);
                Data.Lx_RightToe = L_RightToe(1);
                Data.Lx_LeftToe_vg = L_LeftToe_vg(1);
                Data.Lx_RightToe_vg = L_RightToe_vg(1);
                
                Data.Ly_G = LG(2);
                Data.Ly_LeftToe = L_LeftToe(2);
                Data.Ly_RightToe = L_RightToe(2);
                Data.Ly_LeftToe_vg = L_LeftToe_vg(2);
                Data.Ly_RightToe_vg = L_RightToe_vg(2);
                
                Data.Lx_stToe = L_stToe(1);
                Data.Ly_stToe = L_stToe(2);
                Data.Lx_swToe = L_swToe(1);
                Data.Ly_swToe = L_swToe(2);
                
                Data.Lx_stToe_kf = obj.Lx_stToe_kf;
                Data.Ly_stToe_kf = obj.Ly_stToe_kf;
                
                Data.dx0_next = dx0_next;
                Data.x0_next = x0_next;
                Data.dxf_next_goal = dxf_next_goal;
                
                Data.hr = hr;
                Data.dhr = dhr;
                Data.h0 = h0;
                Data.dh0 = dh0;
                
                
                Data.p_stT = p_stT;
                Data.p_swT = p_swT;
                Data.p_LT = p_LT;
                Data.p_RT = p_RT;
                
                Data.v_stT = v_stT;
                Data.v_swT = v_swT;
                Data.v_LT = v_LT;
                Data.v_RT = v_RT;
                
                Data.rp_stT = rp_stT;
                Data.rv_stT = rv_stT;
                
                Data.p_com = p_com;
                Data.v_com = v_com;
                Data.vx_com = v_com(1);
                Data.vy_com = v_com(2);
                Data.vz_com = v_com(3);
                Data.px_com = p_com(1);
                Data.py_com = p_com(2);
                Data.pz_com = p_com(3);
                Data.pseudo_com_vx = pseudo_com_vx;
                Data.q = q;
                Data.dq = dq;
                Data.u = u;
                Data.s = s;
                
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
        
        function [name_1, name_2]  = getInputNamesImpl(~)
            %GETINPUTNAMESIMPL Return input port names for System block
            name_1 = 'EstStates';
            name_2 = 't_total';
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