%Yukai controller.

classdef Cassie_Controller_1 <matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime %#codegen
    % PROTECTED PROPERTIES ====================================================
    properties
        
    end
    properties (Access = private, Constant)
        TorqueLimits = repmat([112.5;112.5;195.2;195.2;45],[2,1]);
        ActuatorLimits = [-0.2618, 0.3927;    -0.3927, 0.3927;    -0.8727, 1.3963;    -2.8623, -0.7330;   -2.4435, -0.5236; ...
            -0.3927, 0.2618;    -0.3927, 0.3927;    -0.8727, 1.3963;    -2.8623, -0.7330;   -2.4435, -0.5236];
        Ks1 = 1500;
        Ks2 = 1250;
    end
    properties (Access = protected)
        
    end
    % PRIVATE PROPERTIES ====================================================
    properties (Access = private)
       stanceLeg = -1;
       t0= 0;
       total_mass = 32;
       rp_swT_ini = zeros(3,1);
       rv_swT_ini = zeros(3,1); 
    end % properties
    
    % PROTECTED METHODS =====================================================
    methods (Access = protected)
        
        function [userInputs, Data] = stepImpl(obj, t_total, cassieOutputs, isSim, GaitLibrary, encoder_fil, DynamicMatrixLibrary,x)
            %STEPIMPL System output and state update equations.
            
            %% Initialize --------------------------------------------------------
            Data = PreFunctions.Construct_Data;
            % Reset the desired motor torques to zero in case they aren't defined
            userInputs = CassieModule.getUserInStruct;
            u = zeros(10,1);
            
            q = x(1:20);
            dq = x(21:40);
            % Let the output be torso angle, com height and delta x,delta z of swing
            % feet and com. delta = p_com - p_swfeet.
            T = 0.3; % walking period
            Vx = 2; % Desired velocity at the end of a step
            Vy = 0.1; % left right should be different
            Kd = 50;
            Kp = 500;
            g = 9.81; 
            ds = 1/T;
            
            % get ground reaction force
            dqj = CassieModule.getJointProperty(cassieOutputs,'velocity');
            GRF_L = dqj(3);
            GRF_R = dqj(6);

            if obj.stanceLeg == -1
                GRF_sw_z = GRF_R;
                GRF_st_z = GRF_L;
            else
                GRF_sw_z = GRF_L;
                GRF_st_z = GRF_R;
            end
            
            t = t_total - obj.t0;
            s = (t_total - obj.t0)/T;

            
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
            
            LG = getCassieAngularMomentum(p_com,x);
            L_LeftToe = getCassieAngularMomentum(p_LT,x);
            L_RightToe = getCassieAngularMomentum(p_RT,x);
            L_LeftToe_vg = obj.total_mass*cross(rp_LT,v_com);
            L_RightToe_vg = obj.total_mass*cross(rp_RT,v_com);
            
            
            if (GRF_sw_z >= 150 && s>0.5) || s>1.1
                obj.stanceLeg = -obj.stanceLeg;
                obj.t0 = t_total;
                if obj.stanceLeg == -1
                    obj.rp_swT_ini = rp_RT;
                    obj.rv_swT_ini = rv_RT;
                else
                    obj.rp_swT_ini = rp_LT;
                    obj.rv_swT_ini = rv_LT;
                end
            end
            
            
            if obj.stanceLeg == -1
                
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
            end
            
            % order the index of stance leg and swing leg
            if obj.stanceLeg == -1 % right stanceleg
                st_abduction = 7;
                st_rotation = 8;
                st_thigh = 9;
                st_knee = 10;
                st_toe = 13;
                sw_abduction = 14;
                sw_rotation = 15;
                sw_thigh = 16;
                sw_knee = 17;
                sw_toe =20;
            else
                sw_abduction = 7;
                sw_rotation = 8;
                sw_thigh = 9;
                sw_knee = 10;
                sw_toe = 13;
                st_abduction = 14;
                st_rotation = 15;
                st_thigh = 16;
                st_knee = 17;
                st_toe =20;
            end
            


            
            
            T_left = T - t;
            l = sqrt(g/q(3));
            
            pseudo_com_vx = L_stToe(2)/(32*q(3));
            one_step_max_vel_gain = T*l*0.2;
%             dx0_next = rp_stT(1)*l*sinh(l*T_left) + rv_stT(1)*cosh(l*T_left);
            dx0_next = rp_stT(1)*l*sinh(l*T_left) + pseudo_com_vx*cosh(l*T_left);
            dxf_next_goal = median([dx0_next + one_step_max_vel_gain, dx0_next - one_step_max_vel_gain, Vx]);
            x0_next = (dxf_next_goal - dx0_next*cosh(l*T))/(l*sinh(l*T));
            % x0_next is the desired relative position of COM to stance foot swing foot in the beginning of next step,(at this step it is still swing foot) so that COM velocity can be V at time T
            
            pseudo_com_vy = -L_stToe(1)/(32*q(3));
            dy0_next = rp_stT(2)*l*sinh(l*T_left) + pseudo_com_vy*cosh(l*T_left);
            dyf_next_goal = -obj.stanceLeg*Vy;
            y0_next = (dyf_next_goal - dy0_next*cosh(l*T))/(l*sinh(l*T));
            
            w = pi/T;
            H = 0.8;
            CL = 0.1;
            
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

                        % Jh is jacobian for output
            Jh = zeros(8,20);
            dJh = zeros(8,20);
            
            Jh(1,5) = 1; % torso pitch
            Jh(2,6) = 1; % torso roll
            Jh(3,st_rotation) = 1; % stance rotation
            Jh(4,:) = Jrp_stT(3,:); % com to stance toe height
            Jh([5,6,7],:) = Jrp_swT([1,2,3],:); % com to swing toe x y z pos
            Jh(8,sw_rotation) = 1; % swing rotation.

            dJh(4,:) = dJrp_stT(3,:);
            dJh([5,6,7],:) = dJrp_swT([1,2,3],:);
            
            h0 = [q(5);q(6);q(st_rotation);rp_stT(3);rp_swT([1,2,3]);q(sw_rotation)];
            dh0 = Jh*dq;
            
            hr= [0;0;0;H;ref_rp_swT_x;ref_rp_swT_y;ref_rp_swT_z;0];
            dhr = [0;0;0;0;ref_rv_swT_x;ref_rv_swT_y;ref_rv_swT_z;0];
            ddhr = [0;0;0;0;ref_ra_swT_x;ref_ra_swT_y;ref_ra_swT_z;0];
            
            y = h0 - hr;
            dy = dh0 - dhr;

            
%             if obj.stanceLeg == -1
%                 motor_index = [3,4,6,7,8,9];
%             else
%                 motor_index = [1,2,3,4,8,9];
%             end

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
            if obj.stanceLeg == -1
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
            userInputs.torque = u;
            
            % log Data
            Data.stanceLeg = obj.stanceLeg;
            Data.lG_x = LG(1);
            Data.l_LeftToe_x = L_LeftToe(1);
            Data.l_RightToe_x = L_RightToe(1);
            Data.l_LeftToe_vg_x = L_LeftToe_vg(1);
            Data.l_RightToe_vg_x = L_RightToe_vg(1);
            
            Data.lG_y = LG(2);
            Data.l_LeftToe_y = L_LeftToe(2);
            Data.l_RightToe_y = L_RightToe(2);
            Data.l_LeftToe_vg_y = L_LeftToe_vg(2);
            Data.l_RightToe_vg_y = L_RightToe_vg(2);
            
            Data.dx0_next = dx0_next;
            Data.x0_next = x0_next;
            Data.dxf_next_goal = dxf_next_goal;
            
            Data.hr = hr;
            Data.dhr = dhr;
            Data.h0 = h0;
            Data.dh0 = dh0;
            
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
        end % stepImpl
        
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
            name_7 = 'x';
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