%Yukai controller.

classdef StateEstimator_2 <matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime %#codegen
    % PROTECTED PROPERTIES ====================================================
    properties
        CovPos_abduction;
        CovPos_rotation;
        CovPos_thigh;
        CovPos_knee;
        CovPos_toe;
        CovPos_qj1;
        CovPos_qj2;
        CovPos_qj3;
        
        Cov_Euler;
        
        Cov_LinearAccelerator_xy;
        Cov_LinearAccelerator_z;
        
        Cov_rpx_LRToe_body;
        Cov_rpy_LRToe_body;
        Cov_rpz_LRToe_body;
        
        vx_slide_toe_1;
        vx_slide_toe_2;
        Tx_slide_toe_2;
        
        vy_slide_toe_1;
        vy_slide_toe_2;
        Ty_slide_toe_2;
        
        vz_slide_toe_1;
        vz_slide_toe_2;
        Tz_slide_toe_2;
        
        KsL_1;
        KsL_2;
        KsR_1;
        KsR_2;
        sample_time;
    end
    properties(Constant, Access = private)
        
        LAx_std_1 = 10;
        LAx_std_2 = 0.01;
        LAx_T_1 = 0.05;
        LAx_T_2 = 0.05;
        
        LAy_std_1 = 10;
        LAy_std_2 = 0.01;
        LAy_T_1 = 0.05;
        LAy_T_2 = 0.05;
        
        LAz_std_1 = 10;
        LAz_std_2 = 0.01;
        LAz_T_1 = 0.05;
        LAz_T_2 = 0.05;
        
        v_swing_toe_cov = 10000;
    end
    properties (Access = private)
        t_prev = 0;
        t0 = 0;
        s = 0;
        initialized = 0;
        z_bias = 0;
    end
    properties(Access = private) % change when swing leg change
        GRF_sw_z = 0;
        GRF_st_z = 0;
        stToe_pos = zeros(3,1);
        swToe_pos = zeros(3,1);
        stanceLeg = -1;
    end
    properties(Access = private) % kalman filter
        ITx_kf = zeros(4,1); % IT means IMU and Toe
        ITy_kf = zeros(4,1);
        ITz_kf = zeros(4,1);
        sigma_ITx = 1000*eye(4);
        sigma_ITy = 1000*eye(4);
        sigma_ITz = 1000*eye(4);
    end
    % PROTECTED METHODS =====================================================
    methods (Access = protected)
        
        function [EstStates,q,EulerRates,qxyz,dqxyz,IT,a_world, StateEstimator_Data] = stepImpl(obj,cassieOutputs, IRC, t_total,isSim)
            %% Initialize
            EstStates = PreFunctions.Construct_EstStates();
            StateEstimator_Data = PreFunctions.Construct_StateEstimator_Data();
            q = zeros(20,1);
            EulerRates = zeros(3,1);
            qxyz = zeros(3,1);
            dqxyz = zeros(3,1);
            IT = zeros(12,1);
            a_world = zeros(3,1);
            if t_total > 0.001 && cassieOutputs.isCalibrated
                
                %% get values
                [qyaw, qpitch, qroll, dqyaw, dqpitch, dqroll] = IMU_to_Euler_v2(cassieOutputs.pelvis.vectorNav.orientation, cassieOutputs.pelvis.vectorNav.angularVelocity);
                a_raw = cassieOutputs.pelvis.vectorNav.linearAcceleration;
                a_world = YToolkits.Angles.Rz(qyaw) * YToolkits.Angles.Ry(qpitch) * YToolkits.Angles.Rx(qroll) * a_raw;
                a_world = a_world -[0;0;9.806];
                qa = CassieModule.getDriveProperty(cassieOutputs,'position');
                dqa = CassieModule.getDriveProperty(cassieOutputs,'velocity');
                qj = CassieModule.getJointProperty(cassieOutputs,'position');
                dqj = CassieModule.getJointProperty(cassieOutputs,'velocity');
                qq = cassieOutputs.pelvis.vectorNav.orientation;
                qaL = qa(1:5);
                qaR = qa(6:10);
                qjL =  qj(1:2);
                qjR =  qj(4:5);
                dqaL = dqa(1:5);
                dqaR = dqa(6:10);
                dqjL =  dqj(1:2);
                dqjR =  dqj(4:5);
                qsL = getSpringDeflectionAngleV2(qaL(4),qjL(1),qjL(2));
                qsR = getSpringDeflectionAngleV2(qaR(4),qjR(1),qjR(2));
                dqsL = getSpringDeflectionRateV2(qaL(4),qjL(1),qjL(2),dqaL(4),dqjL(1),dqjL(2));
                dqsR = getSpringDeflectionRateV2(qaR(4),qjR(1),qjR(2),dqaR(4),dqjR(1),dqjR(2));
                % assign the value
                q_abduction_R = qaR(1);
                q_rotation_R = qaR(2);
                q_thigh_R = qaR(3);
                q_thigh_knee_R = qaR(4);
                q_knee_shin_R = qjR(1);
                q_shin_tarsus_R = qjR(2);
                q_toe_R = qaR(5);
                
                q_abduction_L = qaL(1);
                q_rotation_L = qaL(2);
                q_thigh_L = qaL(3);
                q_thigh_knee_L = qaL(4);
                q_knee_shin_L = qjL(1);
                q_shin_tarsus_L = qjL(2);
                q_toe_L = qaL(5);
                
                dq_abduction_R = dqaR(1);
                dq_rotation_R = dqaR(2);
                dq_thigh_R = dqaR(3);
                dq_thigh_knee_R = dqaR(4);
                dq_knee_shin_R = dqjR(1);
                dq_shin_tarsus_R = dqjR(2);
                dq_toe_R = dqaR(5);
                
                dq_abduction_L = dqaL(1);
                dq_rotation_L = dqaL(2);
                dq_thigh_L = dqaL(3);
                dq_thigh_knee_L = dqaL(4);
                dq_knee_shin_L = dqjL(1);
                dq_shin_tarsus_L = dqjL(2);
                dq_toe_L = dqaL(5);
                
                
                q_pre = [  0;  0;              0;              qyaw;           qpitch;              qroll;
                    q_abduction_L;	q_rotation_L;	q_thigh_L;      q_thigh_knee_L;     q_knee_shin_L;      q_shin_tarsus_L;    q_toe_L;
                    q_abduction_R;	q_rotation_R;	q_thigh_R;      q_thigh_knee_R;     q_knee_shin_R;      q_shin_tarsus_R;    q_toe_R];
                dq_pre_AR = [  0;  0;              0;              dqyaw;           dqpitch;              dqroll;
                    dq_abduction_L;	dq_rotation_L;	dq_thigh_L;      dq_thigh_knee_L;     dq_knee_shin_L;      dq_shin_tarsus_L;    dq_toe_L;
                    dq_abduction_R;	dq_rotation_R;	dq_thigh_R;      dq_thigh_knee_R;     dq_knee_shin_R;      dq_shin_tarsus_R;    dq_toe_R];
                %% Calculation
                GRF_L = zeros(2,1); GRF_R = zeros(2,1); 
                [ GRF_L, GRF_R  ] = get_GRF(obj,q_pre,qsR,qsL);
                if isSim == 2
                    GRF_L(2) = dqj(3);
                    GRF_R(2) = dqj(6);
                else
                    [ GRF_L, GRF_R  ] = get_GRF(obj,q_pre,qsR,qsL);
                end
                
                t = t_total - obj.t0;
                obj.s = obj.s + IRC.ds*(t-obj.t_prev);
                
                if (obj.GRF_sw_z >= 200 && obj.s>0.5) || obj.s>1.2
                    obj.stanceLeg = -obj.stanceLeg;
                    obj.t0 = t_total;
                    t = 0;
                    obj.s = 0;
                    LegSwitch = 1;
                    obj.stToe_pos = obj.swToe_pos;
                else
                    LegSwitch = 0;
                end
                
                if obj.stanceLeg == -1
                    obj.GRF_sw_z = GRF_R(2);
                    obj.GRF_st_z = GRF_L(2);
                else
                    obj.GRF_sw_z = GRF_L(2);
                    obj.GRF_st_z = GRF_R(2);
                end
                

                
                rp_Origin2LToe = - p_LeftToeJoint(q_pre); % relative position between hip and left toe
                rp_Origin2RToe = - p_RightToeJoint(q_pre);                
                Jrp_Origin2LToe = - Jp_LeftToeJoint(q_pre); % relative position between hip and left toe
                Jrp_Origin2RToe = - Jp_RightToeJoint(q_pre);               
                if obj.stanceLeg == -1
                    rp_Origin2stToe = rp_Origin2LToe;
                    rp_Origin2swToe = rp_Origin2RToe;
                else
                    rp_Origin2stToe = rp_Origin2RToe;
                    rp_Origin2swToe = rp_Origin2LToe;
                end
                
                if IRC.reset_bias
                    obj.z_bias = YToolkits.first_order_filter(obj.z_bias, a_world(3), 0.0002);
                end
                a_world(3) = a_world(3) - obj.z_bias;
                if IRC.reset_IMU_KF || ~obj.initialized
                    obj.ITx_kf = zeros(4,1); % IT means IMU and Toe
                    obj.ITy_kf = zeros(4,1);
                    obj.ITz_kf = zeros(4,1);
                    
                    obj.ITx_kf(2) = rp_Origin2LToe(1);
                    obj.ITx_kf(4) = rp_Origin2LToe(1) - rp_Origin2RToe(1);
                    
                    obj.ITy_kf(2) = rp_Origin2LToe(2);
                    obj.ITy_kf(4) = rp_Origin2LToe(2) - rp_Origin2RToe(2);
                    
                    obj.ITz_kf(2) = rp_Origin2LToe(3);
                    obj.ITz_kf(4) = rp_Origin2LToe(3) - rp_Origin2RToe(3);
                end
                
                if obj.initialized == 0
                    obj.initialized = 1;
                    LegSwitch = 1;
                end
                    
                %% KF for Origin(at IMU) Assume x,y,z are uncorrelated.
                At = eye(4,4); At(2,1) = obj.sample_time;
                Bt = [obj.sample_time;0;0;0];
                Ct = [0 1 -1 0; 0 1 0 -1];
                Cov_q_measured = get_Cov_q_measured(obj);
                Cov_q_fake = zeros(20,20); Cov_q_fake(4:20, 4:20) = Cov_q_measured;
                Cov_LinearAccelerator = diag([obj.Cov_LinearAccelerator_xy; obj.Cov_LinearAccelerator_xy; obj.Cov_LinearAccelerator_z]);
%                 Qt = [Jrp_Origin2LToe; Jrp_Origin2RToe] * Cov_q_fake * [Jrp_Origin2LToe; Jrp_Origin2RToe]';
                Qt = YToolkits.Angles.Rz(qyaw) * diag([obj.Cov_rpx_LRToe_body; obj.Cov_rpy_LRToe_body; obj.Cov_rpz_LRToe_body]) * YToolkits.Angles.Rz(qyaw)';
%                 Qtx = [Qt(1,1) 0; 0 Qt(4,4)]; 
%                 Qty = [Qt(2,2) 0; 0 Qt(5,5)];  
%                 Qtz = [Qt(3,3) 0; 0 Qt(6,6)];
                Qtx = [Qt(1,1) 0; 0 Qt(1,1)]; 
                Qty = [Qt(2,2) 0; 0 Qt(2,2)];  
                Qtz = [Qt(3,3) 0; 0 Qt(3,3)];               
                
                if obj.stanceLeg == -1
                    cov_pstToe_index = 3;
                    cov_pswToe_index = 4;
                else
                    cov_pstToe_index = 4;
                    cov_pswToe_index = 3;
                end
                
                Rtx = zeros(4,4);
                Rtx(1,1) = Cov_LinearAccelerator(1,1)*(1+abs(a_world(1)))^2 * obj.sample_time^2;
%                 Rtx(1,1) = obj.Cov_LinearAccelerator(1)* obj.sample_time^2;
                Rtx(cov_pstToe_index,cov_pstToe_index) = SlideToeCovariance(obj,t,'x') * obj.sample_time^2;
                Rtx(cov_pswToe_index,cov_pswToe_index) = obj.v_swing_toe_cov * obj.sample_time^2;
                
                Rty = zeros(4,4);
                Rty(1,1) = Cov_LinearAccelerator(2,2)*(1+abs(a_world(2)))^2 * obj.sample_time^2;
%                 Rty(1,1) = obj.Cov_LinearAccelerator(2) * obj.sample_time^2;
                Rty(cov_pstToe_index,cov_pstToe_index) = SlideToeCovariance(obj,t,'y') * obj.sample_time^2;
                Rty(cov_pswToe_index,cov_pswToe_index) = obj.v_swing_toe_cov * obj.sample_time^2;
                
                Rtz = zeros(4,4);
                Rtz(1,1) = Cov_LinearAccelerator(3,3)*(1+abs(a_world(3)))^2 * obj.sample_time^2;
%                 Rtz(1,1) = obj.Cov_LinearAccelerator(3) * obj.sample_time^2;
                Rtz(cov_pstToe_index,cov_pstToe_index) = SlideToeCovariance(obj,t,'z') * obj.sample_time^2;
                Rtz(cov_pswToe_index,cov_pswToe_index) = obj.v_swing_toe_cov * obj.sample_time^2;
                
                if t_total<0.3
                     Rtx(cov_pstToe_index,cov_pstToe_index) = 1000;
                     Rty(cov_pstToe_index,cov_pstToe_index) = 1000;
                     Rtz(cov_pstToe_index,cov_pstToe_index) = 1000;
                end
                
                utx = a_world(1);
                uty = a_world(2);
                utz = a_world(3);
                
                ztx = [rp_Origin2LToe(1); rp_Origin2RToe(1)];
                zty = [rp_Origin2LToe(2); rp_Origin2RToe(2)];
                ztz = [rp_Origin2LToe(3); rp_Origin2RToe(3)];
                
                ITx_bar = At*obj.ITx_kf + Bt*utx;
                sigma_ITx_bar = At*obj.sigma_ITx*At' + Rtx;
%                 Ktx = sigma_ITx_bar*Ct'*(Ct*sigma_ITx_bar*Ct'+Qtx)^-1;
%                 obj.ITx_kf = ITx_bar + Ktx*(ztx - Ct*ITx_bar);
%                 obj.sigma_ITx = (eye(4)-Ktx*Ct)*sigma_ITx_bar;
                obj.ITx_kf = ITx_bar + sigma_ITx_bar*Ct'*((Ct*sigma_ITx_bar*Ct'+Qtx)\(ztx - Ct*ITx_bar));
                obj.sigma_ITx = (eye(4)-sigma_ITx_bar*Ct'*((Ct*sigma_ITx_bar*Ct'+Qtx)\Ct))*sigma_ITx_bar;
                
                ITy_bar = At*obj.ITy_kf + Bt*uty;
                sigma_ITy_bar = At*obj.sigma_ITy*At' + Rty;
%                 Kty = sigma_ITy_bar*Ct'*(Ct*sigma_ITy_bar*Ct'+Qty)^-1; % try if use \ can save computation time
%                 obj.ITy_kf = ITy_bar + Kty*(zty - Ct*ITy_bar);
%                 obj.sigma_ITy = (eye(4)-Kty*Ct)*sigma_ITy_bar;
                obj.ITy_kf = ITy_bar + sigma_ITy_bar*Ct'*((Ct*sigma_ITy_bar*Ct'+Qty)\(zty - Ct*ITy_bar));
                obj.sigma_ITy = (eye(4)-sigma_ITy_bar*Ct'*((Ct*sigma_ITy_bar*Ct'+Qty)\Ct))*sigma_ITy_bar;
                
                ITz_bar = At*obj.ITz_kf + Bt*utz;
                sigma_ITz_bar = At*obj.sigma_ITz*At' + Rtz;
%                 Ktz = sigma_ITz_bar*Ct'*(Ct*sigma_ITz_bar*Ct'+Qtz)^-1;
%                 obj.ITz_kf = ITz_bar + Ktz*(ztz - Ct*ITz_bar);
%                 obj.sigma_ITz = (eye(4)-Ktz*Ct)*sigma_ITz_bar;
                obj.ITz_kf = ITz_bar + sigma_ITz_bar*Ct'*((Ct*sigma_ITz_bar*Ct'+Qtz)\(ztz - Ct*ITz_bar));
                obj.sigma_ITz = (eye(4)-sigma_ITz_bar*Ct'*((Ct*sigma_ITz_bar*Ct'+Qtz)\Ct))*sigma_ITz_bar;
                
                %% Assign out put
                qxyz = [obj.ITx_kf(2); obj.ITy_kf(2); obj.ITz_kf(2)];
%                 qxyz = a_world;
                dqxyz = [obj.ITx_kf(1); obj.ITy_kf(1); obj.ITz_kf(1)];
                q = [qxyz; q_pre([4:end])];
                dq_AR = [dqxyz; dq_pre_AR([4:end])];
                EulerRates = [dqyaw; dqpitch; dqroll];
                IT = [obj.ITx_kf; obj.ITy_kf; obj.ITz_kf];
                
                %% Log
                obj.t_prev = t;
                %% Construct EstStates
                EstStates.isCalibrated = cassieOutputs.isCalibrated;
                EstStates.q = q;
                EstStates.dq_AR = dq_AR;
                EstStates.s = obj.s;
                EstStates.t = t;
                EstStates.Voltage = cassieOutputs.rightLeg.kneeDrive.dcLinkVoltage;
                EstStates.a_world = a_world;
                
                EstStates.stanceLeg = obj.stanceLeg;
                EstStates.LegSwitch = LegSwitch;
                EstStates.GRF_z = [GRF_L(2);GRF_R(2)];
                EstStates.GRF_L = GRF_L;
                EstStates.GRF_R = GRF_R;
                
                EstStates.qsL_1 = qsL(1);
                EstStates.qsL_2 = qsL(2);
                EstStates.qsR_1 = qsR(1);
                EstStates.qsR_2 = qsR(2);
                
                EstStates.dqsL_1 = dqsL(1);
                EstStates.dqsL_2 = dqsL(2);
                EstStates.dqsR_1 = dqsR(1);
                EstStates.dqsR_2 = dqsR(2);
                
                
                EstStates.dqxyz_cov = diag([obj.sigma_ITx(1,1),obj.sigma_ITy(1,1),obj.sigma_ITz(1,1)]);
                EstStates.qxyz_cov = diag([obj.sigma_ITx(2,2),obj.sigma_ITy(2,2),obj.sigma_ITz(2,2)]);
                
                %% EstStates_Data
                StateEstimator_Data.a_world = a_world;
                StateEstimator_Data.rp_Origin2LToe = rp_Origin2LToe;
                StateEstimator_Data.rp_Origin2RToe = rp_Origin2RToe;
                StateEstimator_Data.ITx_kf = obj.ITx_kf;
                StateEstimator_Data.ITy_kf = obj.ITy_kf;
                StateEstimator_Data.ITz_kf = obj.ITz_kf;
            end
        end % stepImpl
        
        function [ GRF_L, GRF_R  ] = get_GRF(obj,qall,qsR,qsL)
            qall(4) = 0;
            JR = Jp_RightToeJoint(qall);
            JL = Jp_LeftToeJoint(qall);
            [Fs1R, Fs2R, Fs1L, Fs2L] = get_spring_force(obj,qsR,qsL);
            JL_H = J_HeelSpringDeflectionEst(qall(10),qall(11),qall(12));
            JR_H = J_HeelSpringDeflectionEst(qall(17),qall(18),qall(19));
            JL_s = JL([1,3],[11,12]);
            JR_s = JR([1,3],[18,19]);
            GRF_L = (-JL_s')^-1*[Fs1L+JL_H(2)*Fs2L; JL_H(3)*Fs2L];
            GRF_R = (-JR_s')^-1*[Fs1R+JR_H(2)*Fs2R; JR_H(3)*Fs2R];
        end
        function [Fs1R, Fs2R, Fs1L, Fs2L] = get_spring_force(obj,qsR,qsL)
            Fs1R =- obj.KsR_1 * qsR(1);
            Fs2R =- obj.KsR_2 * qsR(2);
            Fs1L =- obj.KsL_1 * qsL(1);
            Fs2L =- obj.KsL_2 * qsL(2);
        end
        %% Default functions
        function setupImpl(obj)
            %SETUPIMPL Initialize System object.
        end % setupImpl
        
        function resetImpl(~)
            %RESETIMPL Reset System object states.
        end % resetImpl
        
%         function [name_1, name_2, name_3]  = getInputNamesImpl(~)
%             %GETINPUTNAMESIMPL Return input port names for System block
%             name_1 = 'cassieOutputs';
%             name_2 = 't_total';
%             name_3 = 'isSim';
%         end % getInputNamesImpl
        
%         function [name_1, name_2, name_3, name_4, name5, name6] = getOutputNamesImpl(~)
%             %GETOUTPUTNAMESIMPL Return output port names for System block
%             name_1 = 'EstStates';
%             name_2 = 'q';
%             name_3 = 'EulerRates';
%             name_4 = 'qxyz';
%             name_5 = 'dqxyz';
%             name_5 = 'dqxyz';
%         end % getOutputNamesImpl
        
        % PROPAGATES CLASS METHODS ============================================
        function [EstStates, q, EulerRates, qxyz, dqxyz, IT, a_world, StateEstimator_Data] = getOutputSizeImpl(~)
            %GETOUTPUTSIZEIMPL Get sizes of output ports.
            EstStates = [1, 1];
            q = [20, 1];
            EulerRates = [3, 1];
            qxyz = [3, 1];
            dqxyz = [3, 1];
            IT = [12, 1];
            a_world = [3, 1];
            StateEstimator_Data = [1, 1];
        end % getOutputSizeImpl
        
        function [EstStates, q, EulerRates, qxyz, dqxyz, IT, a_world, StateEstimator_Data] = getOutputDataTypeImpl(~)
            %GETOUTPUTDATATYPEIMPL Get data types of output ports.
            EstStates = 'EstStatesBus';
            q = 'double';
            EulerRates = 'double';
            qxyz = 'double';
            dqxyz = 'double';
            IT = 'double';
            a_world = 'double';
            StateEstimator_Data = 'StateEstimator_DataBus';
        end % getOutputDataTypeImpl
        
        function [EstStates, q, EulerRates, qxyz, dqxyz, IT, a_world, StateEstimator_Data] = isOutputComplexImpl(~)
            %ISOUTPUTCOMPLEXIMPL Complexity of output ports.
            EstStates = false;
            q = false;
            EulerRates = false;
            qxyz = false;
            dqxyz = false;
            IT = false;
            a_world = false;
            StateEstimator_Data = false;
        end % isOutputComplexImpl
        
        function [EstStates, q, EulerRates, qxyz, dqxyz, IT, a_world, StateEstimator_Data] = isOutputFixedSizeImpl(~)
            %ISOUTPUTFIXEDSIZEIMPL Fixed-size or variable-size output ports.
            EstStates = true;
            q = true;
            EulerRates = true;
            qxyz = true;
            dqxyz = true;
            IT = true;
            a_world = true;
            StateEstimator_Data = true;
        end % isOutputFixedSizeImpl

    end % methods
end % classdef