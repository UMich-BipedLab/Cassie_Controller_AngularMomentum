%Yukai controller.

classdef StateEstimator_1 <matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime %#codegen
    % PROTECTED PROPERTIES ====================================================
    properties
        step_time;
    end
    properties(Constant)
        Ks1 = 1500;
        Ks2 = 1250;
    end
    properties (Access = private)
        t_prev = 0;
        t0 = 0;
        s = 0;
        initialized = 0;
    end
    properties(Access = private) % change when swing leg change
        GRF_sw_z = 0;
        GRF_st_z = 0;
        stToe_pos = zeros(3,1);
        swToe_pos = zeros(3,1);
        stanceLeg = -1;
    end
    
    
    % PROTECTED METHODS =====================================================
    methods (Access = protected)
        
        function [EstStates,q, EulerRates] = stepImpl(obj,cassieOutputs,t_total,isSim)
            %% Initialize
            EstStates = PreFunctions.Construct_EstStates();
            q = zeros(20,1);
            EulerRates = zeros(3,1);
            if t_total > 0.001
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
                qsL = getSpringDeflectionAngleV2(qaL(4),qjL(1),qjL(2));
                qsR = getSpringDeflectionAngleV2(qaR(4),qjR(1),qjR(2));
                
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
                
                
                q_pre = [  0;  0;              0;              qyaw;           qpitch;              qroll;
                    q_abduction_L;	q_rotation_L;	q_thigh_L;      q_thigh_knee_L;     q_knee_shin_L;      q_shin_tarsus_L;    q_toe_L;
                    q_abduction_R;	q_rotation_R;	q_thigh_R;      q_thigh_knee_R;     q_knee_shin_R;      q_shin_tarsus_R;    q_toe_R];
                %% Calculation
                [ GRF_L, GRF_R  ] = get_GRF(obj,q_pre,qsR,qsL);
                if isSim == 2
                    GRF_L(2) = dqj(3);
                    GRF_R(2) = dqj(6);
                end
                ds = 1/obj.step_time;
                
                t = t_total - obj.t0;
                obj.s = obj.s + ds*(t-obj.t_prev);
                
                if (obj.GRF_sw_z >= 150 && obj.s>0.5) || obj.s>1.2
                    obj.stanceLeg = -obj.stanceLeg;
                    obj.t0 = t_total;
                    t = 0;
                    obj.s = 0;
                    LegSwitch = 1;
                    obj.stToe_pos = obj.swToe_pos;
                else
                    LegSwitch = 0;
                end
                
                if obj.initialized == 0
                    obj.initialized = 1;
                    LegSwitch = 1;
                end
                
                if obj.stanceLeg == -1
                    obj.GRF_sw_z = GRF_R(2);
                    obj.GRF_st_z = GRF_L(2);
                else
                    obj.GRF_sw_z = GRF_L(2);
                    obj.GRF_st_z = GRF_R(2);
                end
                
                
                rp_Origin2LT = - p_LeftToeJoint(q_pre); % relative position between hip and left toe
                rp_Origin2RT = - p_RightToeJoint(q_pre);
                
                if obj.stanceLeg == -1
                    origin_pos = rp_Origin2LT + obj.stToe_pos;
                    obj.swToe_pos = origin_pos - rp_Origin2RT;
                else
                    origin_pos = rp_Origin2RT + obj.stToe_pos;
                    obj.swToe_pos = origin_pos - rp_Origin2LT;
                end
                q = [origin_pos([1,2,3]); q_pre([4:end])];
                EulerRates = [dqyaw; dqpitch; dqroll];
                %% Log
                obj.t_prev = t;
                %% Construct EstStates
                EstStates.q = q;
                EstStates.s = obj.s;
                EstStates.t = t;
                
                EstStates.stanceLeg = obj.stanceLeg;
                EstStates.LegSwitch = LegSwitch;
                EstStates.GRF_z = [GRF_L(2);GRF_R(2)];
                EstStates.GRF_L = GRF_L;
                EstStates.GRF_R = GRF_R;
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
            Fs1R =- obj.Ks1 * qsR(1);
            Fs2R =- obj.Ks2 * qsR(2);
            Fs1L =- obj.Ks1 * qsL(1);
            Fs2L =- obj.Ks2 * qsL(2);
        end
        %% Default functions
        function setupImpl(obj)
            %SETUPIMPL Initialize System object.
        end % setupImpl
        
        function resetImpl(~)
            %RESETIMPL Reset System object states.
        end % resetImpl
        
        function [name_1, name_2, name_3]  = getInputNamesImpl(~)
            %GETINPUTNAMESIMPL Return input port names for System block
            name_1 = 'cassieOutputs';
            name_2 = 't_total';
            name_3 = 'isSim';
        end % getInputNamesImpl
        
        function [name_1, name_2, name_3] = getOutputNamesImpl(~)
            %GETOUTPUTNAMESIMPL Return output port names for System block
            name_1 = 'EstStates';
            name_2 = 'q';
            name_3 = 'EulerRates';
        end % getOutputNamesImpl
        
        % PROPAGATES CLASS METHODS ============================================
        function [EstStates, q, EulerRates] = getOutputSizeImpl(~)
            %GETOUTPUTSIZEIMPL Get sizes of output ports.
            EstStates = [1, 1];
            q = [20, 1];
            EulerRates = [3, 1];
        end % getOutputSizeImpl
        
        function [EstStates, q, EulerRates] = getOutputDataTypeImpl(~)
            %GETOUTPUTDATATYPEIMPL Get data types of output ports.
            EstStates = 'EstStatesBus';
            q = 'double';
            EulerRates = 'double';
        end % getOutputDataTypeImpl
        
        function [EstStates, q, EulerRates] = isOutputComplexImpl(~)
            %ISOUTPUTCOMPLEXIMPL Complexity of output ports.
            EstStates = false;
            q = false;
            EulerRates = false;
        end % isOutputComplexImpl
        
        function [EstStates, q, EulerRates] = isOutputFixedSizeImpl(~)
            %ISOUTPUTFIXEDSIZEIMPL Fixed-size or variable-size output ports.
            EstStates = true;
            q = true;
            EulerRates = true;
        end % isOutputFixedSizeImpl

    end % methods
end % classdef