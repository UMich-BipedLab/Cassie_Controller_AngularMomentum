%Yukai controller.

classdef RadioCommandInterpreter <matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime %#codegen
    % PROTECTED PROPERTIES ====================================================
    properties
        max_turn_per_sec;
        
        Vx_tgd_fil_param;
        Vy_tgd_avg_fil_param;
        H_fil_param;
        desired_com2stToe_lateral_fil_param;
        
        sample_time;
    end
    properties(Constant, Access = private)

    end
    properties (Access = private)
        Vx_tgd_fil = 0;
        Vy_tgd_avg_fil = 0;
        H_fil = 0.8;
        desired_com2stToe_lateral_fil = -0.15;
        step_time_fil = 0.4;
        CL_fil = 0.1;
    end
    properties(Access = private) 

    end
    properties(Access = private) 

    end
    % PROTECTED METHODS =====================================================
    methods (Access = protected)
        
        function [IRC] = stepImpl(obj,cassieOutputs,t_total,isSim)
            %% Initialize
            IRC = PreFunctions.Construct_IRC();
            RadioButton = RadioChannelToButton(cassieOutputs.pelvis.radio.channel);
            
            %% Calculation
            
            
            Vx_tgd = (RadioButton.SGA + 2) * RadioButton.LVA ;
            Vy_tgd_avg = -0.4*RadioButton.LHA;
            obj.Vx_tgd_fil = YToolkits.first_order_filter(obj.Vx_tgd_fil, Vx_tgd, obj.Vx_tgd_fil_param);
            obj.Vy_tgd_avg_fil = YToolkits.first_order_filter(obj.Vy_tgd_avg_fil, Vy_tgd_avg, obj.Vy_tgd_avg_fil_param);
            H = 1/5 * RadioButton.LSA + 0.6;
            obj.H_fil = YToolkits.first_order_filter(obj.H_fil, H, obj.H_fil_param);
            desired_com2stToe_lateral = -0.15 - 0.2*RadioButton.S2A;
            obj.desired_com2stToe_lateral_fil = YToolkits.first_order_filter(obj.desired_com2stToe_lateral_fil, desired_com2stToe_lateral, obj.desired_com2stToe_lateral_fil_param);
            
            % no foot placement?
            if RadioButton.SCA == 1
                IRC.direct_up_down = 1;
            else
                IRC.direct_up_down = 0;
            end
            
            if RadioButton.RSA < 0 
                step_time = 0.4 - 0.4*RadioButton.RSA;
            else
                step_time = 0.4 - 0.2*RadioButton.RSA;
            end
            if RadioButton.RSA < -0.98
                if IRC.direct_up_down == 1
                    step_time = 1000;
                end
            end
            
            CL = 0.1;
            switch RadioButton.SBA 
                case -1
                    CL = 0.1;
                case 0
                    CL = 0.15;
                case 1
                    CL = 0.2;
            end
            obj.CL_fil = YToolkits.first_order_filter(obj.CL_fil, CL, 0.0003);
            
            %% Assign to IRC
            % Boolean Command =============================================
            % sto
            if RadioButton.SAA == 1
                IRC.motor_power = 1;
            else
                IRC.motor_power = 0;
            end
            % Reset IMU KF?
            if RadioButton.SDA == 1
                IRC.reset_IMU_KF = 1;
                IRC.reset_bias = 1;
            else
                IRC.reset_IMU_KF = 0;
                IRC.reset_bias = 0;
            end

            
            % Continuous Command ==========================================
            % turning
            IRC.turn_rps = - obj.max_turn_per_sec*RadioButton.RHA;
            % step frequency
            IRC.step_time = step_time;
            IRC.ds = 1/IRC.step_time;
            % foward/backward velocity
            IRC.Vx_tgd = obj.Vx_tgd_fil + RadioButton.S1A;
            IRC.Vy_tgd_avg = obj.Vy_tgd_avg_fil;
            % COM height
            IRC.H = obj.H_fil;
            % leg width
            IRC.desired_com2stToe_lateral = obj.desired_com2stToe_lateral_fil;
            % foot clearance
            IRC.CL = obj.CL_fil;
        end % stepImpl
        
        %% Default functions
        function setupImpl(obj)
            %SETUPIMPL Initialize System object.
        end % setupImpl
        
        function resetImpl(~)
            %RESETIMPL Reset System object states.
        end % resetImpl

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
        function [IRC] = getOutputSizeImpl(~)
            %GETOUTPUTSIZEIMPL Get sizes of output ports.
            IRC = [1, 1];
        end % getOutputSizeImpl
        
        function [IRC] = getOutputDataTypeImpl(~)
            %GETOUTPUTDATATYPEIMPL Get data types of output ports.
            IRC = 'IRCBus';
        end % getOutputDataTypeImpl
        
        function [IRC] = isOutputComplexImpl(~)
            %ISOUTPUTCOMPLEXIMPL Complexity of output ports.
            IRC = false;
        end % isOutputComplexImpl
        
        function [IRC] = isOutputFixedSizeImpl(~)
            %ISOUTPUTFIXEDSIZEIMPL Fixed-size or variable-size output ports.
            IRC = true;
        end % isOutputFixedSizeImpl

    end % methods
end % classdef
