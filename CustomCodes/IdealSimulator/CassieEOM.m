%PDCONTROLLER PD controller
%
% Syntax:
%   Implements a MATLAB System block for use in Simulink with inputs:
%     cassieOutputs (CassieOutBus)
%   and outputs:
%     userInputs (CassieUserInBus)
%     userLog (double(20,1))
%
% Description:
%   This class implements a simple PD controller for basic robot systems
%   checking. The gains can be edited on the fly using the system block dialog
%   in the simulator or using Simulink Real-Time Explorer interface when on
%   hardware. This is useful for checking and tuning joint level PD gains. The
%   desired position corresponds to a crouched standing pose.

% Copyright 2017-2018 Agility Robotics

classdef CassieEOM < ...
        matlab.System & ...
        matlab.system.mixin.Propagates & ...
        matlab.system.mixin.SampleTime %#codegen
    
    % CONSTANT PROPERTIES ========================================================
    
    
    % PUBLIC PROPERTIES ==========================================================
    
    % PROTECTED PROPERTIES =======================================================
    
    % PROTECTED METHODS ==========================================================
    methods (Access = protected)
        % SYSTEM CLASS METHODS =====================================================
        function setupImpl(obj)
            
        end % setupImpl
        function dx  = stepImpl(obj,x,u,Fg)
            qall = x(1:20);
            dqall = x(21:40);
            dx = zeros(40,1);
            M = EnertiaMatrix(qall);
            C = CoriolisTerm(qall,dqall);
            G = GravityVector(qall);
            B = eye(14); B = B(:,[1,2,3,4,7,8,9,10,11,14]); B = [zeros(6,10);B];
            
            % Cartesian Jacobian on foot( and derivative)
            J_BL = J_LeftToeBottomBack(qall);
            J_BR = J_RightToeBottomBack(qall);
            J_FL = J_LeftToeBottomFront(qall);
            J_FR = J_RightToeBottomFront(qall);
            % Jacobian for foot constraint
            Jg = [J_BL;J_BR;J_FL;J_FR];
            [Jgd1,Jgd2] = size(Jg);
            % Jacobian for spring
            Js_L = zeros(2,20); Js_L(1,11) = 1; Js_L(2,[10 12]) = 1;
            Js_R = zeros(2,20); Js_R(1,18) = 1; Js_R(2,[17 19]) = 1;
            Js = [Js_L; Js_R];
            [Jsd1,Jsd2] = size(Js);
            % Jacobian for fixing torso
            Jt = [eye(6), zeros(6,14)];
            [Jtd1,Jtd2] = size(Jt);
            
            % Extended Model
            Me = [M, -Js'; Js, zeros(Jsd1,Jsd1)];
            He = [-G-C;zeros(Jsd1,1)];
            Be = [B;zeros(Jsd1,10)];
            Jg_Transpose_e = [Jg';zeros(Jsd1,Jgd1)];
            % M*dqall -C -G = B*u+Jg'*Fg;
            Fe = Me^-1*(-He+Be*u+Jg_Transpose_e*Fg);
            ddqall = Fe(1:20);
            
            dx = [dqall;ddqall];
        end % stepImpl
        
        function resetImpl(~)
            %RESETIMPL Reset System object states
        end % resetImpl
        
        function [name_1, name_2, name_3] = getInputNamesImpl(~)
            %GETINPUTNAMESIMPL Return input port names for System block
            name_1 = 'x';
            name_2 = 'u';
            name_3 = 'Fg';
            
        end % getInputNamesImpl
        
        function [name_1] = getOutputNamesImpl(~)
            %GETOUTPUTNAMESIMPL Return output port names for System block
            name_1 = 'dx';
        end % getOutputNamesImpl
        
        % PROPAGATES CLASS METHODS =================================================
        function [sz_1] = getOutputSizeImpl(~)
            %GETOUTPUTSIZEIMPL Get sizes of output ports
            sz_1 = [40, 1];
        end % getOutputSizeImpl
        
        function [dt_1] = getOutputDataTypeImpl(~)
            %GETOUTPUTDATATYPEIMPL Get data types of output ports
            dt_1 = 'double';
        end % getOutputDataTypeImpl
        
        function [cp_1] = isOutputComplexImpl(~)
            %ISOUTPUTCOMPLEXIMPL Complexity of output ports
            cp_1 = false;
        end % isOutputComplexImpl
        
        function [flag_1] = isOutputFixedSizeImpl(~)
            %ISOUTPUTFIXEDSIZEIMPL Fixed-size or variable-size output ports
            flag_1 = true;
        end % isOutputFixedSizeImpl
    end % protected methods
end % classdef
