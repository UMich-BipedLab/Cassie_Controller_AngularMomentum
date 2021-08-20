% RemoteSpoofer
classdef RemoteSpoofer < matlab.System & matlab.system.mixin.Propagates %#codegen
    
    % PROTECTED PROPERTIES ==================================================
    properties (Access = protected)
    end % properties
    
    % Private variables
    properties(Access = private)
    end
    
    % Pre-computed constants
    properties(Access = private, Constant)
    end
    
    % PROTECTED METHODS =====================================================
    methods (Access = protected)
        
        function setupImpl(obj)
            %SETUPIMPL Initialize System object.
        end % setupImpl
        
        function [cassieOutputs] = stepImpl(obj, cassieOutputs,t)
            
            RadioButton = RadioChannelToButton(cassieOutputs.pelvis.radio.channel);
            

            % Height =======================================================
            
            RadioButton.LSA = 1;
            % Longitudinal Velocity ========================================
            RadioButton.LVA = 3;
            RadioButton.SGA = -1;
            % Turning speed ================================================
%             if t > 10
%                 RadioButton.RHA = 1;
%             end
            % Step Frequency ===============================================
            RadioButton.RSA = 0.5;
            % Foot Width
            RadioButton.S2A = 0;
            % direct up and down
            RadioButton.SCA = 0;
            % foot clearance
            RadioButton.SBA = -1;
            
            cassieOutputs.pelvis.radio.channel = RadioButtonToChannel(RadioButton);

        end % stepImpl
        
        %% Define Outputs
        function resetImpl(~)
            %RESETIMPL Reset System object states.
        end % resetImpl
        
        function out = getOutputSizeImpl(~)
            %GETOUTPUTSIZEIMPL Get sizes of output ports.
            out = [1, 1];           
        end % getOutputSizeImpl
        
        function out = getOutputDataTypeImpl(~)
            %GETOUTPUTDATATYPEIMPL Get data types of output ports.
            out = 'CassieOutBus';
        end % getOutputDataTypeImpl
        
        function out = isOutputComplexImpl(~)
            %ISOUTPUTCOMPLEXIMPL Complexity of output ports.
            out = false;
        end % isOutputComplexImpl
        
        function out= isOutputFixedSizeImpl(~)
            %ISOUTPUTFIXEDSIZEIMPL Fixed-size or variable-size output ports.
            out = true;
        end % isOutputFixedSizeImpl
    end % methods
end % classdef

