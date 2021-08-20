function [qc_index,motor_index,Kp_PBC,Kd_PBC,M_diag] = ControlTarget(obj)
% if obj.stanceLeg == -1 % on ground torso control
%     qc_index = [6;8;5;10;14;15;16;17];
%     motor_index = [1;2;3;4;6;7;8;9];
%     Kp_PBC = [obj.Kp_roll; obj.Kp(2); obj.Kp_pitch; obj.Kp(4);obj.Kp(6:9)];
%     Kd_PBC = [obj.Kd_roll; obj.Kd(2); obj.Kd_pitch; obj.Kd(4);obj.Kd(6:9)];
%     M_diag = diag([1.78,1.67,2.09,0.80,2.92,8.7,4.4,5.1]);
% else
%     qc_index = [7;8;9;10;6;15;5;17];
%     motor_index = [1;2;3;4;6;7;8;9];
%     Kp_PBC = [obj.Kp(1:4); obj.Kp_roll; obj.Kp(7); obj.Kp_pitch; obj.Kp(9)];
%     Kd_PBC = [obj.Kd(1:4); obj.Kd_roll; obj.Kd(7); obj.Kd_pitch; obj.Kd(9)];
%     M_diag = diag([2.92,8.7,4.4,5.1,1.78,1.67,2.09,0.80]);
% end

if obj.stanceLeg == -1 % on ground torso control
    qc_index = [5;6;8;10;14;15;16;17];
    motor_index = [3;1;2;4;6;7;8;9];
    Kp_PBC = [obj.Kp_pitch; obj.Kp_roll; obj.Kp(2); obj.Kp(4);obj.Kp(6:9)];
    Kd_PBC = [obj.Kd_pitch; obj.Kd_roll; obj.Kd(2); obj.Kd(4);obj.Kd(6:9)];
    M_diag = diag([1.78,1.67,2.09,0.80,2.92,8.7,4.4,5.1]);
else
    qc_index = [5;6;7;8;9;10;15;17];
    motor_index = [8;6;1;2;3;4;7;9];
    Kp_PBC = [obj.Kp_pitch; obj.Kp_roll; obj.Kp(1:4);  obj.Kp(7);  obj.Kp(9)];
    Kd_PBC = [obj.Kd_pitch; obj.Kd_roll; obj.Kd(1:4);  obj.Kd(7);  obj.Kd(9)];
    M_diag = diag([2.92,8.7,4.4,5.1,1.78,1.67,2.09,0.80]);
end


%                     % choice of state to control (in air)
%                     if obj.stanceLeg == -1
% %                         qc_index = [7;8;9;10;13;14;15;16;17;20];
% %                         h_index = 1:10;
%                         qc_index = [7;8;9;10;14;15;16;17];
%                         h_index = [1;2;3;4;6;7;8;9];
%                     else
% %                         qc_index = [7;8;9;10;13;14;15;16;17;20];
% %                         h_index = 1:10;
%                         qc_index = [7;8;9;10;14;15;16;17];
%                         h_index = [1;2;3;4;6;7;8;9];
%                     end

% % choice of state to control (torso fixed except z)
%                     if obj.stanceLeg == -1
%                         qc_index = [10;14;15;16;17];
%                         motor_index = [4;6;7;8;9];
%                         Kp_PBC = [obj.Kp(4);obj.Kp(6:9)];
%                         Kd_PBC = [obj.Kd(4);obj.Kd(6:9)];
%                         M_diag = diag([1.78,1.67,2.09,0.80,2.92,8.7,4.4,5.1]);
%                     else
%                         qc_index = [7;8;9;10;17];
%                         motor_index = [1;2;3;4;9];
%                         Kp_PBC = [obj.Kp(1:4);obj.Kp(9)];
%                         Kd_PBC = [obj.Kd(1:4);obj.Kd(9)];
%                         M_diag = diag([1.78,1.67,2.09,0.80,2.92,8.7,4.4,5.1]);
%                     end
%                     
                    
%                         qc_index = [7;15;17;8;9;10;14;16];
%                         h_index = [1;2;3;4;6;7;8;9];
%                         qc_index = [7;9;10;14;16;17];
%                         h_index = [1;3;4;6;8;9];
%                         qc_index = [9;10;16;17];
%                         h_index = [3;4;8;9];
%                         qc_index = [7;10;13;14;17;20];
%                         h_index = [1;4;5;6;9;10];
%                         qc_index = [10;17];
%                         h_index = [4;9];
%                         qc_index = [7;8;9;10;14;15;16;17];
%                         h_index = [1;2;3;4;6;7;8;9];
% choice of state to control (on ground)
%                     if obj.stanceLeg == -1
%                         qc_index = [5;6;8;10;14;15;16;17];
%                         h_index = [2;4;6;7;8;9];
%                         Jg = Jg_L;
%                     else
%                         qc_index = [5;6;7;8;9;10;15;17];
%                         h_index = [1;2;3;4;7;9];
%                         Jg = Jg_R;
%                     end



end

