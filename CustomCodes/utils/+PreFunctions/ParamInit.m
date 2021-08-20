%%General
c_KsL_1 = +YToolkits.ifelse(isSim,1500,2422);
c_KsL_2 = +YToolkits.ifelse(isSim,800,828);
c_KsR_1 = +YToolkits.ifelse(isSim,1500,1943);
c_KsR_2 = +YToolkits.ifelse(isSim,800,887);
%% controller
% PD gain on Joints (walk)
c_Kp_pitch = 400;
c_Kd_pitch = 20;
c_Kp_roll = 900;
c_Kd_roll = 30;
c_Kp_yaw = 200;
c_Kd_yaw = 4;

c_Kp_abduction = 800;
c_Kp_rotation = 400;
c_Kp_thigh = 500;
c_Kp_knee = 500;
c_Kp_toe = 10;

c_Kd_abduction = 10;
c_Kd_rotation = 4;
c_Kd_thigh = 15;
c_Kd_knee = 15;
c_Kd_toe = 2;

c_Kp_st_rotation = 200;
c_Kd_st_rotation = 4;

c_u_abduction_swing_cp = 10;
c_u_roll_cp = 40;
c_u_pitch_cp = -15;
c_u_knee_cp = 120;

% Linear Regression Filter

c_dop = 1;
c_history_length = 10;

% Covriance of motors, joints and IMU
c_CovPos_abduction = 1e-8;
c_CovPos_rotation = 1e-8;
c_CovPos_thigh = 1e-8;
c_CovPos_knee = 1e-8;
c_CovPos_toe = 1e-8;
c_CovPos_qj1 = 1e-8;
c_CovPos_qj2 = 1e-8;
c_CovPos_qj3 = 1e-8;

c_CovVel_abduction = LR_CovPos2CovVel(c_dop, c_history_length, sample_time, 0, c_CovPos_abduction);
c_CovVel_rotation = LR_CovPos2CovVel(c_dop, c_history_length, sample_time, 0, c_CovPos_rotation);
c_CovVel_thigh = LR_CovPos2CovVel(c_dop, c_history_length, sample_time, 0, c_CovPos_thigh);
c_CovVel_knee = LR_CovPos2CovVel(c_dop, c_history_length, sample_time, 0, c_CovPos_knee);
c_CovVel_toe = LR_CovPos2CovVel(c_dop, c_history_length, sample_time, 0, c_CovPos_toe);
c_CovVel_qj1 = LR_CovPos2CovVel(c_dop, c_history_length, sample_time, 0, c_CovPos_qj1);
c_CovVel_qj2 = LR_CovPos2CovVel(c_dop, c_history_length, sample_time, 0, c_CovPos_qj2);
c_CovVel_qj3 = LR_CovPos2CovVel(c_dop, c_history_length, sample_time, 0, c_CovPos_qj3);

c_Cov_AngularVelocity = 1e-3 *ones(3,1);

c_Cov_Orientation = 0e-8 *ones(4,1);
c_Cov_Euler = 1e-8 *ones(3,3);
c_Cov_Euler_rates = 1e-3 *ones(3,3);

c_Cov_Lx_LRToe_stTD0 = (0.5)^2;
c_Cov_Ly_LRToe_stTD0 = (0.5)^2;

c_Cov_rpx_LRToe_stTD0 = 0.1^2;
c_Cov_rpy_LRToe_stTD0 = 0.1^2;

c_stance_thre_lb = 100;
c_stance_thre_ub = 200;
c_stance_thre_lb_2 = 90;
c_stance_thre_ub_2 = 150;
c_toe_leave_ground_lb = 0.005;
c_toe_leave_ground_ub = 0.008;
        
c_IK_max_iter_num = 2;
c_rp_LRToe_fil_param = YToolkits.cutoff_frequency_to_alpha( 71.02 ,sample_time );
c_yaw_error_fil_param = YToolkits.cutoff_frequency_to_alpha( 6.43,sample_time );
c_adjusted_desired_roll_angle_fil_param = YToolkits.cutoff_frequency_to_alpha( 6.43,sample_time );
c_use_dq_AR_for_PD = 1;
c_lateral_correction_gain= 0.5;
c_torque_transition_time = 0.02;
c_knee_cp_time = 0.1;
c_st_rotation_torque_ramp_time = 0.05;
c_one_step_max_vel_gain = 0.8;
c_max_st_rotation_motor_torque = 10;

c_com_x_offset = +YToolkits.ifelse(isSim,0,0);
c_Vy_tgd_offset = +YToolkits.ifelse(isSim,0,0);
c_Vx_tgd_offset = 0;



%% for state estimator
c_Cov_LinearAccelerator_xy = (0.05)^2;
c_Cov_LinearAccelerator_z = (0.05)^2;
c_Cov_rpx_LRToe_body = 0.01^2;
c_Cov_rpy_LRToe_body = 0.01^2;
c_Cov_rpz_LRToe_body = 0.01^2;

c_vx_slide_toe_1 = 2;
c_vx_slide_toe_2 = 0.02;
c_Tx_slide_toe_2 = 0.05;

c_vy_slide_toe_1 = 2;
c_vy_slide_toe_2 = 0.02;
c_Ty_slide_toe_2 = 0.05;

c_vz_slide_toe_1 = 2;
c_vz_slide_toe_2 = 0.02;
c_Tz_slide_toe_2 = 0.05;
%% for IRC
c_max_turn_per_sec = 0.5;
c_Vx_tgd_fil_param = YToolkits.cutoff_frequency_to_alpha( 0.0955 , sample_time );
c_Vy_tgd_avg_fil_param = YToolkits.cutoff_frequency_to_alpha( 0.9564 , sample_time );
c_H_fil_param = YToolkits.cutoff_frequency_to_alpha( 0.0955 , sample_time );
c_desired_com2stToe_lateral_fil_param = YToolkits.cutoff_frequency_to_alpha( 0.0955 , sample_time );
%% Run only when new tunable parameter is created
if 1
    S = whos('c_*');
    root_block = 'FG_RealTime';
    FG_RealTime;
    % Make the base workspace variables tunable
    NameList = [];
    StorageClassList = [];
    TypeQualifierList = [];
    for i = 1:length(S)
        NameList = [NameList, S(i).name,','];
        StorageClassList = [StorageClassList, 'Auto,'];
        TypeQualifierList = [TypeQualifierList,','];
    end
    NameList = NameList(1:end-1); % Remove the last ','
    StorageClassList = StorageClassList(1:end-1);
    TypeQualifierList = TypeQualifierList(1:end-1);
    
    set_param(root_block,'TunableVars',NameList);
    set_param(root_block,'TunableVarsStorageClass',StorageClassList);
    set_param(root_block, 'TunableVarsTypeQualifier',TypeQualifierList);


    %% Assign the Model WorkSpace variables of the referenced model to the block parameters block 5
    sys_name = 'Cassie_CommandBlock_Library_5';
    load_system(sys_name);
    set_param(sys_name,'Lock','off')
    % Controller Block
    num_no_c_params = 1;
    parameters = get_param([ sys_name '/Subsystem/MATLAB System1'],'DialogParameters');
    fields = fieldnames(parameters);
    for i = 1:length(fields)-num_no_c_params - 1
        set_param([ sys_name '/Subsystem/MATLAB System1'],fields{i},['c_' fields{i}]);
    end
    parameters = get_param([ sys_name '/Subsystem/MATLAB System3'],'DialogParameters');
    fields = fieldnames(parameters);
    for i = 1:length(fields)-num_no_c_params - 1
        set_param([ sys_name '/Subsystem/MATLAB System3'],fields{i},['c_' fields{i}]);
    end
    %Estimator Block
%     num_no_c_params = 1;
%     parameters = get_param([ sys_name '/Subsystem/MATLAB System'],'DialogParameters');
%     fields = fieldnames(parameters);
%     for i = 1:length(fields)-num_no_c_params - 1
%         set_param([ sys_name '/Subsystem/MATLAB System'],fields{i},['c_' fields{i}]);
%     end
    
    num_no_c_params = 1;
    parameters = get_param([ sys_name '/Subsystem/MATLAB System6'],'DialogParameters');
    fields = fieldnames(parameters);
    for i = 1:length(fields)-num_no_c_params - 1
        set_param([ sys_name '/Subsystem/MATLAB System6'],fields{i},['c_' fields{i}]);
    end
    %IRC Block
    num_no_c_params = 1;
    parameters = get_param([ sys_name '/Subsystem/MATLAB System4'],'DialogParameters');
    fields = fieldnames(parameters);
    for i = 1:length(fields)-num_no_c_params - 1
        set_param([ sys_name '/Subsystem/MATLAB System4'],fields{i},['c_' fields{i}]);
    end
    %% Assign the Model WorkSpace variables of the referenced model to the block parameters block 6
    sys_name = 'Cassie_CommandBlock_Library_6';
    load_system(sys_name);
    set_param(sys_name,'Lock','off')
    % Controller Block
    num_no_c_params = 1;
    parameters = get_param([ sys_name '/Subsystem/MATLAB System1'],'DialogParameters');
    fields = fieldnames(parameters);
    for i = 1:length(fields)-num_no_c_params - 1
        set_param([ sys_name '/Subsystem/MATLAB System1'],fields{i},['c_' fields{i}]);
    end
    parameters = get_param([ sys_name '/Subsystem/MATLAB System3'],'DialogParameters');
    fields = fieldnames(parameters);
    for i = 1:length(fields)-num_no_c_params - 1
        set_param([ sys_name '/Subsystem/MATLAB System3'],fields{i},['c_' fields{i}]);
    end
    %Estimator Block
    num_no_c_params = 1;
    parameters = get_param([ sys_name '/Subsystem/MATLAB System'],'DialogParameters');
    fields = fieldnames(parameters);
    for i = 1:length(fields)-num_no_c_params - 1
        set_param([ sys_name '/Subsystem/MATLAB System'],fields{i},['c_' fields{i}]);
    end
    %IRC Block
    num_no_c_params = 1;
    parameters = get_param([ sys_name '/Subsystem/MATLAB System4'],'DialogParameters');
    fields = fieldnames(parameters);
    for i = 1:length(fields)-num_no_c_params - 1
        set_param([ sys_name '/Subsystem/MATLAB System4'],fields{i},['c_' fields{i}]);
    end
end

% S = whos('c_*');
% if isSim == 1
%     for i = 1:length(S)
%     assignin('base',['m' S(i).name],eval(S(i).name));
%     end
% end
% 
% %% Run only when new tunable parameter is created
% if 0
% referenced_block = 'YukaiRealTime_Block';
% root_block = 'YukaiRealTime_2';
% 
% % Make the base workspace variables tunable
% NameList = [];
% StorageClassList = [];
% TypeQualifierList = [];
% for i = 1:length(S)
%     NameList = [NameList, S(i).name,','];
%     StorageClassList = [StorageClassList, 'Auto,'];
%     TypeQualifierList = [TypeQualifierList,','];
% end
% NameList = NameList(1:end-1); % Remove the last ','
% StorageClassList = StorageClassList(1:end-1);
% TypeQualifierList = TypeQualifierList(1:end-1);
% 
% set_param(root_block,'TunableVars',NameList);
% set_param(root_block,'TunableVarsStorageClass',StorageClassList);
% set_param(root_block, 'TunableVarsTypeQualifier',TypeQualifierList);
% 
% 
% % Create Model WorkSpace variables in the referenced block. And make them Model argument
% load_system(referenced_block);
% mdlWks = get_param(referenced_block,'ModelWorkspace');
% mdlWksNameList = [];
% for i = 1:length(S)
%     assignin(mdlWks,['m' S(i).name],eval(S(i).name)); % this line is used to create the model workspace variable, the assigend value is useless
%     mdlWksNameList = [mdlWksNameList,'m', S(i).name,','];
% end
% mdlWksNameList = mdlWksNameList(1:end - 1);% Remove the last ','
% set_param(referenced_block,'ParameterArgumentNames',mdlWksNameList);
% 
% % In the Root Block assign the tunable parameters to the model argument
% load_system(root_block);
% for i = 1:length(S)
%     set_param('YukaiRealTime_2/Model','ParameterArgumentValues',struct(['m' S(i).name],S(i).name))
% end
% 
% % Assign the Model WorkSpace variables of the referenced model to the block parameters
% load_system('FG_Controller_Library');
% set_param('FG_Controller_Library','Lock','off')
% parameters = get_param('FG_Controller_Library/Subsystem/MATLAB System1','DialogParameters');
% fields = fieldnames(parameters);
% for i = 1:length(fields)-1
%     set_param('FG_Controller_Library/Subsystem/MATLAB System1',fields{i},['mc_' fields{i}]);
% end
% end