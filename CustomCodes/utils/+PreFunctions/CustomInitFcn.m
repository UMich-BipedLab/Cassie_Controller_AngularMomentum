sample_time = 0.0005;
sim_step_length = 0.0005;
SpringDamping = 0.2;
% Create Data Bus
Data = PreFunctions.Construct_Data;
cassieDataBusInfo = Simulink.Bus.createObject(Data);
cassieDataBus = eval(cassieDataBusInfo.busName);

EstStates = PreFunctions.Construct_EstStates;
EstStatesBusInfo = Simulink.Bus.createObject(EstStates);
EstStatesBus = eval(EstStatesBusInfo.busName);

IRC = PreFunctions.Construct_IRC;
IRCBusInfo = Simulink.Bus.createObject(IRC);
IRCBus = eval(IRCBusInfo.busName);


StateEstimator_Data = PreFunctions.Construct_StateEstimator_Data;
StateEstimator_DataBusInfo = Simulink.Bus.createObject(StateEstimator_Data);
StateEstimator_DataBus = eval(StateEstimator_DataBusInfo.busName);

DML = load('DynamicMatrixLibrary_Sim_v1.mat');
DynamicMatrixLibrary = DML.DynamicMatrixLibrary;
cassieDynamicMatrixLibraryBusInfo = Simulink.Bus.createObject(DynamicMatrixLibrary);
cassieDynamicMatrixLibraryBus = eval(cassieDynamicMatrixLibraryBusInfo.busName);

PreFunctions.ParamInit;
PreFunctions.PolyRegressionFilterInit;

%% Covariance
fname = which('all_buses.m');
delete(fname)
Simulink.Bus.save([root_dir '\all_buses.m'])
