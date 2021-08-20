gait = GaitLibrary{9};
X_states = [gait.opt.states{1}.x([1:6,8:14,16:22],:), ...
            gait.opt.states{3}.x([1:6,8:14,16:22],:)];
two_step_length = X_states(1,end) - X_states(1,1) ;
X_states_total = [];       
for i = 1:27
    X_states_total = [X_states_total, X_states];
    X_states(1,:) = X_states(1,:)+two_step_length;
end
T_states = 0:0.02:21.6;

CA = CassieAnimation;
CA.Initializaztion(X_states_total,T_states');