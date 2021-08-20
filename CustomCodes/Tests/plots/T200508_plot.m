figure;
plot(Data.qd_control_adjusted); 
hold on;
plot(Data.q0_control);
plot(Data.stanceLeg,'g-.') 
hold off;
legend('qd control 1', 'qd control 2',  'qd control 3',  'qd control 4',  'qd control 5',  'qd control 6',  'qd control 7',  'qd control 8',  ...
    'q0 control 1', 'q0 control 2',  'q0 control 3',  'q0 control 4',  'q0 control 5',  'q0 control 6',  'q0 control 7',  'q0 control 8')
ylim([-10,10])
plotbrowser('on')

figure;
plot(Data.dqd_control); 
hold on;
plot(Data.dq0_control);
plot(Data.stanceLeg,'g-.') 
hold off;
legend('dqd control 1', 'dqd control 2',  'dqd control 3',  'dqd control 4',  'dqd control 5',  'dqd control 6',  'dqd control 7',  'dqd control 8',  ...
    'dq0 control 1', 'dq0 control 2',  'dq0 control 3',  'dq0 control 4',  'dq0 control 5',  'dq0 control 6',  'dq0 control 7',  'dq0 control 8')
ylim([-10,10])
plotbrowser('on')

figure;
plot(Data.u)
hold on;
plot(100*Data.stanceLeg,'g-.') 
legend('u 1', 'u 2',  'u 3',  'u 4',  'u 5',  'u 6',  'u 7',  'u 8', 'u 9', 'u 10')
ylim([-500,500])
plotbrowser('on')

%%
figure;
plot(Data.hr);
hold on;
plot(Data.hr_recover)
hold off
legend('hr 1', 'hr 2',  'hr 3',  'hr 4',  'hr 5',  'hr 6',  'hr 7',  'hr 8',  ...
    'hr recover 1', 'hr recover 2',  'hr recover 3',  'hr recover 4',  'hr recover 5',  'hr recover 6',  'hr recover 7',  'hr recover 8')

figure;
plot(Data.hr);
hold on;
plot(Data.h0)
hold off
legend('hr 1', 'hr 2',  'hr 3',  'hr 4',  'hr 5',  'hr 6',  'hr 7',  'hr 8',  ...
    'h0 1', 'h0 2',  'h0 3',  'h0 4',  'h0 5',  'h0 6',  'h0 7',  'h0 8')


