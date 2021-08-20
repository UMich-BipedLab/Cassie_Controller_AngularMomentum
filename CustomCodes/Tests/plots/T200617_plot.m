figure; 
plot(t,vx_avg,'LineWidth',3)
hold on;
plot(t,vx,'g')
hold off;
title('Velocity in Longitudinal Direction')
xlabel('Time')
ylabel('Vx')
legend('average velocity in a 0.8s window', 'instant velocity')
grid on;
ax1 = gca;
dcmObj = datacursormode; set(dcmObj, 'UpdateFcn', @NewCallback)


figure;
plot(log.Data.t, log.Data.qd_control_adjusted(:,5))
hold on
plot(log.Data.t, log.Data.q10)
hold off
legend('st knee reference','left knee actual')
grid on;
ax2 = gca;
dcmObj = datacursormode; set(dcmObj, 'UpdateFcn', @NewCallback)

figure;
plot(log.Data.t, log.Data.qd_control_adjusted(:,6))
hold on;
plot(log.Data.t, log.Data.q14)
hold off;
legend('sw abduction reference', 'right abduction actual')
grid on;
ax3 = gca;
dcmObj = datacursormode; set(dcmObj, 'UpdateFcn', @NewCallback)

figure;
plot(log.Data.t, log.Data.qd_control_adjusted(:,7))
hold on;
plot(log.Data.t, log.Data.q16)
hold off;
legend('sw thigh reference', 'right thigh actual')
grid on;
ax4 = gca;
dcmObj = datacursormode; set(dcmObj, 'UpdateFcn', @NewCallback)

figure;
plot(log.Data.t, log.Data.qd_control_adjusted(:,8))
hold on;
plot(log.Data.t, log.Data.q17)
hold off;
legend('sw knee reference', 'right knee actual')
grid on;
ax5 = gca;
dcmObj = datacursormode; set(dcmObj, 'UpdateFcn', @NewCallback)


figure;
plot(log.Data.t, log.Data.dq3)
legend('IMU vz')
grid on;
ax6 = gca;
dcmObj = datacursormode; set(dcmObj, 'UpdateFcn', @NewCallback)

figure;
plot(log.Data.t, log.Data.qd_control_adjusted(:,6))
hold on;
plot(log.Data.t, log.Data.q7)
hold off;
legend('sw knee reference', 'left knee actual')
grid on;
ax7 = gca;
dcmObj = datacursormode; set(dcmObj, 'UpdateFcn', @NewCallback)

figure;
plot(log.Data.t, log.Data.qd_control_adjusted(:,5))
hold on;
plot(log.Data.t, log.Data.q17)
hold off;
legend('st knee reference', 'right knee actual')
grid on;
ax8 = gca;
dcmObj = datacursormode; set(dcmObj, 'UpdateFcn', @NewCallback)


linkaxes([ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8],'x')

%%
figure; 
plot(log.Data.t,log.Data.qsL_1,'b'); 
hold on; 
plot(log.Data.t,log.Data.qsR_1,'g'); 
plot(log.Data.t,log.Data.tau_kneespring_st_est/(-1285),'b.');

plot(log.Data.t,log.Data.tau_kneespring_st_est/(-1420),'g.');
hold off
legend('left knee deflection', 'right knee deflection', 'est_ st kneespring deflection(left coefficient)', 'est_ st kneespring deflection(right coefficient)')

figure; 
plot(log.Data.t,log.Data.qsL_2,'b'); 
hold on; 
plot(log.Data.t,log.Data.qsR_2,'g'); 
plot(log.Data.t,log.Data.tau_heelspring_st_est/(-845),'b.');
plot(log.Data.t,log.Data.tau_heelspring_st_est/(-755),'g.');
hold off
legend('left heel deflection', 'right heel deflection', 'est_ st heelspring deflection(left coefficient)', 'est_ st heelspring deflection(right coefficient)')