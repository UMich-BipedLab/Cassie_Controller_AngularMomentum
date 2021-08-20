figure;
plot(Data_perfect.Lx_stToe);
hold on;
plot(Data.Lx_stToe);
plot(Data.Lx_stToe_stTD0_kf);
plot(Data.stanceLeg,'g-.');
hold off
legend('Lx stToe true', 'Lx stToe obs', 'Lx stToe kf')

figure;
plot(Data_perfect.Ly_stToe);
hold on;
plot(Data.Ly_stToe);
plot(Data.Ly_stToe_stTD0_kf);
plot(Data.stanceLeg,'g-.');
hold off
legend('Ly stToe true', 'Ly stToe obs', 'Ly stToe kf')

figure;plot(Data_perfect.v_stT)

figure;
plot(Data_perfect.dq1);
hold on;
plot(Data.dq1);
plot(Data.stanceLeg,'g-.');
hold off

Lx_stToe_stTD0_obs

figure;
% plot(Data_perfect.Lx_stToe_stTD0_obs);
hold on;
plot(Data.Lx_stToe_stTD0_obs);
plot(Data.Lx_stToe_stTD0_kf);
plot(Data.stanceLeg,'g-.');
hold off
legend('Lx stToe true', 'Lx stToe obs', 'Lx stToe kf')

figure;
% plot(Data_perfect.Ly_stToe_stTD0_obs);
hold on;
plot(Data.Ly_stToe_stTD0_obs);
plot(Data.Ly_stToe_stTD0_kf);
plot(Data.stanceLeg,'g-.');
hold off
legend('Ly stToe true', 'Ly stToe obs', 'Ly stToe kf')