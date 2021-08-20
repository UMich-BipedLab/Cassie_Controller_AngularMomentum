figure;
plot(Data.vy_com_tgy);
hold on;
plot(-1*Data.Lx_stToe_stTD0_kf/(32*0.75))
% plot(Data.dyf_this_tgy)
plot(Data.dyf_this_stTD0)
plot(Data.stanceLeg,'g-.');
grid on;

figure;
plot(Data.vx_com_tgy);
hold on;
plot(Data.Ly_stToe_stTD0_kf/(32*0.8))
grid on;