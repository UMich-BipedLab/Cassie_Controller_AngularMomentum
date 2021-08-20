figure;
plot(Data.qsL_1);
hold on;
plot(-1* Data.tau_kneespring_st_est/1500)
plot(Data.stanceLeg,'g-.')
hold off;

figure;
plot(Data.qsR_2);
hold on;
plot(-1* Data.tau_kneespring_st_est/1500)
plot(Data.stanceLeg,'g-.')
hold off;
