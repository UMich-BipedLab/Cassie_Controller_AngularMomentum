[Fe,t] = YToolkits.timeseries2vector(Data.Fe);
[qsL,t] = YToolkits.timeseries2vector(Data.qsL);
[q4,t] = YToolkits.timeseries2vector(Data.q_thigh_knee_L);
[q5,t] = YToolkits.timeseries2vector(Data.q_knee_shin_L);
[q6,t] = YToolkits.timeseries2vector(Data.q_shin_tarsus_L);

qs2_est = [1.2174   1.7088   1.2122]*[q4;q5;q6] - 1.2122*deg2rad(13);
% qs2_est = [1   1   1]*[q4;q5;q6] - deg2rad(13);

[left_knee_spring_p,t2] = YToolkits.timeseries2vector(Left_knee_spring_p);
[left_heel_spring_p,t2] = YToolkits.timeseries2vector(Left_heel_spring_p);
left_knee_spring_p = interp1(t2,left_knee_spring_p,t);
left_heel_spring_p = interp1(t2,left_heel_spring_p,t);

left_knee_diff = -1500*left_knee_spring_p - Fe(23,:)';
left_heel_diff = 1250*left_heel_spring_p - Fe(24,:)';

figure; plot(t,Fe(23:24,:))
hold on;
plot(t,-1500*left_knee_spring_p)
plot(t,-1250*left_heel_spring_p)
grid on;

figure;
plot(t,left_knee_diff);
hold on
plot(t,left_heel_diff);
grid on

figure;
plot(t,qsL)
hold on;
plot(Left_knee_spring_p);plot(-1*Left_heel_spring_p);

figure;
plot(t,qs2_est)
hold on;
plot(-1*Left_heel_spring_p);



% figure;plot(t,-1250*qsL(2,:));hold on;plot(1250*left_heel_spring_p)