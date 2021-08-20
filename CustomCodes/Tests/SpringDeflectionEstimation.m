[q4,t] = YToolkits.timeseries2vector(Data.q_thigh_knee_L);
[q5,t] = YToolkits.timeseries2vector(Data.q_knee_shin_L);
[q6,t] = YToolkits.timeseries2vector(Data.q_shin_tarsus_L);
[qHeel,t2] = YToolkits.timeseries2vector(Left_heel_spring_p);
q4 = q4';q5 = q5';q6 = q6';qHeel = qHeel';

% time_period = 2:0.0005:3;
% q4_choice =  interp1(t,q4,time_period)';
% q5_choice =  interp1(t,q5,time_period)';
% q6_choice =  interp1(t,q6,time_period)';
% qHeel_choice = interp1(t2,qHeel,time_period)';

% First method
% A = [q4_choice q5_choice q6_choice ones(size(q4_choice))];
% Ae = [q4 q5 q6 ones(size(q4))];
% x = (A'*A)^-1*A'*qHeel_choice;
% test = Ae*x;
% figure;plot(t2,qHeel);hold on;plot(t,test);

% Second method
% b = qHeel_choice + (q4_choice + q6_choice-13/180*pi);
% A = q5_choice;
% x = (A'*A)^-1*A'*b;
% test = A*x-(q4_choice + q6_choice-13/180*pi);
% plot(time_period,qHeel_choice);hold on;plot(time_period,test);

% Third method
% b = qHeel_choice + ( q6_choice-13/180*pi);
% A = [ q4_choice q5_choice] ;
% x = (A'*A)^-1*A'*b;
% test = A*x-( q6_choice-13/180*pi);
% plot(time_period,qHeel_choice);hold on;plot(time_period,test);

% Fourth method
% b = qHeel_choice + ( -13/180*pi);
% A = [ q4_choice q5_choice q6_choice] ;
% x = (A'*A)^-1*A'*b;
% test = A*x-( q6_choice-13/180*pi);
% plot(time_period,qHeel_choice);hold on;plot(time_period,test);

% function test
HeelSpring_est = zeros(size(q4));
for i = 1:length(t)
    HeelSpring_est(i) = HeelSpringDeflection(q4(i),q5(i),q6(i));
end