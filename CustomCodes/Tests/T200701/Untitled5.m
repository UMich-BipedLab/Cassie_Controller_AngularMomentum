% L = length(log.Data.t);
% begin_index = find(log.Data.t == 1);
% end_index = find(log.Data.t == 258);
begin_index = 1;
end_index = length(log.Data.t);
L = end_index - begin_index + 1;
t = log.Data.t(begin_index:end_index);
vx = zeros(L,1);
vy = zeros(L,1);
for i = begin_index:end_index
    v = YToolkits.Angles.Rz(log.Data.q4(i))'*[log.Data.dq1(i);log.Data.dq2(i);0];
    vx(i - begin_index + 1) = v(1);
    vy(i - begin_index + 1) = v(2);
end
%%
vx_avg = zeros(L,1);
avg_time = 1;
samples = avg_time*2000; % it is in fact samples - 1
for i = (samples/2 + 1):(L - samples/2)
%     a = zeros(1,L);
%     a(i - samples/2:i+samples/2) = 1/(samples+1);
    vx_avg(i) = sum(vx(i - samples/2:i+samples/2))/(samples+1);
end
%%
figure; 
plot(t,vx_avg,'b','LineWidth',3)
hold on;
plot(0,0,'g.','MarkerSize',30)
plot(t,vx,'g.','MarkerSize',0.05)
title('Speed Plot','FontSize',20)
xlabel('Time','FontSize',20)
ylabel('Vx','FontSize',20)
% legend('average velocity in a 0.8s window', 'instant velocity')
legend({'Average Forward Speed (1 s moving window)', 'Instantaneous Forward Speed'},'FontSize',20)

grid on

%%
save('speed_data','vx','vy','vx_avg','t')

