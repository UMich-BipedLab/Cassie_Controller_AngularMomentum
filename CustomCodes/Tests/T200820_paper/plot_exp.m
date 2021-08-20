exp_data_path = [root_dir '\ExpLog\2020-7-17\11-13']
load([exp_data_path '\Log.mat'])

begin_index = 1;
end_index = length(log.Data.t);
Length = end_index - begin_index + 1;
t = log.Data.t(begin_index:end_index);
vx = zeros(Length,1);
vy = zeros(Length,1);
Lx_LToe = zeros(Length,1);
Ly_LToe = zeros(Length,1);
Lx_RToe = zeros(Length,1);
Ly_RToe = zeros(Length,1);

q4_fil = zeros(size(log.Data.q4))

for i = begin_index:end_index
    alpha = 0.0002;
    q4_fil(i) = (1-alpha) * q4_fil(max(1,i - 1)) + alpha * log.Data.q4(i);
    v = YToolkits.Angles.Rz(q4_fil(i))'*[log.Data.dq1(i);log.Data.dq2(i);0];
    L_LToe = YToolkits.Angles.Rz(q4_fil(i))'*[log.Data.Lx_LToe(i);log.Data.Ly_LToe(i);0];
    L_RToe = YToolkits.Angles.Rz(q4_fil(i))'*[log.Data.Lx_RToe(i);log.Data.Ly_RToe(i);0];
    vx(i - begin_index + 1) = v(1);
    vy(i - begin_index + 1) = v(2);
    Lx_LToe(i - begin_index + 1) = L_LToe(1);
    Ly_LToe(i - begin_index + 1) = L_LToe(2);
    Lx_RToe(i - begin_index + 1) = L_RToe(1);
    Ly_RToe(i - begin_index + 1) = L_RToe(2);
end
%%
f1 = figure; 
% scatter(t,Ly_LToe)
plot(t,Ly_LToe,'LineWidth',2)
ax(1) = gca;
xlabel('\textbf{Time (s)}','Interpreter','latex','FontSize',12)
ylabel('{\boldmath$\frac{L^y}{mH}$} \textbf{(m/s)}','Interpreter','latex','FontSize',12)
grid on
f2 = figure;
% scatter(t,vx)
plot(t,vx,'LineWidth',2)
ax(2) = gca;
xlabel('\textbf{Time (s)}','Interpreter','latex')
ylabel('{\boldmath$v^x_{\rm CoM}$} \textbf{(m/s)}','Interpreter','latex')
grid on
linkaxes(ax,'x')

xlim([201.75,202.15]);

figure_path = [root_dir, '\PicsData\PD200820'];
cd(figure_path)
saveas(f2,'Vorigin_Cassie_exp.jpg')
saveas(f1,'L_Cassie_exp.jpg')