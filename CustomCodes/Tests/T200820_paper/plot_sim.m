figure
plot(Data.Ly_stToe_stTD0_obs)

figure
plot(Data.Lx_G)

figure
plot(Data.vx_com_tgd)

%%
f1 = figure;
ax1 = gca;
set(gcf, 'Position',  [100, 100, 300, 200])
plot(Data_perfect.Ly_stToe,'LineWidth',2)
hold on
plot(100*Data_perfect.stanceLeg,'g-.','LineWidth',2)
grid on
title('')
xlabel('\textbf{Time (s)}','Interpreter','latex')
ylabel('{\boldmath$L^y$} \textbf{(kg-${\rm \textbf{m}}^{\boldmath2}$/s)}','Interpreter','latex')
ylim([40,60])

f2 = figure;
ax2 = gca;
set(gcf, 'Position',  [100, 100, 300, 200])
plot(Data_perfect.Ly_G,'LineWidth',2)
hold on
plot(100*Data_perfect.stanceLeg,'g-.','LineWidth',2)
grid on
title('')
xlabel('\textbf{Time (s)}','Interpreter','latex')
ylabel('{\boldmath$L^y_{\rm CoM}$} \textbf{(kg-${\rm \textbf{m}}^{\boldmath2}$/s)}','Interpreter','latex')
ylim([-5,5])

f3 = figure;
ax3 = gca;
set(gcf, 'Position',  [100, 100, 300, 200])
plot(Data_perfect.vx_com,'LineWidth',2)
hold on
plot(100*Data_perfect.stanceLeg,'g-.','LineWidth',2)
grid on
title('')
xlabel('\textbf{Time (s)}','Interpreter','latex')
ylabel('{\boldmath$v^x_{\rm CoM}$} \textbf{(m/s)}','Interpreter','latex')
ylim([1.7,2.3])

linkaxes([ax1,ax2,ax3],'x')
xlim([10.7,11.4])

% figure_path = [root_dir, '\PicsData\PD200820'];
% cd(figure_path)
% saveas(f1,'L_Cassie.png')
% saveas(f2,'LCOM_Cassie.png')
% saveas(f3,'vxCOM_Cassie.png')

%%
f1 = figure;
ax(1) = gca;
set(gcf, 'Position',  [100, 100, 400, 250])
plot(Data_perfect.Ly_stToe/32/0.8,'LineWidth',2)
hold on
plot(Data_perfect.vx_com)
plot(10*Data_perfect.stanceLeg,'g-.','LineWidth',2)
hold off
grid on
legend('Ly/mH','vx com')
xlabel('\textbf{Time (s)}','Interpreter','latex')
% ylabel('{\boldmath$v^x_{\rm CoM}$} \textbf{(m/s)}','Interpreter','latex')

f2 = figure;
ax(2) = gca;
set(gcf, 'Position',  [100, 100, 400, 250])
plot(Data_perfect.Ly_stToe/32/0.8,'LineWidth',2)
hold on; 
plot(Data_perfect.dx0_next_stTD0,'LineWidth',2)
plot(10*Data_perfect.stanceLeg,'g-.','LineWidth',2)
hold off;
grid on
legend({'Instantaneous value of {\boldmath$\frac{L^y}{mH}$}','Predicted value at end of step'},'Interpreter','latex')
xlabel('\textbf{Time (s)}','Interpreter','latex','FontSize',12)
ylabel('{\boldmath$\frac{L^y}{mH}$} \textbf{(m/s)}','Interpreter','latex','FontSize',12)


f3 = figure;
ax(3) = gca;
set(gcf, 'Position',  [100, 100, 400, 250])
plot(Data_perfect.vx_com,'LineWidth',2)
hold on; 
plot(Data_perfect.vx_com0_next_stTd0,'LineWidth',2)
plot(10*Data_perfect.stanceLeg,'g-.','LineWidth',2)
hold off;
grid on
legend({'Instantaneous value of {\boldmath$v^x_{\rm CoM}$}','Predicted value at end of step'},'Interpreter','latex')
xlabel('\textbf{Time (s)}','Interpreter','latex')
ylabel('{\boldmath$v^x_{\rm CoM}$} \textbf{(m/s)}','Interpreter','latex')

linkaxes(ax,'xy')

axis([10.7, 11.4, 1.4,2.2])

figure_path = [root_dir, '\PicsData\PD200820'];
cd(figure_path)
saveas(f1,'L_VxCOm.png')
saveas(f2,'L_Predict.png')
saveas(f3,'VxCOM_predict.png')

%%
f1 = figure;
set(gcf, 'Position',  [100, 100, 400, 250])
plot(Data_perfect.vx_com,'LineWidth',1.5)
hold on;
plot(Data_perfect.Ly_stToe/32/0.8,'LineWidth',1.5)
hold off;
legend({'\boldmath$v^x_{\rm CoM}$','{\boldmath$\frac{L^y}{mH}$}'},'Interpreter','latex')
xlabel('\textbf{Time (s)}','Interpreter','latex')
ylabel('(m/s)','Interpreter','latex')
grid on
axis([0,8,0,3.5])

figure_path = [root_dir, '\PicsData\PD200820'];
cd(figure_path)
saveas(f1,'sim_fast.png')