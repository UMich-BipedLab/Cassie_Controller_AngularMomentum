% Date = '2018-7-10';
% Time = '16-8';
% load([root_dir,'/ExpLog/',Date,'/',Time,'/','Log']) 

figure(1)
plot(log.Data.t,180/pi*log.Data.hd_joint)
hold on
plot(log.Data.t,180/pi*log.Data.h0_joint)
ps = plot(log.Data.t,200*log.Data.stanceLeg,'--')
hold off
lg = legend('hd_joint_1','hd_joint_2','hd_joint_3','hd_joint_4','hd_joint_5','hd_joint_6','hd_joint_7','hd_joint_8','hd_joint_9','hd_joint_10','h0_joint_1','h0_joint_2','h0_joint_3','h0_joint_4','h0_joint_5','h0_joint_6','h0_joint_7','h0_joint_8','h0_joint_9','h0_joint_10')
set(lg,'visible','off');
plotbrowser('on')
% title('Left Knee Motor')
% xlabel('Time (s)')
% ylabel('Position (deg)')
% legend('referenc','actual','impact')
% axis([120,130,-110, -60])
% axis([96,117,-110, -60])
% axis([75,77,-110, -60])


figure(2)
plot(log.Data.t,180/pi*log.Data.dhd_joint)
hold on
plot(log.Data.t,180/pi*log.Data.dh0_joint)
ps = plot(log.Data.t,200*log.Data.stanceLeg,'--')
hold off
lg = legend('dhd_joint_1','dhd_joint_2','dhd_joint_3','dhd_joint_4','dhd_joint_5','dhd_joint_6','dhd_joint_7','dhd_joint_8','dhd_joint_9','dhd_joint_10','dh0_joint_1','dh0_joint_2','dh0_joint_3','dh0_joint_4','dh0_joint_5','dh0_joint_6','dh0_joint_7','dh0_joint_8','dh0_joint_9','dh0_joint_10')
set(lg,'visible','off');
plotbrowser('on')

figure(3)
plot(log.Data.t,180/pi*log.Data.torso_angle)
lg = legend('yaw','pitch','roll');
set(lg,'visible','off');
plotbrowser('on')