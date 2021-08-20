function [outputArg1,outputArg2] = PlotCassie(x,t)
linkwidth = 2;
qall = x(1:20);
center = qall(1:3);
P_LeftAbductionJoint = LeftAbductionJoint(qall);
P_LeftRotationJoint = LeftRotationJoint(qall);
P_LeftThighJoint = LeftThighJoint(qall);
P_LeftKneeJoint = LeftKneeJoint(qall);
P_LeftKneeSpringJoint = LeftKneeSpringJoint(qall);
P_LeftAnkleJoint = LeftAnkleJoint(qall);
P_LeftToeJoint = LeftToeJoint(qall);

P_LeftToeBottomBack = LeftToeBottomBack(qall);
P_LeftToeBottomFront = LeftToeBottomFront(qall);

P_RightAbductionJoint = RightAbductionJoint(qall);
P_RightRotationJoint = RightRotationJoint(qall);
P_RightThighJoint = RightThighJoint(qall);
P_RightKneeJoint = RightKneeJoint(qall);
P_RightKneeSpringJoint = RightKneeSpringJoint(qall);
P_RightAnkleJoint = RightAnkleJoint(qall);
P_RightToeJoint = RightToeJoint(qall);

P_RightToeBottomBack = RightToeBottomBack(qall);
P_RightToeBottomFront = RightToeBottomFront(qall);

% plot ground
ground_size = 10;
patch(ground_size*[1,1,-1,-1],ground_size*[1,-1,-1,1],[0,0,0,0],[0,1,0],'FaceAlpha',0.1);
hold on;
% plot torso
% PlotCube(qall(1:3),qall(4:6),0.05*[2,1,3],[0.5,0.5,0.5])

% plot left leg
X = [P_LeftAbductionJoint(1);
    P_LeftRotationJoint(1);
    P_LeftThighJoint(1);
    P_LeftKneeJoint(1);
    P_LeftKneeSpringJoint(1);
    P_LeftAnkleJoint(1);
    P_LeftToeJoint(1)];
Y = [P_LeftAbductionJoint(2);
    P_LeftRotationJoint(2);
    P_LeftThighJoint(2);
    P_LeftKneeJoint(2);
    P_LeftKneeSpringJoint(2);
    P_LeftAnkleJoint(2);
    P_LeftToeJoint(2)];
Z = [P_LeftAbductionJoint(3);
    P_LeftRotationJoint(3);
    P_LeftThighJoint(3);
    P_LeftKneeJoint(3);
    P_LeftKneeSpringJoint(3);
    P_LeftAnkleJoint(3);
    P_LeftToeJoint(3)];
plot3(X,Y,Z,'b','LineWidth',linkwidth);

% plot right leg
X = [P_RightAbductionJoint(1);
    P_RightRotationJoint(1);
    P_RightThighJoint(1);
    P_RightKneeJoint(1);
    P_RightKneeSpringJoint(1);
    P_RightAnkleJoint(1);
    P_RightToeJoint(1)];
Y = [P_RightAbductionJoint(2);
    P_RightRotationJoint(2);
    P_RightThighJoint(2);
    P_RightKneeJoint(2);
    P_RightKneeSpringJoint(2);
    P_RightAnkleJoint(2);
    P_RightToeJoint(2)];
Z = [P_RightAbductionJoint(3);
    P_RightRotationJoint(3);
    P_RightThighJoint(3);
    P_RightKneeJoint(3);
    P_RightKneeSpringJoint(3);
    P_RightAnkleJoint(3);
    P_RightToeJoint(3)];
plot3(X,Y,Z,'r','LineWidth',linkwidth);

%plot left feet
X = [P_LeftToeJoint(1);
    P_LeftToeBottomBack(1);
    P_LeftToeBottomFront(1);
    P_LeftToeJoint(1);];
Y = [P_LeftToeJoint(2);
    P_LeftToeBottomBack(2);
    P_LeftToeBottomFront(2);
    P_LeftToeJoint(2);];
Z = [P_LeftToeJoint(3);
    P_LeftToeBottomBack(3);
    P_LeftToeBottomFront(3);
    P_LeftToeJoint(3);];
plot3(X,Y,Z,'b','LineWidth',linkwidth)

%plot right feet
X = [P_RightToeJoint(1);
    P_RightToeBottomBack(1);
    P_RightToeBottomFront(1);
    P_RightToeJoint(1)];
Y = [P_RightToeJoint(2);
    P_RightToeBottomBack(2);
    P_RightToeBottomFront(2);
    P_RightToeJoint(2)];
Z = [P_RightToeJoint(3);
    P_RightToeBottomBack(3);
    P_RightToeBottomFront(3);
    P_RightToeJoint(3)];
plot3(X,Y,Z,'r','LineWidth',linkwidth)    

% connect left and right leg to torso
X = [P_LeftAbductionJoint(1); P_RightAbductionJoint(1)];
Y = [P_LeftAbductionJoint(2); P_RightAbductionJoint(2)];
Z = [P_LeftAbductionJoint(3); P_RightAbductionJoint(3)];
plot3(X,Y,Z,'k','LineWidth',linkwidth)    

hold off
axis equal
grid on
axis([center(1)-1, center(1)+1, center(2)-1, center(2)+1, 0, 1.5])
    


end

