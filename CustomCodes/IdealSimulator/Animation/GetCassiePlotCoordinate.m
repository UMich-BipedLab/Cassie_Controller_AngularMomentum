function [LeftLegCoordinate, RightLegCoordinate, LeftFeetCoordinate, RightFeetCoordinate, ConnectionCoordinate] = GetCassiePlotCoordinate(x,t)
qall = x(1:20);
center = qall(1:3);
P_LeftAbductionJoint = p_LeftAbductionJoint(qall);
P_LeftRotationJoint = p_LeftRotationJoint(qall);
P_LeftThighJoint = p_LeftThighJoint(qall);
P_LeftKneeJoint = p_LeftKneeJoint(qall);
P_LeftKneeSpringJoint = p_LeftKneeSpringJoint(qall);
P_LeftAnkleJoint = p_LeftAnkleJoint(qall);
P_LeftToeJoint = p_LeftToeJoint(qall);

P_LeftToeBottomBack = p_LeftToeBottomBack(qall);
P_LeftToeBottomFront = p_LeftToeBottomFront(qall);

P_RightAbductionJoint = p_RightAbductionJoint(qall);
P_RightRotationJoint = p_RightRotationJoint(qall);
P_RightThighJoint = p_RightThighJoint(qall);
P_RightKneeJoint = p_RightKneeJoint(qall);
P_RightKneeSpringJoint = p_RightKneeSpringJoint(qall);
P_RightAnkleJoint = p_RightAnkleJoint(qall);
P_RightToeJoint = p_RightToeJoint(qall);

P_RightToeBottomBack = p_RightToeBottomBack(qall);
P_RightToeBottomFront = p_RightToeBottomFront(qall);


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

LeftLegCoordinate = [X,Y,Z]';

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

RightLegCoordinate = [X,Y,Z]';
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
LeftFeetCoordinate = [X,Y,Z]';

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
RightFeetCoordinate = [X,Y,Z]';

% connect left and right leg to torso
X = [P_LeftAbductionJoint(1); P_RightAbductionJoint(1)];
Y = [P_LeftAbductionJoint(2); P_RightAbductionJoint(2)];
Z = [P_LeftAbductionJoint(3); P_RightAbductionJoint(3)];
ConnectionCoordinate = [X,Y,Z]';


end

