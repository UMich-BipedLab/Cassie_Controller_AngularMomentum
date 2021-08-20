test = q_iter;
test(10) = test(10)-1;
test(12) = test(12)+1;
test(9) = -test(10)/2;

q = test;
step_size = 0.01;

p_com_seq = [];
p_LToe_seq = [];
q_knee_seq = [];
for i = 0:step_size:1
    q(10) = q(10) + step_size;
    q(12) = q(12) - step_size;
    q(9) = -q(10)/2;
    p_com =  p_COM(q);
    p_LToe =  p_LeftToeJoint(q);
    p_com_seq = [p_com_seq, p_com];
    p_LToe_seq = [p_LToe_seq, p_LToe];
    q_knee_seq = [q_knee_seq, q(10)];
end