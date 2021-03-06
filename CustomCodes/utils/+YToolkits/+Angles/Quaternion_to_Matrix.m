function R = Quaternion_to_Matrix(in1)
%QUATERNION_TO_MATRIX
%    R = QUATERNION_TO_MATRIX(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    26-Mar-2019 11:01:02

a = in1(1,:);
b = in1(2,:);
c = in1(3,:);
d = in1(4,:);
t2 = b.*c.*2.0;
t3 = a.^2;
t4 = b.^2;
t5 = c.^2;
t6 = d.^2;
t7 = a.*c.*2.0;
t8 = b.*d.*2.0;
t9 = c.*d.*2.0;
R = reshape([t3+t4-t5-t6,t2+a.*d.*2.0,-t7+t8,t2-a.*d.*2.0,t3-t4+t5-t6,t9+a.*b.*2.0,t7+t8,t9-a.*b.*2.0,t3-t4-t5+t6],[3,3]);
