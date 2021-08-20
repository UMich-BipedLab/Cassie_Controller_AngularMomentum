function  PlotCube(center,EA,LWH,color)
L = LWH(1);
W = LWH(2);
H = LWH(3);
lwh = [L;W;H];
% Define vertex of six faces
vertex_set1 = [1,1,1,1;
               1,1,-1,-1;
               1,-1,-1,1];
           
vertex_set2 = vertex_set1;
vertex_set2([1,2],:) = vertex_set2([2,1],:);

vertex_set3 = vertex_set1;
vertex_set3([1,3],:) = vertex_set3([3,1],:);

vertex_set4 = vertex_set1;
vertex_set4(1,:) = -vertex_set4(1,:);

vertex_set5 = vertex_set2;
vertex_set5(2,:) = -vertex_set5(2,:);

vertex_set6 = vertex_set3;
vertex_set6(3,:) = -vertex_set6(3,:);

%Add information of LWH
vertex_set1 = lwh.*vertex_set1;
vertex_set2 = lwh.*vertex_set2;
vertex_set3 = lwh.*vertex_set3;
vertex_set4 = lwh.*vertex_set4;
vertex_set5 = lwh.*vertex_set5;
vertex_set6 = lwh.*vertex_set6;



% Add information of orientation
Rz = YToolkits.Angles.Rz(EA(1));
Ry = YToolkits.Angles.Ry(EA(2));
Rx = YToolkits.Angles.Rx(EA(3));
R = Rz*Ry*Rx;

vertex_set1 = R*vertex_set1;
vertex_set2 = R*vertex_set2;
vertex_set3 = R*vertex_set3;
vertex_set4 = R*vertex_set4;
vertex_set5 = R*vertex_set5;
vertex_set6 = R*vertex_set6;


% Add information of center
vertex_set1 = vertex_set1 + center;
vertex_set2 = vertex_set2 + center;
vertex_set3 = vertex_set3 + center;
vertex_set4 = vertex_set4 + center;
vertex_set5 = vertex_set5 + center;
vertex_set6 = vertex_set6 + center;


%plot
patch(vertex_set1(1,:),vertex_set1(2,:),vertex_set1(3,:),color);
patch(vertex_set2(1,:),vertex_set2(2,:),vertex_set2(3,:),color);
patch(vertex_set3(1,:),vertex_set3(2,:),vertex_set3(3,:),color);
patch(vertex_set4(1,:),vertex_set4(2,:),vertex_set4(3,:),color);
patch(vertex_set5(1,:),vertex_set5(2,:),vertex_set5(3,:),color);
patch(vertex_set6(1,:),vertex_set6(2,:),vertex_set6(3,:),color);

    
          
end

