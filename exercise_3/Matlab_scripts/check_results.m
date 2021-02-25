% Joint variables
q1 = 0;
q2 = 0+(pi/2);
q3 = 0;
q4 = 0;
q5 = 0;
q6 = 0;
% Check results

%(link1 -> link2)
a1 = 0.15;
d1 = 0.525;
alpha1 = pi/2;
A_1_2 = create_homogeneous_transformation(a1, d1, alpha1, q1);

%(link2 -> link3)
a2 = 0.79;
d2 = 0;
alpha2 = 0;
A_2_3 = create_homogeneous_transformation(a2, d2, alpha2, q2);

%(link3 -> link4)
a3 = 0.15;
d3 = 0;
alpha3 = pi/2;
A_3_4 = create_homogeneous_transformation(a3, d3, alpha3, q3);

%(link4 -> link5)
a4 = 0;
d4 = 0.860;
alpha4 = -pi/2;
A_4_5 = create_homogeneous_transformation(a4, d4, alpha4, q4);

%(link5 -> link6)
a5 = 0;
d5 = 0;
alpha5 = pi/2;
A_5_6 = create_homogeneous_transformation(a5, d5, alpha5, q5);

%(link6 -> flange)
a6 = 0;
d6 = 0.1;
alpha6 = 0;
A_6_f = create_homogeneous_transformation(a6, d6, alpha6, q6);

A = [A_1_2; A_2_3; A_3_4; A_4_5; A_5_6; A_6_f];

disp('"Link1->flange"');
%[tform, quaternion, axang, eul_XYZ] = perfom_transformation(A, 1, 6)
[tform, quaternion, axang, roll, pitch, yaw] = perfom_transformation(A, 1, 6)

disp('"Link2->flange"');
%[tform, quaternion, axang, eul_XYZ] = perfom_transformation(A, 2, 6)
[tform, quaternion, axang, roll, pitch, yaw] = perfom_transformation(A, 2, 6)


disp('"Link3->flange"');
%[tform, quaternion, axang, eul_XYZ] = perfom_transformation(A, 3, 6)
[tform, quaternion, axang, roll, pitch, yaw] = perfom_transformation(A, 3, 6)


disp('"Link4->flange"');
%[tform, quaternion, axang, eul_XYZ] = perfom_transformation(A, 4, 6)
[tform, quaternion, axang, roll, pitch, yaw] = perfom_transformation(A, 4, 6)

disp('"Link5->flange"');
%[tform, quaternion, axang, eul_XYZ] = perfom_transformation(A, 5, 6)
[tform, quaternion, axang, roll, pitch, yaw] = perfom_transformation(A, 5, 6)


disp('"Link6->flange"');
%[tform, quaternion, axang, eul_XYZ] = perfom_transformation(A, 6, 6)
[tform, quaternion, axang, roll, pitch, yaw] = perfom_transformation(A, 6, 6)
