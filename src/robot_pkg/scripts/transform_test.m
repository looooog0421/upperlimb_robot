syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 lb ls lh wd;

%%     
%      a        alpha     d      theta
DH1 = [0,       0,        0,     theta1];
DH2 = [0,       pi/2,     0,     theta2];
DH3 = [0,       pi/2,     lb,    theta3];
DH4 = [0,      -pi/2,     0,     theta4];
DH5 = [0,       pi/2,     ls,    theta5];
DH6 = [0,       pi/2,     0,     theta6];
DH7 = [lh,      pi/2,     0,     theta7];

T1 = transform(DH1);
T2 = transform(DH2);
T3 = transform(DH3);
T4 = transform(DH4);
T5 = transform(DH5);
T6 = transform(DH6);
T7 = transform(DH7);

T = T1*T2*T3*T4*T5;
T_t = T1*T2*T3*T4*T5*T6*T7;



function T = transform(DH)
    
    a = DH(1);
    alpha = DH(2);
    d = DH(3);
    theta = DH(4)

    T = [cos(theta),             -sin(theta),            0,           a;
         sin(theta)*cos(alpha),  cos(theta)*cos(alpha),  -sin(alpha), -d*sin(alpha);
         sin(theta)*sin(alpha),  cos(theta)*sin(alpha),  cos(alpha),  d*cos(alpha);
         0,                      0,                      0,           1];
end