% generate the dynamics for the hopping leg with boom
% Author: Yanran Ding and João Ramos
% Last modified: 2020/03/05
clear all

%% --- define symbols ---
syms q1 q2 q3 q4 real
syms dq1 dq2 dq3 dq4 real
syms HB LB DB LH DK LK real
syms M1 M2 M3 M4 real
syms rx1 ry1 rz1 rx2 ry2 rz2 rx3 ry3 rz3 rx4 ry4 rz4 real
syms Jxx1 Jyy1 Jzz1 Jxy1 Jxz1 Jyz1 real
syms Jxx2 Jyy2 Jzz2 Jxy2 Jxz2 Jyz2 real
syms Jxx3 Jyy3 Jzz3 Jxy3 Jxz3 Jyz3 real
syms Jxx4 Jyy4 Jzz4 Jxy4 Jxz4 Jyz4 real
syms g real
syms px py pz real

%% --- variable lists ---
% Physical parameters of the robot
m_list_params = {
    'g'  'p(1)'; %Gravity
    
    'HB' 'p(2)'; %Lengths of links
    'LB' 'p(3)';
    'DB' 'p(4)';
    'LH' 'p(5)';
    'DK' 'p(6)';
    'LK' 'p(7)';
    
    'M1' 'p(8)'; %Total mass of links
    'M2' 'p(9)';
    'M3' 'p(10)';
    'M4' 'p(11)';
    
    'rx1' 'p(12)'; %Positio of CoM in local link frame
    'ry1' 'p(13)';
    'rz1' 'p(14)';
    
    'rx2' 'p(15)';
    'ry2' 'p(16)';
    'rz2' 'p(17)';
    
    'rx3' 'p(18)';
    'ry3' 'p(19)';
    'rz3' 'p(20)';
    
    'rx4' 'p(21)';
    'ry4' 'p(22)';
    'rz4' 'p(23)';
    
    'Jxx1' 'p(24)'; %Inertia tensor term in local link frame
    'Jyy1' 'p(25)';
    'Jzz1' 'p(26)';
    'Jxy1' 'p(26)';
    'Jxz1' 'p(26)';
    'Jyz1' 'p(26)';
    
    'Jxx2' 'p(24)'; %Inertia tensor term in local link frame
    'Jyy2' 'p(25)';
    'Jzz2' 'p(26)';
    'Jxy2' 'p(27)';
    'Jxz2' 'p(28)';
    'Jyz2' 'p(29)';
    
    'Jxx3' 'p(30)'; %Inertia tensor term in local link frame
    'Jyy3' 'p(31)';
    'Jzz3' 'p(32)';
    'Jxy3' 'p(33)';
    'Jxz3' 'p(34)';
    'Jyz3' 'p(35)';
    
    'Jxx4' 'p(36)'; %Inertia tensor term in local link frame
    'Jyy4' 'p(37)';
    'Jzz4' 'p(38)';
    'Jxy4' 'p(39)';
    'Jxz4' 'p(40)';
    'Jyz4' 'p(41)'};

J1 = [Jxx1 Jxy1 Jxz1;
      Jxy1 Jyy1 Jyz1;
      Jxz1 Jyz1 Jzz1];
  
J2 = [Jxx2 Jxy2 Jxz2;
      Jxy2 Jyy2 Jyz2;
      Jxz2 Jyz2 Jzz2];
  
J3 = [Jxx3 Jxy3 Jxz3;
      Jxy3 Jyy3 Jyz3;
      Jxz3 Jyz3 Jzz3];
  
J4 = [Jxx4 Jxy4 Jxz4;
      Jxy4 Jyy4 Jyz4;
      Jxz4 Jyz4 Jzz4];

% joint positions
m_list_q = {
    'q1' 'q(1)';
    'q2' 'q(2)';
    'q3' 'q(3)';
    'q4' 'q(4)'};

% joint velocities
m_list_dq = {
    'dq1' 'dq(1)';
    'dq2' 'dq(2)';
    'dq3' 'dq(3)';
    'dq4' 'dq(4)'};

% Foot position at impact:
m_list_pt = {
    'px' 'pt(1)';
    'py' 'pt(2)';
    'pz' 'pt(3)'};

%% --- variables ---
q = [q1 q2 q3 q4]';
dq = [dq1 dq2 dq3 dq4]';
pt = [px py pz]';

%% --- forward kinematics ---
% Frames:
% 0 - origin
% 1 - boom top
% 2 - hip axis
% 3 - knee axis
% toe - robot foot

r0 = [0; 0; HB];
R01 = rz(q1);
T01 = [R01 R01*r0;
       0, 0, 0, 1];

r1 = [LB; DB; 0];
R12 = ry(q2);
T12 = [R12 R12*r1;
       0, 0, 0, 1];

r2 = [0; 0; -LH];
R23 = rx(q3);
T23 = [R23 R23*r2;
       0, 0, 0, 1];

L = sqrt(LK^2 + DK^2);
r3 = [0; 0; -L];
R3toe = rx(q4);
T3toe = [R3toe R3toe*r3;
         0, 0, 0, 1];

%% Joints positions
% 1 - boom top
% 2 - hip axis
% 3 - knee axis
% toe - robot foot

p1 = T01(1:3,4);
write_fcn_m('fcn_p1.m',{'q','p'},[m_list_q;m_list_params],{p1,'p1'});

R02 = R01 * R12;
T02 = T01 * T12;
p2_aux = T02*[0; -DB; 0; 1];
p2_aux = p2_aux(1:3,1);
write_fcn_m('fcn_p2_aux.m',{'q','p'},[m_list_q;m_list_params],{p2_aux,'p2_aux'});
p2 = T02(1:3,4);
write_fcn_m('fcn_p2.m',{'q','p'},[m_list_q;m_list_params],{p2,'p2'});

R03 = R02 * R23;
T03 = T02 * T23;
p3 = T03(1:3,4);
write_fcn_m('fcn_p3.m',{'q','p'},[m_list_q;m_list_params],{p3,'p3'});

R0toe = R03 * R3toe;
T0toe = T03 * T3toe;
p_toe = T0toe(1:3,4);       % toe position
J_toe = jacobian(p_toe,q);  % toe linear velocity Jacobian 
write_fcn_m('fcn_p_toe.m',{'q','p'},[m_list_q;m_list_params],{p_toe,'p_toe'});
write_fcn_m('fcn_J_toe.m',{'q','p'},[m_list_q;m_list_params],{J_toe,'J_toe'});

% p_h2f_b
p_h2f_s = p_toe - p2;
p_h2f_b = simplify(R02' * p_h2f_s);
write_fcn_m('fcn_p_h2f_b.m',{'q','p'},[m_list_q;m_list_params],{p_h2f_b,'p_h2f_b'});

% J_h2f_b
J_h2f_b = simplify(jacobian(p_h2f_b,q));
write_fcn_m('fcn_J_h2f_b.m',{'q','p'},[m_list_q;m_list_params],{J_h2f_b,'J_h2f_b'});


%% Linear Jacobian of foot in respect to hip: 
T2toe = T23 * T3toe;
p_toe_HIP = T2toe(2:3,4);
J_toe_HIP = jacobian(p_toe_HIP,q);
J_toe_HIP = J_toe_HIP(:,3:4);
write_fcn_m('fcn_p_toe_HIP.m',{'q','p'},[m_list_q;m_list_params],{p_toe_HIP,'p_toe_HIP'});
write_fcn_m('fcn_J_toe_HIP.m',{'q','p'},[m_list_q;m_list_params],{J_toe_HIP,'J_toe_HIP'});

%% Center of Mass (com) positions and velocities of each link
p_1com = T01*[rx1; ry1; rz1; 1]; 
p_1com = p_1com(1:3,1);
v_1com = jacobian(p_1com,q) * dq;

p_2com = T02*[rx2; ry2; rz2; 1];
p_2com = p_2com(1:3,1);
v_2com = jacobian(p_2com,q) * dq;

p_3com = T03*[rx3; ry3; rz3; 1];
p_3com = p_3com(1:3,1);
v_3com = jacobian(p_3com,q) * dq;

p_4com = T0toe*[rx4; ry4; rz4; 1];
p_4com = p_4com(1:3,1);
v_4com = jacobian(p_4com,q) * dq;

CoM = (M1*p_1com + M2*p_2com + M3*p_3com + M4*p_4com)/(M1 + M2 + M3 + M4);
write_fcn_m('fcn_CoM.m',{'q', 'p'},[m_list_q;m_list_params],{CoM,'CoM'});

%% Angular velocity jacobians
u1 = [0; 0; 1];
u2 = [0; 1; 0];
u3 = [1; 0; 0];
u4 = [1; 0; 0];
Jw1 = [u1 [0; 0; 0] [0; 0; 0] [0; 0; 0]];
Jw2 = [u1 R01*u2 [0; 0; 0] [0; 0; 0]];
Jw3 = [u1 R01*u2 R02*u3 [0; 0; 0]];
Jw4 = [u1 R01*u2 R02*u3 R03*u4];
Jw_toe = Jw4; %Angular velocity Jacobian of foot.

% Angular velocity of frames in respect to the world frame
w1 = Jw1 * dq;
w2 = Jw2 * dq;
w3 = Jw3 * dq;
w4 = Jw4 * dq;

%% --- Energy and Lagrangian ---
KE_1 = 0.5 * v_1com' * M1 * v_1com + 0.5 * w1' * R01 * J1 * transpose(R01) * w1;
KE_2 = 0.5 * v_2com' * M2 * v_2com + 0.5 * w2' * R02 * J2 * transpose(R02) * w2;
KE_3 = 0.5 * v_3com' * M3 * v_3com + 0.5 * w3' * R03 * J3 * transpose(R03) * w3;
KE_4 = 0.5 * v_4com' * M4 * v_4com + 0.5 * w4' * R0toe * J4 * transpose(R0toe) * w4;

% Kinetic energy
KE = simplify(KE_1 + KE_2 + KE_3 + KE_4);   

% Potential energy
PE = M1*[0 0 g]*p_1com + M2*[0 0 g]*p_2com + M3*[0 0 g]*p_3com + M4*[0 0 g]*p_4com;

%To calculate the actuation selection matrix:
Upsilon = [q3 q4]; %where control torques go: hip and knee only, first two joints are passive

%% --- Euler-Lagrange Equation ---
[De, Ce, Ge, Be] = std_dynamics(KE,PE,q,dq, Upsilon);

write_fcn_m('fcn_De.m',{'q', 'p'},[m_list_q;m_list_params],{De,'De'});
write_fcn_m('fcn_Ce.m',{'q', 'dq', 'p'},[m_list_q;m_list_dq;m_list_params],{Ce,'Ce'});
write_fcn_m('fcn_Ge.m',{'q', 'p'},[m_list_q;m_list_params],{Ge,'Ge'});
write_fcn_m('fcn_Be.m',{'q', 'p'},[m_list_q;m_list_params],{Be,'Be'});

%% --- Holonomic Constraints ---
% pt is the toe position at impact time
r = [pt([1,2]); 0];
x_hat = r / norm(r);
z_hat = [0; 0; 1];
y_hat = cross(z_hat,x_hat);

Rt = [x_hat y_hat z_hat];       % the toe frame

p_toe_t = Rt' * p_toe;          % p_toe in the toe frame

% The Holonomic constraints are:
% foot position y and z in {Rt} frame do not change during stance
% foot position x in {Rt} frame does change during stance
hc = p_toe_t([2,3]);
Jhc = jacobian(hc,q);
dJhc = sym('dJhc',size(Jhc));
for ii = 1:size(Jhc,2)
    dJhc(:,ii) = jacobian(Jhc(:,ii),q) * dq;
end
write_fcn_m('fcn_Jhc.m',{'q', 'pt', 'p'},[m_list_q;m_list_pt;m_list_params],{Jhc,'Jhc'});
write_fcn_m('fcn_dJhc.m',{'q', 'dq', 'pt', 'p'},[m_list_q;m_list_dq;m_list_pt;m_list_params],{dJhc,'dJhc'});










