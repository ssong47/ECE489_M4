function p = get_params()

p.Tst = 0.3;        % stance time
p.N_animate = 20;   % for animation time smoothness

% % parameters for matrix calculations
% [g, HB, LB, DB, LH, DK, LK, M1, M2, M3, M4, ...
%        rx1, ry1, rz1, rx2, ry2, rz2, rx3, ry3, rz3, rx4, ry4, rz4, J1, J2, J3, J4] = fcn_params;

g = 9.81;   %Gravity

HB = 245e-3;    %Link lengths
LB = 440e-3;
DB = 48e-3;
LH = 96e-3;
LK = 155e-3;
DK = 52e-3;

M1 = 0.172;     %Link masses
M2 = 0.738;
M3 = 0.304;
M4 = 0.123;

rx1 = 0.0;      %Position of the CoM 1 in link frame 1 (constant)
ry1 = 0.0;
rz1 = -0.074;
    
rx2 = -0.143; %Position of the CoM 2 in link frame 2 (constant)
ry2 = -0.032;
rz2 = 0.0;
    
rx3 = 0.032;
ry3 = 0.0;
rz3 = 0.068;
    
rx4 = 0.001;
ry4 = 0.020;
rz4 = 0.139;

Jx1 = 0.00064; %Components of inertia tensor of link 1 in link frame
Jy1 = 0.00062; 
Jz1 = 0.00017; 

Jx2 = 0.00077; 
Jy2 = 0.02262; 
Jz2 = 0.02306; 

Jx3 = 0.00046; 
Jy3 = 0.00108; 
Jz3 = 0.00071; 

Jx4 = 0.00039; 
Jy4 = 0.00034; 
Jz4 = 0.00008;

Jxy1 = 0.00000000;
Jxz1 = 0.00000593;
Jyz1 = 0.00000000;

Jxy2 = 0.00116033;
Jxz2 = -0.00001650;
Jyz2 = 0.00000022;

Jxy3 = -0.00000525;
Jxz3 = 0.00026769;
Jyz3 = -0.00000651;

Jxy4 = -0.00000437;
Jxz4 = 0.00000366;
Jyz4 = 0.00003475;

Irotor = 0.000007;
NH = 26.9;
NK = 28.8;
Kv = 0.0186;
KT = 0.0135;
Rw = 1.3;

%Spring Parameters
K_spring = 1.67e3; %Sprin Stiffness in N/m
L0 = 80; %Natural Length of spring in mm
%Spring ends physical parameters (mm)
a = 32; b = 32;
c = 52; d = 46;
offset = 0.384832950610572; 


p.params = [g, HB, LB, DB, LH, DK, LK, M1, M2, M3, M4, ...
    rx1, ry1, rz1, rx2, ry2, rz2, rx3, ry3, rz3, rx4, ry4, rz4, ...
    Jx1, Jy1, Jz1, Jx2, Jy2, Jz2, Jx3, Jy3, Jz3, Jx4, Jy4, Jz4, ...
    Jxy1, Jxz1, Jyz1, Jxy2, Jxz2, Jyz2, ...
    Jxy3, Jxz3, Jyz3, Jxy4, Jxz4, Jyz4, ...
    Irotor, NH, NK, Kv, KT, Rw, K_spring, L0, a, b, c, d, offset];
