function p = get_params()

p.Tst = 0.2;        % stance time
p.N_animate = 30;   % for animation time smoothness

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

Jxx1 = 0.00064578;
Jyy1 = 0.00062034; 
Jzz1 = 0.00017348; 
Jxy1 = 0;
Jxz1 = 0.00000593;
Jyz1 = 0;

Jxx2 = 0.00076962;
Jyy2 = 0.02261852;
Jzz2 = 0.02306310;
Jxy2 = 0.00116033;
Jxz2 = -0.0000165;
Jyz2 = 0.00000022;

Jxx3 = 0.00045696;
Jyy3 = 0.00108532;
Jzz3 = 0.00070883;
Jxy3 = -0.00000525;
Jxz3 = 0.00026769;
Jyz3 = -0.00000651;

Jxx4 = 0.00039564;
Jyy4 = 0.00033576;
Jzz4 = 0.00008091;
Jxy4 = -0.00000437;
Jxz4 = 0.00000366;
Jyz4 = 0.00003475;

p.params = [g, HB, LB, DB, LH, DK, LK, M1, M2, M3, M4, ...
    rx1, ry1, rz1, rx2, ry2, rz2, rx3, ry3, rz3, rx4, ry4, rz4, ...
    Jxx1, Jyy1, Jzz1, Jxy1, Jxz1, Jyz1,...
    Jxx2, Jyy2, Jzz2, Jxy2, Jxz2, Jyz2,...
    Jxx3, Jyy3, Jzz3, Jxy3, Jxz3, Jyz3,...
    Jxx4, Jyy4, Jzz4, Jxy4, Jxz4, Jyz4];
    
    
    
    
    