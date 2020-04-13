% Hopping leg with boom
% Author: Yanran Ding and Joao Ramos 
% Last modified: 2020/04/02
clc
clear all
close all

addpath gen
addpath fcns

global Fz

fprintf('------ ME446 Milestone 4 -------\n')
fprintf('Initializing ..............\n')

% --- parameters ---
p = get_params;     % Getting physical parameters of the robot
Nstep = 5;          % number of desired hops

% Initial condition
q0 = [0; 0; pi/3; -pi/2]; %Joint angles
dq0 = [0; 0; 0; 0];       %Joint velocities
ic = [q0; dq0];

%Ploting the robot in the initial configuration:
%plotRobot(ic,p);

% Recording
tstart = 0;
tfinal = 2*Nstep;   %Maximum simulation time
tout = tstart;
Xout = ic';
Uout = [0,0];
Fout = [0,0];

for istep = 1:Nstep
    %% aerial phase
    options = odeset('Events',@(t,X)event_touchDown(t,X,p));
    [t,X] = ode45(@(t,X)dyn_aerial(t,X,p),[tstart, tfinal], Xout(end,:),options);
    Fz = 1;

    p.tTD = t(end);                             % touchdown time
    p.ptTD = fcn_p_toe(X(end,1:4),p.params);    % touchdown toe pos

    % log
    nt = length(t);
    tout = [tout; t(2:nt)];
    Xout = [Xout; X(2:nt,:)];
    [~,u,F] = dyn_aerial(t,X,p);
    Uout = [Uout;u(2:nt,:)];
    Fout = [Fout;F(2:nt,:)];
    tstart = tout(end);
    
    %% Impact map (hard contact)
    X_prev = Xout(end,:);
    X_post = fcn_impactMap(X_prev,p);
    Xout(end,:) = X_post';

    %% stance phase
    options = odeset('Events',@(t,X)event_liftOff(t,X,p));
    [t,X] = ode45(@(t,X)dyn_stance(t,X,p),[tstart, tfinal], Xout(end,:), options);

    nt = length(t);
    tout = [tout; t(2:nt)];
    Xout = [Xout; X(2:nt,:)];
    [~,u,F] = dyn_stance(t,X,p);
    Uout = [Uout;u(2:nt,:)];
    Fout = [Fout;F(2:nt,:)];
    tstart = tout(end);

    fprintf('%d out of %d steps complete!\n',istep,Nstep)
end
fprintf('Simulation Complete!\n')

%% Visualing the motion
[t,HIP] = animateRobot(tout,Xout,Uout,Fout,p);

