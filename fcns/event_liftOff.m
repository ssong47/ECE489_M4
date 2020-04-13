function [zeroCrossing,isterminal,direction] = event_liftOff(t,X,p)

global Fz

Tst = p.Tst;
tTD = p.tTD;

% zeroCrossing =  t - tTD - Tst;
zeroCrossing =  Fz;
isterminal   =  1;
direction    =  1;




