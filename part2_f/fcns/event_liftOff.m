function [zeroCrossing,isterminal,direction] = event_liftOff(t,X,p)

global GRFz
Tst = p.Tst;
tTD = p.tTD;

% zeroCrossing =  t - tTD - Tst;
zeroCrossing =  GRFz;
isterminal   =  1;
direction    =  -1;




