%% NMPC -- Theory and Applications
% Course at Karlsruhe Institute of Technology
% Problem Set #2 Exercise 2
%
%
% Author(s):    
% Date:         April 10, 2017
%% Heun integrator
function xf = heun(ode,h,t,x,u)
    xt = x + h*ode(t,x,u);
    xf = x + h/2*(ode(t,x,u)+ode(t+h,xt,u));
end