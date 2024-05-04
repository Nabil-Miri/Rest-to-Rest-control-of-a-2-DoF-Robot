%% NMPC -- Theory and Applications
% Course at Karlsruhe Institute of Technology
% Problem Set #2 Exercise 2
%
%
% Date:         April 10, 2017
%% Euler forward integrator
function xf = eulerf(f,h,t,x,u)
    xf = x + h*f(t,x,u);
end