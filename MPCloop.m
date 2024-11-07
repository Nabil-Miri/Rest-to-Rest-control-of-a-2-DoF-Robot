%% NMPC -- TU Dortmund, ie3, Prof. Dr.-Ing. Timm Faulwasser 
%% MPC Loop function

function [fx, fu, x,u] = MPCloop(U,X,X0,ocp,myfun,dt,tf,xinit,uinit)
% inputs:
%   X0      optipar for initial condition for each OCP
%   U       optivar for MPC inputs
%   sol     parameterized solution to OCP
%   x0      numerical value of initial condition
    t           =   0:dt:tf;
    N           =   length(t)-1;
    nx          =   length(xinit);
    [nu,Nmpc]   =   size(U);
    x           =   zeros(nx,N+1);
    u           =   zeros(nu,N);
    x(:,1)      =   xinit;
    Xsol        =   repmat(xinit,1,Nmpc+1);
    Usol        =   repmat(uinit,1,Nmpc);
    measurement_noise_variance = 0.01;

    % Define obstacle position and size
    obstacle_position = [1; 1]; % Example position
    obstacle_radius = 0.5; % Example radius

    for k = 1:N
        % set current state
        ocp.set_value(X0, x(:,k) ) % + sqrt(measurement_noise_variance) * randn(size(x(:, k)))); 
        % initialize OCP
        ocp.set_initial(X,Xsol);
        ocp.set_initial(U,Usol);
        
        % Add obstacle constraints
        for i = 1:Nmpc+1
            distance_to_obstacle = norm(Xsol(1:2, i) - obstacle_position);
            ocp.subject_to(distance_to_obstacle >= obstacle_radius);
        end
        
        % resolve OCP
        sol = ocp.solve();
        Xsol    =   sol.value(X);
        Usol    =   sol.value(U);
        if k == 1
            fx = Xsol; fu = Usol;
        end      
        % get first element
        u(:,k)   =   Usol(:,1);
        % apply to system using rk4
        x(:,k+1) =   rk4(@(t,x,u)myfun(t,x,u),dt,t(k),x(:,k),u(:,k));
    end
end
