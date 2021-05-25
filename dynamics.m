classdef dynamics
    properties
        % simulation time
        dt;
        sim_t;
        t;
        
        % states
        states;
    end
    
    methods
        function dX = update_dynamics(~, t, X, u)
            % x_dot = v
            % v_dot = u
            dx = X(2);
            dv = u;
            
            dX = [dx; dv];
        end
    end
end
