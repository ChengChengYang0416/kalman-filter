classdef trajectory
    methods 
        function out = traj_generate(~, t)
            f = 0.1;
            out = zeros(2, 1);
            out(1) = sin(2*pi*f*t);
            out(2) = 2*pi*f*cos(2*pi*f*t);
        end
    end
end
