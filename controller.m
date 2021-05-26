classdef controller
    properties
        kp = 20;
        kd = 20;
    end
    methods
        function out = pd_controller(obj, e, e_dot)
            out = -obj.kp*e + -obj.kd*e_dot;
        end
    end
end
