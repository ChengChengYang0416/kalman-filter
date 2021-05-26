classdef my_filter
    methods
        function filtered = first_order_lpf(obj, new, old)
            alpha = 0.2;
            filtered = alpha*new + (1-alpha)*old;
        end
    end
end
