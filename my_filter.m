classdef my_filter < handle
    properties
        % for first order low pass filter
        alpha = 0.2
        
        % for extended kalman filter
        A;
        C;
        P = [1, 0, 0;
             0, 1, 0;
             0, 0, 1];
        Q = [2, 0, 0;
             0, 2, 0;
             0, 0, 2];
        R = 1;
    end
    methods
        function filtered = first_order_lpf(obj, new, old)
            filtered = obj.alpha*new + (1-obj.alpha)*old;
        end
        
        function filtered = extended_kalman_filter(obj, dt, x_last, v_last, u, x_m)
            obj.A = [1, dt,  0;
                     0,  1, dt;
                     0,  0,  0];
            obj.C = [1, 0, 0];
            x_predict = [x_last + v_last*dt;
                         v_last + u*dt;
                         u];
            P_predict = obj.A*obj.P*obj.A'+obj.Q;
            K = P_predict*obj.C'*(inv(obj.C*P_predict*obj.C'+obj.R));
            filtered = x_predict + K*(x_m - obj.C*x_predict);
            obj.P = (1 - K*obj.C)*P_predict;
        end
    end
end
