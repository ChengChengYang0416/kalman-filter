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
        
        % for unscented kalman filter
        alpha_ukf = 1e-3;
        beta = 2;
        L = 3;
        kappa = 0;
        lambda;
        
        weight_state;
        weight_covariance;
        weight_state_original;
        weight_covariance_original;
        
        P_ukf = [1, 0, 0;
                 0, 1, 0;
                 0, 0, 1];
             
        Q_ukf = [2, 0, 0;
             0, 2, 0;
             0, 0, 2];
         
        sigma_points_delta;
        sigma_points;
        sigma_points_predict;
        sigma_points_correct;
        C_ukf;
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
        
        function filtered = unscented_kalman_filter(obj, dt, x_last, v_last, u_last, x_m)
            obj.lambda = obj.alpha_ukf^2*(obj.L+obj.kappa) - obj.L;
            obj.sigma_points_delta = sqrtm((obj.L+obj.lambda)*obj.P_ukf);
            obj.sigma_points_delta = obj.sigma_points_delta';
            obj.sigma_points = zeros(obj.L, 2*obj.L+1);
            
            % sigma points
            obj.sigma_points(:, 1) = [x_last; v_last; u_last];
            obj.sigma_points(:, 2) = [x_last; v_last; u_last] + obj.sigma_points_delta(:, 1);
            obj.sigma_points(:, 3) = [x_last; v_last; u_last] + obj.sigma_points_delta(:, 2);
            obj.sigma_points(:, 4) = [x_last; v_last; u_last] + obj.sigma_points_delta(:, 3);
            obj.sigma_points(:, 5) = [x_last; v_last; u_last] - obj.sigma_points_delta(:, 1);
            obj.sigma_points(:, 6) = [x_last; v_last; u_last] - obj.sigma_points_delta(:, 2);
            obj.sigma_points(:, 7) = [x_last; v_last; u_last] - obj.sigma_points_delta(:, 3);
            
            % sigma points through dynamics
            for i = 1:2*obj.L+1
                obj.sigma_points_predict(1, i) = obj.sigma_points(1, i) + obj.sigma_points(2, i)*dt;
                obj.sigma_points_predict(2, i) = obj.sigma_points(2, i) + obj.sigma_points(3, i)*dt;
                obj.sigma_points_predict(3, i) = obj.sigma_points(3, i) + u_last;
            end
            
            % weight
            obj.weight_state_original = obj.L/(obj.L+obj.lambda);
            obj.weight_state = 1/(2*(obj.L+obj.lambda));
            obj.weight_covariance_original = obj.lambda/(obj.L+obj.lambda)+(1-obj.alpha_ukf^2+obj.beta);
            obj.weight_covariance = 1/(2*(obj.L+obj.lambda));
            
            % prediction
            x_predict = obj.weight_state_original*obj.sigma_points_predict(:, 1);
            for i = 2:2*obj.L+1
                x_predict = x_predict + obj.weight_state*obj.sigma_points_predict(:, i);
            end
            
            % covariance of prediction
            P_prediction = obj.weight_covariance_original*(obj.sigma_points_predict(:, 1)-x_predict)*...
                            (obj.sigma_points_predict(:, 1)-x_predict)';
            for i = 2:2*obj.L+1
                P_prediction = P_prediction + obj.weight_covariance*(obj.sigma_points_predict(:, 1)-x_predict)*...
                            (obj.sigma_points_predict(:, 1)-x_predict)';
            end
            
            % correction
            H = [1, 0, 0];
            obj.sigma_points_correct = zeros(1, 2*obj.L+1);
            for i = 1:2*obj.L+1
                obj.sigma_points_correct(i) = H*obj.sigma_points_predict(:, i);
            end
            
            y_correct = obj.weight_state_original*obj.sigma_points_correct(1);
            for i = 2:2*obj.L+1
                y_correct = y_correct + obj.weight_state*obj.sigma_points_correct(i);
            end
            
            P_yy = obj.weight_covariance_original*(obj.sigma_points_correct(1)-y_correct)*...
                    (obj.sigma_points_correct(1)-y_correct)';
            P_xy = obj.weight_covariance_original*(obj.sigma_points_predict(:, 1)-x_predict)*...
                    (obj.sigma_points_correct(1)-y_correct)';
            for i = 2:2*obj.L+1
                P_yy = P_yy + obj.weight_covariance*(obj.sigma_points_correct(i)-y_correct)*...
                        (obj.sigma_points_correct(i)-y_correct);
                P_xy = P_xy + obj.weight_covariance*(obj.sigma_points_predict(:, i)-x_predict)*...
                        (obj.sigma_points_correct(i)-y_correct)';
            end
            
            K = P_xy/P_yy;
            x_correct = x_predict + K*(x_m-y_correct);
            filtered = x_correct;
        end
    end
end
