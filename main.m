% kalman filter
close all;

% whether to impose noise or not
WITH_NOISE = 0;
WITHOUT_NOISE = 1;
SELECT_W_WO_NOISE = WITH_NOISE;

% select filter
USE_FIRST_ORDER_LPF = 0;
USE_EKF = 1;
USE_UKF = 2;
SELECT_FILTER = USE_EKF;

% simulation time
dt = 0.01;
sim_t = 20;

% initialization
model = dynamics;
model.dt = dt;
model.sim_t = sim_t;
model.t = 0:dt:sim_t;
model.states = zeros(2, length(model.t));

% trajectory initialization
traj = trajectory;
tra = zeros(2, length(model.t));

% controller initilaization
ctrl = controller;
control = zeros(1, length(model.t));

% generate noise
noise = wgn(length(model.t), 2, -100);
noise = noise';
noise = 100*noise;

% filter
myFilter = my_filter;

for i = 2:length(model.t)
    t_now = model.t(i);
    
    % 1-D trajectory
    tra(:, i) = traj.traj_generate(t_now);
    
    % error
    e = model.states(1, i-1) - tra(1, i-1);
    e_dot = model.states(2, i-1) - tra(2, i-1);
    
    % control input
    u = ctrl.pd_controller(e, e_dot);
    control(i) = u;
    
    % dynamics
    X0 = model.states(:, i-1);
    [T, X_new] = ode45(@(t, x) model.update_dynamics(t, x, u), [0, dt], X0, u);
    
    if SELECT_W_WO_NOISE == WITHOUT_NOISE
        model.states(1, i) = X_new(end, 1);
        model.states(2, i) = (model.states(1, i) - model.states(1, i-1))/dt;
    elseif SELECT_W_WO_NOISE == WITH_NOISE
        model.states(1, i) = X_new(end, 1) + noise(1, i);
        model.states(2, i) = (model.states(1, i) - model.states(1, i-1))/dt;
        
        % use filter to deal with the noise
        if SELECT_FILTER == USE_FIRST_ORDER_LPF
            model.states(2, i) = myFilter.first_order_lpf(model.states(2, i), model.states(2, i-1));
        elseif SELECT_FILTER == USE_EKF
            filtered = myFilter.extended_kalman_filter(dt, model.states(1, i-1), model.states(2, i-1), control(i-1), model.states(1, i));
            model.states(:, i) = filtered(1:2, 1);
        elseif SELECT_FILTER == USE_UKF
            filtered = myFilter.unscented_kalman_filter(dt, model.states(1, i-1), model.states(2, i-1), control(i-1), model.states(1, i));
            model.states(:, i) = filtered(1:2, 1);
        end
    end    
end

% plot the position and the velocity
figure
subplot(211)
plot(model.t, model.states(1, :))
y = ylabel('$X$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
hold on
plot(model.t, tra(1, :))
legend('$X$', '$X_{d}$' , 'Interpreter', 'latex')
title('$Position$ $(m)$', 'Interpreter', 'latex')

subplot(212)
plot(model.t, model.states(2, :))
y = ylabel('$V$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
hold on
plot(model.t, tra(2, :))
xlabel('$Time(sec)$', 'Interpreter', 'latex')
legend('$V$', '$V_{d}$' , 'Interpreter', 'latex')
title('$Velocity$ $(m/s)$', 'Interpreter', 'latex')

figure
plot(model.t, noise(1, :))
y = ylabel('$n_{x}$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
xlabel('$Time(sec)$', 'Interpreter', 'latex')
title('$Noise$ $of$ $Position$ $(m)$', 'Interpreter', 'latex')
