% kalman filter
close all;

% whether to impose noise or not
WITH_NOISE = 0;
WITHOUT_NOISE = 1;
SELECT_W_WO_NOISE = WITH_NOISE;

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

% generate noise
noise = wgn(length(model.t), 2, -100);
noise = noise';
noise = 100*noise;

for i = 2:length(model.t)
    t_now = model.t(i);
    
    % 1-D trajectory
    tra(:, i) = traj.traj_generate(t_now);
    
    % error
    e = model.states(1, i-1) - tra(1, i-1);
    e_dot = model.states(2, i-1) - tra(2, i-1);
    
    % control input
    u = ctrl.pd_controller(e, e_dot);
    
    % dynamics
    X0 = model.states(:, i - 1);
    [T, X_new] = ode45(@(t, x) model.update_dynamics(t, x, u), [0, dt], X0, u);
    
    if SELECT_W_WO_NOISE == WITHOUT_NOISE
        model.states(1, i) = X_new(end, 1);
        model.states(2, i) = (model.states(1, i) - model.states(1, i-1))/dt;
    elseif SELECT_W_WO_NOISE == WITH_NOISE
        model.states(1, i) = X_new(end, 1) + noise(1, i);
        model.states(2, i) = (model.states(1, i) - model.states(1, i-1))/dt;
    end    
end

figure
subplot(211)
plot(model.t, model.states(1, :))
hold on
plot(model.t, tra(1, :))

subplot(212)
plot(model.t, model.states(2, :))
hold on
plot(model.t, tra(2, :))

figure
plot(model.t, noise(1, :))
hold on
plot(model.t, noise(2, :))
