% kalman filter
close all;

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
    
    % save the states
    model.states(1, i) = X_new(end, 1);
    model.states(2, i) = X_new(end, 2);
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
