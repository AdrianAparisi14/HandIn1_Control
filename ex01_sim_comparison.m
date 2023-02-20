clc 
clear
close all
%% Load Robot Model
robot = load("ur5e\ur5e_3.mat");
robot = robot.robotUR5e;

robot.DataFormat = 'column';
robot.Gravity = [0, 0, -9.82]';

%% Solution to Exercise 3
q = [1, pi/3, pi/3]';
dq = [0, 0, 0]';
ddq = [0, 0, 0]';

disp('Solution to Exercise 3:')
robot.inverseDynamics(q, dq, ddq)
%% Solution to Exercise 4
N = length(q);
D = 5 * eye(N);

q = [1, pi/3, pi/3]';
dq = [0, 0, 0]';
ddq = [0, 0, 0]';

forwardDyn = @(q, dq) inv(robot.massMatrix(q)) * (-D * dq - robot.velocityProduct(q, dq) - robot.gravityTorque(q));

odefun = @(t, y) [y(N+1:end); 
                  forwardDyn(y(1:N), y(N+1:end))];
              
tic
[t, y] = ode45(odefun, [0 5], [q; dq]);
toc

sim_q = y(:, 1:3);
sim_dq = y(:, 4:6);
sim_tau = -D*sim_dq';


%% plots
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

fig = figure;
fig.Units               = 'centimeters';
fig.Position(3)         = 8; % width
fig.Position(4)         = 7; % height

subplot(6,1,1:2)
plot(t, sim_q)
ylabel('$q$ [rad]');
grid on
ylim([-0.1, 1.8])
yticks([0, pi/3, pi/2])
yticklabels({'0', '$\frac{\pi}{3}$', '$\frac{\pi}{2}$'})
xticklabels({})

subplot(6,1,3:4)
plot(t, sim_dq)
grid on
xlabel("Time [s]")
ylim([-1.2, 1.2])
ylabel('$\dot{q}$ [rad/s]')
legend(["Joint 1", "Joint 2", "Joint 3"], ...
    'NumColumns', 1, ...
    'Location', 'northeast')

subplot(6,1,5:6)
plot(t, sim_tau)
grid on
xlabel("Time [s]")
ylim([-5.5, 5.5])
ylabel('\tau [N \cdot t/m]')
legend(["Joint 1", "Joint 2", "Joint 3"], ...
    'NumColumns', 1, ...
    'Location', 'northeast')

% ax = gca;
% Requires R2020a or later
exportgraphics(fig,'ex1_simluation.pdf', 'BackgroundColor', 'none') 
