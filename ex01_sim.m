clc 
clear
close all
%%
N = 3;

a = [0, -0.425, -0.3922];
d = [0.1625, 0, 0];

m = [3.761, 8.058, 2.846];
pl1 = [0, -0.02561, 0.00193];
pl2 = [0.2125, 0, 0.11336];
pl3 = [0.15, 0, 0.0265];

I1 = [0.0084, 0.0064, 0.0084];
I2 = [0.0078, 0.21, 0.21];
I3 = [0.0016, 0.0462, 0.0462];

g = 9.82;


q = [1, pi/3, pi/3]';
dq = [0, 0, 0]';    
ddq = [0, 0, 0]';
Q = [0, 0, 0]';

% syms q [N 1] real
% syms dq [N 1] real
% syms ddq [N 1] real

getG = @(q) gravity(a(2),a(3),g,m(2),m(3),pl2(1),pl2(2),pl3(1),pl3(2),q(2),q(3));
getB = @(q) inertiaMatrix(I1(2),I2(1),I2(2),I2(3),I3(1),I3(2),I3(3),a(2),a(3),m(1),m(2),m(3),pl1(1),pl1(3),pl2(1),pl2(2),pl2(3),pl3(1),pl3(2),pl3(3),q(1),q(2),q(3));
getC = @(q, dq) coriolisMatrix(I2(1),I2(2),I3(1),I3(2),I3(3),a(2),a(3),d(1),dq(1),dq(2),dq(3),m(2),m(3),pl2(1),pl2(2),pl2(3),pl3(1),pl3(2),pl3(3),q(1),q(2),q(3));
%% Compute joint torques at a specific configuration
tau = getB(q)*ddq + getC(q, dq)*dq + getG(q);
% disp("With the given conf of: " + num2str(q(:).'));
% disp("the required torque is: " + num2str(tau(:).'));
%% Simulate the system using ode45
% *** Insert code here ***
% Simple example:
% odefun = @(t,   y) [y(2); -y(1)];
% [t, y] = ode45(odefun,[0 9],[0 1])

q = [1, pi/3, pi/3]';
dq = [0, 0, 0]';
ddq = [0, 0, 0]';

N = length(q);
D = 5 * eye(N);

forwardDyn = @(q, dq) inv(getB(q)) * (-D * dq - getC(q, dq)*dq - getG(q)); %zeros(3,1) actually should be getC but in this script gives a matrix 3x3 and must be 3x1

% odefun and ode45 are matlab tools to solve differential equations.
% First we have to convert to a state-variable form so the first term will
% be x1dot = x2 = dq  and  x2dot = ddq
odefun = @(t, y) [y(N+1:end); 
                  forwardDyn(y(1:N), y(N+1:end))];
              
tic
[t, y] = ode45(odefun, [0 5], [q; dq]);
toc

% I've computed tau as it is indicated in the title of this exercise
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
ylabel('$\tau [N \cdot t/m]$')
legend(["Joint 1", "Joint 2", "Joint 3"], ...
    'NumColumns', 1, ...
    'Location', 'northeast')

% ax = gca;
% Requires R2020a or later
exportgraphics(fig,'ex1_simluation.pdf', 'BackgroundColor', 'none') 
