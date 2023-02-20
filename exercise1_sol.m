clc
clear
close all
%%

N = 3;

% Define symbols
syms q [N 1] real
syms dq [N 1] real
syms ddq [N 1] real

% DH parameters
% Kinematics
syms a [N 1] real
syms alpha [N 1] real
syms d [N 1] real

a = [0, a2, a3]; 
alpha = [sym(pi)/2, 0, 0];
d = [d1, 0, 0];

% Dynamics
syms m [N 1] real
syms pl [N 3] real

syms g real
g0 = [0; 0; -g];

% syms Q [N 1] real

% % Inertia tensor
% I3 = sym('I3', [3,3]);
% I3 = tril(I3,0) + tril(I3,-1).';
% I2 = sym('I2', [3,3]);
% I2 = tril(I2,0) + tril(I2,-1).';
% I1 = sym('I1', [3,3]);
% I1 = tril(I1,0) + tril(I1,-1).';

syms I1 [3 1] real;
syms I2 [3 1] real;
syms I3 [3 1] real;

I1 = diag(I1);
I2 = diag(I2);
I3 = diag(I3);


%%
% Transformation between from link i-1 to link i.
for i = 1:N
    ci = cos(q(i));
    si = sin(q(i));
    T{i} = [ci, -si * cos(alpha(i)),  si * sin(alpha(i)), a(i) * ci; 
            si,  ci * cos(alpha(i)), -ci * sin(alpha(i)), a(i) * si;
             0,       sin(alpha(i)),       cos(alpha(i)),      d(i);
             0,                   0,                   0,         1];
end

T_0_0 = eye(4);
T_0_1 = simplify(T{1});
T_0_2 = simplify(T{1} * T{2});
T_0_3 = simplify(T{1} * T{2} * T{3});

% vpa(T_0_1, 4)
% vpa(T_0_2, 4)
% vpa(T_0_3, 4)
%% Coordinates for CoM for each link in base frame.
p_0_l0 = zeros(3, 1);
p_0_l1 = eye(3, 4) * T_0_1 * [pl(1, :)'; 1];
p_0_l2 = eye(3, 4) * T_0_2 * [pl(2, :)'; 1];
p_0_l3 = eye(3, 4) * T_0_3 * [pl(3, :)'; 1];
% 
% p_0_l1 = simplify(p_0_l1)
% p_0_l2 = simplify(p_0_l2)
% p_0_l3 = simplify(p_0_l3)

%% Compute the Jacobians (given in base frame)
% Orientation
J_O_l1 = [T_0_0(1:3, 3), zeros(3, 2)];
J_O_l2 = [T_0_0(1:3, 3), T_0_1(1:3, 3), zeros(3, 1)];
J_O_l3 = [T_0_0(1:3, 3), T_0_1(1:3, 3), T_0_2(1:3, 3)];

% % J_O_l1 = simplify(J_O_l1)
% J_O_l2 = simplify(J_O_l2)
% J_O_l3 = simplify(J_O_l3)

% Translational
J_P_l1 = [cross(T_0_0(1:3, 3), (p_0_l1 - T_0_0(1:3, 4))), ...
          zeros(3, 2)];
J_P_l2 = [cross(T_0_0(1:3, 3), (p_0_l2 - T_0_0(1:3, 4))), ...
          cross(T_0_1(1:3, 3), (p_0_l2 - T_0_1(1:3, 4))), ...
          zeros(3, 1)];
J_P_l3 = [cross(T_0_0(1:3, 3), (p_0_l3 - T_0_0(1:3, 4))), ...
          cross(T_0_1(1:3, 3), (p_0_l3 - T_0_1(1:3, 4))), ...
          cross(T_0_2(1:3, 3), (p_0_l3 - T_0_2(1:3, 4)))];
      
% J_P_l1 = simplify(J_P_l1)
% J_P_l2 = simplify(J_P_l2)
% J_P_l3 = simplify(J_P_l3)

%% Export end effector Jacobian
J_O_ee = [T_0_0(1:3, 3) T_0_1(1:3, 3) T_0_2(1:3, 3)];
J_P_ee = [cross(T_0_0(1:3, 3), T_0_3(1:3, 4) - T_0_0(1:3, 4)), ...
          cross(T_0_1(1:3, 3), T_0_3(1:3, 4) - T_0_1(1:3, 4)), ...
          cross(T_0_2(1:3, 3), T_0_3(1:3, 4) - T_0_2(1:3, 4))];


Jee = simplify([J_P_ee; J_O_ee]);
% matlabFunction(Jee, 'File', 'jacobian', 'Outputs', {'J'});


%% Compute inertia tensor for each link (given in base frame)
I_0_l1 = T_0_1(1:3, 1:3) * I1 * T_0_1(1:3, 1:3)';
I_0_l2 = T_0_2(1:3, 1:3) * I2 * T_0_2(1:3, 1:3)';
I_0_l3 = T_0_3(1:3, 1:3) * I3 * T_0_3(1:3, 1:3)';

% I_0_l1 = simplify(I_0_l1)
% I_0_l2 = simplify(I_0_l2)
% I_0_l3 = simplify(I_0_l3)

%% Compute gravity term
Epot = -(m(1) * g0' * p_0_l1 + ...
         m(2) * g0' * p_0_l2 + ...
         m(3) * g0' * p_0_l3);
% Epot = simplify(Epot);
grav = jacobian(Epot, q)';
% grav = simplify(grav)
% matlabFunction(grav, 'File', 'gravity', 'Outputs', {'grav'});



%% Compute inertia tensor
B = m(1) * (J_P_l1' * J_P_l1) + ...
        J_O_l1' * I_0_l1 * J_O_l1 + ...
    m(2) * (J_P_l2' * J_P_l2) + ...
        J_O_l2' * I_0_l2 * J_O_l2 + ...
    m(3) * (J_P_l3' * J_P_l3) + ...
        J_O_l3' * I_0_l3 * J_O_l3
    
% Ekin = (1/2) * dq' * B * dq;
% Ekin = simplify(Ekin)

% matlabFunction(B, 'File', 'inertiaMatrix', 'Outputs', {'B'});

%% Compute coriolis and centrifugal term
C = sym(zeros(N, N));
for i=1:N
    for j=1:N
        for k=1:N
            C(i, j) = C(i, j) + 0.5 * (...
                diff(B(i, j), q(k)) + ...
                diff(B(i, k), q(j)) - ...
                diff(B(j, k), q(i)) ...
            ) * dq(k);
            C(i, j) = simplify(C(i, j));
        end
    end
end

% C;
% matlabFunction(C, 'File', 'coriolisMatrix', 'Outputs', {'C'});
