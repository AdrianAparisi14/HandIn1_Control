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

syms I1 [3 1] real;
syms I2 [3 1] real;
syms I3 [3 1] real;

I1 = diag(I1);
I2 = diag(I2);
I3 = diag(I3);
%% Kinematic structure of robot
% we have to find with the Denavit Hartenberg method the transformation
% matrix between the frame and from that from the base to each frame
T00 = eye(4);
T01 = DHParamGen(d(1), q(1), a(1), alpha(1));
T12 = DHParamGen(d(2), q(2), a(2), alpha(2));
T23 = DHParamGen(d(3), q(3), a(3), alpha(3));
T02 = T01*T12;
T03 = T02*T23;
Ts = reshape([T00 T01 T02 T03], [4,4,4]); % the three transform matrix to base

% ok
%% Coordinates for CoM for each link in base frame.
% they are calculated with the transformation between the base frame and
% the frame we want to calculate the center of mass of each column of pcl
% will be the center of mass for that joint
CoM1 = eye(3,4) * T01 * [pl(1,:) 1]';
CoM2 = eye(3,4) * T02 * [pl(2,:) 1]';
CoM3 = eye(3,4) * T03 * [pl(3,:) 1]';
CoM = [CoM1 CoM2 CoM3]; % each columns is a CoM

% ok
%% Compute the Jacobians (given in base frame)
% we use the formulas to find the jacobian, tha jacobian is related to a
% point, usually is the end effector or a center of mass. It is divided in
% translational Jacobian and Orientational, each column is a joint and each
% row is a axis (x, y, z)

% J=[[-70   0   0   ]  This is the translational part of the jacobian                     
%    [  0  805  0   ]  a value that indicates how much a revolution of a
%    [  0   0   0   ]  joint moves the point the jacobian is reffered to

%    [  0  -1  -1   ]  From here there is the orientation part, the vertical
%    [  0   0   0   ]  vector are always unitari, it indicates the axis around
%    [  1   0   0   ]] the joint rotates to create the said movement 
%                      (first: "one" rotation around the z axis will cause 
%                       "-70" movement in the x axis on the end effector)

% to find the J for the center of mass of the link we use the formulas below, 
% because our robot is made of revolute joint 

[JP, JO] = computeJacNDOFRevolute(Ts, CoM, N);
% ok
%% Compute inertia tensor for each link (given in base frame)
% We have to found the momento d'inerzia of each frame joint 
% I01 = R01 * I1 * R01' where R01 is the rotation matrix from frame 0 to 1 
% and I1 is the inertia tensor for the joint given by the constructor
I01 = T01(1:3, 1:3) * I1 * T01(1:3, 1:3)';
I02 = T02(1:3, 1:3) * I2 * T02(1:3, 1:3)';
I03 = T03(1:3, 1:3) * I3 * T03(1:3, 1:3)';
Is = reshape([I01 I02 I03], [3, 3, N]);
%% Compute gravity term
% the gravity term of each link is gonna be the the force that will act on
% the robot, since its gave in the system of reference of the base its
% dependent on the pcl beside the g and the masses
gravTerms = calcGravity(CoM, g0, N, m);
Epot = -sum(gravTerms);
%grav = simplify(jacobian(Epot, q)');
grav = jacobian(Epot, q)';
matlabFunction(grav, 'File', 'gravity', 'Outputs', {'grav'}); % takes your syms value and define a function depending on the "basics" parameters

%% Compute inertia tensor
% The B matrix gives us the kinetic energy of the manipulator depending
% just on the Q, it will depend on the mass by the speed and also by the
% rotational factor depending on omega and I of the joints
B = calcInertiaMatrix(JO, JP, m, Is, N);
matlabFunction(B, 'File', 'inertiaMatrix', 'Outputs', {'B'}); 
%% Compute coriolis and centrifugal term
% the Coriolis forces are apparent forces that an object moving has in
% respect to another object moving
C = calcCoriolisMatrix(q, dq, B, N);
matlabFunction(C, 'File', 'coriolisMatrix', 'Outputs', {'C'});