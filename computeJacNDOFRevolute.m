function [JP, JO] = computeJacNDOFRevolute(Ts, p, N)
%computeJacNDOFRevolute compute the jacobian for a revolute robot with N
% joints 
JP = sym(zeros(N, N, N));
JO = sym(zeros(N, N, N));
for i = 1:N
    for j = 1:N
        JP(:, j, i) = cross(Ts(1:3, 3, j), p(1:3, i) - Ts(1:3, 4, j)); % the 3 in the ps is because they are not homogenized
        JO(:, j, i) = Ts(1:3, 3, j);
        if j >= i % the nth joint depend only on the n-1th, n-2th, ... joints
            break
        end
    end
end
end