function grav = calcGravity(CoM, gAxis, N, m)
syms grav [1, N];
for i = 1:N
    grav(i) = CoM(1:3, i)' * gAxis * m(i);
end
end