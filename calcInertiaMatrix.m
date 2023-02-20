function B = calcInertiaMatrix(JO, JP, m, I, N)
B = sym(zeros(N, N));
for i=1:N
    B = B + ...
        m(i)*(JP(:,:,i)'*JP(:,:,i)) + ... % term of the kinetic energy based on mass and speed
        JO(:,:,i)'*I(:,:,i)*JO(:,:,i); % term based on Is and angular speed
end
end

