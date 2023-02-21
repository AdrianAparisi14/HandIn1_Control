function C = calcCoriolisMatrix(q, dq, B, N)
C = sym(zeros(N, N));
for k=1:N
    for j=1:N
        for i=1:N
            C(k,j) = C(k,j) + 0.5 * (diff(B(k,j), q(i)) + diff(B(k,i), q(j)) - diff(B(i,j), q(k))) * dq(i);
        end
    end
end
end

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