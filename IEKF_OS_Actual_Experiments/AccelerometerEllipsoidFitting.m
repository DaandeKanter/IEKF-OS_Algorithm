function [Bias_Acc, Dinv_Acc] = AccelerometerEllipsoidFitting(RawAcc)

    % First normalize the accelerometer data for later use
    Acc_Normalized = RawAcc./9.81;

    % Parameters of IMU1 data
    N = size(Acc_Normalized,2);
    M = ones(N,13);

    for i = 1:N
        M(i,1:9)   = (kron(Acc_Normalized(:,i),Acc_Normalized(:,i)))';
        M(i,10:12) = (Acc_Normalized(:,i))';
    end

    % Solve the convex optimization problem using CVX
    cvx_begin
        variable A(3,3)
        variable b(3,1)
        variable c(1,1)
        minimize( norm( M * [vec(A); b; c], 2) ) % 2-norm
        subject to
        trace(A) == 1
        A - 0.0001*eye(3) == semidefinite(3)
    cvx_end

    % Determine the inverse of the misallignment matrix D: Dinv
    DinvTDinv_Acc  = inv(0.25 * (b' / A * b) - c) * A;

    % Using the Cholesky decomposition chol(D) factorizes the symmetric
    % positive definite matrix D into an upper triangular R that satisfies:
    % D = R'*R
    % Hence, using chol(DinvTDinv_Acc), the output is Dinv_Acc
    Dinv_Acc = chol(DinvTDinv_Acc,'lower');

    % Determine the bias term for accelerometer
    Bias_Acc = -0.5 * ( A \ b);
    Bias_Acc = Bias_Acc.*9.81;

end