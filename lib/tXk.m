function tilde_Xk = tXk(dyn, K, X0, k)
    
    E = cell2mat(dyn.Fd);
    Ac = dyn.A + dyn.B * K;
    x_dim = size(dyn.A, 2);
    u_dim = size(dyn.B, 2);
    I = eye(x_dim);

    H = cell(k+1, 1);
    h = cell(k+1, 1);
    
    for t = 0:k
        Lambda_t = (I - Ac) \ (I - Ac ^ t) * dyn.B;

        d_X0 = @(tau) dyn.D.support((X0.A * Ac ^ tau * E)');
        H_X0 = X0.A * [Ac ^ t, Lambda_t];
        h_X0 = X0.b - sum(cell2mat(arrayfun(d_X0, 0:max(t-1, 0), 'UniformOutput', false)), 2);
    
        U = dyn.XU.projection(x_dim+1:dyn.XU.Dim);
        d_U = @(tau) dyn.D.support((U.A * K * Ac ^ tau * E)');
        H_U = U.A * [K * Ac ^ t, (K * Lambda_t + eye(u_dim))];
        h_U = U.b - sum(cell2mat(arrayfun(d_U, 0:max(t-1, 0), 'UniformOutput', false)), 2);
    
        H{t+1} = [H_X0; H_U];
        h{t+1} = [h_X0; h_U];
    end

    Pi = Polyhedron('A', cell2mat(H), 'b', cell2mat(h));
    tilde_Xk = Pi.projection(1:x_dim);
end