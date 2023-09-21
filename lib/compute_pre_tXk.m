function [tX_xu, pre_tXk] = compute_pre_tXk(dyn, tXk)
    nx = size(dyn.A, 2);
    nu = size(dyn.B, 2);
    nv = length(dyn.D.V);
    ntXk = length(tXk.b);
    H = zeros(nv * ntXk, nx + nu);
    h = zeros(nv * ntXk, 1);
    Fd = cell2mat(dyn.Fd);
    for i = 1:nv
        H((i-1)*ntXk + (1:ntXk),:) = tXk.A * [dyn.A, dyn.B];
        h((i-1)*ntXk + (1:ntXk)) = tXk.b - tXk.A * Fd * dyn.D.V(i, :);
    end

    tX_xu = Polyhedron('H', [H, h]).intersect(dyn.XU);
    tX_xu.minHRep;
    
    pre_tXk = tX_xu.projection(1:nx);
    
end