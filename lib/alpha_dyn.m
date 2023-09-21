function [ dyn1, Sx1, dyn2, Sx2] = alpha_dyn(dyn, Sx, alpha)
% Construct the axiliary system \Sigma_{[0,1]} and \Sigma_{\alpha}
% Systems considered are in form
%               x(k+1) = A x(k) + B u(k) + Ew w(k) + F
% Inputs:
%       dyn (Dyn) --- the original system (only with nonmeasurable
%                     disturbance)
%       Sx (Polyhedron) -- the safe set of the original system
%       alpha (double or interval) --- the value range of alpha or the
%       specified value of alpha
% Outputs:
%       dyn1 (Dyn) --- the lifted system with alpha in [0,1]
%       Sx1 (Polyhedron) --- the lifted safe set of dyn1
%       dyn2 (Dyn) --- the lifted system with specificed alpha
%       Sx2 (Polyhedron) --- the lifted safe set of dyn2

  if ~isa(dyn, 'Dyn')
    error('dyn must be an instance of Dyn');
  end

  if ~isa(Sx, 'Polyhedron')
    error('S_x must be an instance of Polyhedron');
  end
  
  dyn.D.minHRep();
  if ~dyn.D.hasVRep()
    dyn.D.computeVRep(); 
  end

  if ~dyn.XU.hasHRep()
    dyn.XU.computeHRep();
  end

  matFd = cell2mat(dyn.Fd);

  A = blkdiag(dyn.A, 1);

  F = [dyn.F; zeros(1, size(dyn.F, 2))];
  
  B = [dyn.B, matFd; zeros(1, size(dyn.B, 2) + size(matFd, 2))];

  Ew = [matFd; zeros(1, size(matFd, 2))];

  [~, n_x] = size(dyn.A);
  [N_u, n_u] = size(dyn.XU.A(:, n_x+1:end));
  [N_d, n_d] = size(dyn.D.A);
  
  H_XU = [zeros(1, n_x), 1, zeros(1, n_u + n_d);
        zeros(1, n_x), -1, zeros(1, n_u + n_d);
        dyn.XU.A(:, 1:n_x), zeros(N_u, 1), dyn.XU.A(:, n_x+1:end), zeros(N_u, n_d);
        zeros(N_d, n_x), dyn.D.b, zeros(N_d, n_u), dyn.D.A];
  h_XU = [1; 0; dyn.XU.b; dyn.D.b];
  XU = Polyhedron('A', H_XU, 'b', h_XU );
  XU.minHRep;
  
  [n_w, n_d] = size(dyn.D.V);
%   verts = mat2cell(dyn.D.V, ones(1, n_w), n_d);
  XW_V = cell(n_w, 1);
  for i = 1:n_w
    XW_V{i} = [zeros(n_d, n_x), dyn.D.V(i,:)', zeros(n_d, 1)];
  end
  
  dyn1 = Dyn(A, F, B, XU, [], [], [], [], [], [], [], [], Ew, XW_V);

  Alpha = Polyhedron('A', [1; -1], 'b', [1; 0]);
  Sx1 = Sx * Alpha;
  Sx1.minHRep;
  dyn1.check();
  
  H_XU = [zeros(1, n_x), 1, zeros(1, n_u + n_d);H_XU(3:end,:)];
  h_XU = [0;h_XU(3:end)];
  XU = Polyhedron('A', H_XU, 'b', h_XU );
  XU.minHRep;
  dyn2 = Dyn(A, F, B, XU);
  dyn2.check()
  if nargin < 3
      Alpha = Polyhedron('A', 1, 'b', 0);
  elseif isvector(alpha) && all(size(alpha) == [1, 2])
      Alpha = Polyhedron('lb', alpha(1), 'ub', alpha(2));
  elseif isscalar(alpha)
      Alpha = Polyhedron('lb', alpha, 'ub', alpha);
  end
  Sx2 = Sx * Alpha;
  Sx2.minHRep;
end