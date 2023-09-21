function [ dyn1] = alpha_dyn_d(dyn, alpha)
% Construct a substituted discrete-time system with alpha relaxation
% Systems considered are in form
%               x(k+1) = A x(k) + B u(k) + Ew w(k) + F
% Inputs:
%       dyn (Dyn) --- the original system (only with nonmeasurable
%                     disturbance)
%       alpha (double) --- the value range of alpha or the
%       specified value of alpha
% Outputs:
%       dyn1 (Dyn) --- the auxiliary system with the given alpha

  if ~isa(dyn, 'Dyn')
    error('dyn must be an instance of Dyn');
  end

  
  dyn.D.minHRep();
  if ~dyn.D.hasVRep()
    dyn.D.computeVRep(); 
  end

  if ~dyn.XU.hasHRep()
    dyn.XU.computeHRep();
  end

  matFd = cell2mat(dyn.Fd);

  A = dyn.A;

  F = dyn.F;
  
  B = [dyn.B, matFd];
  [~, n_x] = size(dyn.A);
  [N_u, n_u] = size(dyn.XU.A(:, n_x+1:end));
  [N_d, n_d] = size(dyn.D.A);
  
  H_XU = [dyn.XU.A, zeros(N_u, n_d);
        zeros(N_d, n_x+n_u), dyn.D.A];
  h_XU = [dyn.XU.b; (1-alpha)*dyn.D.b];
  XU = Polyhedron('A', H_XU, 'b', h_XU );
  XU.minHRep;
  D = alpha*dyn.D;
    
  dyn1 = Dyn(A, F, B, XU, [], [], [], dyn.Ad, dyn.Fd, D);
  dyn1.check();
end