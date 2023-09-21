function [ dyn_, Sx_ ] = k_dyn(dyn, K, Sx)

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
    
  n = size(dyn.A, 2);
  m = dyn.XU.Dim - n;

  A = [dyn.A + dyn.B * K, dyn.B;
       zeros(m, n), eye(m)];

  H_x = dyn.XU.A(:, 1:n);
  H_u = dyn.XU.A(:, n+1:end);
  H_XU = [H_u * K + H_x, H_u];
  h_XU = dyn.XU.b;          
  XU = Polyhedron('A', H_XU, 'b', h_XU );
  XU.minHRep;

  nd = length(dyn.Fd);
  Ad = cellfun(@(x) zeros(m + n), cell(1, nd), 'UniformOutput', false);
  Fd = cellfun(@(x) [x; zeros(m, 1)], dyn.Fd, 'UniformOutput', false);
  D = dyn.D;
  
  F = [dyn.F; zeros(1, size(dyn.F, 2))];
  
  dyn_ = Dyn(A, F, [], [], [], [], [], Ad, Fd, D);

  V = Polyhedron('lb', -inf * ones(m, 1), 'ub', inf * ones(m, 1));

  Sx_ = XU.intersect(Sx * V);
  Sx_.minHRep;
  dyn_.check();
  
end