% Test file for alpha_dyn.m
A = eye(2);
B = eye(2);
XU = Polyhedron('lb', [-inf;-inf;-1;-1], 'ub',[inf;inf;1;1]);
XU.minHRep;
D = Polyhedron('lb', [-1.1;-1.1],'ub',[1.1;1.1]);
D.minHRep;
dyn = Dyn(A, [0;0], B, XU,[], [], [], ...
    {zeros(2,2), zeros(2,2)}, {[1;0], [0;1]}, D);
Sx = Polyhedron('lb', [-10, -10], 'ub', [10, 100]);
% Formulate alpha-relaxation dynamic system
[dyn1, Sx1, dyn2, Sx2] = alpha_dyn(dyn, Sx);

[~, ~, ~, Sx3] = alpha_dyn(dyn, Sx, -10);


A_ = eye(3);
B_ = [eye(2), eye(2);zeros(1,4)];
Fd_ = [eye(2);0 0];
XU1 = Polyhedron('lb', [-inf;-inf;0;-1;-1;-inf;-inf],'ub',[inf;inf;1;1;1;inf;inf]);
XU2 = Polyhedron('A',[zeros(4,2), D.b, zeros(4,2),D.A], 'b', D.b);
XU3 = Polyhedron('lb', [-inf;-inf;-inf;-1;-1;-inf;-inf],'ub',[inf;inf;0;1;1;inf;inf]);
%% Test the outputs of alpha_dyn
assert(isequal(A_, dyn1.A));
assert(isequal(A_, dyn2.A));
assert(isequal(B_, dyn1.B));
assert(isequal(B_, dyn2.B));
assert(isequal(Fd_, dyn1.Ew));
assert(XU1.intersect(XU2) == dyn1.XU);
assert(XU3.intersect(XU2) == dyn2.XU);
for i=1:size(D.V,1)
    assert(isequal(dyn1.XW_V{i}, [zeros(2,2),D.V(i,:)',zeros(2,1)]));
end
assert(Sx1 == Sx*Polyhedron('lb',0,'ub',1))
assert(Sx3 == Sx*Polyhedron('lb',-10,'ub',-10));