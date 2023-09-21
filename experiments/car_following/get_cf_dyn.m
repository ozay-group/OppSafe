function dyn = get_cf_dyn()

% parameters
dT = 0.25;
u_max = 2;
w_max = 1;

A = [1 dT; 0 1];

B = [-dT^2/2; -dT];

K = [0;0];

E = [dT^2/2; dT];

XU = Polyhedron('H', [0 0 1 u_max;
                      0 0 -1 u_max]);

% constraints on the disturbances
P = Polyhedron('H', [1 w_max;
                     -1 w_max]); 
dyn = Dyn(A, K, B, XU, [],[],[],{zeros(2,2)}, {E}, P);

end