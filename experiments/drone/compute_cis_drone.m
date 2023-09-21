% Compute the CIS for drone dynamics
clc;clear all;close all;
%% drone dynamics (3d)
% load the drone dynamics identified from data
load data/drone_dyn_3d.mat
n = size(A,1);
nd = size(E,2);
I = eye(n);
Ad = cell(1,nd);
Fd = cell(1,nd);
for i = 1:nd
    Ad{i} = zeros(n);
    Fd{i} = E(:,i);
end

u_max = 1; % maximal velocity
a_max = 5; % maximal acceleration
XU1 = Polyhedron('lb',[-inf(n,1);-u_max], 'ub', [inf(n,1);u_max]);
XU2 = Polyhedron('H', [0 0 1 -1 0.1*a_max;0 0 -1 1 0.1*a_max]);
XU = XU1.intersect(XU2).minHRep();
dyn = Dyn(A, zeros(n,1), B, XU, [],[],[], Ad, Fd, D);
dyn.check()
Safe = Polyhedron('lb',[-1;-1;-1], 'ub', [1;1;1]);
% Obtain the auxiliary system Sigma_{[0,1]}
[dyn1, Safe1, dyn2, Safe2] = alpha_dyn(dyn,Safe,-10);
%% Compute the maximal RCIS C_{max,1} of the original sytem
C_max_1 = dyn.win_always(Safe,0, 0, 1);
%% Compute the maximal RCIS C_{max,[0,1]} of the axuiliary system
C_max = dyn1.win_always(Safe1,0,0,1);

%% Compute C_xu (used at runtime to compute admissible input set)
H_xu = [];
h_xu = [];
for i = 1:length(dyn1.XW_V) 
    H_xu = [H_xu;C_max.A*[dyn1.A + dyn1.Ew*dyn1.XW_V{i}(:,1:4), dyn1.B]];
    h_xu = [h_xu;C_max.b];
end

C_xu = Polyhedron('A', H_xu, 'b', h_xu).minHRep;
C_xu = C_xu.intersect(dyn1.XU);
C_xu.minHRep



