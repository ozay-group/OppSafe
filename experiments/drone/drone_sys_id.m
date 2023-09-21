% Identify a linear model based on logged data.
clc; clear all; close all;
filename = 'data/v_flight_data_20230829_203756.csv';
data = readmatrix(filename);
timestamp = data(:, 1) / 1000;
input = data(:, 2:3);
state = data(:, 4:7);
sampling_time = mean(diff(timestamp));

%% least square (1D, 1-step delay, unify x and y dim)
Xp1 = [[state(3:end,[1,3])', state(3:end,[2,4])'];[input(2:end-1,1)', input(2:end-1,2)']];
X = [[state(2:end-1,[1,3])', state(2:end-1,[2,4])'];[input(1:end-2,1)', input(1:end-2,2)']];
U = [input(2:end-1,1)', input(2:end-1,2)'];

SYS = Xp1/[X;U]
A = SYS(:,1:3);
B = SYS(:,4);
C = [eye(2), zeros(2,1)];
A(3,:) = 0;

Xp1_est = [A, B]*[X;U];


figure(1);
for i = 1:2
    subplot(2,1,i);
    plot(Xp1(i,:));
    hold on;
    plot(Xp1_est(i,:));
end

Q = 4*eye(3);
R = eye(1);

[K, S, e] = dlqr(A, B, Q, R);

D_samples = Xp1-Xp1_est;
D = Polyhedron('V',D_samples(1:2,:)');
D.minHRep()
E = [eye(2);0 0];
% examine for outliers in D_samples
plot(D)
hold on;
plot(D_samples(1,:), D_samples(2,:),'x')
% remove outlier
D_samples(:,1864) = [];
D = Polyhedron('V',D_samples(1:2,:)');
D.minHRep()
save data/drone_dyn_3d.mat