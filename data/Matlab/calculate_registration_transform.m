% Registers two sets of 3DoF data
% Assumes A and B are d,n sets of data
% where d is the dimension of the system 
% typically d = 2,3
% and n is the number of points
% typically n>3
%
% Mili Shah
% July 2014
clear all
close all 
clc

%% Get .csv file into table T
% Filename
name = 'registration_points_all';

% Read data from table
T = readtable(strcat(name,'.csv'), 'HeaderLines',0);

%% Extract values from table
n = width(T);                   
A_orig = T{1:3,:};
B_orig = T{4:6,:};

%% Calculate registration transform
%Mean Center Data
Ac = mean(A_orig,2);
Bc = mean(B_orig,2);
A = A_orig-repmat(Ac,1,n);
B = B_orig-repmat(Bc,1,n);

%Calculate Optimal Rotation
M = A*B';
N = [M(1,1)+M(2,2)+M(3,3)  M(2,3)-M(3,2)          M(3,1)-M(1,3)          M(1,2)-M(2,1);...
     M(2,3)-M(3,2)         M(1,1)-M(2,2)-M(3,3)   M(1,2)+M(2,1)          M(3,1)+M(1,3);...
     M(3,1)-M(1,3)         M(1,2)+M(2,1)         -M(1,1)+M(2,2)-M(3,3)   M(2,3)+M(3,2);...
     M(1,2)-M(2,1)         M(3,1)+M(1,3)          M(2,3)+M(3,2)         -M(1,1)-M(2,2)+M(3,3)];
[u,v] = eig(N);
[m,ind] = max(diag(v));
q = u(:,ind);
R = (q(1)^2-q(2:4)'*q(2:4))*eye(3) + 2*q(2:4)*q(2:4)'+2*q(1)*[0 -q(4) q(3); q(3) 0 -q(2); -q(3) q(2) 0];
%Calculate Optimal Translation
t = Bc - R*Ac;
regist_tf = [t;q];
csvwrite('registration_matlab.csv',regist_tf)

qr = quaternion(q(1), q(2), q(3), q(4));
qt = quaternion(0, t(1), t(2), t(3));
p_A_orig = quaternion(zeros(1,n), A_orig(1,:), A_orig(2,:), A_orig(3,:));
p_B_orig = quaternion(zeros(1,n), B_orig(1,:), B_orig(2,:), B_orig(3,:));

% Register points
for i=1:n
    p_B(i) = qr*p_A_orig(i)*conj(qr) + qt;
    p_A(i) = conj(qr)*(p_B_orig(i) - qt)*qr;
end

% Aurora registered to robot
B_reg = compact(p_B)';
B_reg = B_reg(2:4,:);

% Robot registered to aurora
A_reg = compact(p_A)';
A_reg = A_reg(2:4,:);

figure(1);
plot3(A_orig(1,:), A_orig(2,:), A_orig(3,:), 'r');
hold on;
xlabel('X'); 
ylabel('Y');
zlabel('Z');
plot3(B_orig(1,:), B_orig(2,:), B_orig(3,:), 'b');
legend({'robot','aurora'})
axis equal

figure (2);
plot3(B_orig(1,:), B_orig(2,:), B_orig(3,:), 'b');
hold on;
xlabel('X'); 
ylabel('Y');
zlabel('Z');
plot3(B_reg(1,:), B_reg(2,:), B_reg(3,:), 'o-r');
legend({'robot','aurora to robot'})
axis equal

figure(3);
plot3(A_orig(1,:), A_orig(2,:), A_orig(3,:), 'r');
hold on;
xlabel('X'); 
ylabel('Y');
zlabel('Z');
plot3(A_reg(1,:), A_reg(2,:), A_reg(3,:), 'o-b');
legend({'aurora','robot to aurora'})
axis equal
