%% 本文件是进行一阶倒立摆的线性化模型的能控能观性、极点配置、状态观测器设计等的计算文件
clear; clc; close all; warning off;

M = 2;
m = 0.1;
l =0.5;
I = 1/3*m*l^2;  % 注意转动惯量的计算
g = 9.8;

a23 = -m*m*g*l*l/(I*(m+M)+M*m*l*l);
a43 = m*g*l*(M+m)/(I*(m+M)+M*m*l*l);
b2 = (I+m*l*l)/(I*(m+M)+M*m*l*l);
b4 = -m*l/(I*(m+M)+M*m*l*l);

A = [0 1 0 0;
    0 0 a23 0;
    0 0 0 1;
    0 0 a43 0];
B = [0;
    b2;
    0;
    b4];
C =[1 0 0 0]; 

%% 能控能观性判断
S_c = [B A*B A^2*B A^3*B];
fprintf('rank(S_c) = %d\n', rank(S_c));

Q_o = [C;
    C*A;
    C*A^2;
    C*A^3]
fprintf('rank(Q_o) = %d\n', rank(Q_o));

%% 稳定性判断
% 劳斯判据
disp('eig(A)=');
eig(A)

% 李雅普诺夫稳定性判据
% P = lyap(A',eye(4))
% eig(P)

%% 状态反馈的极点配置问题
J = [-1 -2 -1+j*sqrt(3) -1-j*sqrt(3)];
K1 = acker(A,B,J); 
K2 = place(A,B,J); 
disp('K1 = ');
disp(K1);
disp('K2 = ');
disp(K2);
K = K2;

%% 全阶观测器
A_ = A';
B_ = C';
C_ = B';
J_ = [-2 -6 -2-j*2*sqrt(3) -2+j*2*sqrt(3)];
Ke = (acker(A_, B_, J_))';
disp('Ke = ');
disp(Ke);
B_All = zeros(4,2);
B_All(:, 1) = B;
B_All(:, 2) = Ke;

%% 最小阶观测器
Aaa = A(1, 1);
Aab = A(1, 2:end);
Aba = A(2:end, 1);
Abb = A(2:end, 2:end);
J_j = [-6 -2-j*2*sqrt(3) -2+j*2*sqrt(3)];
Kb = K(2:end);
Ke_j = (acker(Abb',Aab',J_j))';
disp('Ke_j = ');
disp(Ke_j);

%% LQR控制
Q = eye(4);
R = eye(1);
K_lqr = lqr(A,B,Q,R);
disp('K_lqr = ');
disp(K_lqr);