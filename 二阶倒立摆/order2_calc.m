%% 本文件是进行一阶倒立摆的线性化模型的能控能观性、极点配置、状态观测器设计等的计算文件
%% 求线性化模型，使用符号求逆
clear; clc; close all; warning off;
% syms M m_1 m_2 l_1 l_2 L g I_1 I_2    % 求线性化模型，使用符号求逆
% 具体数据求解
M=2; m_1=0.5; m_2=0.5; l_1=0.2; l_2=0.2; L=0.4; g=9.8;
I_1 = 1/12*m_1*(2*l_1)^2; I_2 = 1/12*m_2*(2*l_2)^2;

M_11 = M+m_1+m_2; M_12 = m_1*l_1+m_2*L;  M_13 = m_2*l_2;
M_21 = M_12; M_22 = I_1+m_1*l_1*l_1+m_2*L*L;  M_23 = m_2*L*l_2;   
M_31 = M_13; M_32 = M_23; M_33 = I_2+m_2*l_2*l_2;
M = [M_11 M_12 M_13; M_21 M_22 M_23; M_31 M_32 M_33];
G = [0 0 0; 0 (m_1*l_1+m_2*L)*g 0; 0 0 m_2*g*l_2];
U = [1; 0; 0];
A_a = M\G;
B_b = M\U;

A = zeros(6,6);
B = zeros(6,1);
A(1:3, 4:end) = A_a;
A(4:end, 1:3) = eye(3);
B(1:3) = B_b;
C = [0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];
 
%% 能控性、能观性判断
S_c = [B A*B A^2*B A^3*B A^4*B A^5*B];
fprintf('rank(S_c) = %d\n', rank(S_c));

Q_o = [C; C*A; C*A^2; C*A^3; C*A^4; C*A^5];
fprintf('rank(Q_o) = %d\n', rank(Q_o));

%% 劳斯判据
disp('eig(A)=');
eig(A)

%% 李雅普诺夫稳定性判据
% P = lyap(A',eye(6))
% eig(P)

%% 状态反馈极点配置
J = [-2+j*2 -2-j*2 -6 -7 -8 -9];
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
J_ = 3*J;
Ke = (place(A_, B_, J_))';  % 注意此时对偶系统是多输入系统了，只能用place，不能用acker
disp('Ke = ');
disp(Ke);

%% 最小阶观测器
Abb = A(1:3, 1:3);
Aba = A(1:3, 4:end);
Aab = A(4:end, 1:3);
Aaa = A(4:end, 4:end);
J_j = 3*J(1:3);   % 选择前三个极点
Kb = K(4:end);
Ke_j = (place(Abb',Aab',J_j))';
disp('Ke_j = ');  % 注意这里把状态变量顺序换了，变成[x1;x2;x3;dx1;dx2;dx3]
disp(Ke_j);

%% LQR控制
Q = eye(6);
R = eye(1);
K_lqr = lqr(A,B,Q,R);
disp('K_lqr = ');
disp(K_lqr);
