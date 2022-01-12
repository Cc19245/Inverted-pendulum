%% 一阶倒立摆系统的最小阶观测器s-function建模
function [sys,x0,str,ts,simStateCompliance] = order1_min_observer_sfun(t,x,u,flag, x_0, th_0)
switch flag,
    case 0,
        [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(t,x,u, x_0, th_0);
    case 1,
        sys=mdlDerivatives(t,x,u);
    case 2,
        sys=mdlUpdate(t,x,u);
    case 3,
        sys=mdlOutputs(t,x,u);
    case 4,
        sys=mdlGetTimeOfNextVarHit(t,x,u);
    case 9,
        sys=mdlTerminate(t,x,u);
    otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
% 主函数结束


%% ---------------------------------------------
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(t,x,u,x_0, th_0)
% 初始化
sizes = simsizes;% 生成sizes数据结构

sizes.NumContStates  = 3;% 连续状态数
sizes.NumDiscStates  = 0;% 离散状态数,缺省为 0
sizes.NumOutputs     = 4;% 输出量个数,缺省为 0, 
sizes.NumInputs      = 2;% 输入量个数，缺省为 0, 这里的输入取为输入u和y
sizes.DirFeedthrough = 1;% 是否存在直接馈通。1：存在；0：不存在，缺省为 1 
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);       
x0  = [0; th_0; 0] - [10; -152.2041; -684.4898]*x_0;  % 设置初始状态
str = [];% 保留变量置空
ts  = [0 0]; % 连续系统
simStateCompliance = 'UnknownSimState';
% end mdlInitializeSizes

%% ---------------------------------------------
function sys=mdlDerivatives(t, x, u)
%  计算导数例程子函数
M = 2; m = 0.1; l =0.5; I = 1/3*m*l^2; g = 9.8;
a23 = -m*m*g*l*l/(I*(m+M)+M*m*l*l);
a43 = m*g*l*(M+m)/(I*(m+M)+M*m*l*l);
b2 = (I+m*l*l)/(I*(m+M)+M*m*l*l);
b4 = -m*l/(I*(m+M)+M*m*l*l);
A = [0 1 0   0;
     0 0 a23 0;
     0 0 0   1;
     0 0 a43 0];
B = [0; b2; 0; b4]; 
Ke_j = [10;  -152.2041; -684.4898];
A_hat = A(2:end, 2:end) - Ke_j*A(1, 2:end);
B_hat = A_hat*Ke_j + A(2:end, 1) - Ke_j*A(1,1);
F_hat = B(2:end) - Ke_j*B(1);
sys = A_hat*x + B_hat*u(2) + F_hat*u(1) ;  % u1就是输入，u2是原系统输出y

%% ---------------------------------------------
function sys=mdlUpdate(t,x,u)
%3. 状态更新例程子函数
sys = [];

%% ---------------------------------------------
function sys=mdlOutputs(t,x,u)
%4. 计算输出例程子函数
C_hat = [0 0 0; 1 0 0; 0 1 0; 0 0 1];
D_hat = [1; 10; -152.2041; -684.4898];
sys = C_hat*x + D_hat*u(2);

%% ---------------------------------------------
function sys=mdlGetTimeOfNextVarHit(t,x,u)
 % 5. 计算下一个采样时间，仅在系统是变采样时间系统时调用
sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

%% ---------------------------------------------
function sys=mdlTerminate(t,x,u)
 % 6. 仿真结束时要调用的例程函数
sys = [];
