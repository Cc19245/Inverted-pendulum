%% 一阶倒立摆系统的全阶观测器s-function建模
function [sys,x0,str,ts,simStateCompliance] = order1_all_observer_sfun(t,x,u,flag, x_0, th_0)
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

sizes.NumContStates  = 4;% 连续状态数, 分别是x', theta1', theta2', x, theta1, theta2
sizes.NumDiscStates  = 0;% 离散状态数,缺省为 0
sizes.NumOutputs     = 4;% 输出量个数,缺省为 0, 
sizes.NumInputs      = 1;% 输入量个数，缺省为 0, 这里的输入取为e
sizes.DirFeedthrough = 1;% 是否存在直接馈通。1：存在；0：不存在，缺省为 1 
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [x_0; 0; th_0; 0];% 设置初始状态
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
A = [0 1 0 0;
    0 0 a23 0;
    0 0 0 1;
    0 0 a43 0];
b = [0; b2; 0; b4]; 
K = [-1.1020, -2.2041, -37.5147, -8.2194];  
Ke = [12; 75.2; -988.9; -3689.2];
sys = (A-b*K)*x + Ke*u ;  % 注意这里的u就是x(1)的观测误差e(1)


%% ---------------------------------------------
function sys=mdlUpdate(t,x,u)
%3. 状态更新例程子函数
sys = [];

%% ---------------------------------------------
function sys=mdlOutputs(t,x,u)
%4. 计算输出例程子函数
sys=x;

%% ---------------------------------------------
function sys=mdlGetTimeOfNextVarHit(t,x,u)
 % 5. 计算下一个采样时间，仅在系统是变采样时间系统时调用
sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

%% ---------------------------------------------
function sys=mdlTerminate(t,x,u)
 % 6. 仿真结束时要调用的例程函数
sys = [];
