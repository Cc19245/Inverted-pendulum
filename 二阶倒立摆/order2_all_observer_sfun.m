%% 二阶倒立摆系统的全阶观测器s-function建模
function [sys,x0,str,ts,simStateCompliance] = order2_all_observer_sfun(t,x,u,flag, x_0, th1_0, th2_0)
switch flag,
    case 0,
        [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(t,x,u, x_0, th1_0, th2_0);
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
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(t,x,u,x_0, th1_0, th2_0)
% 初始化
sizes = simsizes;% 生成sizes数据结构

sizes.NumContStates  = 6;% 连续状态数
sizes.NumDiscStates  = 0;% 离散状态数,缺省为 0
sizes.NumOutputs     = 6;% 输出量个数,缺省为 0
sizes.NumInputs      = 3;% 输入量个数，缺省为 0
sizes.DirFeedthrough = 1;% 是否存在直接馈通。1：存在；0：不存在，缺省为 1 
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [0; 0; 0; x_0; th1_0; th2_0];% 设置初始状态
str = [];% 保留变量置空
ts  = [0 0]; % 连续系统
simStateCompliance = 'UnknownSimState';
% end mdlInitializeSizes


%% ---------------------------------------------
function sys=mdlDerivatives(t, x, u)
assert(all(imag(u)==0), 'u is imaginary or nan');
%  计算导数例程子函数
A =[0, 0, 0, 0,   -4.4100,    0.4900;
    0, 0, 0, 0,   77.1750,  -33.0750;
    0, 0, 0, 0,  -99.2250,   84.5250;
    1, 0, 0, 0,         0,         0;
    0, 1, 0, 0,         0,         0;
    0, 0, 1, 0,         0,         0];
B = [0.4667; -1.5000; 0.5000; 0; 0; 0];
K = [23.4125   -0.5709   44.4357   22.3907 -283.0923  379.2252];  
Ke = [132.9964  143.1468    0.4900;
     -122.4083  214.2149  -33.0750;
             0  -99.2250  570.5250;
       27.2900    6.8972         0;
       -5.1003   29.7100         0;
             0         0   45.0000];
sys = (A-B*K)*x + Ke*u ;  % 注意这里的u就是x(1)的观测误差e(1)


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
