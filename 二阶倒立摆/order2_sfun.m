%% 二阶倒立摆系统的动力学方程s-function建模
function [sys,x0,str,ts,simStateCompliance] = order2_sfun(t,x,u,flag, dx_0, dth1_0, dth2_0, x_0, theta1_0, theta2_0)
switch flag,
    case 0,
        [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(t,x,u, dx_0, dth1_0, dth2_0, x_0, theta1_0, theta2_0);
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
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(t,x,u, dx_0, dth1_0, dth2_0, x_0, theta1_0, theta2_0)
% 初始化
sizes = simsizes;% 生成sizes数据结构

sizes.NumContStates  = 6;% 连续状态数, 分别是x', theta1', theta2', x, theta1, theta2
sizes.NumDiscStates  = 0;% 离散状态数,缺省为 0
sizes.NumOutputs     = 6;% 输出量个数,缺省为 0, 
sizes.NumInputs      = 1;% 输入量个数，缺省为 0, 这里的输入取为u，也就是作用在q1上的力矩\tau
sizes.DirFeedthrough = 1;% 是否存在直接馈通。1：存在；0：不存在，缺省为 1 
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [dx_0; dth1_0; dth2_0; x_0; theta1_0; theta2_0];% 设置初始状态
str = [];% 保留变量置空
ts  = [0 0]; % 连续系统
simStateCompliance = 'UnknownSimState';
% end mdlInitializeSizes


%% ---------------------------------------------
function sys=mdlDerivatives(t,x,u)
%  计算导数例程子函数
assert(all(imag(u)==0), 'u is imaginary or nan');
M=2; m1=0.5; m2=0.5; l1=0.2; l2=0.2; L=0.4; g=9.8;
I1 = 1/12*m1*(2*l1)^2; I2 = 1/12*m2*(2*l2)^2;
dx_ = x(1); dth1_ = x(2); dth2_ = x(3);
x_ = x(4); th1_ = x(5); th2_ = x(6); 
M11 = M + m1 + m2;
M12 = (m1*l1+m2*L)*cos(th1_);
M13 = m2*l2*cos(th2_);
M21 = M12;
M22 = I1 + m1*l1^2 + m2*L^2;
M23 = m2*L*l2*cos(th2_ - th1_);
M31 = M13;
M32 = M23;
M33 = I2 + m2*l2^2;
C11 = 0; C12 = -(m1*l1+m2*L)*sin(th1_)*dth1_; C13 = -m2*l2*sin(th2_)*dth2_;
C21 = 0; C22 = 0; C23 = -m2*L*l2*dth2_*sin(th2_ - th1_);
C31 = 0; C32 = m2*L*l2*dth1_*sin(th2_ - th1_); C33 = 0;
G1 = 0; G2 = -(m1*l1+m2*L)*g*sin(th1_); G3 = -m2*g*l2*sin(th2_);

A = [M11 M12 M13 C11 C12 C13;
     M21 M22 M23 C21 C22 C23;
     M31 M32 M33 C31 C32 C33;
      0   0   0   1   0   0 ;
      0   0   0   0   1   0 ;
      0   0   0   0   0   1 ];
B = [u-G1;
     -G2;
     -G3;
     dx_;
     dth1_;
     dth2_];
sys = A\B;

%% ---------------------------------------------
function sys=mdlUpdate(t,x,u)
%3. 状态更新例程子函数
sys = [];

%% ---------------------------------------------
function sys=mdlOutputs(t,x,u)
%4. 计算输出例程子函数
sys=[x(1);x(2);x(3);x(4);x(5);x(6)];

%% ---------------------------------------------
function sys=mdlGetTimeOfNextVarHit(t,x,u)
 % 5. 计算下一个采样时间，仅在系统是变采样时间系统时调用
sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

%% ---------------------------------------------
function sys=mdlTerminate(t,x,u)
 % 6. 仿真结束时要调用的例程函数
sys = [];
