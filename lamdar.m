function [sys,x0,str,ts] = lamdar(t,x,u,flag)

global lamdar;

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
    lamdar = 0;
    
  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts]=mdlInitializeSizes

sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs = 1;
sizes.NumInputs  = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [];
str = [];
ts  = [-2 0];



function sys=mdlDerivatives(t,x,u)

sys = [];


function sys=mdlUpdate(t,x,u)

sys = [];


function sys=mdlOutputs(t,x,u)
global lamdar;
sys = lamdar; 

function sys=mdlGetTimeOfNextVarHit(t,x,u)
global lamdar;
ism = u(1);
ts = u(2);
Tr = 0.671 / 3.24; 
Lm = 0.651;
lamdar = (-ts/Tr + 1) * lamdar + Lm * ts/Tr * ism;
sys = t + ts;    

function sys=mdlTerminate(t,x,u)

sys = [];

function [output] = roundn(input, digit)
temp = input * 10^digit;
output = round(temp) / 10^digit;