function [sys,x0,str,ts] = speed(t,x,u,flag)

global we wsl;
global thetak;

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
    we = 0;
    wsl = 0;
    thetak = 0;

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
sizes.NumOutputs = 3;
sizes.NumInputs  = 5;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [-2 0];



function sys=mdlDerivatives(t,x,u)

sys = [];


function sys=mdlUpdate(t,x,u)

sys = [];


function sys=mdlOutputs(t,x,u)
global we wsl;
if abs(we - wsl) < 90
    sys = [we wsl we-wsl];
else
    sys = [0 wsl 0];
end


function sys=mdlGetTimeOfNextVarHit(t,x,u)
global we wsl;
global thetak;
lamdax = u(1);
lamday = u(2);
ix = u(3);
iy = u(4);
ts = u(5);
Lm = 0.27325;
Tr = 0.289368 / 3.3278;

if lamdax ~= 0 && ~isnan(lamdax) && ~isnan(lamday)
    theta = atan(lamday / lamdax);
    if abs(theta - thetak) < 0.5*pi
        we = (theta - thetak) / ts;
    elseif theta <= 0
        we = (theta - thetak + pi) / ts;
    else
        we = (theta - thetak - pi) / ts;
    end
    thetak = theta;

    wsl = Lm/Tr * (lamdax*iy - lamday*ix) / (lamdax^2 + lamday^2);
else
    we = 0;
    wsl = 0;
end


sys = t + ts;

function sys=mdlTerminate(t,x,u)

sys = [];

