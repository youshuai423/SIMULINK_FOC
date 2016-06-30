function [sys,x0,str,ts] = ualbeSVM(t,x,u,flag)

global section  % SVM不同阶段
global vec  % 8个矢量
global s_inv  % 实际输出开关状态
global Tinv % SVM不同阶段的时间

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
    section = 1;
    vec = [
        1, 0, 0, 1, 0, 1; % 100
        1, 0, 1, 0, 0, 1; % 110
        0, 1, 1, 0, 0, 1; % 010
        0, 1, 1, 0, 1, 0; % 011
        0, 1, 0, 1, 1, 0; % 001
        1, 0, 0, 1, 1, 0; % 101
        0, 1, 0, 1, 0, 1; % 000
        1, 0, 1, 0, 1, 0; % 111
    ];
    s_inv = []; 
    Tinv = []; 

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
sizes.NumOutputs = 6;
sizes.NumInputs  = 4;
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
global section
global s_inv
global Tinv

% ========================== 
%        SVM控制阶段
% ==========================
switch section
    case {1,8},
        if section == 1
            section = section + 1;
        else
            section = 1;
        end
        sys = s_inv(1,:);
        
    case {2, 7},
        sys = s_inv(2,:);
        if Tinv(1) == 0 && section == 7
            section = 1;
        else
            section = section + 1;
        end
    case {3, 6},
        sys = s_inv(3,:);
        if Tinv(1) + Tinv(2) == 0 && section == 6
            section = 1;
        else
            section = section + 1;
        end
    case {4, 5}
        sys = s_inv(4,:);
        if Tinv(1) + Tinv(2) + Tinv(3) == 0 && section == 5
            section = 1;
        else
            section = section + 1;
        end
end
    

function sys=mdlGetTimeOfNextVarHit(t,x,u)

global section
global s_inv
global vec
global Tinv

ux = u(1);
uy = u(2);
ts = u(3);
ud = u(4);

% ========================== 
%        SVM控制阶段
% ==========================
if section == 1 % 第一阶段之前进行占空比等计算   
    % 扇区判断
    k = uy / ux;
    if (ux > 0 && uy >= 0 && k < sqrt(3) && k >= 0)
        sector = 1;
        Dm = (ux - uy/sqrt(3)) / ud;
        Dn = 2/sqrt(3) * uy / ud;
    elseif (uy > 0 && (k >= sqrt(3) || k < -sqrt(3)))
        sector = 2;
        Dm = (ux + uy/sqrt(3)) / ud;
        Dn = (-ux + uy/sqrt(3)) / ud;
    elseif (ux < 0 && uy > 0 && k >= -sqrt(3) && k < 0)
        sector = 3;
        Dm = 2/sqrt(3) * uy / ud;
        Dn = (-ux - uy/sqrt(3)) / ud;
    elseif (ux < 0 && uy <= 0 && k >= 0 && k < sqrt(3))
        sector = 4;
        Dm = (-ux + uy/sqrt(3)) / ud;
        Dn = -2/sqrt(3) * uy / ud;
    elseif (uy < 0 && (k >= sqrt(3) || k < -sqrt(3)))
        sector = 5;
        Dm = (-ux - uy/sqrt(3)) / ud;
        Dn = (ux - uy/sqrt(3)) / ud;
    elseif (ux > 0 && uy < 0 && k < 0 && k >= -sqrt(3))
        sector = 6;
        Dm = -2/sqrt(3) * uy / ud;
        Dn = (ux + uy/sqrt(3)) / ud;
    else
        sector = 1;
        Dm = 0;
        Dn = 0;        
    end

    if (Dm + Dn >= 1) % 过调制等比例缩小
        temp = Dm / (Dm + Dn);
        Dn = Dn / (Dm + Dn);
        Dm = temp;
        D0 = 0;
    else
        D0 = 1 - Dm - Dn;
    end
    
    switch sector
        case 1,
            s_inv = [vec(7,:); vec(1,:); vec(2,:); vec(8,:)];
            Tinv = ts * [D0/4, Dm/2, Dn/2, D0/4];
        case 2,
            s_inv = [vec(7,:); vec(3,:); vec(2,:); vec(8,:)]; % 考虑到开关次数最少，中间两个向量颠倒
            Tinv = ts * [D0/4, Dn/2, Dm/2, D0/4];
        case 3,
            s_inv = [vec(7,:); vec(3,:); vec(4,:); vec(8,:)];
            Tinv = ts * [D0/4, Dm/2, Dn/2, D0/4];
        case 4,
            s_inv = [vec(7,:); vec(5,:); vec(4,:); vec(8,:)];
            Tinv = ts * [D0/4, Dn/2, Dm/2, D0/4];
        case 5,
            s_inv = [vec(7,:); vec(5,:); vec(6,:); vec(8,:)];
            Tinv = ts * [D0/4, Dm/2, Dn/2, D0/4];
        case 6,
            s_inv = [vec(7,:); vec(1,:); vec(6,:); vec(8,:)];
            Tinv = ts * [D0/4, Dn/2, Dm/2, D0/4];
    end
    Tinv = roundn(Tinv,8);
end

if section == 1
    if Tinv(1) ~= 0 
        sys = t + Tinv(1);
    else
        section = 2; % 第一段时间等于0直接跳到第二段
    end
end

if section == 2
    if Tinv(2) ~= 0
        sys = t + Tinv(2); 
    else
        section = 3; % 第二段时间等于0直接跳到第三段
    end
end

if section == 3
    if Tinv(3) ~= 0
        sys = t + Tinv(3);
    else
        section = 4; % 第三段时间等于0直接跳到第四段
    end
end

if section == 4
    if Tinv(4) ~= 0
        sys = t + Tinv(4);
    else
        section = 5; % 第四段时间等于0直接跳到第五段
    end
end

if section == 5
    if Tinv(4) ~= 0
        sys = t + Tinv(4);
    elseif Tinv(3) ~= 0
        section = 6;
    elseif Tinv(2) ~= 0
        section = 7;
    elseif Tinv(1) ~= 0
        section = 8;
    else
        section = 1;
    end
end

if section == 6
    if Tinv(3) ~= 0
        sys = t + Tinv(3);
    elseif Tinv(2) ~= 0
        section = 7;
    elseif Tinv(1) ~= 0
        section = 8;
    else
        section = 1;
    end
end

if section == 7
    if Tinv(2) ~= 0
        sys = t + Tinv(2);
    elseif Tinv(1) ~= 0
        section = 8;
    else
        section = 1;
    end
end

if section == 8
    sys = t + Tinv(1);
end

if section > 8
    error('out of range!')
end
 
    

function sys=mdlTerminate(t,x,u)

sys = [];

function [output] = roundn(input, digit)
temp = input * 10^digit;
output = round(temp) / 10^digit;