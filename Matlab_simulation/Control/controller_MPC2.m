function [sys,x0,str,ts,simStateCompliance] = controller_MPC2(t,x,u,flag,reference,params)

%ues quadprog instead

%SFUNTMPL General MATLAB S-Function Template
%   With MATLAB S-functions, you can define you own ordinary differential
%   equations (ODEs), discrete system equations, and/or just about
%   any type of algorithm to be used within a Simulink block diagram.
%
%   The general form of an MATLAB S-function syntax is:
%       [SYS,X0,STR,TS,SIMSTATECOMPLIANCE] = SFUNC(T,X,U,FLAG,P1,...,Pn)
%
%   What is returned by SFUNC at a given point in time, T, depends on the
%   value of the FLAG, the current state vector, X, and the current
%   input vector, U.
%
%   FLAG   RESULT             DESCRIPTION
%   -----  ------             --------------------------------------------
%   0      [SIZES,X0,STR,TS]  Initialization, return system sizes in SYS,
%                             initial state in X0, state ordering strings
%                             in STR, and sample times in TS.
%   1      DX                 Return continuous state derivatives in SYS.
%   2      DS                 Update discrete states SYS = X(n+1)
%   3      Y                  Return outputs in SYS.
%   4      TNEXT              Return next time hit for variable step sample
%                             time in SYS.
%   5                         Reserved for future (root finding).
%   9      []                 Termination, perform any cleanup SYS=[].
%
%
%   The state vectors, X and X0 consists of continuous states followed
%   by discrete states.
%
%   Optional parameters, P1,...,Pn can be provided to the S-function and
%   used during any FLAG operation.
%
%   When SFUNC is called with FLAG = 0, the following information
%   should be returned:
%
%      SYS(1) = Number of continuous states.
%      SYS(2) = Number of discrete states.
%      SYS(3) = Number of outputs.
%      SYS(4) = Number of inputs.
%               Any of the first four elements in SYS can be specified
%               as -1 indicating that they are dynamically sized. The
%               actual length for all other flags will be equal to the
%               length of the input, U.
%      SYS(5) = Reserved for root finding. Must be zero.
%      SYS(6) = Direct feedthrough flag (1=yes, 0=no). The s-function
%               has direct feedthrough if U is used during the FLAG=3
%               call. Setting this to 0 is akin to making a promise that
%               U will not be used during FLAG=3. If you break the promise
%               then unpredictable results will occur.
%      SYS(7) = Number of sample times. This is the number of rows in TS.
%
%
%      X0     = Initial state conditions or [] if no states.
%
%      STR    = State ordering strings which is generally specified as [].
%
%      TS     = An m-by-2 matrix containing the sample time
%               (period, offset) information. Where m = number of sample
%               times. The ordering of the sample times must be:
%
%               TS = [0      0,      : Continuous sample time.
%                     0      1,      : Continuous, but fixed in minor step
%                                      sample time.
%                     PERIOD OFFSET, : Discrete sample time where
%                                      PERIOD > 0 & OFFSET < PERIOD.
%                     -2     0];     : Variable step discrete sample time
%                                      where FLAG=4 is used to get time of
%                                      next hit.
%
%               There can be more than one sample time providing
%               they are ordered such that they are monotonically
%               increasing. Only the needed sample times should be
%               specified in TS. When specifying more than one
%               sample time, you must check for sample hits explicitly by
%               seeing if
%                  abs(round((T-OFFSET)/PERIOD) - (T-OFFSET)/PERIOD)
%               is within a specified tolerance, generally 1e-8. This
%               tolerance is dependent upon your model's sampling times
%               and simulation time.
%
%               You can also specify that the sample time of the S-function
%               is inherited from the driving block. For functions which
%               change during minor steps, this is done by
%               specifying SYS(7) = 1 and TS = [-1 0]. For functions which
%               are held during minor steps, this is done by specifying
%               SYS(7) = 1 and TS = [-1 1].
%
%      SIMSTATECOMPLIANCE = Specifices how to handle this block when saving and
%                           restoring the complete simulation state of the
%                           model. The allowed values are: 'DefaultSimState',
%                           'HasNoSimState' or 'DisallowSimState'. If this value
%                           is not speficified, then the block's compliance with
%                           simState feature is set to 'UknownSimState'.


%   Copyright 1990-2010 The MathWorks, Inc.
%   $Revision: 1.18.2.5 $

%
% The following outlines the general structure of an S-function.
%
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

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
    sys=mdlOutputs(t,x,u,reference,params);

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

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 5;
sizes.NumInputs      = 8;
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
ts  = [1/60 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

sys = [];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,reference,params)


%s, X, Y, Vx, kap, psi (reference)

% (u) X,Y,psi,vx,vy,dpsi+ index
Vx=u(4);
dt=1/60;
s_ahead=Vx*dt;
s=u(8);
predict_s=s+s_ahead;

ds=0.02;
%% ---------------------------------------------find the nearest point---------------------------------------
predict_points=40*ds;
possible_points=predict_s-predict_points/2:ds:predict_s+predict_points/2;

idx1=(possible_points>reference(end,1)); % exceed 
idx2=(possible_points<reference(1,1)); % below
possible_points(idx1)=possible_points(idx1)-reference(end,1);
possible_points(idx2)=possible_points(idx2)+reference(end,1);

X_p=spline(reference(:,1),reference(:,2),possible_points);
Y_p=spline(reference(:,1),reference(:,3),possible_points);

dist_list=(X_p-u(1)).^2+(Y_p-u(2)).^2;
[ey2,index]=min(dist_list);

s=possible_points(index);
sys(3)=s;

%% -------------------------------------get ey, epsi------------------------------------------------------
psi_ref=spline(reference(:,1),reference(:,6),s);
X_ref=X_p(index);
Y_ref=Y_p(index);
epsi=wrapToPi(u(3))-psi_ref;

% back to small ange
epsi=wrapToPi(epsi);

% ey=sqrt(min_num)*sign(epsi);
angle_ey=atan2(u(2)-Y_ref,u(1)-X_ref);
ey=sqrt(ey2)*sign(wrapToPi(angle_ey-psi_ref));
sys(4)=epsi;
sys(5)=ey;



%% -----------------------------MPC controller---------------------------
% set hyperparameters
Ns=6; % states number
Nc=2; % control number
Np=30; % prediction number (horizion)
% dt=1/20; % sample period
dt=1/30; % sample period
% vx,vy,dpsi, epsi,ey, steer
% R=[0.1,500]';
% Q=[10,0,0,100,20,1]'; 

R = params.R;
Q = params.Q;

d_steerlimit=100;
steerlimit=30;

% get following points
% interpolate along the referenceline: s, X, Y, Vx, kap, psi (reference)
Vx=u(4);
Vy=u(5);
delta=u(7);
dpsi=u(6);

current_s=s;
horizion_s=current_s+(0:Np)*dt*Vx;
ind3=(horizion_s>reference(end,1));  % if exceed the terminal points
horizion_s(ind3)=horizion_s(ind3)-reference(end,1);
Vx_all=interp1(reference(:,1),reference(:,4),horizion_s,'spline');    
kap=interp1(reference(:,1),reference(:,5),horizion_s,'spline');   

% get matrix
% m=1060;
% Iz=1493.4;
% lf=1.0462;
% lr=1.2638;
% 
% a11=4800*2;
% a12=7.0;
% a21=3720*2;
% a22=10.3;
m = params.m;
Iz = params.Izz;
lf = params.lf;
lr = params.lr;
a11 = params.a11*2;
a12 = params.a12;
a21 = params.a21*2;
a22 = params.a22;

% MPC sys(1)=a, sys(2)=steer

% A11=-(Caf+Car)/(m*Vx);
% A12=-Vx-(Caf*lf-Car*lr)/(m*Vx);
% A21=-(Caf*lf-Car*lr)/(Iz*Vx);
% A22=-(Caf*lf^2+Car*lr^2)/(Iz*Vx);
% B1=Caf/m;
% B2=Caf*lf/Iz;

alpf=delta-atan2(Vy+lf*dpsi,Vx);
alpr=-atan2(Vy-lr*dpsi,Vx);
Fyf=a11*tanh(a12*alpf);
Fyr=a21*tanh(a22*alpr);
Kf=a11*a12*(1-tanh(a12*alpf)^2);
Kr=a21*a22*(1-tanh(a22*alpr)^2);
bf=Fyf-Kf*alpf;
br=Fyr-Kr*alpr;

A22=-(Kf+Kr)/(m*Vx);
A23=-Vx-(Kf*lf-Kr*lr)/(m*Vx);
A32=-(Kf*lf-Kr*lr)/(Iz*Vx);
A33=-(Kf*lf^2+Kr*lr^2)/(Iz*Vx);
B26=Kf/m;
B36=Kf*lf/Iz;

% w_wind=2.7e-4;
w_wind = params.w_wind;
a_wind=w_wind*Vx.^2;
D1=-(Fyf*sin(delta))/m+dpsi*Vy-a_wind;
D2=(bf+br)/m;
D3=(lf*bf-lr*br)/Iz;


A_combined1=repmat({zeros(Ns,Ns)},Np+1,Np+1);
A_combined2=repmat({zeros(Ns,Nc)},Np+1,Np+1);
I=eye(Ns);

% build matrix continuous A and B 
Ac=sparse([2,2,3,3,4,5,5,2,3],[2,3,2,3,3,2,4,6,6],[A22,A23,A32,A33,1,1,Vx,B26,B36],Ns,Ns);
Bc=sparse([1,6],[1,2],[1,1],Ns,Nc);
Ac=full(Ac);
Bc=full(Bc);
Ad=Ac*dt+I;
Bd=Bc*dt;

B0=sparse([6],[2],[1],Ns,Nc)*dt;
B0=full(B0);

A_combined1{1,1}=I;
A_combined2{1,1}=-B0;
for i=2:Np+1
    A_combined1{i,i}=I;
    A_combined1{i,i-1}=-Ad;
    A_combined2{i,i}=-Bd;
end
A_combined=[A_combined1,A_combined2];
A_qp=cell2mat(A_combined);

% states
x0=[u(4),u(5),u(6),epsi,ey,u(7)]';
b=zeros((Np+1)*Ns,1);
b(1:Ns)=x0;
% ref_dpsi=-kap.*Vx_all;
ref_dpsi=-kap.*Vx;
b(Ns+4:Ns:Ns*(Np+1))=ref_dpsi(1:Np)*dt;

b(Ns+1:Ns:Ns*(Np+1))=D1*dt;
b(Ns+2:Ns:Ns*(Np+1))=D2*dt;
b(Ns+3:Ns:Ns*(Np+1))=D3*dt;


% boundary
% lbu=[-60,-d_steerlimit/180*pi]'; % acc,steer
% ubu=[10,d_steerlimit/180*pi]'; 
lbu=[-9.5,-d_steerlimit/180*pi]'; % acc,steer
ubu=[1.3,d_steerlimit/180*pi]'; 
lbx=[0,-100,-100,-100,-100,-steerlimit/180*pi]';  % Vx,Vy,dpsi,epsi,ey, steer
ubx=[100,100,100,100,100,steerlimit/180*pi]';

lb_qp=zeros((Np+1)*Ns+(Np+1)*Nc,1);
ub_qp=zeros((Np+1)*Ns+(Np+1)*Nc,1);

lb_qp(1:(Np+1)*Ns)=repmat(lbx,Np+1,1);
ub_qp(1:(Np+1)*Ns)=repmat(ubx,Np+1,1);

lb_qp((Np+1)*Ns+1:end)=repmat(lbu,Np+1,1);
ub_qp((Np+1)*Ns+1:end)=repmat(ubu,Np+1,1);


% cost function
target_x=zeros((Np+1)*Ns+(Np+1)*Nc,1);
target_x(1:Ns:Ns*(Np+1))=Vx_all;

weight=[repmat(Q,Np+1,1);repmat(R,Np+1,1)];

H=diag(weight);
lh=length(weight);
id1=4+(0:Np)*Ns;  % line4
id2=2+(0:Np)*Ns;  % line2
% H0=sparse(id1,id2,Q(4)./Vx_all,lh,lh);
% H=H+H0;

% H22=sparse(id2,id2,Q(4)./Vx_all.^2+Q(2),lh,lh);
H22=sparse(id2,id2,Q(4)./Vx_all.^2,lh,lh);
H42=sparse(id1,id2,Q(4)./Vx_all,lh,lh);
H24=sparse(id2,id1,Q(4)./Vx_all,lh,lh);

H=H+H22+H42+H24;

f=-target_x'*H;
% options = optimoptions('fmincon');
% options = optimset('Algorithm','interior-point-convex')
options = optimset('Algorithm','interior-point-convex','display','off'); 
% options = optimset('Algorithm','active-set','display','off'); 
% options = optimset('Algorithm','active-set'); 

[xout,fval,exitflag,output]=quadprog(H,f,[],[],A_qp,b,lb_qp,ub_qp,[],options);


if exitflag==1
a0=xout((Np+1)*Ns+Nc+1);
steer0=xout(6);

t_steerlimit=6;
    if abs(steer0)>t_steerlimit/180*pi
        steer0=sign(steer0)*t_steerlimit/180*pi;
    end    
    
    % confine a
%     aup=1.3;
%     adn=-9.5;
%     a0=min([aup,a0]);
%     a0=max([adn,a0]);
    
    sys(1)=a0;
    sys(2)=steer0;

else
    sys(1)=0;
    sys(2)=u(7);
    
end

% if rem(t,0.1)<0.02 & t>50
%     run MPC_test
%     t
%     pause(0.01)
% end


%% 



% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
