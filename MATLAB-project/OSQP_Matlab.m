%% Digital Control Systems Design project, sampling of the transfer function 
%% to obtain the discrete tf then simulation of the MPC controller with OSQP
% Nicola ROMANO         0622701549  N.ROMANO24@STUDENTI.UNISA.IT
% Antonio ROTONDO       0622701489  A.ROTONDO9@STUDENTI.UNISA.IT
% Catello SORRENTINO    06227001490 C.SORRENTINO61@STUDENTI.UNISA.IT
%%  ay = bu is the model
G=tf([167773.98],[1 392.96 15992.15]);
Ts = 0.05;
Gd=c2d(G,Ts);
[num,den] = tfdata(Gd,'v');
a=den;
b=num(2:3);
ny=10;      % prediction horizon
nu=2;       % input horizon
Wu=1;       % weights on input (responsiveness)
Wy=1;       % weights on output
Dumax=inf;  % input rate limit
umax=12;    % max input
umin=-12;   % min input
ymax=130;   % max output
ymin=-130;  % min output
sizeu=1; 
ref=[zeros(1,10),100*ones(1,40)];  %% target
dist=[zeros(1,20),0*zeros(1,30)];   %%% output disturbance signal
noise=ref*0;  %% measurement noise
sizey = size(a,1);

[H,P,Q] = mpc_predmat(a,b,ny);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%   Find control law and parameters of the cost function
%%%%   Dufut = Pr*rfut - Dk*Dupast - Nk*ypast 
%%%%    J = Dufut'*S*Dufut + Dufut'*2X*[Dupast;ypast;rfut]
[Nk,Dk,Pr,S,X] = mpc_law(H,P,Q,nu,Wu,Wy,sizey);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%   Define constraint matrices
%%%%%%   CC*Du(future) - dd - du*u(k-1))-ddu*Dupast-dy*ypast <= 0
[CC,dd,du,ddu,dy]  = mpc_constraints2(Dumax,umax,umin,ymax,ymin,sizey,nu,H(:,1:nu*sizey),P,Q);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% full simulation 

[y,u,Du,r] = mpc_simulate_outputconstraints_osqp(b,a,nu,ny,Wu,Wy,Dumax,umax,umin,ymax,ymin,ref,dist,noise);
