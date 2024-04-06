close all 
clear
%clc

J = 0.304;
h1 = 0.3;
h2 = 0.2;
g = 9.82;
Tm = 4;
m = 2.43;

F = 4;

%system dynamics
     %x  xdot gam  gdot y  ydot beta bdot  z    zdot
a = [0   1    0    0    0    0    0    0    0    0;
     0   0    -g   0    0    0    0    0    0    0;
     0   0    0    1    0    0    0    0    0    0;
     0   0    0    0    0    0    0    0    0    0;
     0   0    0    0    0    1    0    0    0    0;
     0   0    0    0    0    0    -g   0    0    0;
     0   0    0    0    0    0    0    1    0    0;
     0   0    0    0    0    0    0    0    0    0;
     0   0    0    0    0    0    0    0    0    1;
     0   0    0    0    0    0    0    0    1    0];

   %xdot %gam  gdot ydot bet bdot  z   zdot
A = [0    -g   0    0    0    0    0    0;
     0    0    1    0    0    0    0    0;
     0    0    0    0    0    0    0    0;
     0    0    0    0    -g   0    0    0;
     0    0    0    0    0    1    0    0;
     0    0    0    0    0    0    0    0;
     0    0    0    0    0    0    0    1;
     0    0    0    0    0    0    1    0];

   %xdot gam  ydot gam2  z   zdot
AA = [0    -g   0    0    0    0;
      0    0    0    0    0    0;
      %0    0    0    0    0;%    0;
      0    0    0    -g   0    0;
      0    0    0    0    0    0;
      %0    0    0    0    0;%    0;
      0    0    0    0    0    1;
      0    0    0    0    1    0];

B = [0          g       0       0;
     0          0       0       0;
     0          m*g*h1/J 0      0;
     0          0       g       0;
     0          0       0       0;
     0          0       m*g*h2/J 0;
     0          0       0       0;
     Tm/m       0       0       -g];

BB = [0          g       0;
     0          0       0;
     0          0       g;
     0          0       0;
     0          0       0;
     Tm/m       0       0];

CC = eye(5);

DD = zeros(5,3);
    %x  xdot gam gdot y ydot be bdot z  zdot
C = eye(8);
 

% eye(10);    %[0  0   1   0   0   0   1   0   0   1];
   
D = zeros(8,4);    %0;  %[0  0   0 ];

%initial value and reference vector

%initial conditions:
x0 = [-10; %x
      0; %xdot
      0; %gamma
      0; %gammadot
      10; %y
      0; %ydot
      0; %beta
      0; %betadot
      8; %z
      0];%zdot

% %reference
% xd = [2; %x
%       0; %xdot
%       0; %gamma
%       0; %gammadot
%       2; %y
%       0; %ydot
%       0; %beta
%       0; %betadot
%       5; %z
%       0];%zdot
% %calculate ud
% ud = -inv(B'*B)*B'*A*xd;

% control law
q = [1  0   0   0   0   0   0   0   0   0; %x
     0  10   0   0   0   0   0   0   0   0;  %xdot
     0  0   5   0   0   0   0   0   0   0; %gamma
     0  0   0   1   0   0   0   0   0   0; %gammadot
     0  0   0   0   1   0   0   0   0   0; %y
     0  0   0   0   0   10   0   0   0   0;  %ydot
     0  0   0   0   0   0   5   0   0   0; %beta
     0  0   0   0   0   0   0   1   0   0; %betadot
     0  0   0   0   0   0   0   0   5   0;  %z
     0  0   0   0   0   0   0   0   0   1];%zdot

Q = [10   0   0   0   0   0   0   0;  %xdot
     0   5   0   0   0   0   0   0; %gamma
     0   0   1   0   0   0   0   0; %gammadot
     0   0   0   10   0   0   0   0;  %ydot
     0   0   0   0   5   0   0   0; %beta
     0   0   0   0   0   1   0   0; %betadot
     0   0   0   0   0   0   10   0;  %z
     0   0   0   0   0   0   0   1000];%zdot
%Q = eye(10);
     
R = [1  0   0   0; %thrust
    0   1  0    0;  %theta1
    0   0   1   0
    0   0   0   1]; %theta2


% Define the time span for simulation
tspan = 0:0.01:10; 

% Calculate K
sys = ss(A, B, C, D);
[K,S,P] = lqr(sys,Q,R);

sys2 = ss(A,B,C,D);
sysd = c2d(sys2,0.01);
[Kd,S2,P2] = lqr(sys2,Q,R);

%Qi = eye(16);

%[Ki,Si,e] = lqi(sysd,Q,R);

%rank(ctrb(A,B))
%rank(obsv(A,C))

%xdot = Ax + Bu 
%edot = -Cx + r


for i=1:length(tspan)
    tant(i,2) = (tanh(tspan(i)-6))*6+8.5;
    tant(i,1) = tspan(i);
end
%tant

for i=1:length(tspan)
    if i ~= length(tspan)
        tantd(i,2) = (tant(i+1,2) - tant(i,2))/(tspan(i+1)-tspan(i));
    else
        tantd(i,2) = (tant(i,2) - tant(i-1,2))/(tspan(i)-tspan(i-1));
    end
    tantd(i,1) = tspan(i);
end
%tantd
%plot(tantd)


vsim = [0   0;
        0.5 1;
        1   2;
        2   2;
        3   2;
        3.5 1;
        4   0;
        5   0;
        6   0;
        7   0;
        8   0;
        9   0;
        10  0;
        11  0;
        30  0];

hsim = [0   0;
        4   6;
        5   6;
        6   6;
        7   6;
        8   6;
        9   6;
        10  6;
        11  6;
        30  6];


Ae = [0    -g   0    0    0    0    0    0  0;
     0    0    1    0    0    0    0    0   0;
     0    0    0    0    0    0    0    0   0;
     0    0    0    0    -g   0    0    0   0;
     0    0    0    0    0    1    0    0   0;
     0    0    0    0    0    0    0    0   0;
     0    0    0    0    0    0    0    1   0;
     0    0    0    0    0    0    1    0   0;
     1    0    0    0    0    0    0    0   0;
     0    1    0    0    0    0    0    0   0;
     0    0    1    0    0    0    0    0   0;  
     0    0    0    1    0    0    0    0   0;
     0    0    0    0    1    0    0    0   0;
     0    0    0    0    0    1    0    0   0;  
     0    0    0    0    0    0    1    0   0;
     0    0    0    0    0    0    0    1   0];


%[A   zeros(8,8);
% -C  zeros(8,8)];

Be = [0          g       0;
     0          0       0;
     0          m*g*h1/J 0;
     0          0       g;
     0          0       0;
     0          0       m*g*h2/J;
     0          0       0;
     Tm/m       0       0;
     0          0       0;
     0          0       0;
     0          0       0;
     0          0       0;
     0          0       0;
     0          0       0;
     0          0       0;
     0          0       0;];
     
%[B;
%zeros(8,3)];

Ce = [1     0     0     0     0     0     0     0   0;
     0     1     0     0     0     0     0     0    0;
     0     0     1     0     0     0     0     0    0;
     0     0     0     1     0     0     0     0    0;  
     0     0     0     0     1     0     0     0    0;
     0     0     0     0     0     1     0     0    0;  
     0     0     0     0     0     0     1     0    0;
     0     0     0     0     0     0     0     1    0];


%[C zeros(8,8)];

De = zeros(8,9);
Br = [0;
     1];

Qi = eye(16);

%sysde = ss(Ae,Be,Ce,De);

%[Ki,~,~] = lqr(sysde,Qi ,R); 


%[Ke,Se,Pe] = lqi(sysd,Qi,R);


%%% KALMAN

Qk = [1   0   0   0   0   0   0   0;  %xdot
     0   1   0   0   0   0   0   0; %gamma
     0   0   1   0   0   0   0   0; %gammadot
     0   0   0   1   0   0   0   0;  %ydot
     0   0   0   0   1   0   0   0; %beta
     0   0   0   0   0   1   0   0; %betadot
     0   0   0   0   0   0   1   0;  %z
     0   0   0   0   0   0   0   1];%zdot

Rk = [1   0   0   0   0   0   0   0;  %xdot
     0   1   0   0   0   0   0   0; %gamma
     0   0   1   0   0   0   0   0; %gammadot
     0   0   0   1   0   0   0   0;  %ydot
     0   0   0   0   1   0   0   0; %beta
     0   0   0   0   0   1   0   0; %betadot
     0   0   0   0   0   0   1   0;  %z
     0   0   0   0   0   0   0   1];%zdot


%[kalmf,L,~,Mx,Z] = kalman(sysd,Qk,Rk);