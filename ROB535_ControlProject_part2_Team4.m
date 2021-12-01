%% Self Driving Cars Control Part 2
% worked on by Dyllian Powell, Spencer Talbot, and Joseph Drockton
% This file is empty currently :]

% Problem 3 homework 1 
%vehicle parameters
delta = [-.5, .5];
Fx = [-5000, 5000];
m = 1400;
Nw = 2;
f = 0.01;
Iz = 2667;
a = 1.35;
b = 1.45;
By = 0.27;
Cy = 1.2;
Dy = 0.7;
Ey = -1.6;
Shy = 0;
Svy = 0;
g = 9.806;

%trajectory (nominal) inputs (idk what these are), see homework 1 
%delta_f=0.3;
%u=5;

%% note that the solutions Y refer to the full state vector i.e. (Y(t) is 3x1)
%3.1 Simulate the non-linear system to generate the circular trajectory (nominal trajectory)
IC = [287, 5, -176, 0, 2, 0]'; % x, u, y, v, phi, r
fun1=@(t,x) [u*cos(x(3)) - (b/L)*u*tan(delta_f)*sin(x(3)); u*sin(x(3)) + (b/L)*u*tan(delta_f)*cos(x(3)); 1/L * u*tan(delta_f)];
tspan = [0:0.01:5];

[T,Y]= ode45(fun1,tspan,[0;0;0]);%use ode45 to simulate the dynamics with delta_f, and u as inputs. 
%T should be 501x1. Y, the nominal trajectory, should be 501x3
%plot(T,Y)


%3.2 Find the linearized time varying A and B matrices
%syms t
%psi = (1/L) * u * tan(delta_f) * t;
%A = [0 0 -u*sin(psi) - u*tan(delta_f)*cos(psi); 0 0 u*cos(psi) + u*tan(delta_f)*sin(psi)/2; 0 0 0]

A=@(t) [0 0 -u*sin((1/L) * u * tan(delta_f) * t) - (b/L)*u*tan(delta_f)*cos( (1/L) * u * tan(delta_f) * t);
    0 0 u*cos((1/L) * u * tan(delta_f) * t) - (b/L)*u*tan(delta_f)*sin((1/L) * u * tan(delta_f) * t);
    0 0 0];

B=@(t) [cos((1/L) * u * tan(delta_f) * t) - (b/L)*tan(delta_f)*sin((1/L) * u * tan(delta_f) * t), -(b/L)*u*(tan(delta_f)^2 + 1)*sin((1/L) * u * tan(delta_f) * t);
    sin((1/L) * u * tan(delta_f) * t)+(b/L)*tan(delta_f)*cos((1/L) * u * tan(delta_f) * t), (b/L)*u*(tan(delta_f)^2 + 1)*cos((1/L) * u * tan(delta_f) * t);
    (1/L)*tan(delta_f), (1/L)*u*(tan(delta_f)^2 +1)];

%3.3 find the optimal feedback gains
Q= eye(3);
R= eye(2);
tpsan = [0 : 0.01 : 5];

Afun = @(t) [0 0 -u*sin((1/L) * u * tan(delta_f) *  (t-1)/100) - (b/L)*u*tan(delta_f)*cos( (1/L) * u * tan(delta_f) *  (t-1)/100);
     0 0 u*cos((1/L) * u * tan(delta_f) *  (t-1)/100) - (b/L)*u*tan(delta_f)*sin((1/L) * u * tan(delta_f) *  (t-1)/100);
     0 0 0];
Bfun = @(t) [cos((1/L) * u * tan(delta_f) * (t-1)/100) - (b/L)*tan(delta_f)*sin((1/L) * u * tan(delta_f) * (t-1)/100), -(b/L)*u*(tan(delta_f)^2 + 1)*sin((1/L) * u * tan(delta_f) * (t-1)/100);
        sin((1/L) * u * tan(delta_f) * (t-1)/100)+(b/L)*tan(delta_f)*cos((1/L) * u * tan(delta_f) * (t-1)/100), (b/L)*u*(tan(delta_f)^2 + 1)*cos((1/L) * u * tan(delta_f) * (t-1)/100);
        (1/L)*tan(delta_f), (1/L)*u*(tan(delta_f)^2 +1)];

[K,P]= lqr_LTV(Afun,Bfun,Q,R,tspan);%use provided lqr_ltv function, remember the input to the A,B matrices is a discrete time index

%3.4 Simulate the LQR controller
x0=[0.1;0.8;0.01];
odeFun = @(i,x) ((Afun(i) - Bfun(i)*cell2mat(K(i))))*x;
YL=ode1(odeFun,tspan,x0);%use ode 1 to simulate the linear system i.e (A-BK)X; 
YL= Y + YL;
%then add nominal trajectory  it
%store the final trajectory in a 501x3 matrix named YL

%3.5 Simulate the Nonlinear system (the grader only checks that the final trajectory YNL is correct, a sketch of a for loop is provided below if you want to use it)
YNL=NaN(size(YL));

%set initial condition 
YNL(1,:)= x0';


for i=1:length(T)-1
    
   %calculate error between YNL and Y at time step i (state of linearized system)
   x=YNL(i,:) - Y(i,:);
    
   %compute input to nonlinear system (u_in(t)=-K{i}x(t)+nominal_input)
   u_in=-K{i}*x' + [5;0.3];

   %use ode45 to simulate nonlinear system, f, forward 1 timestep 
   %(where f is the kinematic bicycle model, equation 9 in the pdf)
   %and T is the time vector given in the pdf
   f2=@(t,x, u, delta_f) [u*cos(x(3)) - (b/L)*u*tan(delta_f)*sin(x(3)); u*sin(x(3)) + (b/L)*u*tan(delta_f)*cos(x(3)); 1/L * u*tan(delta_f)];

   [~,ytemp]=ode45(@(t,z) f2(t,z,u_in(1), u_in(2)),[T(i),T(i+1)],YNL(i,:));
   
   %set next state  
   YNL(i+1,:)=ytemp(end,:);
   
end
   
function [K, P] = lqr_LTV(AFun,BFun,Q,R,tSpan)
    nSteps = length(tSpan);
    P{nSteps} = zeros(size(Q));
    K{nSteps} = zeros(length(R),length(Q));
        
    for i = nSteps-1:-1:1
        A_ = AFun(i+1);
        B_ = BFun(i+1);
        P_ = P{i+1};
        P{i} = P_ + (tSpan(i+1)-tSpan(i)) * ( P_*A_ + A_'*P_ - P_*B_*(R\(B_'*P_)) + Q);
        K{i} = R\(B_'*P_);
    end
end

function x = ode1(odeFun, tSpan, x0)
    x = zeros(length(tSpan),length(x0));
    x(1,:) = x0';
    for i = 1:length(tSpan)-1
        x(i+1,:) = x(i,:) + (tSpan(i+1) - tSpan(i))*odeFun(i,x(i,:)')';
    end
