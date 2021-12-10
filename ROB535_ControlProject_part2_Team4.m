%% Self Driving Cars Control Part 2
% worked on by Dyllian Powell, Spencer Talbot, and Joseph Drockton
% This file is empty currently :]

% Problem 3 homework 1 
%vehicle parameters
close all 
clear all
clc 
% lpinp = load('lowpassheadingdiff.mat');
% lpinp = horzcat(0,lpinp.lpinp);

testtrack = load('TestTrack.mat');
center = testtrack.TestTrack.cline;

right = testtrack.TestTrack.br;
left = testtrack.TestTrack.bl;
thet = testtrack.TestTrack.theta;
track_struct = testtrack.TestTrack;


% figure(1)
% plot(lpinp,'r')
% hold on
% plot(diff(thet),'b')
% lpinp = lowpass(movagin,100)
dthet = angdiff(thet);

% plot(lowpass(dthet,100,1000),'k')

%low pass filter the derivate of headings on the track
lpinp=lowpass(dthet,1000,10000);
plot(lpinp)
hold on
%pad with a 0 so sizes match for lookup
plot(diff(thet),'b')
lpinp = horzcat(lpinp,0);
delta = [-.5, .5]; % need to use this range to generate Delta_feedback most likely
Fx = [-5000, 5000];% not sure how we constrain this range, probably similar to delta_fb
m = 1400;
Nw = 2;
f = 0.01;
Iz = 2667;
a = 1.35;
b = 1.45;
By = 0.27; %tire stuff. 
Cy = 1.2;
Dy = 0.7;
Ey = -1.6;
Shy = 0; %it's shy uwu
Svy = 0;
g = 9.806; %what kind of messed up gravity has 3 sig figs

%trajectory (nominal) inputs (idk what these are), see homework 1 
%delta_f=0.3;
%u=5;

% Timespan for all simulations
T=0:0.01:0.5;

% 1.1 Compute the front and rear cornerning stiffness
%ca_f = Fzf * B * C * D

%Ca_r= F_zr*B*C*D;
%Ca_f= F_zf*B*C*D;
initial_z = [287; 5; -176; 0; 2; 0];
Yprev = initial_z;

outputsteps = 2;
loopsteps=43100;

% initialize vectors to store
% state vector
Ystore = zeros(6,outputsteps*loopsteps);
% timestep vector
Tstore = zeros(1,outputsteps*loopsteps);
% system inputs [steering angle; Fz; steeringdecisionrule @ current track
% location
Inputstore = zeros(3,outputsteps*loopsteps);
testinput = [0 100 0];
accum_error = 0;
for i=1:loopsteps
    [testinput,accum_error] = steering(lpinp,track_struct,Yprev,testinput,outputsteps,accum_error);
    [Y, T]=forwardIntegrateControlInput(testinput(:,1:2),Yprev);
    Y = Y';
    Ystore(:,(i-1)*outputsteps+1:(i-1)*outputsteps+outputsteps) = Y;
    Yprev = Y(:,end); 
%     Tstore = horzcat(Tstore,T);
    Inputstore(:,(i-1)*outputsteps+1:(i-1)*outputsteps+outputsteps) = testinput';
end


%plot results

figure(2)
subplot(2,1,1)
x = center(1,:);
y = center(2,:);
plot(center(1,:),center(2,:));

hold on
plot(right(1,:),right(2,:),'r')
plot(left(1,:),left(2,:),'r')
title('Vehicle State Colored by turning decision rule');
% quiver(x(1:10:end),y(1:10:end),cos(thet(1:10:end)),sin(thet(1:10:end)))

%subplot(3,1,1);
Y = Ystore'; 
plot(Y(:,1),Y(:,3),'k');

pointsize = 10;

% keySet = {-2,-1,0,1,2};
% valueSet = ["Hard Left","Soft Left", "Straight","Soft Right", "Hard Right"];
% M = containers.Map(keySet,valueSet)
% labels =values(M,num2cell(Inputstore(3,:)));
gscatter(Y(:,1),Y(:,3),Inputstore(3,:));

% legend({"Hard Left","Soft Left", "Straight","Soft Right", "Hard Right"},'FontSize',14)

hold on

%subplot(3,1,2)
%plot(T,Y(:,2))


subplot(2,1,2)


plot(center(1,:),center(2,:));

hold on
plot(right(1,:),right(2,:),'r')
plot(left(1,:),left(2,:),'r')
title('Center Line of Track');

plot(Y(:,1),Y(:,3),'k');

pointsize = 10;
scatter(Y(:,1),Y(:,3),pointsize,Y(:,2));
colorbar




% Yfin = [Ystore(1,:); Ystore(3,:)];
Yfin = [Ystore(1,:); Ystore(3,:)];
%call to see if we crash
info = getTrajectoryInfo(Yfin',Inputstore(1:2,:)',[],T,testtrack.TestTrack);

% 1.2.1 Generate the equilibrium trajectory using Euler integration and linear tire forces

Z_eq=zeros(101,6);
dt = 0.01;

for i = 2:length(T)
   Z_eq(i,:) = Z_eq(i-1,:) + tiredynamics(T(i-1), Z_eq(i-1,:))' * dt;
end

Z_eq = Z_eq';

%{

% 1.2.2 Linearize bicycle model for feedback gains with linear tire forces
Q= eye(5);
R= 0.5;
[K,P]= lqr_LTV(@(i) Afunction(T(i),Z_eq(:,i)'),@(i) Bfunction(T(i),Z_eq(:,i)'),Q,R,T);
%K=

% 1.2.3 Plot linear vs nonlinear tire forces and find max % difference
delta_fun=@(t) pi/180*sin(2*pi*t)-0.00175;
alphaf = delta_fun(T) - (Z_eq(4,:) + a*Z_eq(5,:))/u_0;
alphar = -(Z_eq(4,:) - b*Z_eq(5,:))/u_0;
Fyf_lin = F_zf*B*C*D * alphaf;
Fyr_lin = F_zr*B*C*D*alphar;

%Z_nl=zeros(101,5);
%dt = 0.01;
%for i = 2:length(T)
%   Z_nl(i,:) = Z_nl(i-1,:) + magicequations(T(i-1), Z_nl(i-1,:))' * dt;
%end
%Z_nl = Z_nl';
af_nl = delta_fun(T) - atan((Z_eq(4,:) + a*Z_eq(5,:))/u_0);
ar_nl = - atan((Z_eq(4,:) - b*Z_eq(5,:))/u_0);
Fyf_nl = F_zf*D*sin(C*atan(B*(1-E)*alphaf + E*atan(B*af_nl)));
Fyr_nl = F_zr*D*sin(C*atan(B*(1-E)*alphar + E*atan(B*ar_nl)));
    

efront = max(abs(Fyf_lin - Fyf_nl)./abs(Fyf_nl));
erear = max(abs(Fyr_lin - Fyr_nl)./abs(Fyr_nl));
tireforce_percent_error = 100*max(efront,erear);
%}
% computation of non-linear model? 
Z_nl=zeros(101,6);
dt = 0.01;
Zprimed = Z_eq';
delta = delta_fun(T);
for i = 2:length(T)
   delta_fb = deltafeedback(Zprimed(i-1,:), Z_nl(i-1,:), K{i-1}, delta(i-1));
   Z_nl(i,:) = Z_nl(i-1,:) + magicequations(T(i-1), Z_nl(i-1,:), delta_fb)' * dt;
end

%Z_nl is array or positions(Z) computed using non-linear methods 

%error = ((Zprimed(:,1) - Z_nl(:,1)).^2 + (Zprimed(:,2) - Z_nl(:,2)).^2 ).^.5;

%max_distance_error = max(error)

%%FINAL OUTPUT WE WANT IS ARRAY OF [delta, Fx,n]
% how do we generate Fx? 
function [move, accumlated_poserr] = steering(lpinp, ref_track, curr_state,lastoutput,outputsteps,accumlated_poserr)
    
    maxPinput = 0.07;
    maxsteeringout = .15;
    maxsteeringRateOfChange = .07;
    max_accumerrorlimit = 5;
    Igain = -.00001;
    desire_speed = 5;
    pGainoffCenter = .1;
    Pdeadband = 1;
    Ideadband = 1;
    output = zeros(outputsteps,3);
    output(:,2) = 90;

    curr_state = curr_state';
    cenpts = ref_track.cline;
    ref_heading = ref_track.theta;
    curr_pt = [curr_state(1), curr_state(3)];

    %Find closest position on track to current location
    Idx = knnsearch(cenpts', curr_pt);
    %look up change in heading using same index
    matchpt = lpinp(1, Idx);
    
    poseerror = angdiff(curr_state(4),ref_heading(Idx));
    cenpts = cenpts';
    
    %compute which side of center using cross product and
    %apply proportional steering correction
    whichside = whichwayoffcenter(Idx, cenpts, curr_state);
    refpt = [cenpts(Idx,1) cenpts(Idx,2)];

    howfar = norm(refpt - curr_pt);

    %proportional centerline control
    if abs(howfar) > Pdeadband
        Pgain = -sign(whichside(3)) * howfar * pGainoffCenter;
        if abs(Pgain) > maxPinput
            Pgain = sign(Pgain) * maxPinput;
        end
    else
        Pgain = 0;
    end

    %integral centerline control
%     accumlated_poserr = sign(whichside(3))* abs(howfar) + accumlated_poserr;

    if howfar > Ideadband
        accumlated_poserr = sign(whichside(3))* abs(howfar) + accumlated_poserr;
    end

    
    %protect for windup
    if abs(accumlated_poserr) >= max_accumerrorlimit & sign(whichside(3)) ~= sign(accumlated_poserr)
        accumlated_poserr = 0;
    end



    Igain = accumlated_poserr * Igain;

    %proportional speed control (only decrease force from baseline if exceed desired speed)
    if curr_state(2) > desire_speed
        output(:,2) = lastoutput(end,2)*.9;
%     elseif curr_state(2) <= desire_speed
%         output(:,2) = lastoutput(end,2)*1.1; 
    end


    %% FOR SOME REASON LEFT AND RIGHT ARE FLIPPED RIGHT NOW SO DEAL WITH IT

    output = steeringrule(matchpt, output);

    output(:,1) = output(:,1) + Pgain + Igain;

    %rate limit
    for i = 1:length(output)
        outrate = lastoutput(end,1) - output(1,1);
    if abs(outrate) > maxsteeringRateOfChange
        output(i,1) = lastoutput(end,1) + sign(output(i,1))*maxsteeringRateOfChange;
        output(i,1) = sign(output(i,1)) * min(abs(output(i,1)),maxsteeringout);
        lastoutput = output(i,:);
    end
    end

    move = output;
    
end

function whichside = whichwayoffcenter(Idx, cenpts, curr_state)
if Idx <= length(cenpts) -1
    myposline = [curr_state(1)-cenpts(Idx,1), curr_state(3) - cenpts(Idx,2),0];
    trackline = [cenpts(Idx+1,1)- cenpts(Idx,1), cenpts(Idx+1,2) - cenpts(Idx,2),0]; 
    whichside = cross(trackline,myposline);
else
    whichside = 0;
end
end

function output = steeringrule(matchpt, output)

s_l = -0.02;
%slight right
s_r = 0.02; 
%hard left
h_l = -0.1;
%hard right
h_r = 0.1; 

hard_turn = .06;
soft_turn = 0.0175;

if(matchpt < h_l)
    %turn hard left
    output(:,1) = -hard_turn;
    output(:,3) = -2;
elseif (matchpt < s_l)
    %turn slight left
    output(:,1) = -soft_turn;   
    output(:,3) = -1;
elseif (matchpt > h_r)
    %   %turn hard right
    output(:,1) = hard_turn;
    output(:,3) = 2;
elseif (matchpt > s_r)
    %turn slight right
    output(:,1) = soft_turn;
    output(:,3) = 1;
else
    %go straight
    output(:,1) = 0;
    output(:,3) = 0;
end
end




function d_fb = deltafeedback(Z_eq, Z_nl, K,delta)
    d_fb = K * (Z_eq - Z_nl)' + delta;
    if abs(d_fb) > 45*pi/180
        d_fb = sign(d_fb)*(45*pi)/180;
    end
end

function zdot = tiredynamics(t, z)
    % Vehicle Parameters
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
    
    %z is the initial state here, aka the state we pass in at each time
    %iteration 
    % z = [x,u,y,v,psi,r]' 
    x = z(1);
    u = z(2);
    y = z(3);
    v = z(4);
    psi = z(5);
    r = z(6);
    %% NEED TO UPDATE DELTA FUNCTION AND THIS STUFF, MIGHT NOT BE NECESSARY DEPENDING ON IF WE ONLY DO MAGIC EQAUATIONS AND IGNORE TIRE DYNAMICS.
    delta_fun=@(t) pi/180*sin(2*pi*t)-0.00175;
    
    F_zf = (b/L)*m*g;
    F_zr = (a/L)*m*g;
    
    Ca_r= F_zr*B*C*D;
    Ca_f= F_zf*B*C*D;

    alphaf = delta_fun(t) - (z(4) + a*z(5))/u_0;
    alphar = - (z(4) - b*z(5))/u_0;
    
    fyf = Ca_f*alphaf;
    fyr = Ca_r*alphar;
    % z = [x,y,psi,v,r]
    zdot = zeros(5,1);
    zdot(1) = u_0*cos(z(3)) - z(4)*sin(z(3));
    zdot(2) = z(4)*cos(z(3)) + u_0*sin(z(3));
    zdot(3) = z(5);
    zdot(4) = (1/m) * (fyr + fyf - m*u_0*z(5));
    zdot(5) = (1/Iz) * (-b*fyr + a*fyf);

end

function Afun = Afunction(t, z)
    % Vehicle Parameters
    a   =  1.14;		% distance c.g. to front axle (m) 
    L   =  2.54;		% wheel base (m)
    m   =  1500;		% mass (kg)
    Iz  =  2420.0;	        % yaw moment of inertia (kg-m^2)
    b=L-a;                  % distance of c.g to rear axel (m) 
    g=9.81;
    u_0=20;
    
    % Tire forces
    B=10;
    C=1.3;
    D=1;
    E=0.97;
    
    delta_fun=@(t) pi/180*sin(2*pi*t)-0.00175;
    
    F_zf = (b/L)*m*g;
    F_zr = (a/L)*m*g;
    
    Ca_r= F_zr*B*C*D;
    Ca_f= F_zf*B*C*D;

    alphaf = delta_fun(t) - (z(4) + a*z(5))/u_0;
    alphar = - (z(4) - b*z(5))/u_0;
    
    fyf = Ca_f*alphaf;
    fyr = Ca_r*alphar;
    % z = [x,y,psi,v,r]
    zdot = zeros(5,1);
    zdot(1) = u_0*cos(z(3)) - z(4)*sin(z(3));
    zdot(2) = z(4)*cos(z(3)) + u_0*sin(z(3));
    zdot(3) = z(5);
    zdot(4) = (1/m) * (fyr + fyf - m*u_0*z(5));
    zdot(5) = (1/Iz) * (-b*fyr + a*fyf);
    
    Afun = zeros(5,5);
    
    Afun(1,3) = -z(4)*cos(z(3))-u_0*sin(z(3));
    Afun(1,4) = -sin(z(3));
    Afun(2,3) = u_0*cos(z(3))-z(4)*sin(z(3)); 
    Afun(2,4) = cos(z(3));
    Afun(3,5) = 1;
    Afun(4,4) = (1/m)*((-Ca_f/u_0 - Ca_r/u_0));
    Afun(4,5) = (1/m) * (-m*u_0 + (-Ca_f*a/u_0) + (Ca_r*b/u_0));
    Afun(5,4) = (1/Iz)*((-Ca_f*a/u_0) + (Ca_r*b/u_0));
    Afun(5,5) = (1/Iz)*((-Ca_f*a^2/u_0) - (Ca_r*b^2/u_0));
end

function Bfun = Bfunction(t, z)
    % Vehicle Parameters
    a   =  1.14;		% distance c.g. to front axle (m) 
    L   =  2.54;		% wheel base (m)
    m   =  1500;		% mass (kg)
    Iz  =  2420.0;	        % yaw moment of inertia (kg-m^2)
    b=L-a;                  % distance of c.g to rear axel (m) 
    g=9.81;
    u_0=20;
    
    % Tire forces
    B=10;
    C=1.3;
    D=1;
    E=0.97;
    
    delta_fun=@(t) pi/180*sin(2*pi*t)-0.00175;
    
    F_zf = (b/L)*m*g;
    F_zr = (a/L)*m*g;
    
    Ca_r= F_zr*B*C*D;
    Ca_f= F_zf*B*C*D;

    alphaf = delta_fun(t) - (z(4) + a*z(5))/u_0;
    alphar = - (z(4) - b*z(5))/u_0;
    
    fyf = Ca_f*alphaf;
    fyr = Ca_r*alphar;
    % z = [x,y,psi,v,r]
    zdot = zeros(5,1);
    zdot(1) = u_0*cos(z(3)) - z(4)*sin(z(3));
    zdot(2) = z(4)*cos(z(3)) + u_0*sin(z(3));
    zdot(3) = z(5);
    zdot(4) = (1/m) * (fyr + fyf - m*u_0*z(5));
    zdot(5) = (1/Iz) * (-b*fyr + a*fyf);
    
    Bfun = zeros(5,1);
    Bfun(4,1) = Ca_f/m; 
    Bfun(5,1) = Ca_f*a/Iz; 
end

function zdot = magicequations(t, z, delta_fb, Fx)
    %delta = [-.5, .5]; %% NEED TO FIGURE OUT HOW TO GET THESE INTO THIS FUNCTION
    %Fx = [-5000, 5000];
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
    
    %z is the initial state here, aka the state we pass in at each time
    %iteration 
    % z = [x,u,y,v,psi,r]' 
    x = z(1);
    u = z(2);
    y = z(3);
    v = z(4);
    psi = z(5);
    r = z(6);
    %magic equations, input is delta_f given presumably by the delta
    %feedback function 
    delta_f = delta_fb;
    alpha_f = delta_f - inv(tan((v+a*r)/u));
    alpha_r = -inv(tan((v-b*r)/u));
    
    phi_yf = (1-Ey) * (alpha_f + Shy) + (Ey/By)*inv(tan(By*(alpha_f + Shy)));
    phi_yr = (1-Ey) * (alpha_r + Shy) + (Ey/By)*inv(tan(By*(alpha_r + Shy)));

    F_zf = (b/(a+b))*m*g;
    F_zr = (a/(a+b))*m*g;

    F_yf = F_zf * Dy * sin(Cy * inv(tan(By*phi_yf))) + Svy;
    F_yr = F_zr * Dy * sin(Cy * inv(tan(By*phi_yr))) + Svy;
    
    
    % zdot = [xdot, udot, ydot, vdot, psidot, rdot]' 
    zdot = zeros( 6, 1 );
    zdot( 1 ) = u * cos(psi) - v*sin(psi); %Xdot
    zdot( 2 ) = (1/m)*(-f*m*g + Nw*Fx - F_yf*sin(delta_f)) + v*r; % udot
    zdot( 3 ) = u*sin(psi) + v*cos(psi); %Ydot
    zdot( 4 ) = (1/m) * (F_yf * cos(delta_f) + F_yr) - u*r; % vdot
    zdot( 5 ) = r; %psidot
    zdot( 6 ) = (a*F_yf*cos(delta_f) - b*F_yr)/Iz ;%rdot
    
    %delta_fun=@(t) pi/180*sin(2*pi*t)-0.00175; 
    % homework 2 equations
    %{
    F_zf = (b/L)*m*g;
    F_zr = (a/L)*m*g;

    alphaf = delta_fb - atan((z(4) + a*z(5))/u_0);
    alphar = - atan((z(4) - b*z(5))/u_0);
    
    fyf = F_zf * D * sin(C * atan( B*(1-E)* alphaf + E * atan(B * alphaf)));
    fyr = F_zr * D * sin(C * atan( B*(1-E)* alphar + E * atan(B * alphar)));

    % z = [x,y,psi,v,r]
    zdot = zeros( 5, 1 );
    zdot( 1 ) = u_0 * cos( z(3) ) - z(4) * sin( z(3)); %Xdot
    zdot( 2 ) = z(4) * cos( z(3) ) + u_0 * sin( z(3)); % Ydot
    zdot( 3 ) = z(5); %psi
    zdot( 4 ) = (fyf*cos(delta_fb) + fyr - m*u_0*z(5) )/m ; % vdot
    zdot( 5 ) = ( - b* fyr + a* fyf *cos(delta_fb))/Iz ;%rdot
    %}
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
