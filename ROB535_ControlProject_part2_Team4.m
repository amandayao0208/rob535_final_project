%% Self Driving Cars Control Part 2
% worked on by Dyllian Powell, Spencer Talbot, and Joseph Drockton
% This file is empty currently :]

close all 
clear all
clc 

%% Getting track parameters
load('TestTrack.mat'); % loading the test track
center = TestTrack.cline;
right = TestTrack.br;   % defining right boundary of track
left = TestTrack.bl;    % defining left boundary of track
theta = TestTrack.theta; % defining heading angle of track

%% Finding relative angles of the track and low passing them
dtheta = ang_diff(theta); % unfortunately we can't use Matlab's angdiff b/c it's not in one of the allowed toolboxes
lpinp=lowpass(dtheta,1000,10000); % low passing relative angles

%% Plotting smoothed relative angles against the original relative angles
hold on
plot(lpinp) % plotting smoothed relative angles
plot(diff(theta),'b') % plotting original relative angles
lpinp = horzcat(lpinp,0); % pad with a 0 so sizes match for lookup

%% Vehicle Parameters
delta = [-.5, .5]; % need to use this range to generate Delta_feedback most likely
Fx = [-5000, 5000];% not sure how we constrain this range, probably similar to delta_fb
m = 1400;   % mass
Nw = 2; 
f = 0.01;
Iz = 2667;  % moment of inertia about z
a = 1.35;
b = 1.45;
By = 0.27; %tire stuff. 
Cy = 1.2;
Dy = 0.7;
Ey = -1.6;
Shy = 0; %it's shy uwu
Svy = 0;
g = 9.806; %what kind of messed up gravity has 3 sig figs

%% Setting up for modeling
T=0:0.01:0.5; % Timespan for all simulations
initial_z = [287; 5; -176; 0; 2; 0]; % defining initial conditions
Yprev = initial_z; % initializing previous y value

outputsteps = 2; % ??
loopsteps=43100; % ??

%% Initializing vectors to store for plotting
% system inputs [steering angle; Fz; steeringdecisionrule @ current track
% location
Ystore = zeros(6,outputsteps*loopsteps); % state vector
Tstore = zeros(1,outputsteps*loopsteps); % timestep vector
Inputstore = zeros(3,outputsteps*loopsteps); % storing the inputs over time ***this is what we are submitting for part 1
testinput = [0 100 0]; % testing an input?
accum_error = 0; % accumulated error? if so, I'm not sure what it's for

%% Running the model and performing calculations
for i=1:loopsteps
    [testinput,accum_error] = steering(lpinp,TestTrack,Yprev,testinput,outputsteps,accum_error); % calling steering function
    [Y, T]=forwardIntegrateControlInput(testinput(:,1:2),Yprev); % new state calculation
    Y = Y'; % transposing state vector
    Ystore(:,(i-1)*outputsteps+1:(i-1)*outputsteps+outputsteps) = Y; % storing history of state
    Yprev = Y(:,end); % setting previous state to current state
%     Tstore = horzcat(Tstore,T);
    Inputstore(:,(i-1)*outputsteps+1:(i-1)*outputsteps+outputsteps) = testinput'; % storing inputs
end

%% Plotting path with colored turning decisions
figure(2)
subplot(2,1,1) % top plot in figure
hold on
plot(center(1,:),center(2,:)); % plotting center line
plot(right(1,:),right(2,:),'r'); % plotting right boundary
plot(left(1,:),left(2,:),'r'); % plotting left boundary
plot(Ystore(1,:),Ystore(3,:),'k'); % plotting car path taken
gscatter(Ystore(1,:),Ystore(3,:),Inputstore(3,:)); % plotting rules by color for steering
title('Vehicle State Colored by turning decision rule'); % setting plot title

%% Plotting path with forward velocity as colored
subplot(2,1,2)
hold on
plot(center(1,:),center(2,:)); % plotting center line
plot(right(1,:),right(2,:),'r'); % plotting right boundary
plot(left(1,:),left(2,:),'r');  % plotting left boundary
plot(Ystore(1,:),Ystore(3,:),'k'); % plotting car path taken
pointsize = 10; % setting point color to not look like poop
scatter(Ystore(1,:),Ystore(3,:),pointsize,Ystore(2,:)); 
colorbar
title('Center Line of Track');  

%% Testing to see if we finish the track using GSI code
% Yfin = [Ystore(1,:); Ystore(3,:)];
Yfin = [Ystore(1,:); Ystore(3,:)];
info = getTrajectoryInfo(Yfin',Inputstore(1:2,:)',[],T,TestTrack); % call to see if we crash

%% Functions
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
    
    poseerror = ang_diff(curr_state(4),ref_heading(Idx));
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

function dangle = ang_diff(alpha, beta)
    % this function is only good for vectors of angles, not matrices like
    % the Matlab angdiff function
    if nargin > 2
        error("too many inputs");
    end
    if nargin == 1
        dangle = zeros(1, length(alpha) - 1);
        for i = 1:length(alpha) - 1
            dangle(i) = wrap(alpha(i + 1) - alpha(i));
        end
    end
    if nargin == 2
        dangle = wrap(beta - alpha);
    end
end    

function new_angle = wrap(angle)
    angle = mod(angle + pi, 2*pi);
    if angle < 0
        angle = angle + 2*pi;
    end
    new_angle = angle - pi;
end

