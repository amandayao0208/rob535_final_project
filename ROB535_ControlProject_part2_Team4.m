%% Self Driving Cars Control Part 2
% worked on by Dyllian Powell, Spencer Talbot, and Joseph Drockton

close all 
clear all
clc 

%% Getting track parameters
load('TestTrack.mat'); % loading the test track
center = TestTrack.cline;
right = TestTrack.br;   % defining right boundary of track
left = TestTrack.bl;    % defining left boundary of track
theta = TestTrack.theta; % defining heading angle of track

%% Generating random obstacles (will probably have to remove before submission)

avoid_obs_flag = true;
Xobs = 0;
if avoid_obs_flag == true
    %Nobs = 25;
    %Xobs = generateRandomObstacles(Nobs,TestTrack);
    Xobs = {[267.0753 -121.1613;273.3790 -118.8634;272.5333 -116.5435;266.2296 -118.8413], [246.491470035382,-80.8900077438253;252.791647654848,-78.6999545797396;252.272949213638,-77.2078025333206;245.972771594173,-79.3978556974062]};
end

%% Finding relative angles of the track and low passing them
low_p_flag = true;
dtheta = ang_diff(theta); % unfortunately we can't use Matlab's angdiff b/c it's not in one of the allowed toolboxes
lpinp=lowpass(dtheta,1000,10000); % low passing relative angles

%% Plotting smoothed relative angles against the original relative angles
% hold on
% figure(1)
% plot(lpinp) % plotting smoothed relative angles
% plot(diff(theta),'b') % plotting original relative angles
% lpinp = horzcat(lpinp,0); % pad with a 0 so sizes match for lookup
% hold off
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
Ystore(:,1) = initial_z;
Y = initial_z;
Tstore = zeros(1,outputsteps*loopsteps); % timestep vector
Inputstore = zeros(3,outputsteps*loopsteps); % storing the inputs over time ***this is what we are submitting for part 1
testinput = [0 100 0]; % testing an input?
accum_error = 0; % accumulated error? if so, I'm not sure what it's for
lane_side_prev = "center";
disp("desired lane: " + lane_side_prev);

%% Initializing Simulation
window_size = 160; % changes the figure window size
sensing_radius = 150;   % sensing radius for sensing obstacles
[xunit, yunit] = circle(Ystore(1,1), Ystore(3,1), sensing_radius);
figure(1);
hold on
plot(center(1,:),center(2,:)); % plotting center line
plot(right(1,:),right(2,:),'r'); % plotting right boundary
plot(left(1,:),left(2,:),'r'); % plotting left boundary
if avoid_obs_flag == true
    draw_obs(Xobs)
end
h = plot(xunit, yunit,'b');
hold off
car = get_triangle_points(Ystore(:,1), 3, 4);
p = patch(car(1,:), car(2,:), 'green');


%% Running the model and performing calculations
for i=1:loopsteps
    %steering with obs enabled 
    [testinput,accum_error, lane_side] = steering_obs(lpinp,TestTrack,Yprev,testinput,outputsteps,accum_error, Xobs, avoid_obs_flag, low_p_flag); % calling steering function
    [Y, T]=forwardIntegrateControlInput(testinput(:,1:2),Yprev); % new state calculation
    Y = Y'; % transposing state vector
    Ystore(:,(i-1)*outputsteps+1:(i-1)*outputsteps+outputsteps) = Y; % storing history of state
    Yprev = Y(:,end); % setting previous state to current state
    Inputstore(:,(i-1)*outputsteps+1:(i-1)*outputsteps+outputsteps) = testinput'; % storing inputs
    [p.XData, p.YData, h.XData, h.YData] = simulate(Ystore(:,i), window_size, sensing_radius); % simulation of car with sensing radius
    if lane_side ~= lane_side_prev
        disp("desired lane: " + lane_side);
        lane_side_prev = lane_side;
    end
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
draw_obs(Xobs)
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
%Yfin = [Ystore(1,:); Ystore(3,:)];
%info = getTrajectoryInfo(Yfin',Inputstore(1:2,:)',[],T,TestTrack); % call to see if we crash

%% Functions
function [CarDataX, CarDataY, circleDataX, circleDataY] = simulate(curr_state, window_size, sensing_radius)
    X = curr_state(1);    % current x position of car
    Y = curr_state(3);    % current y position of car
    axis([X - window_size, X + window_size, Y - window_size, Y + window_size])
    car = get_triangle_points(curr_state, 3, 4);
    CarDataX = car(1,:);
    CarDataY = car(2,:);

    [circleDataX, circleDataY] = circle(X, Y, sensing_radius);
%     viscircles([X,Y], 75, 'Color','b', 'LineWidth', 1)
    drawnow limitrate
end

function draw_obs(Xobs)
    for i = 1:length(Xobs)
        points = Xobs{i};
        x_vals = [points(:,1); points(1,1)];
        y_vals = [points(:,2); points(1,2)];

        plot(x_vals, y_vals, 'b')
    end
end

function [throttle, velocity_err_hist] = longitudinal_controller(curr_vel, desired_vel, velocity_err_hist, dt, Xobs)
    kp = 1;
    ki = 0;
    kd = 0;
    
    prev_error = velocity_err_hist(end);
    curr_error = desired_vel - curr_vel;
    velocity_err_hist = [velocity_err_hist curr_error];

    D_err = (curr_error - prev_error) / dt;
    I_err = sum(velocity_err_hist);

    throttle = kp * curr_error + ki * I_err + kd * D_err;
end

function delta = stanleyController(lpinp, ref_track, curr_state, low_p_flag)
    a = 1.35;
    ke = 1;
    kv = 1;
    X = curr_state(1);    % current x position of car
    u = curr_state(2);    % current forward velocity of car
    Y = curr_state(3);    % current y position of car
    psi = curr_state(5);  % current heading of car

    center = ref_track.cline;
    if low_p_flag == true
        theta = lpinp;
    else
        theta = ref_track.theta;
    end
    curr_pos = [X + a * cos(psi); Y + a * sin(psi)];
    cross_track_error = check_pos(lpinp, ref_track, curr_state, curr_pos, psi, low_p_flag);

    heading_error_cross_track = atan2(ke * cross_track_error / (kv + u));

    % control law
    steering_hope = heading_error + heading_error_cross_track;
    steering_hope = wrap(steering_hope);
    steering_hope = clamp(steering_hope, -0.5, 0.5);

    % update delta
    delta = steering_hope;
end

function [desired_heading, cross_track_error, side] = check_pos(lpinp, ref_track, curr_state, curr_pos, heading, low_p_flag)
    psi = heading;  % current heading of car
    center = ref_track.cline;
    if low_p_flag == true
        theta = lpinp;
    else
        theta = ref_track.theta;
    end

    % calculate heading error
    [track_idx, d1] = knnsearch(center', curr_pos);
    flag = "start";
    d2 =  sqrt((curr_pos(1) - center(1, track_idx + 1))^2 + (curr_pos(2) -  center(2, track_idx + 1))^2);
    if track_idx ~= 1 && track_idx ~= length(center(1,:))
        [idx_check, d2] = knnsearch([center(:, track_idx - 1)'; center(:, track_idx + 1)'], curr_pos);
        if idx_check == 1
            flag = "behind";
            desired_heading = theta(track_idx);
        else
            flag = "in_front";
            desired_heading = theta(track_idx + 1);
        end
    else
        desired_heading = theta(track_idx);
    end
    heading_error = desired_heading - psi;
    heading_error = wrap(heading_error);

    % calculate crosstrack error
    % use scalene triangle formula to find crosstrack error
    if flag == "in_front"  || flag == "start"
        point2 = [center(1, track_idx + 1); center(2, track_idx + 1)];
        point1 = [center(1, track_idx); center(2, track_idx)];
        cross_track_error = scalene(point1, point2, d1, d2);
        assert(isreal(cross_track_error));
        side = whichwayoffcenter(track_idx, center', curr_state);
        if side > 0 % left side
            cross_track_error = -cross_track_error;
        end
    end

    if flag == "behind"
        point2 = [center(1, track_idx); center(2, track_idx)]; 
        point1 = [center(1, track_idx - 1); center(2, track_idx - 1)];
        cross_track_error = scalene(point1, point2, d1, d2);
        assert(isreal(cross_track_error));
        side = whichwayoffcenter(track_idx, center', curr_state);
        if side > 0 % left side
            cross_track_error = -cross_track_error;
        end
    end
end

function [move, accumlated_poserr, lane_side] = steering_obs(lpinp, ref_track, curr_state,lastoutput,outputsteps,accumlated_poserr,Xobs, avoid_obs_flag, low_p_flag)
    maxPinput = 0.07;               % maximum p gain
    maxsteeringout = .15;           % delta steering
    maxsteeringRateOfChange = .07;  % derivative clamp 
    max_accumerrorlimit = 5;        % integral error limit
    Igain = 0;                %integral gain
    desire_speed = 5;               % Desired Fx steady state speed 
    pGainoffCenter = .1;            % P gain 
    Pdeadband = 1;                  % deadbands around line (0 = line exactly pretty much)
    Ideadband = 1;
    output = zeros(outputsteps,3);  % initializing output vector of inputs
    output(:,2) = 90;   % not sure what this is for

    curr_state = curr_state';   % transposing current state into a row vector?
    cenpts = ref_track.cline;   % define centerpoints
    right = ref_track.br;       % define right boundary
    left = ref_track.bl;        % define left boundary

    % construct centerlines for left and right lane
    c_right = (right + cenpts)/2;
    c_left = (left+cenpts)/2;
    ref_heading = ref_track.theta;
    curr_pt = [curr_state(1), curr_state(3)];
    
%     %Find closest position on track to current location
%     Idx = knnsearch(cenpts', curr_pt);
%     %look up change in heading using same index
%     matchpt = lpinp(Idx);
%     %poseerror = angdiff(curr_state(4),ref_heading(Idx));
%     cenpts = cenpts';
%     %compute which side of center using cross product and
%     %apply proportional steering correction
%     whichside = whichwayoffcenter(Idx, cenpts, curr_state);
%     refpt = [cenpts(Idx,1) cenpts(Idx,2)];
    
    if avoid_obs_flag == true
        %disp('checking for obs')
        [refpt, lane, lane_side] = avoidObs(lpinp, ref_track, curr_state, Xobs);
    end
    new_ref_track = ref_track;
    new_ref_track.cline = lane;
%     howfar = norm(refpt - curr_pt);
    [matchpt, howfar, whichside] = check_pos(lpinp, new_ref_track, curr_state, curr_pt, curr_state(5), low_p_flag);

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
        accumlated_poserr = sign(whichside(3)) * abs(howfar) + accumlated_poserr;
    end

    
    %protect for windup
    if abs(accumlated_poserr) >= max_accumerrorlimit &&  sign(whichside(3)) ~= sign(accumlated_poserr)
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

function output = steeringrule(matchpt, output)
%derivative threshholds
s_l = -0.02;
%slight right
s_r = 0.02; 
%hard left
h_l = -0.1;
%hard right
h_r = 0.1;

%deltas 
hard_turn = .06;
soft_turn = 0.0175;
%second(3rd element)output is for plotting which rule was chosen 
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

function [refpt, lane, lane_side] = avoidObs(lpinp, ref_track, curr_state, Xobs)
    curr_state = curr_state';   % transposing current state into a 
    cenpts = ref_track.cline;   % define centerpoints
    right = ref_track.br;       % define right boundary
    left = ref_track.bl;        % define left boundary
    lane = cenpts;
    lane_side = "center";


    % construct centerlines for left and right lane
    c_right = (right + cenpts)/2;
    c_left = (left+cenpts)/2;
    ref_heading = ref_track.theta;
    curr_pt = [curr_state(1), curr_state(3)];
    
    % check if we see any obstacles (stores in cell array)
    Xobs_seen = senseObstacles(curr_pt, Xobs);
    
    % calculate center point of each obstacle
    Obs = [];   % we don't know how many obstacles we have, so we can't preallocate a vector
    for i=1:length(Xobs_seen)
        Obs(i,:) = mean(Xobs_seen{i});  % creating a vector of center points of the obstacles seen
    end
    
    % extract heading angle from vehicle and relative angle to obstacle
    % center point
    front_obs =[];  % creating an empty vector for the first obstacle
    heading = curr_state(5);
    % calculate distance between car and obstacles
    if(Obs)
        %disp('There are obstacles within the circle')
        for i=1:size(Obs)
           obx = Obs(i,1); % x location of obs
           oby = Obs(i,2); % y location
           x_d = obx - curr_pt(1); % x distance of triangle
           y_d = oby - curr_pt(2); % y distance of triangle
           theta = atan2(y_d,x_d); % angle of vector to obstacle from x axis
           %alpha = heading - theta; % angle between heading and obstacle
           dist = norm(curr_pt - [obx,oby]);
           obs_vec = [dist,obx,oby,x_d,y_d,theta];
           if (theta > (heading - deg2rad(90)) && theta < (heading+deg2rad(90)))
              front_obs = [front_obs;obs_vec]; 
           %else
              %disp('Obstacle is not in front of us')
              %disp(heading)
              %disp(alpha)
           end
        end
    end
    % final front_obs vector: [distance, x,y, x_d, y_d, alpha] 
    % find closest obstacle
    if(front_obs)
        [~,I] = min(front_obs(:,1));
        %grab global x,y of next closest obstacle
        nxt_obs = front_obs(I,2:3);
        %WANT TO: calculate which side obstacle is on (L,R), then tell car to
        %go to follow the lane oposite to that side.
        %[~,l_point] = knnsearch(left',nxt_obs); % nearest point to obstacle on either side
        %[~,r_point] = knnsearch(right',nxt_obs);
        [cen_idx ,~] = knnsearch(cenpts',nxt_obs); % find nearest centerpt indx
        %side_l = norm(l_point - nxt_obs); % distance between obstacle and track sides
        %side_r = norm(r_point - nxt_obs);
        nxt_obs(3) = front_obs(I,3);
        which_side = whichwayoffcenter(cen_idx, cenpts', nxt_obs);
        
        if (which_side(3) >= 0) % obstacle closer to left side of track
            % set new goal path to right lane
%             disp('Changing lane to right lane @\n')
%             disp(curr_pt)
            lane = c_right';
            lane_side = "right";
        else % else obstacle closer to right side
            % set new goal path to left lane
%             disp('Changing lane to left lane @\n')
%             disp(curr_pt)
            lane = c_left';
            lane_side = "left";
        end
        Idx = knnsearch(lane, curr_pt);
        %look up change in heading using same index
        matchpt = lpinp(1, Idx);
        %poseerror = angdiff(curr_state(4),ref_heading(Idx));
     
        %compute which side of center using cross product and
        %apply proportional steering correction
        whichside = whichwayoffcenter(Idx, lane, curr_state);
        refpt = [lane(Idx,1) lane(Idx,2)];
    else
        %Find closest position on track to current location
        Idx = knnsearch(cenpts', curr_pt);
        %look up change in heading using same index
        matchpt = lpinp(1, Idx);
        %poseerror = angdiff(curr_state(4),ref_heading(Idx));
        cenpts = cenpts';
        %compute which side of center using cross product and
        %apply proportional steering correction
        whichside = whichwayoffcenter(Idx, cenpts, curr_state);
        refpt = [cenpts(Idx,1) cenpts(Idx,2)];
    end
    if size(lane, 1) ~= 2
        lane = lane';
    end
end

function whichside = whichwayoffcenter(Idx, cenpts, curr_state) %cross product of heading x centerline to determine side you're on
if Idx <= length(cenpts) - 1
    myposline = [curr_state(1)-cenpts(Idx,1), curr_state(3) - cenpts(Idx,2),0]; % unit vector from car to center line point 
    trackline = [cenpts(Idx+1,1)- cenpts(Idx,1), cenpts(Idx+1,2) - cenpts(Idx,2),0]; % unit vector between curr pt and next point on track
    whichside = cross(trackline,myposline); % z dim of x-product tells if you are on left or right (left is positive, right is negative)
else
    whichside = [0 0 0];
end
end

function dangle = ang_diff(alpha, beta)
    % this function is only good for vectors of angles, not matrices like
    % the Matlab angdiff function
    if nargin > 2
        error("too many inputs");   % if the number of arguments is greater than 2, throw error and stop code
    end
    if nargin == 1
        dangle = zeros(1, length(alpha) - 1);   % create vectore of size one less than the input vector
        for i = 1:length(alpha) - 1 
            dangle(i) = wrap(alpha(i + 1) - alpha(i)); % find the difference between two angles next to each other and map them to [-pi, pi]
        end
    end
    if nargin == 2
        dangle = wrap(beta - alpha); % find difference between two angles and map to [-pi, pi]
    end
end    

function new_angle = wrap(angle)
    % mapping angle to [-pi. pi]
    angle = mod(angle + pi, 2*pi);
    if angle < 0
        angle = angle + 2*pi;
    end
    new_angle = angle - pi;
end

function var = clamp(var, amin, amax)
    % clamps a value or vector, var, to a range of [amin, amax]
    var(var>amax) = amax;   % clamping max value(s)
    var(var<amin) = amin;   % clamping min value(s)
end

function cross_track_error = scalene(point1, point2, d1, d2)
        % caluclates the cross track error using the formula for a scalene
        % triangle. d1 and d2 are the distances from the car to the two
        % closest waypoints, point1 and point2.
        assert(d1 > 0, "d1 must be greater than 0");
        assert(d2 > 0, "d2 must be greater than 0");
        x2 = point2(1); % x-value point 2
        x1 = point1(1); % x-value of point 1
        y2 = point2(2); % y-value of point 2
        y1 = point1(2); % y-value of point 1
        D = sqrt((x2 - x1)^2 + (y2 - y1)^2);    % distance between the two waypoints
        s = (d1 + d2 + D) / 2;  % semi perimeter of scalene triangle
        A = sqrt(s*(s - d1)*(s - d2)*(s - D));  % area calculation of triangle
        cross_track_error = 2 * A / D;     % height of the triangle with D as the base
end

function points = get_triangle_points(curr_state, w, h)
    X = curr_state(1);    % current x position of car
    Y = curr_state(3);    % current y position of car
    psi = curr_state(5);  % current heading of car
    centroid = [X; Y];
    point1 = rotate_point([X + 2/3*h; Y], psi, centroid);
    point2 = rotate_point([X - 1/3*h; Y + w/2], psi, centroid);
    point3 = rotate_point([X - 1/3*h; Y - w/2], psi, centroid);

    points = [point1 point2 point3];
end

function rotated_point = rotate_point(point, angle, centroid)
    R = [cos(angle) -sin(angle);
         sin(angle) cos(angle)];

    point = [point(1); point(2)] - centroid;
    rotated_point = R * point + centroid;
end

function [xunit, yunit] = circle(x,y,r)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
end
