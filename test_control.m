%%% Making simple waypoint line path to test controller
clc
clear

%% Various test cases
% straight line
% line = 100;
% x1 = linspace(0, line, line + 1);
% y1 = linspace(0, line, line + 1);
% center = [x1;y1];
% right = linspace(0, 0, (line + 1));
% left = linspace(0, 0, (line + 1));
% theta = linspace(pi/4, pi/4, (line + 1));

% appending points for a lane change maneuver
% x2 = linspace(line + 2, 2*line, line + 1);
% y2 = linspace(10, 10, line + 1);
% center = [x1 x2;y1 y2];
% right = linspace(0, 0, 2 * (line + 1));
% left = linspace(0, 0, 2 * (line + 1));
% theta = linspace(0, 0, 2 * (line + 1));

% ref_track.cline = center;
% ref_track.br = right;
% ref_track.bl = left;
% ref_track.theta = theta;

%% loading main track and defining inputs
load('TestTrack.mat');      % loading the test track
center = TestTrack.cline;   % defining right bonudary of track
right = TestTrack.br;       % defining right boundary of track
left = TestTrack.bl;        % defining left boundary of track
theta = TestTrack.theta;    % defining heading angle of track
ref_track = TestTrack;      % I don't really need to define a second struct, but this is how it is currently set up

% initial_state = [-5; 5; 5; 0; pi/4; 0];   % state vector for other cases
initial_state = [287; 5; -176; 0; 2; 0];    % normal initial state
initial_input = [0 100];    % initial input--I don't think it really matters what this is
store_state = [];   % making empty vector to store state
store_inputs = [];  % making empty vector to store inputs

%% Finding relative angles of the track and low passing them
low_p_flag = false;         % flag to determine whether or not to use the low passed values for theta--for the new controller, they make it worse
dtheta = ang_diff(theta);   % unfortunately we can't use Matlab's angdiff b/c it's not in one of the allowed toolboxes
lpinp=lowpass(dtheta,1000,10000);   % low passing relative angles
lpinp = horzcat(lpinp,0);           % pad with a 0 so sizes match for lookup


%% initializing simulation
window_size = 60;       % changes the figure window size
sensing_radius = 150;   % sensing radius for sensing obstacles
[xunit, yunit] = circle(initial_state(1), initial_state(3), sensing_radius);    % getting circle coordinates
figure(1);  % creating figure 1
hold on
plot(center(1,:),center(2,:),'k.-');    % plotting center line
plot(right(1,:),right(2,:),'r');        % plotting right boundary
plot(left(1,:),left(2,:),'r');          % plotting left boundary
h = plot(xunit, yunit,'b');             % creating plot object for circle
car = get_triangle_points(initial_state, 3, 4); % getting car coordinates to be represented as a triangle
p = patch(car(1,:), car(2,:), 'green');         % plotting car as green triangle

%% Initializing variables for the model
curr_state = initial_state;     % setting current state to initial state
prev_state = initial_state;     % setting previous state to initial state
dt = 0.01;              % time step as specified in document--I still don't understand this for how we are supposed to turn things in though
velocity_err_hist = 0;  % keeps track of the velocity error history in case we want to use an integral term for the longitudinal controller
U = zeros(2,2);         % creating a matrix to hold the inputs (the inputs are the same for both rows, but this is how the forwardIntegrateControlInput.m function wants it)
track_idx = 0;          % initializing the closest track index to zero (I expect it to throw an error if for some reason the real index isn't calculated)
n_count = 0;            % counter for the while loop
T = 0:dt:0.5;           % timespan for the forwardIntegrateControlInput.m function for ode45

%% Running the model
% continues until the closest index is the final index of the centerline
while track_idx ~= length(center(1,:))
    % computing values
    n_count = n_count + 1; % incrementing counter first since it started at zero
    track_idx = knnsearch(center', [curr_state(1) curr_state(3)]);  % computes the closest track index to the car
    [Fx, velocity_err_hist] = longitudinal_controller(curr_state, velocity_err_hist, dt);   % returns a value for Fx--this function controls the speed using PID (currently just P)
    delta = stanleyController(lpinp, ref_track, curr_state, low_p_flag);    % returns value for steering angle using the Stanley controller
    U = [delta Fx; delta Fx];   % creating matrix of inputs to send through the forwardIntegrateControlInput.m function
    [output_state, ~] = forwardIntegrateControlInput(U, prev_state');   % integrates to get us our next state
    curr_state = output_state(end,:)';  % updates the current state vector

    % storing values
    store_inputs(n_count, :) = [delta Fx];  % stores our inputs in a vector
    store_state(n_count, :) = prev_state;   % stores our states in a vector
    prev_state = curr_state;    % sets the previous state equal to the current state since we are ending the loop

    % simulating
    [p.XData, p.YData, h.XData, h.YData] = simulate(curr_state, window_size, sensing_radius); % simulation of car with sensing radius
end
disp('finished') % displays when the last index is the closest point--we need to append one extra point so the car passes the finish line


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

function [throttle, velocity_err_hist] = longitudinal_controller(curr_state, velocity_err_hist, dt)
    % This function controls the throttle, or Fx, of our car. It takes the
    % current state, the velocity error history, and the control time step
    % as inputs and outputs the throttle value and updated velocity error
    % history variables.

    kp = 20;    % proportional gain    
    ki = 0;     % integral gain
    kd = 0;     % derivative gain
    curr_vel = curr_state(2);   % defining the current velocity
    MIN_DES_VEL = 10;           % maximum desired velocity (determined by trial and error--we can definitely change this)
    MAX_DES_VEL = 20;           % minimum desired velocity " "
    MIN_HEAD_RATE = 0;          % minimum heading rate (used for slowing down the car as it gets bigger)
    MAX_HEAD_RATE = 0.3;        % maximum heading rate " "
    curr_heading_rate = abs(curr_state(6)); % defining the current heading rate (r)
    desired_vel = mapfun(curr_heading_rate, MIN_HEAD_RATE, MAX_HEAD_RATE, MIN_DES_VEL, MAX_DES_VEL);   % determining a desired velocity between MIN_DES_VEL and MAX_DES_VEL using the mapping function (like it's used in Arduino code)

    prev_error = velocity_err_hist(end);    % computing the previous velocity error
    curr_error = desired_vel - curr_vel;   % computing the current velocity error
    velocity_err_hist = [velocity_err_hist curr_error]; % appending the new velocity error histor value to the vector

    D_err = (curr_error - prev_error) / dt; % error rate to be multiplied by kd gain
    I_err = sum(velocity_err_hist); % error sum to be multiplied by ki gain
    
    throttle = kp * curr_error + ki * I_err + kd * D_err;   % computing the value for the throttle
    throttle = clamp(throttle, -5000, 5000);                % clamping Fx to its bounds
end

function delta = stanleyController(lpinp, ref_track, curr_state, low_p_flag)
    % This function controls the lateral motion of the car. It takes in the
    % low-passed heading values of the track (even though it makes
    % performance worse for this controller, I kept it in since we can set
    % that to false). The function als takes in the reference track,
    % current state, and the low pass flag. The output is the steering
    % angle using the stanley controller.

    a = 1.35;   % distance from the center of gravity of the car to the front steering axle
    ke = 2;     % a proportional gain for how fast we want to turn towards the desired target lane
    kv = 1;     % a gain to make sure the denominator isn't zero
    X = curr_state(1);    % current x position of car
    u = curr_state(2);    % current forward velocity of car
    Y = curr_state(3);    % current y position of car
    psi = curr_state(5);  % current heading of car
    
    % calculating heading error
    curr_pos = [X + a * cos(psi); Y + a * sin(psi)];    % current position of the front axis of the vehicle--the stanley controller depends on this assumption
    [desired_heading, cross_track_error, ~, ~] = check_pos(lpinp, ref_track, curr_state, curr_pos, psi, low_p_flag);    % getting the desired heading and the crosstrack error
    psi_e = desired_heading - psi;  % computing the heading error

    % control law
    des_steer = psi_e + atan2(ke * cross_track_error, (kv + u));    % computing the desired steering angle
    des_steer = wrap(des_steer);    % wrapping the desired steering angle to [-pi, pi] just in case
    des_steer = clamp(des_steer, -0.5, 0.5);    % clamping the desired steering angle to be within its bounds

    % update delta
    delta = des_steer;  % setting delta equal to the desired steering angle (this isn't really necessary, but it makes it easier to know what the function output is)
end

function [desired_heading, cross_track_error, side, FLAG_terminate] = check_pos(lpinp, ref_track, curr_state, curr_pos, heading, low_p_flag)
    % This function takes in a number of inputs shown above, and returns
    % the desired ehading, crosstrack error using the formula for a scalene
    % triangle, as well as the FLAG_terminate flag to determine if we have
    % finished the track.

    % initial definitions for readability
    psi = heading;              % current heading of car
    center = ref_track.cline;   % defining centerline as new variable for readability

    % checking whether or not to use low-passed theta value
    if low_p_flag == true       
        theta = lpinp;  % track heading equals low passed values
    else
        theta = ref_track.theta;    % track heading equals original values
    end
    curr_pos = [curr_pos(1) curr_pos(2)];   % defining current position as a row vector since that's how knnsearch wants it

    % calculate heading error
    [track_idx, d1] = knnsearch(center', curr_pos); % finding the closest point to the car and returning the point's index and distance away
    flag = "start"; % setting flag to start initially--this only really matters until the 2nd index is closest, but it also doesn't really affect anything after that
    
    % Finding the crosstrack error
    if track_idx == length(center(1,:)) % if the car has reached the final point in the track
        FLAG_terminate = 1;     % set binary flag to one
        d2 =  sqrt((curr_pos(1) - center(1, track_idx - 1))^2 + (curr_pos(2) -  center(2, track_idx - 1))^2);   % determine distance to point befor the last point so things don't break
        desired_heading = theta(track_idx);                             % finding desired heading
        point2 = [center(1, track_idx); center(2, track_idx)];          % defining position of closest point
        point1 = [center(1, track_idx - 1); center(2, track_idx - 1)];  % defining position of further point
        cross_track_error = scalene(point1, point2, d1, d2);            % finding the crosstrack error
        if ~isreal(cross_track_error)   % checking if the crosstrack error is real--it's computed as imaginary when it's right on the line (i.e. zero crosstrack error)
                cross_track_error = 0;  % setting crosstrack error to zero
        end
        side = whichwayoffcenter(track_idx, center', curr_state);   % determining the side of the track the car is on
        if side(3) > 0 % if greater than zero, the car is on the left side
            cross_track_error = -cross_track_error; % negate the crosstrack error
        end
    else
        % getting the heading
        FLAG_terminate = 0; % binary flag is zero since we haven't finished yet
        d2 =  sqrt((curr_pos(1) - center(1, track_idx + 1))^2 + (curr_pos(2) -  center(2, track_idx + 1))^2);   % determine distance to further point as precaution so things don't break for edge cases
        if track_idx ~= 1 && track_idx ~= length(center(1,:))   % if the closest point isn't the first or last point onf the track
            [idx_check, d2] = knnsearch([center(:, track_idx - 1)'; center(:, track_idx + 1)'], curr_pos);  % checking if we are closer to the point one ahead of the closest point or one index behind the closest point--also finds the distance away d2
            if idx_check == 1   
                flag = "behind"; % first index means we are behind the closest point
                desired_heading = theta(track_idx); % grabbing the desired heading
            else
                flag = "in_front"; % second index means that we are in front of the closest point
                desired_heading = theta(track_idx + 1); % grabbing the desired heading
            end
        else
            desired_heading = theta(track_idx); % grabbing the desired heading even if we are at the first or last index
        end
    
        % calculate crosstrack error
        if flag == "in_front"  || flag == "start"
            point2 = [center(1, track_idx + 1); center(2, track_idx + 1)];  % finding distance to further point
            point1 = [center(1, track_idx); center(2, track_idx)];          % finding distance to closer point (the closer point is always with regards to track_idx)
            cross_track_error = scalene(point1, point2, d1, d2);    % computing the cross_track_error
            if ~isreal(cross_track_error)   % checking if the crosstrack error is real
                cross_track_error = 0;  % setting the crosstrack error to zero if it's not real
            end
            side = whichwayoffcenter(track_idx, center', curr_state);   % determining which side of the road the car is on
            if side(3) > 0 % if the value is greater than zero, thecar is on the left side
                cross_track_error = -cross_track_error; % negate the crosstrack error
            end
        end
    
        if flag == "behind"
            point2 = [center(1, track_idx); center(2, track_idx)]; % finding the distance to the closer point
            point1 = [center(1, track_idx - 1); center(2, track_idx - 1)];  % finding the distance to the further point
            cross_track_error = scalene(point1, point2, d1, d2);    % computing the crosstrack error
            if ~isreal(cross_track_error)   % checking if the crosstrack error is real
                cross_track_error = 0;  % setting the crosstrack error to zero if it's not real
            end
            side = whichwayoffcenter(track_idx, center', curr_state);   % determining which side of the road the car is on
            if side(3) > 0 % if the value is greater than zero, thecar is on the left side
                cross_track_error = -cross_track_error; % negate the crosstrack error
            end
        end
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
    centroid = [X; Y];  % centroid point of car

    % defining each point in the triangle and rotating them by the heading
    point1 = rotate_point([X + 2/3*h; Y], psi, centroid);   % tip of triangle
    point2 = rotate_point([X - 1/3*h; Y + w/2], psi, centroid); % bottom left corner if triangle is up
    point3 = rotate_point([X - 1/3*h; Y - w/2], psi, centroid); % bottom right corner if triangle is up

    points = [point1 point2 point3]; % putting points in a vector
end

function rotated_point = rotate_point(point, angle, centroid)
    R = [cos(angle) -sin(angle);    % defining rotation matrix
         sin(angle) cos(angle)];

    point = [point(1); point(2)] - centroid;    % moving point to the origin
    rotated_point = R * point + centroid;       % rotating point and moving it back to its original position
end

function [xunit, yunit] = circle(x,y,r)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
end

function output = mapfun(value,fromLow,fromHigh,toLow,toHigh)
    narginchk(5,5)
    nargoutchk(0,1)
    output = (value - fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow) + toLow;
end
