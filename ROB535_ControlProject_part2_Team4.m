function [sol_2, FLAG_terminate] = ROB535_ControlProject_part2_Team4(TestTrack,Xobs_seen,curr_state)
    %% defining tracks
    center = [TestTrack.cline, [1480;840]];   % defining right bonudary of track
    right = [TestTrack.br, [1490;840]];       % defining right boundary of track
    left = [TestTrack.bl, [1470;840]];        % defining left boundary of track
    theta = TestTrack.theta ;    % defining heading angle of track
    theta = [theta theta(end)];
    
    % creating new struct to not modify TestTrack
    ref_track.cline = center;
    ref_track.br = right;
    ref_track.bl = left;
    ref_track.theta = theta;
    
    %% Initializing variables for the model
    prev_state = curr_state;
    dt = 0.01;              % time step as specified in document--I still don't understand this for how we are supposed to turn things in though
    velocity_err_hist = 0;  % keeps track of the velocity error history in case we want to use an integral term for the longitudinal controller
    T = 0:dt:0.5;           % timespan for the forwardIntegrateControlInput.m function for ode45
    
    %% Running the model
    % continues until the closest index is the final index of the centerline
    
    for i = 1:length(T)    
        % avoid obstacles stuff
        ref_track.cline = center;
        [lane, ~] = avoidObs(ref_track, curr_state, Xobs_seen);
        ref_track.cline = lane;
    
        % computing values
        [Fx, velocity_err_hist] = longitudinal_controller(curr_state, velocity_err_hist, dt);   % returns a value for Fx--this function controls the speed using PID (currently just P)
        [delta, FLAG_terminate] = stanleyController(ref_track, curr_state);    % returns value for steering angle using the Stanley controller
        U = [delta Fx; delta Fx];   % creating matrix of inputs to send through the forwardIntegrateControlInput.m function
        [output_state, ~] = forwardIntegrateControlInput(U, prev_state');   % integrates to get us our next state
        curr_state = output_state(end,:)';  % updates the current state vector
    
        % storing values
        sol_2(i, :) = [delta Fx];  % stores our inputs in a vector
        prev_state = curr_state;    % sets the previous state equal to the current state since we are ending the loop
    end
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
    MIN_DES_VEL = 5;           % maximum desired velocity (determined by trial and error--we can definitely change this)
    MAX_DES_VEL = 20;           % minimum desired velocity " "
    MIN_HEAD_RATE = 0;          % minimum heading rate (used for slowing down the car as it gets bigger)
    MAX_HEAD_RATE = 0.3;        % maximum heading rate " "
    curr_heading_rate = abs(curr_state(6)); % defining the current heading rate (r)
    desired_vel = mapfun(curr_heading_rate, MIN_HEAD_RATE, MAX_HEAD_RATE, MAX_DES_VEL, MIN_DES_VEL);   % determining a desired velocity between MIN_DES_VEL and MAX_DES_VEL using the mapping function (like it's used in Arduino code)

    prev_error = velocity_err_hist(end);    % computing the previous velocity error
    curr_error = desired_vel - curr_vel;   % computing the current velocity error
    velocity_err_hist = [velocity_err_hist curr_error]; % appending the new velocity error histor value to the vector

    D_err = (curr_error - prev_error) / dt; % error rate to be multiplied by kd gain
    I_err = sum(velocity_err_hist); % error sum to be multiplied by ki gain
    
    throttle = kp * curr_error + ki * I_err + kd * D_err;   % computing the value for the throttle
    throttle = clamp(throttle, -5000, 5000);                % clamping Fx to its bounds
end

function [delta, FLAG_terminate] = stanleyController(ref_track, curr_state)
    % This function controls the lateral motion of the car. It takes in the
    % low-passed heading values of the track (even though it makes
    % performance worse for this controller, I kept it in since we can set
    % that to false). The function als takes in the reference track,
    % current state, and the low pass flag. The output is the steering
    % angle using the stanley controller.

    a = 1.35;   % distance from the center of gravity of the car to the front steering axle
    ke = 2.5;     % a proportional gain for how fast we want to turn towards the desired target lane
    kv = 1;     % a gain to make sure the denominator isn't zero
    X = curr_state(1);    % current x position of car
    u = curr_state(2);    % current forward velocity of car
    Y = curr_state(3);    % current y position of car
    psi = curr_state(5);  % current heading of car
    
    % calculating heading error
    curr_pos = [X + a * cos(psi); Y + a * sin(psi)];    % current position of the front axis of the vehicle--the stanley controller depends on this assumption
    [desired_heading, cross_track_error, ~, FLAG_terminate] = check_pos(ref_track, curr_state, curr_pos);    % getting the desired heading and the crosstrack error
    psi_e = desired_heading - psi;  % computing the heading error

    % control law
    des_steer = psi_e + atan2(ke * cross_track_error, (kv + u));    % computing the desired steering angle
    des_steer = wrap(des_steer);    % wrapping the desired steering angle to [-pi, pi] just in case
    des_steer = clamp(des_steer, -0.5, 0.5);    % clamping the desired steering angle to be within its bounds

    % update delta
    delta = des_steer;  % setting delta equal to the desired steering angle (this isn't really necessary, but it makes it easier to know what the function output is)
end

function [desired_heading, cross_track_error, side, FLAG_terminate] = check_pos(ref_track, curr_state, curr_pos)
    % This function takes in a number of inputs shown above, and returns
    % the desired ehading, crosstrack error using the formula for a scalene
    % triangle, as well as the FLAG_terminate flag to determine if we have
    % finished the track.

    % initial definitions for readability
    center = ref_track.cline;   % defining centerline as new variable for readability
    theta = ref_track.theta;    % track heading equals original values
    
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

function output = mapfun(value,fromLow,fromHigh,toLow,toHigh)
    narginchk(5,5)
    nargoutchk(0,1)
    output = (value - fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow) + toLow;
end

function [lane, lane_side] = avoidObs(ref_track, curr_state, Xobs_seen)
    curr_state = curr_state';   % transposing current state into a 
    cenpts = ref_track.cline;   % define centerpoints
    right = ref_track.br;       % define right boundary
    left = ref_track.bl;        % define left boundary
    lane = cenpts;
    lane_side = "center";


    % construct centerlines for left and right lane
    c_right = (right + cenpts)/2;
    c_left = (left+cenpts)/2;
    curr_pt = [curr_state(1), curr_state(3)];
    
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
        for i=1:size(Obs)
           obx = Obs(i,1); % x location of obs
           oby = Obs(i,2); % y location
           hx = curr_pt(1) + 10*cos(heading);
           hy = curr_pt(2) + 10*sin(heading);
           l_1 = [hx,hy,0] - [curr_pt(1),curr_pt(2),0];
           l_2 = [obx,oby,0] - [curr_pt(1),curr_pt(2),0];
           theta = atan2(norm(cross(l_1,l_2)), dot(l_1,l_2));

           dist = norm(curr_pt - [obx,oby]);
           obs_vec = [dist,obx,oby,hx,hy,theta];
            %status = [h_lb, ' < ', o_ang, ' < ', h_ub];
           if (theta > (deg2rad(-90)) && theta < (deg2rad(90)))
              front_obs = [front_obs;obs_vec]; 
           end
        end
    end
    % final front_obs vector: [distance, x,y, x_d, y_d, alpha] 
    % find closest obstacle
    if(front_obs)
        %disp(status);
        [~,I] = min(front_obs(:,1));
        %grab global x,y of next closest obstacle
        nxt_obs = front_obs(I,2:3);
        %WANT TO: calculate which side obstacle is on (L,R), then tell car to
        %go to follow the lane oposite to that side.
        [cen_idx ,~] = knnsearch(cenpts',nxt_obs); % find nearest centerpt indx
        nxt_obs(3) = front_obs(I,3);
        which_side = whichwayoffcenter(cen_idx, cenpts', nxt_obs);
        
        if (which_side(3) >= 0) % obstacle closer to left side of track
            % set new goal path to right lane
            lane = c_right';
            lane_side = "right";
        else % else obstacle closer to right side
            % set new goal path to left lane
            lane = c_left';
            lane_side = "left";
        end
    end
    if size(lane, 1) ~= 2
        lane = lane';
    end
end
