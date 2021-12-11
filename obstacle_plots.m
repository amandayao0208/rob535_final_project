%%% testing obstacles
clc
clear all
Nobs = 10;
load TestTrack.mat

Xobs = generateRandomObstacles(Nobs, TestTrack);
left = TestTrack.bl;
right = TestTrack.br;
center = TestTrack.cline;

hold on
plot(center(1,:), center(2,:),'r--')
plot(left(1,:), left(2,:), 'k-')
plot(right(1,:), right(2,:), 'k-')
draw_obs(Xobs)


function draw_obs(Xobs)
    for i = 1:length(Xobs)
        points = Xobs{i};
        x_vals = [points(:,1); points(1,1)];
        y_vals = [points(:,2); points(1,2)];

        plot(x_vals, y_vals, 'b')
    end
end