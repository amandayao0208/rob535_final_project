%%% test_script
clc
clear
load('TestTrack.mat')
percent_complete = zeros(100,1);
Xobs_save = {};
counter = 1;
for i = 1:100
    [Y,U,t_total,t_update, Xobs, T] = forwardIntegrate();
    info = getTrajectoryInfo(Y,U,Xobs,T,TestTrack);
    percent_complete(i) = info.percent_of_track_completed;
    if info.percent_of_track_completed < 1
        Xobs_save{counter} = Xobs;
        iteration(counter) = i;
        disp('iteration: ')
        i
        info
        counter = counter + 1;
    end
    i
end


