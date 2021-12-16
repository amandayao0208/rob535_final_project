%%% Test script for a specific set of obstacles
% for testing certain cases, modify Xobs in the forwardIntegrate function.
[Y,U,t_total,t_update, Xobs, T] = forwardIntegrate();
info = getTrajectoryInfo(Y,U,Xobs,T,TestTrack);