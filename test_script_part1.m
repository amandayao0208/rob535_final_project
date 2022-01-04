%%% test script for part 1
clc
clear
load("ROB535_ControlProject_part1_Team4.mat");
load("TestTrack.mat")
T = 0:0.01:0.5;
sol_1 = forwardIntegrateControlInput(ROB535_ControlProject_part1_input);
info = getTrajectoryInfo(sol_1, ROB535_ControlProject_part1_input,[], T, TestTrack)