%% Microrobot Swarm Aggregation using Medial Axes for Path Planning
% ========================================================================
% Author: Li Huang 
% Email:lihuang.mech@gmail.com
% ========================================================================


clear;
close all;
clc

% Specify vascular network maps
mapname = {'figT','figStdMap','figVB'};

% Specify map scaling
scale = {1, 0.5, 1};

% Goal locations for each map
goalloc = {[280,327],[331,333],[297,169]};

% Set distance threshold to distinguish end points from branch points and
% the range of branch point (within this range there is only 1 branch point)
dist_threshold = {[35 35],[35 35], [15 15]};

% Def. channel width
channel_width = {50, 30,12.5};

% Animation ON/OFF switch
animation_switch = 1;

% Process Display ON/OFF
proc_disp = 1;

% Choose an Algorithm
alg = 2;  % 1 for benchmark heuristics
          % 2 for divide-and-conquer

% Def number of robots
NumRob = 512; 

% Functions
funct = 2;
%1: map processing only
%2: Global control
%3: Map evaluation (under construction)
%4: Human control (joystick required)

% Def. variable "maps" with properties
maps = cell(3,1);

for ii = 1:3
    maps{ii} = struct('funct',funct,'Process_Display',proc_disp,'Algorithm',alg,...
        'Animation',animation_switch,'name', mapname(ii), 'goal_loc', goalloc(ii), ...
        'distance_threshold', dist_threshold(ii),'scaling', ...
        scale(ii),'channel_width',channel_width(ii),'NumRob',NumRob);
end

addpath(genpath(pwd));

% User input: choose a map
fprintf('Choose a map number from the list ...\n')
for ii = 1:size(mapname,2)
   fprintf('Map #%d: %s\n', ii, mapname{ii});
end

temp = input('Input the map name number: ');
testmap = strcat(maps{temp,1}.name,'.fig');

while ~exist(testmap)
    fprintf('File name does not exist!\n')
    fprintf('Choose a map number from the list ...\n')
    for ii = 1:size(mapname,2)
       fprintf('Map #%d: %s\n', ii, mapname{ii});
    end
    temp = input('Input the map name number: ');
    testmap = strcat(maps{temp,1}.name,'.fig');
end

fprintf('Map %s is chosen.\n', testmap);

% Call TestObj to process map and initialize swarm aggregation
obj = TestObj(maps{temp});


