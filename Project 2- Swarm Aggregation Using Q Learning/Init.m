load('map1.txt')
Map = map1;

ValueMap = Map;                                     % 'ValueMap' is a cost-to-go value map

NodeCnt = 1;                                        % Waypoint counter (i.e. node counter) 

% goal = [4,4];                                     % Goal location for mapT
goal = [11,10];                                     % Goal location for map1

ValueMap(goal(1),goal(2))=100;                      % Def the minimum cost to goal
                                                    % Note this is not zero because in "ValueMap", 
                                                    % "0" indicates an obstacle                                                   

frtr = goal;                                        % Breadth-first search (BFS) frontier
 
%% Breadth-first first (BFS) search of all accessible paths to goal, 
%% and assign cost-to-go to each waypoint

while ~isempty(frtr)    
    % Extract a 2x2 neighbor (in 'ValueMap') of the current frontier waypoint
    for ii = 1:4
        if ValueMap(round(cos(ii*pi/2))+frtr(1,1),round(sin(ii*pi/2))+frtr(1,2))==1
            ValueMap(round(cos(ii*pi/2))+frtr(1,1),round(sin(ii*pi/2))+frtr(1,2))=ValueMap(frtr(1,1),frtr(1,2))+1;
            frtr = [frtr; round(cos(ii*pi/2))+frtr(1,1),round(sin(ii*pi/2))+frtr(1,2)];            
        end
    end
    
    % Remove the current waypoint is from the frontier set
    frtr(1,:) = []; 
end

% Make goal location cost = 1, and the cost of all other waypoint increases by distance. 
tmp = ValueMap>0;
ValueMap(tmp) = ValueMap(tmp)-99;

% Find the index of each waypoint in freespace
FreeSpace = (find(Map>0));
FreeSpaceIdx = (zeros(max(FreeSpace),1));

for ii = 1:numel(FreeSpace)
   FreeSpaceIdx(FreeSpace(ii)) = ii; 
end

% Initialization reward of each waypoint
R_FreeSpace = zeros(numel(FreeSpace),1);

% Assign the reward of each waypoint as the squared distance to the goal
for ii = 1:numel(FreeSpace)
    tmp1 = floor(FreeSpace(ii,1)./size(Map,1))+1;
    tmp2 = FreeSpace(ii,1)-floor(FreeSpace(ii,1)./size(Map,1))*size(Map,1);
    R_FreeSpace(FreeSpaceIdx(FreeSpace(ii))) = -(ValueMap(tmp2,tmp1)-1)^2 ;
end

% Initialize states
index = 1;

states = uint8(zeros(1,NumRob));

% The size "states" is changing--> increase the row size of the matrix by 2
% when it exceeds the max capacity 
states_cap = 1;

%% States Initialization
stage = 1;

loop_idx = 1;

loop_mat = ones(2,NumRob);

[states,index,states_cap] = forloopfunction(loop_mat,states,numel(FreeSpace),NumRob,FreeSpaceIdx,FreeSpace,loop_idx,stage,index,states_cap);

states = states(1:index-1,:);  

states_len = length(states);

%% Use binning for fast states search
binSize = 0.25e4;

states_bin_Idx = zeros(floor(states_len/binSize)+1,NumRob);

states_Idx_len = size(states_bin_Idx,1);

jj = 1;

for ii = 1:states_len
    if mod(ii,binSize)==1
        states_bin_Idx(jj,:) = states(ii,1:NumRob);
        jj= jj +1;
    end 
end
