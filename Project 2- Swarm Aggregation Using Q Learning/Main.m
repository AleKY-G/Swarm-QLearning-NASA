%% Microrobot Swarm Aggregation using Q Learning
% ========================================================================
% Author: Li Huang 
% Email:lihuang.mech@gmail.com
% ========================================================================

close all
clear
clc

time_rec = datetime

cnt = 0;

%% Animation setting
pause_timer1 = 0.4;         % pause time for each iteration

pause_timer2 = 0.001;       % pause time for each motion

process_display = 1;        % Display ON/OFF


%% Map, robot states and reward matrix etc. initialization
NumRob = 6;                 % Number of robots

disp('Initialization...')
run('Init.m')               

%% Algorithm Initialization
% Let Q(s,a) be the expected discounted reinforcement learning of taking
% action a in state s: 
%       Q(s,a) = Q(s,a) + learnRate*(bonus + discount*max( Q(s_new,a_new), [for all a_new @ s_new] ) -Q(s,a) )

learnRate = 0.75;           % Learning rate

epsilon = 0.5;              % Exploration ratio, so (1-epsilon) is exploitation ratio

epsilonDecay = 1;           % Learning rate decay ratio

discount = 0.8;             % Discount ratio

successRate = 1;            

winBonus = 100;             % Bonus for reaching the final goal 

maxIter = 200000;           % Max iterations   

maxStep = 300;              % Max steps in each iteration

actions = [1 2 3 4];        % Action set

R = sum(R_FreeSpace(states),2);     % Rewards matrix for each state

Q = repmat(R,[1 length(actions)]);  % Discounted reinforcement matrix for each action in each state


%% Robot Location Initialization
% RobLoc = FreeSpace(randi(length(FreeSpace),NumRob,1));
RobLoc = [68; 14; 53; 87; 110; 58];
    % [59; 14; 17; 110; 90; 116];
    % [20; 14; 17; 113; 110; 116]; 
    
% Robot row and col info
LocCol = floor(RobLoc./size(Map,1))+1;
LocRow = mod(RobLoc,size(Map,1));
LocRow(LocRow==0) = size(Map,1);

% Initial state
z0 = RobLoc';

%% Display Initialization
% Robot and map display
fig = figure; hold on
fig.Position = [250 100 800 600];
axis([-1 size(Map,2)+1,-1 size(Map,1)+1])
[row, col] = find(Map>0);
hSpace = scatter(col,row,1000,'filled');
hSpace.CData = [1 1 0];
hGoal = scatter(goal(2),goal(1),1000,'filled');
hGoal.CData = [0 1 0];
hRob = scatter(LocCol,LocRow,200,'filled');
Loc = [LocCol,LocRow];
fp1 = plot(0,0,'b','LineWidth',5);
fp2 = plot(0,0,'*b','MarkerSize',25);
plot(0.001,0,'.k','MarkerSize',30,'MarkerFaceColor',[1 0 0]);
fig.Position = [50 50 700 700];

%% Trainning
for iIter = 1:maxIter
    
    % Fixed robot initial locations
    RobLoc = [68; 14; 53; 87; 110; 58];
    % [59; 14; 17; 110; 90; 116];
    % [20; 14; 17; 113; 110; 116]; 
    
    % Sort location order to reduce state matrix dimension
    RobLoc = sort(RobLoc);
    
    % Compute robot coordinates
    LocCol = floor(RobLoc./size(Map,1))+1;
    LocRow = mod(RobLoc,size(Map,1));
    LocRow(LocRow==0) = size(Map,1);

    z0 = RobLoc';

    Loc = [LocCol,LocRow];      % Robot coordinates in X-Y plane

    % Display ON/OFF
    if process_display == 1
        hRob.XData = Loc(:,1);
        hRob.YData = Loc(:,2);
        pause(pause_timer1);
    end

    % Robot state vector
    z1 = z0;
    z1 = FreeSpaceIdx(z1)';

    % Use binning for fast state index search
    [~, bnewIdx] = min(sum( (repmat(z1(1:3).*[1e4,1e2,1e0],...
                   [states_Idx_len,1])-states_bin_Idx(:,1:3).*repmat([1e4,1e2,1e0],[states_Idx_len,1])).^2,2));
    UpBd = min(binSize*(bnewIdx+1),states_len);
    LwBd = max(binSize*(bnewIdx-2)+1,1);
    [~, snewIdx0] = min(sum( (repmat(z1,[UpBd-LwBd+1,1])-double(states(LwBd:UpBd,1:NumRob))).^2,2));
    snewIdx = snewIdx0 + LwBd-1;        % Update the index of the current state

    
    if process_display ==1
        fig2 = figure(2);
        fp3 = plot(0,-R(snewIdx),'-k','LineWidth',2);
        fig2.Position = [800 200 600 400];
        xlabel('Number of Steps')
        ylabel('Total cost to the goal')
        axis([0 320 0 2200])
    end

    for iStep = 1:maxStep
        
        % Use binning for fast state index search
       [~, bIdx] = min(sum( (repmat(z1(1:3).*[1e4,1e2,1e0],...
                   [states_Idx_len,1])-states_bin_Idx(:,1:3).*repmat([1e4,1e2,1e0],[states_Idx_len,1])).^2,2));
        UpBd = min(binSize*(bIdx+1),states_len);
        LwBd = max(binSize*(bIdx-2)+1,1);
       [~, sIdx0] = min(sum( (repmat(z1,[UpBd-LwBd+1,1])-double(states(LwBd:UpBd,1:NumRob))).^2,2));
        sIdx = sIdx0 + LwBd-1;      % Update the index of the current state

       % Exploration or exploitation
       if (rand()>epsilon || iStep == maxIter) && rand()<=successRate 
           [~,aIdx] = max(Q(sIdx,:));       % Exploitation: pick the action maximize the reward
       else
           aIdx = randi(length(actions),1); % Exploration
       end

       T = actions(aIdx);                   % Translate action number to moving direction

       Last_Loc = Loc;                      % Save the previous robot locations

       % Update robot locations
       Loc = Loc + repmat(( round([sin(T*pi/2) cos(T*pi/2)])),[length(RobLoc),1]);

       % Collision Check
       collision_idx =find(diag(Map(Loc(:,2),Loc(:,1)))==0);
       Loc(collision_idx,:) = Last_Loc(collision_idx,:);

       if process_display ==1
           set(fp1,'XData',[0 sin(T*pi/2)]);
           set(fp1,'YData',[0 cos(T*pi/2)]);
           set(fp2,'XData',sin(T*pi/2));
           set(fp2,'YData',cos(T*pi/2));
           set(fp3,'YData',[fp3.YData,-R(snewIdx)]);
           set(fp3,'XData',[fp3.XData,iStep]);
           hRob.XData = Loc(:,1);
           hRob.YData = Loc(:,2);
           pause(pause_timer2)
       end

       % Update State
       z1 = (Loc(:,1)-1)'*size(Map,1)+Loc(:,2)';

       [z1, z1_idx] = sort(z1);
       
       Loc = Loc(z1_idx',:);

       z1 = FreeSpaceIdx(z1)';
       
      % Use binning for fast state index search
       [~, bnewIdx] = min(sum( (repmat(z1(1:3).*[1e4,1e2,1e0],...
                   [states_Idx_len,1])-states_bin_Idx(:,1:3).*repmat([1e4,1e2,1e0],[states_Idx_len,1])).^2,2));
        UpBd = min(binSize*(bnewIdx+1),states_len);
        LwBd = max(binSize*(bnewIdx-2)+1,1);
       [~, snewIdx0] = min(sum( (repmat(z1,[UpBd-LwBd+1,1])-double(states(LwBd:UpBd,1:NumRob))).^2,2));
        snewIdx = snewIdx0 + LwBd-1;         % Update the index of the current state

      % Do all robots reach the goal? If not, continue
       if R(snewIdx) == 0
           success = true;
           bonus = winBonus;
       else
           bonus = -1;
           success = false;
       end

       % Update reinforcemant matrix Q
       if iIter < maxIter
           Q(sIdx,aIdx) = Q(sIdx,aIdx) + learnRate * ( R(snewIdx) ...
               + discount*max(Q(snewIdx,:)) - Q(sIdx,aIdx) + bonus );
       end


       if success
           break;
       end


    end
    
    % Decay the ratio of exploration each iteration if epsilonDecay<1
    epsilon = epsilon*epsilonDecay;

    if mod(iIter,500)==1
       disp('Iteration index updates every 500 times')
       disp('Iteration = ')
       disp(iIter)
       
   end

end


   time_rec(2) = datetime
