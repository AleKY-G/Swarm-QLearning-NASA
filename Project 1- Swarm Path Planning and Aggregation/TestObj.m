%% Microrobot Swarm Aggregation using Medial Axes for Path Planning
% ========================================================================
% Author: Li Huang 
% Email:lihuang.mech@gmail.com
%
% Comments:
% Run "BenchmarkMaptest.m" first
% This program intializes map and simulate swarm aggregation
% ========================================================================
classdef TestObj < handle
    
   properties (SetAccess = private)
      funct
      PerformanceEval
      animation
      Process_Display
      alg
      NumRob
      Ed
      Ed0
      EdPts
      EdPts0
      psdEdPts
      Brch
      Brch0
      BrchPts
      BrchPts0
      BEdist
      BBdist
      fig
      BW
      channel_width
      EBW
      Skel
      goal
      ConnMat
      Pathway
      Path
      track
      RegionID
      RegionVal
      trackL
      rtrack
      localSkel
      localPathway
      tmpPathway
      localPath
      tar
      tar_brch
      tar_brch_ID
      local_track
      distbt
      localmpath
      loc
      freespace
      localfsp
      medial
      basescore
      Expt
      row0
      col0
      rank0
   end
    
   methods
       %% class constructor
       function this = TestObj(map)
           
           %% Load Map
           tic
          
           testmap = strcat(map.name,'.fig');
           open(testmap);
           this.fig = getframe;

           
           %% Initialize map parameters
           this.channel_width = map.channel_width;              % Def channel width
           bw = imbinarize(this.fig.cdata(:,:,1),0.9);          % RGB --> binary image
           this.BW = imresize(bw,map.scaling);                  % Resize the image for smooth path
           this.goal = map.goal_loc;                            % Goal location
           this.animation = map.Animation;                      % Animation ON/OFF
           this.alg = map.Algorithm;                            % Choose algorithm
           this.NumRob = map.NumRob;                            % Number of robots
           this.Process_Display = map.Process_Display;          % Process display ON/OFF
           this.funct = map.funct;
           if this.Process_Display ==0
                close (figure(1))  
           end
           %% Extend the boundaries of the binary image
           [l10,l20] = size(this.BW);
           l1 = uint16(l10*1.2);
           l2 = uint16(l20*1.2);           
           BW0 = uint16(zeros(l1,l2));
           BW0(uint16((l1-l10)/2):uint16((l1-l10)/2)-1+l10,uint16((l2-l20)/2):l20-1+uint16((l2-l20)/2))=this.BW;
           this.BW = logical(BW0);
           
           %% Image processing: skeletonization 
           this.EBW = ~edge(this.BW,'canny');                   % Extract edges
           
           this.Skel = bwmorph(this.BW,'skel',Inf);             % Obtain medial-axis skeleton of the binary image, targets are marked as 1, o.w. 0
          
           this.Brch = bwmorph(this.Skel,'branchpoints');       % Obtain branch map in the medial-axis map, targets are marked as 1, o.w. 0 
           [row, col] = find(this.Brch);                     
           this.BrchPts = [row, col];                           % Extract branch point coordinates
           
           this.Ed = bwmorph(this.Skel,'endpoints');            % Obtain end-point map in the medial-axis map, targets are marked as 1, o.w. 0
           [row, col] = find(this.Ed);
           this.EdPts = [row, col];                             % Extract end point coordinates
                     
           this.BEdist = map.distance_threshold(1);             % Branch_Point-to-End_point distance threshold
           this.BBdist = map.distance_threshold(2);             % Branch_Point-to-Branch_point distance threshold    
           
           %% Visualize the map
           if this.Process_Display ==1
               figure
               disp(' ')
               disp('Map preprocessing: skeletonization by MATLAB')
               imshow(this.Skel);
               hold on
               f1 = scatter(this.BrchPts(:,2),this.BrchPts(:,1),20,'filled');
               f1.CData = [1 0 0];
               f2 = scatter(this.EdPts(:,2),this.EdPts(:,1),20,'filled');
               f2.CData = [0 0 1];
           end
           
           %% Part I: Customized Map Preprocessing
           MapProcess1(this);
           MapProcess2(this);
           MapProcess3(this);
           MapProcess4(this);
           GlobalSetup(this);
           disp(' ')
           disp('Map Process Time:')
           toc
           %% Part II: Global Aggregation
           switch(this.funct)
               case 1
                    disp('Map processing mode...')
               case 2
                    disp('Automatic simulation mode...')                    

                    GlobalControl(this);
               case 3
                    disp('Map evaluation mode...')
                    MapEvaluation(this);
               case 4
                    disp('Human control mode...')
                    HumanControl(this);
           end
          


% %            EdPts0tmp = this.EdPts0;
% %            for jj = 1:length(EdPts0tmp)
% %                this.col0 = jj;
% %                this.goal = EdPts0tmp(jj,1:2);
% %                MapProcess1(this);
% %                MapProcess2(this);
% %                MapProcess3(this);
% %                MapProcess4(this);
% %                GlobalSetup(this);
% %                for kk = 1:10
% %                     this.row0 = kk+2; 
% %                     GlobalControl(this);
% %                end
% %            end

       end
       
       %% Initilize Cost Function 
       % Assign cost (from the goal)to nodes
       function MapProcess1(this)   
            this.Pathway = uint16(this.Skel);                   % 'this.Pathway' is a cost-to-go value map
             
            this.Path = zeros(sum(this.Skel(:)),6);             % Initialize waypoint properties
            
            NodeCnt = 1;                                        % Waypoint counter (i.e. node counter) 
            
            this.Pathway(this.goal(1),this.goal(2))=1e2;        % Def the minimum cost to goal
                                                                % Note this is not zero because in "this.Pathway", 
                                                                % because initially "0" means it's off medial axes                                                   
                                                               
            frtr = this.goal;                                   % Breadth-first search (BFS) frontier
            this.psdEdPts = [];                                 % Def psedo end points
            
            % Update waypoint list
            this.Path(NodeCnt,1:2) = this.goal;                 % Def goal waypoint coordinates
            this.Path(NodeCnt,4) = 1e2;                         % Def the cost from the current waypoint to goal    
            
            %% Breadth-first first (BFS) search of all accessible (medial-axis) paths to goal
            while ~isempty(frtr)    
                % Extract a 3x3 neighbor (in 'this.Pathway') of the current frontier waypoint 
                nb = this.Pathway(-1+frtr(1,1):1+frtr(1,1),-1+frtr(1,2):1+frtr(1,2));     
                
                % If a medial-axis waypoint has been explored, it cost-to-go is > 100
                % otherwise, it's cost-to-go is 1.
                [row, col] = find(nb==1);      % Search for unexplored neighbor waypoints in medial axes
                row = row-2;
                col = col-2;
                
                if numel(row)>0 % if there exists unexplored neighbor
                    % Assign cost value to the new frontier
                    this.Pathway(-1+frtr(1,1):1+frtr(1,1),-1+frtr(1,2):1+frtr(1,2))=...
                        -this.Pathway(-1+frtr(1,1):1+frtr(1,1),-1+frtr(1,2):1+frtr(1,2)).*uint16(blkdiag(0,1,0))+...
                        uint16(nb ==1).*this.Pathway(frtr(1,1),frtr(1,2))...
                        +this.Pathway(-1+frtr(1,1):1+frtr(1,1),-1+frtr(1,2):1+frtr(1,2));
                    
                    % Update the id of the current waypoint's predecessor 
                    ParentID = find(this.Path(1:NodeCnt,1)==frtr(1,1) & this.Path(1:NodeCnt,2)==frtr(1,2));
                    
                    % Add new frontier waypoints to the frontier set
                    for ij = 1:numel(row)       
                        frtr = [frtr; row(ij)+frtr(1,1), col(ij)+frtr(1,2)];
                        NodeCnt = NodeCnt +1;                           % Update total number of waypoints
                        
                        % Update waypoint list
                        this.Path(NodeCnt,1:2) = frtr(end,1:2);                         % Current waypoint coordinates        
                        this.Path(NodeCnt,3) = ParentID;                                % Current waypoint's predecessor's id 
                        this.Path(NodeCnt,4) = this.Pathway(frtr(1,1),frtr(1,2))+1;     % Current waypoint cost-to-go
                    end
                    
                else
                    % if all neighbors are explored
                    % And if this frontier waypoint is not a end point, then 
                    % add it to the set of psedo end point. 
                    
                    if this.Ed(frtr(1,1),frtr(1,2))==0
                        this.psdEdPts = [this.psdEdPts;frtr(1,1:2)]; 
                       % "this.psdEdPts" stands for pseudo end pts, which is not an
                       % actual end pts, but a waypoint where you can get to the
                       % goal with more than one path at the same cost
                    end
                end
                
                % Remove the current waypoint is from the frontier set
                frtr(1,:) = []; 
            end
            
            
            if ~isempty(this.psdEdPts)
                % Among these psedo end points, there are two kinds of
                % waypoints. The first kind belongs to
                
                % Extract a sub-matrix from the branch-point map (branch points are marked as "1")
                % The diagonal elements of this sub-matrix are the
                % corresponding psedo end points. 
                temp = this.Brch(this.psdEdPts(:,1),this.psdEdPts(:,2));
                
                % Find the diagonal elements that are not branch points,
                % since the exact psedo end points cannot be branch points.
                this.psdEdPts = this.psdEdPts(diag(temp)==0,:);
            end
            
       end
          
       %% Eliminate Fake End Points
       % Note that some fake end points are located at a corner of a channel
       function MapProcess2(this)
             % Make sure the points with the largest cost-to-go are end points 
             [row, col] = find(this.Pathway==max(this.Pathway(:)));
              for jj = 1:numel(row)
                    if this.Ed(row(jj),col(jj))==0
                       this.Ed(row(jj),col(jj))=1;
                    end
              end
              
              % Now copy the end point and branch point information to two
              % new variables
              this.Ed0 = this.Ed;
              this.Brch0 = this.Brch;
              
              % Make sure the goal is an end point AND a branch points
              this.Ed0(this.goal(1),this.goal(2)) = 1;
              this.Brch0(this.goal(1),this.goal(2)) = 1;

              % Extract end point coordinates 
              [row, col] = find(this.Ed0==1);
              this.EdPts0 = [row, col];

              % Extract branch point coodinates 
              [row, col] = find(this.Brch0==1);
              this.BrchPts0 = [row, col];
                
              % This for-loop says that if the distance (cost) of a branch
              % point and the nearest end point is less than the threshold (this.BEdist),
              % then this branch point is regarded as an end pts instead of a
              % branch point. 
              % This happens because MATLAB skeletonization does not end at the medial axis. 
              for ii = 1:length(this.EdPts)
                    temp0 = this.EdPts(ii,1:2);                 % Pick an end point
                    temp = temp0;                               % Intermediate point for BFS
                    cost0 = this.Pathway(temp0(1),temp0(2));    % The cost of the current waypoint
                    cost = cost0;
                    temp_buffer = temp;                         % "temp_buffer" saves all "temp" values
                    
                    while (cost0-cost < this.BEdist) && (cost > 100) 
                        
                        % Extract the neighbors around the intermediate point
                        % from the cost-to-go map 'this.Pathway'
                        nb = this.Pathway(-1+temp(1):1+temp(1),-1+temp(2):1+temp(2));
                        nb(2,2)=0;
                        
                        % Extract the neighbors around the intermediate point
                        % from the branch point map 'this.Brch'
                        nb0 = this.Brch(-1+temp(1):1+temp(1),-1+temp(2):1+temp(2));
                        nb0(2,2)=0;
                        
                        % Search for branch points
                        if ~isempty(find(nb0==1,1))
                            [row, col] = find(nb0==1);
                            row = row-2;
                            col = col-2;
                            if this.Pathway(row(1)+temp(1),col(1)+temp(2)) > cost
                               [row, col] = find(nb==cost-1);
                               row = row-2;
                               col = col-2;
                               temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point
                               temp_buffer = [temp_buffer;temp];
                            else
                               % The branch point is within the threshold distance, and it
                               % will be regarded as an end point.    
                               temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate pt
                               this.Brch0(temp(1),temp(2))=0;
                               this.Ed0(temp(1),temp(2))=1;
                               this.Ed0(temp0(1),temp0(2))=0;
                               for i_tempRcd = 1:size(temp_buffer,1)
                                    this.Skel(temp_buffer(i_tempRcd,1),temp_buffer(i_tempRcd,2)) = 0;
                                    this.Pathway(temp_buffer(i_tempRcd,1),temp_buffer(i_tempRcd,2)) = 0;
                               end
                                Idx = knnsearch(this.Path(:,1:2),temp_buffer);
                                this.Path(Idx,:) = NaN;
                               break;
                            end
                        else
                            [row, col] = find(nb==cost-1);
                            row = row-2;
                            col = col-2;
                            temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point
                            temp_buffer = [temp_buffer;temp];
                        end
                        
                        cost = this.Pathway(temp(1),temp(2));       % Intermediate point cost
                        
                   end
                    
                   
              end
       
              % Update end point list
              [row, col] = find(this.Ed0);
              this.EdPts0 = [row, col];
           
              % This for-loop eliminates any possible end point b/w one end
              % point and the goal
              for ii = 1:size(this.EdPts0,1)
                    temp = this.EdPts0(ii,:);               % Obtain coordinates of this end point                   
                    
                    if this.Ed0(temp(1),temp(2))==0         % Is this an end point? We are updating end point info
                        continue
                    end

                    cost = this.Pathway(temp(1),temp(2));
                    while cost > 100    
                        % Extract the neighbors around the intermediate point
                        % from the cost-to-go map 'this.Pathway'
                        nb = this.Pathway(-1+temp(1):1+temp(1),-1+temp(2):1+temp(2));
                        nb(2,2)=0;
                        
                        % Extract the neighbors around the intermediate point
                        % from the branch point map 'this.Brch'
                        nb0 = this.Ed0(-1+temp(1):1+temp(1),-1+temp(2):1+temp(2));
                        nb0(2,2)=0;
                        
                        % Search for branch points
                        if ~isempty(find(nb0==1,1))
                            [row, col] = find(nb0==1);
                            row = row-2;
                            col = col-2;
                            if this.Pathway(row(1)+temp(1),col(1)+temp(2)) > cost
                               [row, col] = find(nb==cost-1);
                               row = row-2;
                               col = col-2;
                               temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point

                            else
                               % This end point is b/w an end point and the goal, and it
                               % will NOT be regarded as an end point.    
                               temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point
                               this.Ed0(temp(1),temp(2))=0;
                            end
                        else
                            [row, col] = find(nb==cost-1);
                            row = row-2;
                            col = col-2;
                            temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point
                        end
                        
                        cost = this.Pathway(temp(1),temp(2));       % Intermediate point cost

                    end 
              end

            
            
       end
        
       %% Eliminate Fake Branch Points
       function MapProcess3(this)
                
              % This for-loop says that if the distance (cost) of two nearby branch
              % points is less than the threshold (this.BBdist),
              % then the farther branch pt will NOT be regarded as a branch
              % point
              for ii = 1:size(this.BrchPts0,1)
                    temp0 = this.BrchPts0(ii,1:2);                  % Pick an branch point
                    temp = temp0;                                   % Intermediate point for BFS search
                    cost0 = this.Pathway(temp0(1),temp0(2));        % The cost of this branch point
                    cost = cost0;
                    
                    while (cost0-cost < this.BBdist) && cost >100
                        
                        % Extract the neighbors around the intermediate point
                        % from the cost-to-go map 'this.Pathway'
                        nb = this.Pathway(-1+temp(1):1+temp(1),-1+temp(2):1+temp(2));
                        nb(2,2)=0;
                        
                        % Extract the neighbors around the intermediate point
                        % from the branch point map 'this.Brch'
                        nb0 = this.Brch0(-1+temp(1):1+temp(1),-1+temp(2):1+temp(2));
                        nb0(2,2)=0;
                        
                        % Search for branch points
                        if ~isempty(find(nb0==1,1))
                            [row, col] = find(nb0==1);
                            row = row-2;
                            col = col-2;
                            if this.Pathway(row(1)+temp(1),col(1)+temp(2)) > cost
                               [row, col] = find(nb==cost-1);
                               row = row-2;
                               col = col-2;
                               temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point

                            else
                               % The branch point is within the threshold distance, and it
                               % will be regarded as an end point instead of a branch point.    
                               temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point
                               this.Brch0(temp0(1),temp0(2))=0;

                            end
                        else
                            [row, col] = find(nb==cost-1);
                            row = row-2;
                            col = col-2;
                            temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point
                        end
                        
                        cost = this.Pathway(temp(1),temp(2));       % Intermediate the cost
                        
                   end
                    
                   
              end    
              
            this.Brch0(this.goal(1),this.goal(2))=1;
            [row, col] = find(this.Brch0);
            this.BrchPts0 = [row, col];
            this.Ed0(this.goal(1),this.goal(2)) = 1;
            [row, col] = find(this.Ed0);
            this.EdPts0 = [row, col];
            
            if this.Process_Display ==1
                figure
                disp(' ')
                disp('Skeleton map after customized processing')
                disp('red spots are branch points, blue spots are end pints, and smaller spots are psedo end points.')
                imshow(~(~this.BW+this.Skel)); hold on

                f1 = scatter(this.BrchPts0(:,2),this.BrchPts0(:,1),20,'filled');
                f1.CData = [1 0 0];
                f2 = scatter(this.EdPts0(:,2),this.EdPts0(:,1),20,'filled');
                f2.CData = [0 0 1];
                
                % Mark goal location
                hGoal = scatter(this.goal(2),this.goal(1),60,'filled');
                hGoal.CData = [1 0 0];
            
                if ~isempty(this.psdEdPts)
                    f3 = scatter(this.psdEdPts(:,2),this.psdEdPts(:,1),5,'filled');
                    f3.CData = [0,0,0.8];
                end
            end

       end
            
       %% Pair junction (branch points) with end points
       % In this section, each end poiny should find its nearest branch
       % point (junction)
       function MapProcess4(this)
           % Def connectivity matrix
           % The i-th row of the connectivity matrix is associated with the
           % i-th branch point in the branch point list (this.BrchPts0).
           this.ConnMat = zeros(size(this.BrchPts0,1),11);   
           
           % Sort the region by cost-to-go of the branch points
           % # of regions = # of branch points
           [this.RegionVal, this.RegionID] = sort(diag(this.Pathway(this.BrchPts0(:,1),this.BrchPts0(:,2))),'Descend');
           

           
           % Pair branch points w/ branch points        
            for ii = 1:numel(this.RegionID)
                temp1 = this.BrchPts0(this.RegionID(ii),1:2);    % Pick a branch point
                temp = temp1;                                    % Intermediate point for BFS search
                cost1 = this.RegionVal(ii);                      % The cost of the current waypoint
                cost = cost1;

                while cost >100

                    % Extract the neighbors around the intermediate point
                    % from the cost-to-go map 'this.Pathway'
                    nb = this.Pathway(-1+temp(1):1+temp(1),-1+temp(2):1+temp(2));
                    nb(2,2)=0;

                    % Extract the neighbors around the intermediate point
                    % from the branch point map 'this.Brch'
                    nb0 = this.Brch0(-1+temp(1):1+temp(1),-1+temp(2):1+temp(2));
                    nb0(2,2)=0;

                    % Search for branch points
                    if ~isempty(find(nb0==1,1))
                        [row, col] = find(nb0==1);
                        row = row-2;
                        col = col-2;
                        % If the branch point has higher cost than the intermediate point "temp", skip it
                        if this.Pathway(row(1)+temp(1),col(1)+temp(2)) > cost
                           [row, col] = find(nb==cost-1);
                           row = row-2;
                           col = col-2;
                           temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point

                        else
                           % If we reach a branch point, break the while loop
                           % Let this branch point be the new staring point.
                           temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point
                           
                           % Find the current branch point in the branch point list
                           ID = find(this.BrchPts0(:,1)==temp(1) & this.BrchPts0(:,2)==temp(2));
                           
                           % In the connectivity matrix, row i column 1 indicates
                           % how many branch points/end points are
                           % associated with the i-th branch point. That
                           % is, to reach the goal, these branch/end points
                           % need to reach the i-th branch point first.
                           this.ConnMat(ID,1) = this.ConnMat(ID,1)+1;
                           
                                                      
                           % In the connectivity matrix, row i column 2j:2j+1 indicates
                           % the coordinates of the j-th waypoint (branch or end) 
                           % associated with the i-th branch point  
                           this.ConnMat(ID,2*this.ConnMat(ID,1):2*this.ConnMat(ID,1)+1) = temp1;
                           
                           cost = this.Pathway(temp(1),temp(2));
                           break;
                        end
                    else
                        % No branch point found, keep looking for
                        % intermediate point of less cost: (cost - 1)
                        [row, col] = find(nb==cost-1);
                        row = row-2;
                        col = col-2;
                        temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point
                    end

                    cost = this.Pathway(temp(1),temp(2));       % Intermediate point cost

                end


            end
            
            % Sort end points by cost-to-go
            [EndVal, EndID] = sort(diag(this.Pathway(this.EdPts0(:,1),this.EdPts0(:,2))),'Descend');
            
            % Pair end points w/ branch points        
            for ii = 1:numel(EndID)
                temp1 = this.EdPts0(EndID(ii),1:2);         % Pick an end point
                temp = temp1;                               % Intermediate point for BFS search
                cost1 = EndVal(ii);                         % The cost-to-go of the current end point
                cost = cost1;

                while cost > 100

                    % Extract the neighbors around the intermediate point
                    % from the cost-to-go map 'this.Pathway'
                    nb = this.Pathway(-1+temp(1):1+temp(1),-1+temp(2):1+temp(2));
                    nb(2,2)=0;

                    % Extract the neighbors around the intermediate point
                    % from the branch point map 'this.Brch'
                    nb0 = this.Brch0(-1+temp(1):1+temp(1),-1+temp(2):1+temp(2));
                    nb0(2,2)=0;

                    % Search for branch points
                    if ~isempty(find(nb0==1,1))
                        [row, col] = find(nb0==1);
                        row = row-2;
                        col = col-2;
                        
                        % If the branch point has higher cost than the intermediate point "temp", skip it
                        if this.Pathway(row(1)+temp(1),col(1)+temp(2)) > cost
                           [row, col] = find(nb==cost-1);
                           row = row-2;
                           col = col-2;
                           temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point

                        else
                           % If we reach a branch point, break the while loop
                           % Let this branch point be the nbenw starting point. 
                           temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point
                           ID = find(this.BrchPts0(:,1)==temp(1) & this.BrchPts0(:,2)==temp(2));
                           this.ConnMat(ID,1) = this.ConnMat(ID,1)+1;
                           this.ConnMat(ID,2*this.ConnMat(ID,1):2*this.ConnMat(ID,1)+1) = temp1;
                           cost = this.Pathway(temp(1),temp(2));
                           break;
                        end
                    else
                        % No branch points found, keep looking for
                        % intermediate point of less cost: (cost - 1)
                        [row, col] = find(nb==cost-1);
                        row = row-2;
                        col = col-2;
                        temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point
                    end
                    cost = this.Pathway(temp(1),temp(2));       % Intermediate pt cost

                end


            end
            
            % If there exists pseudo end points (the points cannot be found by bwmorph)
            if ~isempty(this.psdEdPts)
                % Sort psdEdPts
                [EndVal, EndID] = sort(diag(this.Pathway(this.psdEdPts(:,1),this.psdEdPts(:,2))),'Descend');
            
                % Pair pseudo end points w/ branch points        
                for ii = 1:numel(EndID)
                    temp1 = this.psdEdPts(EndID(ii),1:2);       % Pick a pseudo end point
                    temp = temp1;                               % Intermediate pt for BFS search
                    cost1 = EndVal(ii);                         % The cost of the current waypoint
                    cost = cost1;

                    while cost > 100

                        % Extract the neighbors around the intermediate point
                        % from the cost-to-go map 'this.Pathway'
                        nb = this.Pathway(-1+temp(1):1+temp(1),-1+temp(2):1+temp(2));
                        nb(2,2)=0;

                        % Extract the neighbors around the intermediate point
                        % from the branch point map 'this.Brch'
                        nb0 = this.Brch0(-1+temp(1):1+temp(1),-1+temp(2):1+temp(2));
                        nb0(2,2)=0;

                        % Search for branch points
                        if ~isempty(find(nb0==1,1))
                            [row, col] = find(nb0==1);
                            row = row-2;
                            col = col-2;
                            % If the branch point has higher cost than the intermediate point "temp", skip it
                            if this.Pathway(row(1)+temp(1),col(1)+temp(2)) > cost
                               [row, col] = find(nb==cost-1);
                               row = row-2;
                               col = col-2;
                               temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point

                            else
                               % If we reach a branch point, break the while loop
                               % Let this branch point be the new starting point. 
                               temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point
                               ID = find(this.BrchPts0(:,1)==temp(1) & this.BrchPts0(:,2)==temp(2));
                               this.ConnMat(ID,1) = this.ConnMat(ID,1)+1;
                               this.ConnMat(ID,2*this.ConnMat(ID,1):2*this.ConnMat(ID,1)+1) = temp1;
                               cost = this.Pathway(temp(1),temp(2));
                               break;
                            end
                        else
                        % No branch points found, keep looking for
                        % intermediate point of less cost: (cost - 1)
                            [row, col] = find(nb==cost-1);
                            row = row-2;
                            col = col-2;
                            temp = [row(1)+temp(1),col(1)+temp(2)];     % Update the intermediate point
                        end

                        cost = this.Pathway(temp(1),temp(2));       % Intermediate point cost

                    end


                end
            
            end
            
 

       end      
       
       function GlobalSetup(this)             
            
           % Extract branch points in the order of descending cost-to-go
           temp = this.BrchPts0(this.RegionID,1:2);
           
           for ii = 1:length(temp)
               Idx = find(this.Path(:,1)==temp(ii,1) & this.Path(:,2)==temp(ii,2));
               this.Path(Idx,5) = 1;    % If this is a branch point, set it to "1"
               this.Path(Idx,6) = ii;   % Assign region id
               this.Path(Idx,8) = this.Path(Idx,4);     % Assign region junction cost
           end
           
           % Extract branch points in this.path
           temp = find(this.Path(:,5)==1);
           
           % Extract nearby neighbors of these branch points
           Idx = rangesearch(this.Path(:,1:2),this.Path(temp,1:2),2);
            
           for ii = 1:size(temp,1)
               % Assign the same region id of the current branch point to nearby neighbors 
               this.Path(Idx{ii},6) = this.Path(temp(ii),6); 
               this.Path(Idx{ii},8) = this.Path(temp(ii),8);
               % Set these neighbors as "0.5"
               this.Path(Idx{ii},5) = 0.5;
           end
           
           
           for ii = 1:length(this.Path)
               % If this waypoint is not a branch point or branch-nearby
               % waypoint, assign it the same region id of its predecessor.
               idx = ii;
               while this.Path(ii,6) ==0
                   idx = this.Path(idx,3);
                   this.Path(ii,6) = this.Path(idx,6);
                   this.Path(ii,8) = this.Path(idx,8);
               end
               
           end
           
           % Branch Segmentation
           for ii = 1:size(this.ConnMat,1)
               Idx0 = (this.Path(:,1)== this.BrchPts0(ii,1) & this.Path(:,2)== this.BrchPts0(ii,2));
               for jj = 1:this.ConnMat(ii,1)
                    temp_coor = this.ConnMat(ii,2*jj:2*jj+1);
                    temp_idx = find(this.Path(:,1)==temp_coor(1) & this.Path(:,2)==temp_coor(2));
                    
                    temp_flag = 1;
                    while temp_flag
                       
                        if this.Path(temp_idx,5) == 0
                            this.Path(temp_idx,7) = jj;
                        end
                        temp_idx = this.Path(temp_idx,3);
                        
                        if (this.Path(temp_idx,5) > 0) && (this.Path(temp_idx,6) == this.Path(Idx0,6))
                            temp_flag = 0;
                        end
                    end
                    
               end
           end
           
           % Branch segmentation continued: just in case
           temp_idx = find(this.Path(:,7)==0 & this.Path(:,5)==0);
           this.Path(temp_idx,7) = this.Path(this.Path(temp_idx,3),7);
           
           % Extract coordinates of all waypoints in freespace
           [row, col] =  find(this.BW==1);
           this.freespace = [row, col];
           
           % Extract coordinates of all medial-axis waypoints
           this.medial = this.Path(:,1:2);
           
           % For each waypoint in freespace, use KNN search to find the
           % nearest neighbor that belongs to the set of medial-axis waypoints.
           fspIdx = knnsearch(this.medial,this.freespace);
           
           % Associate each waypoint in freespace to the nearest medial-axis waypoint 
           this.freespace(:,4) = fspIdx;
           
           % Assign each waypoint in freespace the region id
           this.freespace(:,3) = this.Path(fspIdx,6);
           
           % Region Segmentation Plot
           
            
           if this.Process_Display ==1
                figure
                imshow(this.BW);
                hold on
                for ii = 1:numel(this.RegionID)
                    temp = find(this.freespace(:,3)==ii);
                    scatter(this.freespace(temp,2),this.freespace(temp,1),10,'filled');
                end

                % Mark goal location
                hGoal = scatter(this.goal(2),this.goal(1),20,'filled');
                hGoal.CData = [1 1 1];
            end
       end
       
       function GlobalControl(this)
            

            

            % Distribute robots in freespace
            this.loc = zeros(this.NumRob,2);
            this.loc(:,1:2) = this.freespace(randi((length(this.freespace)),this.NumRob,1),1:2);
            
           %==============================================================%
           % Enable Animation           
           %==============================================================%
            if this.animation ==1
                % close all
                panel = figure;
                RGB = double(cat(3, ~this.BW, ~this.BW, ~this.BW));
                RGB(:,:,1) = RGB(:,:,1).*182./255+ double(this.BW);
                RGB(:,:,2) = RGB(:,:,2).*228./255+ double(this.BW);
                RGB(:,:,3) = RGB(:,:,3).*255./255+ double(this.BW);
                imshow(RGB)
                hold on

                % Mark goal location
                hGoal = scatter(this.goal(2),this.goal(1),20,'filled');
                hGoal.CData = [1 0 0];
                
                % Plot robots
                hRobot = scatter(this.loc(:,2),this.loc(:,1),5,'filled');
                hRobot.CData = [0 0 0];
                
                hRobot_Target = scatter(this.loc(1,2),this.loc(1,1),15,'filled');
                hRobot_Target.CData = [1 0 0];
                
                
                
            end
            %--------------------------------------------------------------%                          
           
            totalcost = 1e12;       % Initial total cost
                            
            nstep = 0;              % # of total control steps
            
            tic
            
            while totalcost > this.NumRob*(100+1/3*this.channel_width)
                
                switch this.alg
                    case 1
                        % For each robot location, associate it with the nearest medial-axis waypoint 
                        Idx0 = knnsearch(this.Path(:,1:2),this.loc);

                        % Find the max robot with max cost-to-go
                        [~, tempID] = max(this.Path(Idx0,4));

                        % The target robot
                        Robot_Target = tempID;

                        % Extract the coordinate of the target robot
                        p1 = this.loc(Robot_Target,1:2);

                        % Extract the coordinate of the nearest medial-axis waypoint
                        p2 = this.Path(Idx0(Robot_Target),1:2);

                        % The predecesssor of p2
                        p2_Parent_ID = this.Path(Idx0(Robot_Target),3);


                        % This while-loop moves the target robot to the goal
                        % location along the medial-axis trajectory
                        while this.Path(p2_Parent_ID,3)>0

                            % Store current robot locations
                            prev = this.loc;

                            % Calculate moving direction vector
                            ds =p2-p1;

                            % Update all robot locations
                            this.loc(:,1:2) = this.loc(:,1:2) + [ds(1).*ones(this.NumRob,1),...
                                ds(2).*ones(this.NumRob,1)];

                            % Collision check
                            collision_idx = diag(this.BW(this.loc(:,1),this.loc(:,2)))==0;

                            % Make sure all robots stop at the obstacle boundaries                    
                            this.loc(collision_idx,:) = prev(collision_idx,:);

                            % Update the target robot location
                            p1 = this.loc(Robot_Target,1:2);

                            if this.animation ==1
                                % Update robot location plot
                                hRobot.XData = this.loc(:,2);
                                hRobot.YData = this.loc(:,1);
                                hRobot_Target.XData = this.loc(Robot_Target,2);
                                hRobot_Target.YData = this.loc(Robot_Target,1);
                                pause(0.00001);
                                
                            end
                            % For each robot location, associate it with the nearest medial-axis waypoint                     
                            Idx0 = knnsearch(this.Path(:,1:2),this.loc);

                            % Update the total cost
                            totalcost = sum(this.Path(Idx0,4));

                            % Update control step
                            nstep = nstep + 1;
                            
                            % Update intermediate target location
                            p2 = this.Path(p2_Parent_ID,1:2);
                            p2_Parent_ID = this.Path(p2_Parent_ID,3);
                        end
                        


                    case 2
                        % For each robot location, associate it with the nearest medial-axis waypoint 
                        Idx0 = knnsearch(this.Path(:,1:2),this.loc);
                        
                        % RobDist: position and cost information of each robot
                        % Column 1: Nearest medial-axis waypoint ID
                        % Column 2: region ID
                        % Column 3: branch ID       
                        % Column 4: robot cost-to-go
                        RobDist = zeros(this.NumRob,3);
                        
                        % RegionDist: robot distribution in each region
                        % Column 1: number of robots in each region
                        % Column 2: accumulated weights of each branch in the
                        % current region. This weights determine the branch
                        % priority.                     
                        RegionDist = zeros(length(this.RegionID),2+max(this.ConnMat(:,1)));
                        
                        % Robot distribution statistics
                        for ii = 1:this.NumRob                              % For each robot:
                            RobDist(ii,1) = Idx0(ii);                       % assign nearest medial-axis waypoint ID
                            RobDist(ii,2) = this.Path(Idx0(ii),6);          % assign region ID        
                            RobDist(ii,3) = this.Path(Idx0(ii),7);          % assign branch ID
                            RobDist(ii,4) = this.Path(Idx0(ii),4);          % assign cost-to-go
                            RegionDist(RobDist(ii,2),1) = RegionDist(RobDist(ii,2),1)+1;        % Update the number of robots in the corresponding region
                            % Update the weights of the corresponding branch
                            RegionDist(RobDist(ii,2),RobDist(ii,3)+2) = RegionDist(RobDist(ii,2),RobDist(ii,3)+2)+...
                                100*0.4^(floor((this.Path(Idx0(ii),4)-this.Path(Idx0(ii),8))/(this.channel_width*0.5)));
                        end
                                                
                        % Find the farthest region where robots exist
                        for ii = 1:length(this.RegionID)
                            if RegionDist(ii,1) > 0
                                break;
                            end
                        end
                        % The region with highest priority
                        iRegionTarget = ii;
                        
                        if iRegionTarget == length(this.RegionID)
                            this.alg = 1;
                            continue;
                        end
                                 
                        
                        % Find the branch with the highest priority
                        [~, tempID] = max(RegionDist(iRegionTarget,2:end));
                        branchID = tempID-1;
                        Robot_w_Priority = find(RobDist(:,2) == iRegionTarget & RobDist(:,3)==branchID);
                        
                        % Find the robot with the highest priority: set it as the
                        % target robot.
                        [~, tempID] = min(RobDist(Robot_w_Priority,4));
                        Robot_Target = Robot_w_Priority(tempID);
                        
                        
                         % Extract the position of the target robot
                        p1 = this.loc(Robot_Target,1:2);

                        % Extract the position of the nearest medial-axis waypoint
                        p2 = this.Path(Idx0(Robot_Target),1:2);

                        % Find the predecesssor of p2
                        p2_Parent_ID = this.Path(Idx0(Robot_Target),3);


                        % This while-loop moves the target robot to the goal
                        % location along the medial-axis trajectory
                        
                        target_flag = 0;
                        target_reach_flag = 0;
                        target_flag_increment = 1/100;
                        
                        while target_flag <=1 

                            % Store current robot locations
                            prev = this.loc;

                            % Calculate moving direction vector
                            ds =p2-p1;

                            % Update all robot locations
                            this.loc(:,1:2) = this.loc(:,1:2) + [ds(1).*ones(this.NumRob,1),...
                                ds(2).*ones(this.NumRob,1)];

                            % Collision check
                            collision_idx = diag(this.BW(this.loc(:,1),this.loc(:,2)))==0;

                            % Make sure all robots stop at the obstacle boundaries                    
                            this.loc(collision_idx,:) = prev(collision_idx,:);

                            % Update the target robot location
                            p1 = this.loc(Robot_Target,1:2);

                            if this.animation ==1
                                % Update robot location plot
                                hRobot.XData = this.loc(:,2);
                                hRobot.YData = this.loc(:,1);
                                hRobot_Target.XData = this.loc(Robot_Target,2);
                                hRobot_Target.YData = this.loc(Robot_Target,1);
                                pause(0.00001);
                            end
                            % For each robot location, associate it with the nearest medial-axis waypoint                     
                            Idx0 = knnsearch(this.Path(:,1:2),this.loc);

                            % Update the total cost
                            totalcost = sum(this.Path(Idx0,4));

                            % Update control step
                            nstep = nstep + 1;
                            
                            % Update intermediate target location
                            p2 = this.Path(p2_Parent_ID,1:2);
                            p2_Parent_ID = this.Path(p2_Parent_ID,3);

                            if p2_Parent_ID == 0
                               break;
                            end
                            if this.Path(p2_Parent_ID, 5) > 0
                                target_reach_flag = 1;
                            end
                            
                            if target_reach_flag == 1
                                target_flag = target_flag + target_flag_increment;
                            end
                            
                        end
                        

                end
           
            end
            
            disp(' ')
            toc
            disp(' ')
            disp(datetime('now'))
            fprintf('Total control steps = %d\n',nstep);
            
% %             this.Expt(1,this.col0) =this.goal(1);
% %             this.Expt(2,this.col0) =this.goal(2);
% %             this.Expt(this.row0,this.col0) = nstep;
% %             assignin('base','Expt',this.Expt)
% %             fprintf('Experiment %d, End Point %d\n',this.row0-2,this.col0);
% %             close all
       end
       
       function MapEvaluation(this)
           
           % Set up performance recorder: 
           % Column 1: global performance: the impact of local
           % aggregation to all cleared regions (check if there is any
           % reentry).
           % Column 2: local performance: we are looking at two regions
           % next to each other (in the sense of priority), when we work on
           % the closer region, check if there is any reenry in the other
           % region
           
           this.PerformanceEval = [0 0];
           
           % Select a random region as the first region
           Reg4Eval = randi(length(this.RegionID)-2,1);
           
           % (this.freespace(:,3) --> region ID information)
           temp = this.freespace(:,3)==Reg4Eval;                % Find all waypoints in the first region    
           Rg1 = this.freespace(temp,:);                        % Extract the waypoints information in the first region 
           
           % Distribute 100 robots to this region
           NumRob1 = 100;
           RLoc1 = Rg1( randi(length(Rg1),NumRob1,1),1:2);      % Select 100 random locations in the first region as the initial positions  
          
           % Select the second region (region ID:= Reg4Eval+1)
           temp = this.freespace(:,3)==Reg4Eval+1;              % Find all waypoints in the second region           
           Rg2 = this.freespace(temp,:);                        % Extract the waypoints information in the second region 
           
           % Distribute 100 robots to this region
           NumRob2 = 100;
           RLoc2 = Rg2( randi(length(Rg2),NumRob2,1),1:2);      % Select 100 random locations in the second region as the initial positions  
           
           RLoc = [RLoc1;RLoc2];                                % Locations of all robots         
           this.NumRob = NumRob1 + NumRob2;                     % Total number of robots
           
           %==============================================================%
           % Find all regions that are closer to the goal than the second
           % chosen region. And distribute 100 robots in each region
           %==============================================================%
           ii = Reg4Eval+2;                                     % Initial region id 
           
           while ii<=length(this.RegionID)
           
               temp = this.freespace(:,3)==ii;                  % Find all waypoints in the ii-th region       
               Rgi = this.freespace(temp,:);                    % Extract the waypoints information in the ii-th region 
              
               NumRobi = 100;
               RLoci = Rgi( randi(length(Rgi),NumRobi,1),1:2);  % Select 100 random locations in the ii-th region as robot initial positions  
               
               RLoc = [RLoc;RLoci];                             % Update the locations of all robots
               
               this.NumRob = this.NumRob + NumRobi;             % Update the total number of robots
               
               ii = ii+1;                                       % Next region id
           end       
           %--------------------------------------------------------------%
         
           
           % Extract waypoints of regions that are farther than the first
           % chosen region
         
           Rg0 = [];
           
           for ii = 1:Reg4Eval-1
                temp = this.freespace(:,3)== ii;                % Find all waypoints in the ii-th region       
                Rg0 = [Rg0;this.freespace(temp,1:2)];           % Extract the waypoints information in the ii-th region 
           end
           
           %==============================================================%
           % Aggregation performance evaluation matrices definition and 
           % initialization 
           %==============================================================%          
           
           EvalBW0 = this.BW.*0;                                % Initialize evaluation matrix 0
           EvalBW1 = EvalBW0;                                   % Initialize evaluation matrix 1
           
           % Evaluation matrix 0 is a binary matrix which has the same
           % size as the map. Set all pixels to 1 for waypoints belong to
           % "Rg0" (regions taht are farther to goal than the first selected region)
           
           if ~isempty(Rg0)
               temp = ((Rg0(:,2)-1).*size(this.BW,1))+Rg0(:,1);
               EvalBW0(temp)=1;
           end
           
           % Evaluation matrix 1 is a binary matrix which has the same
           % size as the map. Set all pixels to 1 for waypoints belong to
           % "Rg1" (the first chosen region)
           temp = ((Rg1(:,2)-1).*size(this.BW,1))+Rg1(:,1);                
           EvalBW1(temp)=1;
           
           %--------------------------------------------------------------%
           
           %==============================================================%
           % Enable Animation           
           %==============================================================%
           if this.animation ==1
                
                panel = figure;
                RGB = double(cat(3, ~this.BW, ~this.BW, ~this.BW));
                RGB(:,:,1) = RGB(:,:,1).*182./255+ double(this.BW);
                RGB(:,:,2) = RGB(:,:,2).*228./255+ double(this.BW);
                RGB(:,:,3) = RGB(:,:,3).*255./255+ double(this.BW);
                imshow(RGB)
                hold on
                
                % Plot Aggregation Region
                hRegion1 = scatter(Rg1(:,2),Rg1(:,1),1,'filled');
                hRegion1.CData = [255 255 0]./255;
                
                hRegion2 = scatter(Rg2(:,2),Rg2(:,1),1,'filled');
                hRegion2.CData = [255,239,213]./255;
                
                if ~isempty(Rg0)
                    hRegion0 = scatter(Rg0(:,2),Rg0(:,1),1,'filled');
                    hRegion0.CData = [192,192,192]./255;
                end
                
                % Plot robots
                hRobot = scatter(RLoc(:,2),RLoc(:,1),5,'filled');
                hRobot.CData = [0 0 0];
                
                hRobot_Target = scatter(RLoc(1,2),RLoc(1,1),15,'filled');
                hRobot_Target.CData = [1 0 0];
           end           
           %--------------------------------------------------------------%                          
           
           nstep = 1;  % # of total control steps
           
           MapEvalFlag = 1; % Enable aggregation performance evaluation
           
           %% Aggregation Evaluation
           while MapEvalFlag
                % For each robot location, associate it with the nearest medial-axis waypoint 
                Idx0 = knnsearch(this.Path(:,1:2),RLoc);

                % RobDist: position and cost information of each robot
                % Column 1: Nearest medial-axis waypoint ID
                % Column 2: region ID
                % Column 3: branch ID       
                % Column 4: robot cost-to-go
                RobDist = zeros(this.NumRob,3);

                % RegionDist: robot distribution in each region
                % Column 1: number of robots in each region
                % Column 2: accumulated weights of each branch in the
                % current region. This weights determine the branch
                % priority.
                RegionDist = zeros(length(this.RegionID),2+max(this.ConnMat(:,1)));

                for ii = 1:(this.NumRob)                                % For each robot:
                    RobDist(ii,1) = Idx0(ii);                           % assign nearest medial-axis waypoint ID
                    RobDist(ii,2) = this.Path(Idx0(ii),6);              % assign region ID                    
                    RobDist(ii,3) = this.Path(Idx0(ii),7);              % assign branch ID
                    RobDist(ii,4) = this.Path(Idx0(ii),4);              % assign cost-to-go
                    
                    RegionDist(RobDist(ii,2),1) = RegionDist(RobDist(ii,2),1)+1;        % Update the number of robots in the corresponding region
                    % Update the weights of the corresponding branch
                    RegionDist(RobDist(ii,2),RobDist(ii,3)+2) = RegionDist(RobDist(ii,2),RobDist(ii,3)+2)+...       
                        100*0.4^(floor((this.Path(Idx0(ii),4)-this.Path(Idx0(ii),8))/(this.channel_width*0.5)));
                end

                % Find the farthest region where robots exist
                for ii = 1:length(this.RegionID)
                    if RegionDist(ii,1) > 0
                        break;
                    end
                end
                
                % The region with highest priority
                iRegionTarget = ii;
                
                % If we have finished aggregation in the chosen two
                % regions, or the targeted region is the goal region,
                % stop aggregation performance evaluation.
                if iRegionTarget > Reg4Eval + 1 || iRegionTarget ==length(this.RegionID)
                    MapEvalFlag = 0;
                    break
                end

                % Find the branch with the highest priority
                [~, tempID] = max(RegionDist(iRegionTarget,2:end));
                branchID = tempID-1;
                Robot_w_Priority = find(RobDist(:,2) == iRegionTarget & RobDist(:,3)==branchID);

                % Find the robot with the highest priority: set it as the
                % target robot.
                [~, tempID] = min(RobDist(Robot_w_Priority,4));
                Robot_Target = Robot_w_Priority(tempID);

                % Extract the position of the target robot
                p1 = RLoc(Robot_Target,1:2);

                % Extract the position of the nearest medial-axis waypoint
                p2 = this.Path(Idx0(Robot_Target),1:2);

                % Find the predecesssor of p2
                p2_Parent_ID = this.Path(Idx0(Robot_Target),3);


                % This while-loop moves the target robot to the goal
                % along the medial-axis trajectory

                target_flag = 0;
                target_reach_flag = 0;
                target_flag_increment = 1/50;
                while target_flag <=1 

                    % Store current robot locations
                    prev = RLoc;

                    % Calculate moving direction vector
                    ds =p2-p1;

                    % Update all robot locations
                    RLoc(:,1:2) = RLoc(:,1:2) + [ds(1).*ones(this.NumRob,1),...
                        ds(2).*ones(this.NumRob,1)];

                    % Collision check
                    collision_idx = diag(this.BW(RLoc(:,1),RLoc(:,2)))==0;

                    % Make sure all robots stop at the obstacle boundaries                    
                    RLoc(collision_idx,:) = prev(collision_idx,:);
                    
                    % Evaluate map performance by checking if there are any
                    % robots in regions that have been aggregated.
                    temp = ((RLoc(:,2)-1).*size(this.BW,1))+RLoc(:,1);
                    
                    % Extend variable capacity
                    if length(this.PerformanceEval)<nstep
                        this.PerformanceEval = [this.PerformanceEval;this.PerformanceEval.*0];
                    end
                    
                    % Record aggregation performance by counting the number
                    % of robots reentry in cleared regions
                    if iRegionTarget > Reg4Eval
                        this.PerformanceEval(nstep,2) = sum(EvalBW1(temp)==1);
                        this.PerformanceEval(nstep,1) = sum(EvalBW0(temp)==1);
                    else
                        this.PerformanceEval(nstep,1) = sum(EvalBW0(temp)==1);
                    end
                    
                    % Update the target robot location
                    p1 = RLoc(Robot_Target,1:2);

                    if this.animation ==1
                        % Update robot location plot
                        hRobot.XData = RLoc(:,2);
                        hRobot.YData = RLoc(:,1);
                        hRobot_Target.XData = RLoc(Robot_Target,2);
                        hRobot_Target.YData = RLoc(Robot_Target,1);
                        pause(0.00001);

                    end


                    % Update control step
                    nstep = nstep + 1;

                    % Update intermediate target location
                    p2 = this.Path(p2_Parent_ID,1:2);
                    p2_Parent_ID = this.Path(p2_Parent_ID,3);

                    if p2_Parent_ID == 0
                       break;
                    end
                    if this.Path(p2_Parent_ID, 5) > 0
                        target_reach_flag = 1;
                    end

                    if target_reach_flag == 1
                        target_flag = target_flag + target_flag_increment;
                    end

                end          
            
           end

           
           this.PerformanceEval= this.PerformanceEval(1:nstep-1,1:2);
            
            % Plot aggreagtion performance evaluation
            figure
            plot(this.PerformanceEval(:,1),'-r','LineWidth',2);hold on
            plot(this.PerformanceEval(:,2),'-k','LineWidth',2);hold off
            xlabel('Number of Steps')
            ylabel('Number of Robots in Cleared Regions')
            legend('Global Reentry','Local Reentry')
       end       
       
       function HumanControl(this)
            joy = vrjoystick(1) ;
            [axes, buttons, povs] = read(joy);
            
            
            cd('C:\Users\lhuang28\Documents\GitHub\MagneticController\lihuang\GlobalAgg\New folder')
            % Distribute robots in freespace
            this.loc = zeros(this.NumRob,2);
            this.loc(:,1:2) = this.freespace(randi((length(this.freespace)),this.NumRob,1),1:2);
         
            panel = figure;
            RGB = double(cat(3, ~this.BW, ~this.BW, ~this.BW));
            RGB(:,:,1) = RGB(:,:,1).*182./255+ double(this.BW);
            RGB(:,:,2) = RGB(:,:,2).*228./255+ double(this.BW);
            RGB(:,:,3) = RGB(:,:,3).*255./255+ double(this.BW);
            imshow(RGB)
            hold on
                         
            % Mark goal location
            hGoal = scatter(this.goal(2),this.goal(1),20,'filled');
            hGoal.CData = [1 0 0];

            % Plot robots
            hRobot = scatter(this.loc(:,2),this.loc(:,1),5,'filled');
            hRobot.CData = [0 0 0];
            
            nstep = 1;
            frame_num = 1;
            
            % For each robot location, associate it with the nearest medial-axis waypoint                     
            Idx0 = knnsearch(this.Path(:,1:2),this.loc);

            % Update the total cost
            totalcost = sum(this.Path(Idx0,4))./this.NumRob-100;
            
            figure(2)
            fp2 = plot(1:frame_num,totalcost,'-k','LineWidth',2);
            tmp = figure(2);
            tmp.Position = [-800, 100, 400 300]; 
            axis([0 400 0 400]);
           
            xlabel('5x Number of Steps')
            ylabel('Average Cost to the Goal')
           
            while buttons(2) ==0
                [axes, buttons, povs] = read(joy);
                axisY = -axes(2);
                axisX = -axes(1);
                if abs(axisY)>0.2 || abs(axisX)>0.2
                    angle = round((atan2(axisX,axisY)+pi)/(pi/4));
                    ds = round([cos(angle*pi/4),sin(angle*pi/4)]);

                    % Store current robot locations
                    prev = this.loc;

                    % Update all robot locations
                    this.loc(:,1:2) = this.loc(:,1:2) + [ds(1).*ones(this.NumRob,1),...
                        ds(2).*ones(this.NumRob,1)];

                    % Collision check
                    collision_idx = diag(this.BW(this.loc(:,1),this.loc(:,2)))==0;

                    % Make sure all robots stop at the obstacle boundaries                    
                    this.loc(collision_idx,:) = prev(collision_idx,:);


                    % Update robot location plot
                    hRobot.XData = this.loc(:,2);
                    hRobot.YData = this.loc(:,1);
                    pause(0.00001);

                    % For each robot location, associate it with the nearest medial-axis waypoint                     
                    Idx0 = knnsearch(this.Path(:,1:2),this.loc);

                    % Update the total cost
                    totalcost = sum(this.Path(Idx0,4))./this.NumRob-100;
                    
                    % Update control step
                    nstep = nstep + 1;
                end
                
                if mod(frame_num,5)==0 
                
                    figure(1)

                    thisframe = getframe;
                    thisfile = sprintf('Agg%04d.tif',frame_num/5);
                    imwrite(thisframe.cdata,thisfile);

                    figure(2)

                    set(fp2,'YData',[fp2.YData,totalcost]);
                    set(fp2,'XData',[fp2.XData,frame_num/5]);
                    thisframe = getframe;
                    thisfile = sprintf('Cost%04d.tif',frame_num/5);
                    imwrite(thisframe.cdata,thisfile);
                end
                frame_num = frame_num+1;
            end
           
       end
       
   end


end