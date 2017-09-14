function [states,index,states_cap,loop_mat] = forloopfunction(loop_mat,states,...
    NumFreeSpace,NumRob,FreeSpaceIdx,FreeSpace,loop_idx,stage,index,states_cap)
 
    if stage<NumRob
      
       stage = stage + 1; 
       for local_idx = loop_idx:NumFreeSpace
           loop_mat(1,stage-1) = local_idx;
           [states,index,states_cap,loop_mat] = ...
               forloopfunction(loop_mat,states,NumFreeSpace,NumRob,...
           FreeSpaceIdx,FreeSpace,local_idx,stage,index,states_cap);       
       end
    else

        for local_idx = loop_idx:NumFreeSpace
            loop_mat(1,stage) = local_idx;
            if states_cap < index
               states = [states;states.*0];
               states_cap = states_cap*2;
            end
            for iR = 1:NumRob
               states(index,iR) = FreeSpaceIdx(FreeSpace(loop_mat(1,iR)));
            end
            index = index + 1;
                      
        end
 
    end
    
end