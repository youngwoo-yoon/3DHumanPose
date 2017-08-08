% Extract the nearest exemplar from 3D pose library
% pool_2d: the 2D pose library for matching
% pool_3d: the 3D pose library
% pred: the prediction 2D pose
% mask: skeleton mask (logical variable)
function [j_p] = NN_pose(pool_2d,pool_3d,pred,mask)
    temp = zeros(1,28);
    temp_root = 0.5 * (pred(9,:) + pred(12,:)); % right and left hip point
    for a = 1:14
        if(mask(a) == 0); continue; end
        temp(1,a*2-1) = pred(a,1) - temp_root(1);
        temp(1,a*2) = pred(a,2) - temp_root(2);
        y_c(a) = temp(1,2*a);
    end
    scale = max(y_c)-min(y_c);
    temp = temp / scale;
    
    array_mask = logical(mask);
    array_mask = repelem(array_mask,2);
    
    %[~,m_idx] = min(pdist2(temp, pool_2d));
    [~,m_idx] = min(pdist2(temp(array_mask), pool_2d(:,array_mask)));
    j_p = pool_3d(m_idx,:);
    j_p = reshape(j_p,3,14);
    j_p = double(j_p');
end
