function child_nodes = child_nodes_cal(parent_node, m, n, obs, closeList)

child_nodes = [];
field = [1,1; n,1; n,m; 1,m];

% 第1个子节点
child_node = [parent_node(1)-1, parent_node(2)+1];
% inpolygon(x,y,xv,yv) 判断x,y是否在多边形内部，其中xv,yv为地图四个顶点格子位置(是否在所画的地图内，不是plot)
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    % ismember当 A 中的行也存在于 B 中时，将返回包含逻辑值 1 (true) 的列向量。
    % 数组中的其他位置将包含逻辑值 0 (false)
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第2个子节点
child_node = [parent_node(1), parent_node(2)+1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第3个子节点
child_node = [parent_node(1)+1, parent_node(2)+1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第4个子节点
child_node = [parent_node(1)-1, parent_node(2)];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第5个子节点
child_node = [parent_node(1)+1, parent_node(2)];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第6个子节点
child_node = [parent_node(1)-1, parent_node(2)-1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第7个子节点
child_node = [parent_node(1), parent_node(2)-1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 第8个子节点
child_node = [parent_node(1)+1, parent_node(2)-1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

%% 排除已经存在于closeList的节点
delete_idx = [];
for i = 1:size(child_nodes, 1)
    if ismember(child_nodes(i,:), closeList , 'rows')
        delete_idx(end+1,:) = i;
    end
end
child_nodes(delete_idx, :) = [];