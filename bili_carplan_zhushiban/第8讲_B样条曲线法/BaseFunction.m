function Bik_u = BaseFunction(i, k , u, NodeVector) % k是次数

if k == 0       % 0次B样条 i从0开始循环但matlab向量从1开始
    if u >= NodeVector(i+1) && u < NodeVector(i+2)
        Bik_u = 1;
    else
        Bik_u = 0;
    end
else
    % 支撑区间的长度 德布尔公式知乎和百度的公式不同，但用知乎的不对，百度的才对？？？三项多了1
    Length1 = NodeVector(i+k+1) - NodeVector(i+1); % 第一个分母
    Length2 = NodeVector(i+k+2) - NodeVector(i+2); % 第二个分母    
    if Length1 == 0       % 规定0/0 = 0
        Length1 = 1;
    end
    if Length2 == 0
        Length2 = 1;
    end
    % 递归(调用自身)直到k == 0 即每次迭代都将等式右侧带入BaseFunction（每次都是求Bik_u）
    Bik_u = (u - NodeVector(i+1)) / Length1 * BaseFunction(i, k-1, u, NodeVector) ...
        + (NodeVector(i+k+2) - u) / Length2 * BaseFunction(i+1, k-1, u, NodeVector);
end
% 递归
% b=a+fun(1),b=a+fun(2);b=a+fun(3);fun(3)=0;
