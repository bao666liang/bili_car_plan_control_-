function NodeVector = U_quasi_uniform(n, k)
% 准均匀B样条的节点向量计算，共n+1个控制顶点，k次B样条，k+1阶（这里的k是次数，main的k是阶数！！）
NodeVector = zeros(1, n+k+2);%节点矢量
piecewise = n - k + 1;       % 曲线的段数（一条贝塞尔曲线由多少段贝塞尔曲线构成）
if piecewise == 1            % 只有一段曲线时，n = k
    for i = k+2 : n+k+2
        NodeVector(1, i) = 1;
    end
else
    flag = 1;           % 不止一段曲线时
    while flag ~= piecewise
        NodeVector(1, k+flag+1) = NodeVector(1, k + flag) + 1/piecewise;
        flag = flag + 1;
    end
    NodeVector(1, n+2 : n+k+2) = 1;   % 节点向量前面和后面有（k+1）个重复值（阶数）
end
