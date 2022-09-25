function Bik_u = BaseFunction(i, k , u, NodeVector) % k�Ǵ���

if k == 0       % 0��B���� i��0��ʼѭ����matlab������1��ʼ
    if u >= NodeVector(i+1) && u < NodeVector(i+2)
        Bik_u = 1;
    else
        Bik_u = 0;
    end
else
    % ֧������ĳ��� �²�����ʽ֪���ͰٶȵĹ�ʽ��ͬ������֪���Ĳ��ԣ��ٶȵĲŶԣ������������1
    Length1 = NodeVector(i+k+1) - NodeVector(i+1); % ��һ����ĸ
    Length2 = NodeVector(i+k+2) - NodeVector(i+2); % �ڶ�����ĸ    
    if Length1 == 0       % �涨0/0 = 0
        Length1 = 1;
    end
    if Length2 == 0
        Length2 = 1;
    end
    % �ݹ�(��������)ֱ��k == 0 ��ÿ�ε���������ʽ�Ҳ����BaseFunction��ÿ�ζ�����Bik_u��
    Bik_u = (u - NodeVector(i+1)) / Length1 * BaseFunction(i, k-1, u, NodeVector) ...
        + (NodeVector(i+k+2) - u) / Length2 * BaseFunction(i+1, k-1, u, NodeVector);
end
% �ݹ�
% b=a+fun(1),b=a+fun(2);b=a+fun(3);fun(3)=0;
