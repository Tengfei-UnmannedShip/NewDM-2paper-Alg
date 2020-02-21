function d = nb_dims(x)
% ndims函数的调试调用函数
% nb_dims - debugged version of ndims.
%
%   d = nb_dims(x);
%
%   Copyright (c) 2004 Gabriel Peyr?

if isempty(x)
    d = 0;
    return;
end
% ndims系统函数，返回向量的维度

d = ndims(x);

if d==2 && (size(x,1)==1 || size(x,2)==1)
    d = 1;
end