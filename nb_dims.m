function d = nb_dims(x)
% ndims�����ĵ��Ե��ú���
% nb_dims - debugged version of ndims.
%
%   d = nb_dims(x);
%
%   Copyright (c) 2004 Gabriel Peyr?

if isempty(x)
    d = 0;
    return;
end
% ndimsϵͳ����������������ά��

d = ndims(x);

if d==2 && (size(x,1)==1 || size(x,2)==1)
    d = 1;
end