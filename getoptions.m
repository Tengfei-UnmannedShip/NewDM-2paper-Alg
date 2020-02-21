function v = getoptions(options, name, v, mendatory)
% getoptions-检索选项参数
% 很有用，以后我也可以使用
% getoptions - retrieve options parameter
%
%   v = getoptions(options, 'entry', v0);
% is equivalent to the code:
%   if isfield(options, 'entry')
%       v = options.entry;
%   else
%       v = v0;
%   end
%
%   Copyright (c) 2007 Gabriel Peyre

if nargin<4
    mendatory = 0;
end
%系统自带函数，确定输入是否为结构数组字段
%Determine whether input is structure array field
if isfield(options, name)   
    
    v = eval(['options.' name ';']);
elseif mendatory
    error(['You have to provide options.' name '.']);
end 
end
