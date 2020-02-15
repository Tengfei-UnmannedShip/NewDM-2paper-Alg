function [map, paths] = FMM(W, start_points, end_points)
map = [];
paths = {};
if nargin==0
	disp('请输入更多参数。');
elseif nargin==1
    disp('请输入更多参数。');
else
	options.nb_iter_max = Inf;
    disp('Performing front propagation.');
    [D,~] = perform_fast_marching(W, start_points, options);
    map = D;
    paths = {};
end

if nargin==3
    npaths = size(end_points, 2);
    disp('Extract paths');
    for i=1:npaths
        paths{i} = compute_geodesic(D,end_points(:,i));
        if length(paths{i}(:))==2
            paths{i} = paths{i-1};
        end        
    end
end
end