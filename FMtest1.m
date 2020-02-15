% Ìí¼ÓÂ·¾¶
addpath('.\simple_sim_without_control_FS','-end');
addpath('.\toolbox','-end');

load FMmap.mat
M = FM_map;
start_point = [2500, 100];
end_point = [1500, 2500];
tic
[Mtotal, paths] = FMM(M, end_point', start_point');
toc
path = paths{:};
figure(1)
imageplot(convert_distance_color(Mtotal)); 
axis image; 
axis on;
plot(path(2, :), path(1, :), '-r'); 
axis on;