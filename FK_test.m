% Author ~ Ahmed Magd Aly
% Innopolis University

clear all
close all
clc
%%
% Initiate figure element with fullscreen
Robot = figure('units','normalized','outerposition',[0 0 1 1]);

% This part is just to clear all the elements in the plot except the path of the end-effector
global axes_plot links_plot joints_plot end_effector_plot
axes_plot = plot3(0,0,0);
hold on
links_plot = plot3(0,0,0);
hold on
joints_plot = plot3(0,0,0);
hold on
end_effector_plot = plot3(0,0,0);

for angle = 0:5:360
    if ~ishandle(Robot), return, end
    delete([links_plot,joints_plot,end_effector_plot])
    delete(axes_plot)
    
    angles = rand(1,6)*360;
    % you can comment the line below and uncomment the one below that and
    % put the angles you want in the simulation
    [O, T] = FK(angles(1), angles(2), angles(3), angles(4), angles(5), angles(6), 1);
    %[O, T] = FK(angle, 0, 0, 0, 0, 0, 1);
    drawnow
end