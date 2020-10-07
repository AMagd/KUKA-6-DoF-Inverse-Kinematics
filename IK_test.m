% Author ~ Ahmed Magd Aly
% Innopolis University

clear all
close all
clc

%% Testing the IK function:
% Shape to draw: 1 -> sinusoidal circle, 2 -> 3D spiral, 3 -> circles, 4 ->
% sine wave with all possible solutions
ShapeToDraw = 1;

% give default z_angle, y_angle and x_angle of the final frame that we want to achieve:
z_angle = 0;
y_angle = 0;
x_angle = 0;

% initiate figure
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


%% Drawing shape 1
if ShapeToDraw == 1


    for t = 0:90
        % This condition is just to check if the figure was closed so
        % that it stops excecution
        if ~ishandle(Robot), return, end
        delete([links_plot,joints_plot,end_effector_plot])
        delete(axes_plot)
        
        z = 900 + 100*sin(0.5*t/pi);
        x = 0 + 10*t;
        y = 0 ;

        [q,n] = IK(x, y, z, z_angle, y_angle, x_angle);
        
        % put n equals to 1 to show only the first solutoin, if you want to
        % show all of them just delete n = 1 line
        n = 1;

        for i = 1:n
            if n == 1
            elseif n == 4
                subplot(2,2,i)
            elseif n == 8
                subplot(2,4,i)
            else
                subplot(1,2,i)
            end
            FK(q(1,i), q(2,i), q(3,i), q(4,i), q(5,i), q(6,i), 1);

        end
        drawnow
    end

    radius = x;
    index = 0;
    angle = linspace(0,2*pi,500);
    for t = linspace(1,2*pi*radius,round(2*pi*radius))
        if ~ishandle(Robot), return, end
        delete([links_plot,joints_plot,end_effector_plot])
        delete(axes_plot)
        
        index = index + 1;
        z = 900 + 100*sin(0.5*(t+90)/pi);
        x = radius*cos(angle(index));
        y = radius*sin(angle(index)) ;

        [q,n] = IK(x, y, z, angle(index), y_angle, x_angle);
        n = 1;

        for i = 1:n
            if ~ishandle(Robot), return, end
            
            if n == 1
            elseif n == 4
                subplot(2,2,i)
            elseif n == 8
                subplot(2,4,i)
            else
                subplot(1,2,i)
            end
            FK(q(1,i), q(2,i), q(3,i), q(4,i), q(5,i), q(6,i), 1);

        end
        drawnow
    end
    
%% Drawing shape 2
elseif ShapeToDraw == 2
    
    % Drawing 3D spiral
    % spiral parameters:
    steps = 500;
    r = linspace(10,1000,steps);
    height = linspace(10,1000,steps);
    rotations = 5;
    angle = linspace(0,rotations*2*pi,steps);
    
    for i=1:steps
        if ~ishandle(Robot), return, end
        delete([links_plot,joints_plot,end_effector_plot])
        delete(axes_plot)
        
        z = height(i);
        x = r(i)*cos(angle(i));
        y = r(i)*sin(angle(i)) ;

        [q,n] = IK(x, y, z, angle(i), y_angle, x_angle);
        % put n equals to 1 to show only the first solutoin, if you want to
        % show all of them just delete n = 1 line
        n = 1;

        for i = 1:n
            
            if n == 1
            elseif n == 4
                subplot(2,2,i)
            elseif n == 8
                subplot(2,4,i)
            else
                subplot(1,2,i)
            end
            FK(q(1,i), q(2,i), q(3,i), q(4,i), q(5,i), q(6,i), 1);
        end
        drawnow
    end
    
%% Drawing shape 3
elseif ShapeToDraw == 3
    
    % Drawing array of circles
    % Parameters to define
    numberOfCircles = 10;
    smallestRadius = 50;
    biggestRadius = 300;
    distanceBetweenCircles = 50;
    
    r = [linspace(smallestRadius, biggestRadius, numberOfCircles) linspace(biggestRadius, smallestRadius, numberOfCircles)];
    r(numberOfCircles) = []; 
    
    y = -(numberOfCircles-1)*distanceBetweenCircles;
    for j = 1:length(r)
        if ~ishandle(Robot), return, end
        
        steps = round(r(j)/3);
        for angle = linspace(0,2*pi,steps)
            delete([links_plot,joints_plot,end_effector_plot])
            delete(axes_plot)

            z = r(j)*sin(angle);
            x = r(j)*cos(angle);
            
            if y<0
                z_angle = pi/2;
            else
                z_angle = -pi/2;
            end

            [q,n] = IK(x, y, z, z_angle, y_angle, x_angle);
            % put n equals to 1 to show only the first solutoin, if you want to
            % show all of them just delete n = 1 line
            n = 1;

            for i = 1:n
                if n == 1
                elseif n == 4
                    subplot(2,2,i)
                elseif n == 8
                    subplot(2,4,i)
                else
                    subplot(1,2,i)
                end
                FK(q(1,i), q(2,i), q(3,i), q(4,i), q(5,i), q(6,i), 1);
                zlim([-1000 1000])
            end
            drawnow
        end
        y = y + distanceBetweenCircles;
    end
   
%% Drawing shape 4
elseif ShapeToDraw == 4
    for t = 0:115
        if ~ishandle(Robot), return, end
        z = 900 + 100*sin(0.5*t/pi);
        x = -100 + 10*t;
        y = 200 ;


        delete([links_plot,joints_plot,end_effector_plot])
        delete(axes_plot)

        [q,n] = IK(x, y, z, z_angle, y_angle, x_angle);

        for i = 1:n
            if n == 1
            elseif n == 4
                subplot(2,2,i)
            elseif n == 8
                subplot(2,4,i)
            else
                subplot(1,2,i)
            end
            FK(q(1,i), q(2,i), q(3,i), q(4,i), q(5,i), q(6,i), 1);

        end
        drawnow
    end
end