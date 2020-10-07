% Author ~ Ahmed Magd Aly
% Innopolis University

function [O, T0_6] = FK(q1, q2, q3, q4, q5, q6, enable_plot)
% Forward Kinematics Function:

d0 = 400;
d1 = 25;
d2 = 560;
d3 = 25;
d4 = 515;
d6 = 90;

q1 = deg2rad(q1);
q2 = deg2rad(q2); % this is to use the kinematic equations as if the robot's home position was in the horizontal mode, but apply them as if the robot was in the vertical position
q3 = deg2rad(q3);
q4 = deg2rad(q4);
q5 = deg2rad(q5);
q6 = deg2rad(q6);

%% Origin Frame
O0 = eye(4); % world reference frame

%% Moving to point 1:
T_base = Tz(d0);
O1 = O0*T_base;

T1 = Rz(q1)*Tx(d1);

O2 = O1*T1;

T2 = Ry(q2)*Tz(d2);

O3 = O2*T2;

T3 = Ry(q3)*Tz(d3);

O4 = O3*T3;

T4 = Tx(d4)*Rx(q4);

O5 = O4*T4;

T5 = Ry(q5);

O6 = O5*T5;

T6 = Tx(90)*Rx(q6);

O7 = O6*T6;

O = round([O0, O1, O2, O3, O4, O5, O6, O7],10);

T_arm = T1*T2*T3;

T_tool = Tx(d6);

T_wrist = Rx(q4)*Ry(q5)*Rx(q6)*T_tool;

T0_6 = T_base*T_arm*T_wrist*T_tool;

global axes_plot links_plot joints_plot end_effector_plot



if enable_plot
    % figure('units','normalized','outerposition',[0 0 1 1])
    


    as = 150; % axes scaler
    color = ['r','g','b']; % axes color
    
    index = 0;
    for i = 1:4:length(O)
        index = index + 1;
        points_x(index) = O(1,i+3);
        points_y(index) = O(2,i+3);
        points_z(index) = O(3,i+3);
        
        if index ~= 2 && index ~= 5
            for axes = 0:2

                axes_plot = [axes_plot plot3([O(1,i+3) as*O(1,i+axes)+O(1,i+3)], [O(2,i+3) as*O(2,i+axes)+O(2,i+3)], [O(3,i+3) as*O(3,i+axes)+O(3,i+3)],'Color',color(axes+1))];
                hold on
            end
        end
    end
    joints_x = [points_x(1) points_x(3:4) points_x(6:8)];
    joints_y = [points_y(1) points_y(3:4) points_y(6:8)];
    joints_z = [points_z(1) points_z(3:4) points_z(6:8)];
    
    links_plot = [links_plot plot3(points_x, points_y, points_z,'Color', "0 0 0",'linewidth',2)];
    hold on
    joints_plot = [joints_plot plot3(joints_x(1:5), joints_y(1:5), joints_z(1:5),'.','Color','0.992 0.788 0.04 1','MarkerSize',20)];
    hold on
    end_effector_plot = [end_effector_plot plot3(joints_x(6), joints_y(6), joints_z(6),'.','Color','0.8 0 0 1','MarkerSize',20)];
    path_plot = plot3(joints_x(6), joints_y(6), joints_z(6),'.','Color','0.8 0 0 1','MarkerSize',7.5);
    
    
    xlim([-1000 1000])
    ylim([-1000 1000])
    zlim([-100 1500])
%     view(0,0)
    grid on
end


