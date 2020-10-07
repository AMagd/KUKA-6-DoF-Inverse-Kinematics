% Author ~ Ahmed Magd Aly
% Innopolis University

function [q, numberOfSolutions] = IK(x_in, y_in, z_in, rotz, roty, rotx)
%% Model dimensions:
d0 = 400;
d1 = 25;
d2 = 560;
d3_dash = 25;
d4 = 515;
d6 = 90;

dq = -atan2(d3_dash,d4); %%%%%% NOTE THE NEGATIVE HERE BECAUSE dq IS IN THE OPPOSITE DIRECTION OF Q3
d3 = sqrt(d3_dash^2 + d4^2);



%% Model Kinematics:
orientation = eye(4);
orientation(1:3,1:3) = eul2rotm([rotz, roty, rotx]);

T = Tx(x_in)*Ty(y_in)*Tz(z_in)*orientation;
% also T = Tz(d0)*T123*Ry(-dq)*T456*Tx(d6)

T0 = inv(Tz(d0))*T*inv(Tx(d6));

x = T0(1,4);
y = T0(2,4);
z = T0(3,4);

%--------------------------------

%% Calculating q1:
% for q1 there might be two solution or just one solution
q1 = atan2(y,x)*ones(1,2);

%% Calculating q2 and q3:
Y = z;
l1 = d2;
l2 = d3;

% first pair of solution X1 (elbow up and elbow down)
X1 = sqrt(x^2 + y^2) - d1;

alpha = acos( (X1^2+Y^2+l1^2-l2^2)/(2*l1*sqrt(X1^2+Y^2)) );
beta = acos( (l1^2+l2^2-X1^2-Y^2)/(2*l1*l2) );
gamma = atan2(X1,Y);

% Check if position is inside the reachable region
if imag(beta) > 0 || imag(alpha)>0
    disp("Position Out of Range")
    return
end

q2 = gamma - alpha; q3 = pi/2 - beta;
q2 = [q2 (gamma + alpha)]; q3 = [q3 (beta + pi/2)];

%% Calculating q4, q5 and q6:
n = length(q2);

for i = 1:n
    T123 = Tz(d0)*Rz(q1(i))*Tx(d1)*Ry(q2(i))*Tz(d2)*Ry(q3(i)-dq)*Tz(d3_dash)*Tx(d4);
    T456bis = inv(T123)*T;
    
    if abs(T456bis(1,1)) ~= 1
        q4(i) = atan2(T456bis(2,1), -T456bis(3,1));
        q6(i) = atan2(T456bis(1,2), T456bis(1,3));
        q5(i) = atan2(sqrt(T456bis(1,2)^2 + T456bis(1,3)^2), T456bis(1,1));
    else
        disp("wrist singularity case has infinite number of solutions")
        q5 = [q5 0];
        q4 = [q4 0]; % assumed an arbitrary value for q4
        q6 = [q6 atan2(T456bis(3,2), T456bis(3,2))];
    end
end


for i = n+1:2*n
    q1(i) = q1(i-n); q2(i) = q2(i-n); q3(i) = q3(i-n);
    T123 = Tz(d0)*Rz(q1(i))*Tx(d1)*Ry(q2(i))*Tz(d2)*Ry(q3(i)-dq)*Tz(d3_dash)*Tx(d4);
    T456bis = inv(T123)*T;
    
    if abs(T456bis(1,1)) ~= 1
        q4(i) = atan2(-T456bis(2,1), T456bis(3,1));
        q6(i) = atan2(-T456bis(1,2), -T456bis(1,3));
        q5(i) = atan2(-sqrt(T456bis(1,2)^2 + T456bis(1,3)^2), T456bis(1,1));
    else
        disp("wrist singularity case has infinite number of solutions")
        q5 = [q5 0];
        q4 = [q4 0]; % assumed an arbitrary value for q4
        q6 = [q6 atan2(T456bis(3,2), T456bis(3,2))];
    end
end



%% checking if there is another solution for q1
lengths = sqrt( (sqrt(x^2 + y^2)+d1)^2 + z^2 );

if lengths <= (d2+d3)
    q1 = [q1, (q1(1) + pi)*ones(1,2)];

    X2 = -sqrt(x^2 + y^2) - d1;
    alpha = acos( (X2^2+Y^2+l1^2-l2^2)/(2*l1*sqrt(X2^2+Y^2)) );
    beta = acos( (l1^2+l2^2-X2^2-Y^2)/(2*l1*l2) );
    gamma = atan2(X2,Y);

    q2 = [q2, gamma - alpha]; q3 = [q3, pi/2 - beta];
    q2 = [q2, gamma + alpha]; q3 = [q3, beta + pi/2];
    
    % wrist solutions:
    n = length(q2);

    for i = n-1:n
        T123 = Tz(d0)*Rz(q1(i))*Tx(d1)*Ry(q2(i))*Tz(d2)*Ry(q3(i)-dq)*Tz(d3_dash)*Tx(d4);
        T456bis = inv(T123)*T;

        if abs(T456bis(1,1)) ~= 1
            q4(i) = atan2(T456bis(2,1), -T456bis(3,1));
            q6(i) = atan2(T456bis(1,2), T456bis(1,3));
            q5(i) = atan2(sqrt(T456bis(1,2)^2 + T456bis(1,3)^2), T456bis(1,1));
        else
            disp("wrist singularity case has infinite number of solutions")
            q5 = [q5 0];
            q4 = [q4 0]; % assumed an arbitrary value for q4
            q6 = [q6 atan2(T456bis(3,2), T456bis(3,2))];
        end
    end
    
    for i = n+1:n+2
        q1(i) = q1(i-2); q2(i) = q2(i-2); q3(i) = q3(i-2);
        T123 = Tz(d0)*Rz(q1(i))*Tx(d1)*Ry(q2(i))*Tz(d2)*Ry(q3(i)-dq)*Tz(d3_dash)*Tx(d4);
        T456bis = inv(T123)*T;

        if abs(T456bis(1,1)) ~= 1
            q4(i) = atan2(-T456bis(2,1), T456bis(3,1));
            q6(i) = atan2(-T456bis(1,2), -T456bis(1,3));
            q5(i) = atan2(-sqrt(T456bis(1,2)^2 + T456bis(1,3)^2), T456bis(1,1));
        else
            disp("wrist singularity case has infinite number of solutions")
            q5 = [q5 0];
            q4 = [q4 0]; % assumed an arbitrary value for q4
            q6 = [q6 atan2(T456bis(3,2), T456bis(3,2))];
        end
    end
    
    
else
    disp("only one solution exists for q1")
end

q3 = q3-dq;

q = round(rad2deg([q1; q2; q3; q4; q5; q6]),10);

numberOfSolutions = size(q,2);
