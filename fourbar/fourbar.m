function fourbar

% Kinematic analysis of fourbar mechanism in body-coordinates
% Position, velocity, and accleration analysis from t = 0 to t = 20 s
% Coordinate-Partitioning Method
clc
% Position analysis
q = zeros(9,1);
t = 0:0.001:20;N = size(t,2);
pcoordsall = zeros(9,N);
for i = 1:N
    [phi1,~,~] = driver(t(i));
    q(3) = phi1;
    pcoordsall(:,i) = NR_fourbar(q,9,1e-14,40,@constraints);
end

% Velocity analysis
qdot = zeros(9,1);
vcoordsall  = zeros(9,N);
for i = 1:N
   [~,phi1dot,~] = driver(t(i));
   qdot(3) = phi1dot;
   [~,D] = constraints(pcoordsall(:,i));
   Dnew = [D(:,1:2) D(:,4:9)];
   rhs = -D(:,3)*qdot(3);
   qv = Dnew\rhs;
   vcoordsall(:,i) = [qv(1:2,1)' qdot(3) qv(3:8,1)']';
end
% Acceleration analysis
qddot = zeros(9,1);
acoordsall  = zeros(9,N);
for i = 1:N
   [~,~,phi1ddot] = driver(t(i));
   qddot(3) = phi1ddot;
   [~,D] = constraints(pcoordsall(:,i));
   Dnew = [D(:,1:2) D(:,4:9)];
   rhs = gamma(pcoordsall(:,i),vcoordsall(:,i))-D(:,3)*qddot(3);
   qa = Dnew\rhs;
   acoordsall(:,i) = [qa(1:2,1)' qddot(3) qa(3:8,1)']';
end
plot(t,acoordsall(7,:))
save 4rkadata.mat t pcoordsall vcoordsall acoordsall
end


function output = gamma(q,qdot)
l1 = 2.16;% Length of the crank
l2 = 4.85;% Length of connecting rod
l3 = 5.81; % Length of arm
l4 = 8.30; % ground link
phi1 = q(3);phi2 = q(6);phi3 = q(9);
phi1dot = qdot(3);phi2dot = qdot(6);phi3dot = qdot(9);
s_1_O = [-l1/2 0]';s_1_A = [l1/2 0]';
s_2_A = [-l2/2 0]';s_2_B = [l2/2 0]';
s_3_B = [l3/2 0]';s_3_Q = [-l3/2 0]';
S_1_O = A(phi1)*s_1_O;S_1_A = A(phi1)*s_1_A;
S_2_A = A(phi2)*s_2_A;S_2_B = A(phi2)*s_2_B;
S_3_B = A(phi3)*s_3_B;S_3_Q = A(phi3)*s_3_Q;

output = [S_1_O*phi1dot^2;
    -S_1_A*phi1dot^2+S_2_A*phi2dot^2;
    -S_2_B*phi2dot^2+S_3_B*phi3dot^2;
    -S_3_Q*phi3dot^2
	];
end

function [Phi,D] = constraints(q)
l1 = 2.16;% Length of the crank
l2 = 4.85;% Length of connecting rod
l3 = 5.81; % Length of arm
l4 = 8.30; % ground link
x1 = q(1);y1 = q(2);phi1 = q(3);
x2 = q(4);y2 = q(5);phi2 = q(6);
x3 = q(7);y3 = q(8);phi3 = q(9);
r1 = [x1 y1]';r2 = [x2 y2]';r3 = [x3 y3]';Q=[l4 0]';
s_1_O = [-l1/2 0]';s_1_A = [l1/2 0]';
s_2_A = [-l2/2 0]';s_2_B = [l2/2 0]';
s_3_B = [l3/2 0]';s_3_Q = [-l3/2 0]';
S_1_O = A(phi1)*s_1_O;S_1_A = A(phi1)*s_1_A;
S_2_A = A(phi2)*s_2_A;S_2_B = A(phi2)*s_2_B;
S_3_B = A(phi3)*s_3_B;S_3_Q = A(phi3)*s_3_Q;
Phi = [r1+S_1_O;
    r2+S_2_A-r1-S_1_A;
    r3+S_3_B-r2-S_2_B;
    r3+S_3_Q-Q];
S_1_O_r = [0 -1;1 0]*S_1_O;
S_1_A_r = [0 -1;1 0]*S_1_A;
S_2_A_r = [0 -1;1 0]*S_2_A;
S_2_B_r = [0 -1;1 0]*S_2_B;
S_3_B_r = [0 -1;1 0]*S_3_B;
S_3_Q_r = [0 -1;1 0]*S_3_Q;
D = [eye(2) S_1_O_r zeros(2,6);
    -eye(2) -S_1_A_r eye(2) S_2_A_r zeros(2,3);
    zeros(2,3) -eye(2) -S_2_B_r eye(2) S_3_B_r;
    zeros(2,6) eye(2) S_3_Q_r
    ];
end

function [phi1,phi1dot,phi1ddot] = driver(t)
if t<=1
    phi1 = pi/3+t^3-t^4/2;
    phi1dot = 3*t^2-2*t^3;
    phi1ddot = 6*t-6*t^2;
else
    phi1 = pi/3+1-1/2+1*(t-1);
    phi1dot = 1;
    phi1ddot = 0;
end
end

function output = A(phi)
output = [cos(phi) -sin(phi);sin(phi) cos(phi)];
end