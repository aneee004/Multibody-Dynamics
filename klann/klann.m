
% Kinematic analysis of klann linkage in body-coordinates
% Position, velocity, and accleration analysis from t = 0 to t = 20 s
% Coordinate-Partitioning Method
clc
% Position analysis
q = zeros(21,1);
t = 0:0.001:20;N = size(t,2);
pcoordsall = zeros(21,N);
for i = 1:N
	[phi1,~,~] = driver(t(i));
	q(3) = phi1;
	pcoordsall(:,i) = NR_klann(q,21,1e-14,40,@constraints);
end
% Velocity analysis
qdot = zeros(21,1);
vcoordsall  = zeros(21,N);
for i = 1:N
   [~,phi1dot,~] = driver(t(i));
   qdot(3) = phi1dot;
   [~,D] = constraints(pcoordsall(:,i));
   Dnew = [D(:,1:2) D(:,4:21)];
   rhs = -D(:,3)*qdot(3);
   qv = Dnew\rhs;
   vcoordsall(:,i) = [qv(1:2,1)' qdot(3) qv(3:20,1)']';
end
% Acceleration analysis
qddot = zeros(21,1);
acoordsall  = zeros(21,N);
for i = 1:N
   [~,~,phi1ddot] = driver(t(i));
   qddot(3) = phi1ddot;
   [~,D] = constraints(pcoordsall(:,i));
   Dnew = [D(:,1:2) D(:,4:21)];
   rhs = gamma(pcoordsall(:,i),vcoordsall(:,i))-D(:,3)*qddot(3);
   qa = Dnew\rhs;
   acoordsall(:,i) = [qa(1:2,1)' qddot(3) qa(3:20,1)']';
end
plot(t,acoordsall(19,:))
save klann.mat t pcoordsall vcoordsall acoordsall
function output = gamma(q,qdot)
l1 = 1.1;	% Length of the crank
l2 = 2.88;	% Length of connecting rod1
l3 = 1.3;	% Length of rocker1
l4 = 1.82;	% Length of rocker2
l5 = 2.65;	%Length of leg
l6 = 2.22;	% Extention of the connecting rod
l7 = 4.9;	% Length of extended leg
phi1 = q(3);phi2 = q(6);phi3 = q(9);phi4 = q(12);phi5 = q(15);phi6 = q(18);phi7 = q(21);
phi1dot = qdot(3);phi2dot = qdot(6);phi3dot = qdot(9);phi4dot = qdot(12);phi5dot = qdot(15);phi6dot = qdot(18);phi7dot = qdot(21);
s1_O1 = [-l1/2 0]';	s1_A  = [l1/2 0]';
s2_A  = [l2/2 0]';	s2_B  = [-l2/2 0]';
s3_B  = [l3/2 0]';	s3_O2 = [-l3/2 0]';
s4_O3 = [-l4/2 0]';	s4_C  = [l4/2 0]';
s5_C  = [l5/2 0]';	s5_D  = [-l5/2 0]';
s6_B  = [l6/2 0]';	s6_D  = [-l6/2 0]';
s7_D  = [l7/2 0]';	s7_E  = [-l7 0]';
S1_O1 = A(phi1)*s1_O1;	S1_A  = A(phi1)*s1_A;
S2_A  = A(phi2)*s2_A;	S2_B  = A(phi2)*s2_B;
S3_B  = A(phi3)*s3_B;	S3_O2 = A(phi3)*s3_O2;
S4_O3 = A(phi4)*s4_O3;	S4_C  = A(phi4)*s4_C;
S5_C  = A(phi5)*s5_C;	S5_D  = A(phi5)*s5_D;
S6_B  = A(phi6)*s6_B;	S6_D  = A(phi6)*s6_D;
S7_D  = A(phi7)*s7_D;	S7_E  = A(phi7)*s7_E;

output = [-S1_O1*phi1dot^2;
	S1_A*phi1dot^2-S2_A*phi2dot^2;
	S2_B*phi2dot^2-S3_B*phi3dot^2;
	-S3_O2*phi3dot^2;
	-S4_O3*phi4dot^2;
	S4_C*phi4dot^2-S5_C*phi3dot^2;
	S5_D*phi5dot^2-S6_D*phi6dot^2;
	S2_B*phi2dot^2-S6_B*phi6dot^2;
	0;
	S6_D*phi6dot^2-S7_D*phi6dot^2;
	0
	];
end
function [Phi,D] = constraints(q)
l1 = 1.1;	% Length of the crank
l2 = 2.88;	% Length of connecting rod1
l3 = 1.3;	% Length of rocker1
l4 = 1.82;	% Length of rocker2
l5 = 2.65;	%Length of leg
l6 = 2.22;	% Extention of the connecting rod
l7 = 4.9;	% Length of extended leg
x1 = q(1);	y1 = q(2);	phi1 = q(3);
x2 = q(4);	y2 = q(5);	phi2 = q(6);
x3 = q(7);	y3 = q(8);	phi3 = q(9);
x4 = q(10);	y4 = q(11);	phi4 = q(12);
x5 = q(13);	y5 = q(14);	phi5 = q(15);
x6 = q(16);	y6 = q(17);	phi6 = q(18);
x7 = q(19);	y7 = q(20);	phi7 = q(21);
r1 = [x1 y1]';	r2 = [x2 y2]';	r3 = [x3 y3]';	r4 = [x4 y4]';	r5 = [x5 y5]';	r6 = [x6 y6]';	r7 = [x7 y7]';
O1 = [0 0]';	O2 = [-2.6616 -1.3]';	O3 = [-2.6616 0.6145]';
s1_O1 = [-l1/2 0]';	s1_A  = [l1/2 0]';
s2_A  = [l2/2 0]';	s2_B  = [-l2/2 0]';
s3_B  = [l3/2 0]';	s3_O2 = [-l3/2 0]';
s4_O3 = [-l4/2 0]';	s4_C  = [l4/2 0]';
s5_C  = [l5/2 0]';	s5_D  = [-l5/2 0]';
s6_B  = [l6/2 0]';	s6_D  = [-l6/2 0]';
s7_D  = [l7/2 0]';	s7_E  = [-l7 0]';
S1_O1 = A(phi1)*s1_O1;	S1_A  = A(phi1)*s1_A;
S2_A  = A(phi2)*s2_A;	S2_B  = A(phi2)*s2_B;
S3_B  = A(phi3)*s3_B;	S3_O2 = A(phi3)*s3_O2;
S4_O3 = A(phi4)*s4_O3;	S4_C  = A(phi4)*s4_C;
S5_C  = A(phi5)*s5_C;	S5_D  = A(phi5)*s5_D;
S6_B  = A(phi6)*s6_B;	S6_D  = A(phi6)*s6_D;
S7_D  = A(phi7)*s7_D;	S7_E  = A(phi7)*s7_E;
Phi = [r1+S1_O1 - O1;
	r2+S2_A-r1-S1_A;
	r3+S3_B-r2-S2_B
	r3+S3_O2 - O2;
	r4+S4_O3 - O3;
	r5+S5_C - r4-S4_C;
	r6+S6_D - r5-S5_D;
	r6+S6_B-r2-S2_B;
	phi6-phi2+(pi/18);
	r7+S7_D - r6-S6_D;
	phi7-phi6-(pi/9);
	];
r=[0 -1;1 0];
S1_O1_r = r*S1_O1;
S1_A_r  = r*S1_A;
S2_A_r  = r*S2_A;
S2_B_r  = r*S2_B;
S3_B_r  = r*S3_B;
S3_O2_r = r*S3_O2;
S4_O3_r = r*S4_O3;
S4_C_r  = r*S4_C;
S5_C_r  = r*S5_C;
S5_D_r  = r*S5_D;
S6_D_r  = r*S6_D;
S6_B_r  = r*S6_B;
S7_D_r  = r*S7_D;
D = [eye(2) S1_O1_r zeros(2,18);
	-eye(2) -S1_A_r eye(2) S2_A_r zeros(2,15);
	zeros(2,3) -eye(2) -S2_B_r eye(2) S3_B_r zeros(2,12);
	zeros(2,6) eye(2) S3_O2_r zeros(2,12);
	zeros(2,9) eye(2) S4_O3_r zeros(2,9);
	zeros(2,9) -eye(2) -S4_C_r eye(2) S5_C_r zeros(2,6);
	zeros(2,12) -eye(2) -S5_D_r eye(2) S6_D_r zeros(2,3);
	zeros(2,3) -eye(2) -S2_B_r zeros(2,9) eye(2) S6_B_r zeros(2,3);
	zeros(1,3) 0 0 -1 zeros(1,9) 0 0 1 zeros(1,3);
	zeros(2,15) -eye(2) -S6_D_r eye(2) S7_D_r;
	zeros(1,15) 0 0 -1 0 0 1;
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