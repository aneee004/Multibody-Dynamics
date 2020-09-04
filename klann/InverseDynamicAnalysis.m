% InverseDynamic Analysys
clc
load klann
t = 0:0.001:20;N = size(t,2);
L = zeros(21,N);
rhs=L;

for i =1:N
	l1 = 1.1;	% Length of the crank
	l2 = 2.88;	% Length of connecting rod1
	l3 = 1.3;	% Length of rocker1
	l4 = 1.82;	% Length of rocker2
	l5 = 2.65;	%Length of leg
	l6 = 2.22;	% Extention of the connecting rod
	l7 = 4.9;	% Length of extended leg
	m1=1;m2=1;m3=1;m4=1;m5=1;m6=1;m7=1;
	J1=l1^2/3;J2=l2^2/3;J3=l3^2/3;J4=l4^2/3;J5=l5^2/3;J6=l6^2/3;J7=l7^2/3;
	M=[m1*eye(2) zeros(2,19);
		zeros(1,2) J1 zeros(1,18);
		zeros(2,3) m2*eye(2) zeros(2,16);
		zeros(1,5) J2 zeros(1,15);
		zeros(2,6) m3*eye(2) zeros(2,13);
		zeros(1,8) J3 zeros(1,12);
		zeros(2,9) m4*eye(2) zeros(2,10);
		zeros(1,11) J4 zeros(1,9);
		zeros(2,12) m5*eye(2) zeros(2,7);
		zeros(1,14) J5 zeros(1,6);
		zeros(2,15) m6*eye(2) zeros(2,4);
		zeros(1,17) J6 zeros(1,3);
		zeros(2,18) m7*eye(2) zeros(2,1);
		zeros(1,20) J7;
		];
	g=-9.81;
	h=[0 m1*g 0 0 m2*g 0 0 m3*g 0 0 m4*g 0 0 m5*g 0 0 m6*g 0 0 m7*g 0]'; % weight
	[phi1,phi1dot,phi1ddot]=driver(t(i));
	dD=[zeros(1,2) 1 zeros(1,18)];
	[~,cD]=constraints(pcoordsall(:,i));
	D=[cD; dD];
	rhs(:,i)=(M*acoordsall(:,i) - h);
	L(:,i)=inv(D')*rhs(:,i); % Lagrange Multipliers
end
close all
figure
plot(t,L(1,:),'b');
hold on
plot(t,L(2,:),'r');
hold on
plot(t,L(3,:),'--k');
hold on
xlabel('time')
ylabel('\lambda')
title('Multipliers')
legend('Fx','Fy','M')
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
	phi1 = -(pi/3+t^3-t^4/2);
	phi1dot = -(3*t^2-2*t^3);
	phi1ddot = -(6*t-6*t^2);
else
	phi1 = -(pi/3+1-1/2+1*(t-1));
	phi1dot = -1;
	phi1ddot = 0;
end
end
function output = A(phi)
output = [cos(phi) -sin(phi);sin(phi) cos(phi)];
end