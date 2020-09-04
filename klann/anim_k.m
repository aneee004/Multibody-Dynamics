clc
clear all
close all

load klann
    l1 = 1.1;   % Length of the crank
    l2 = 2.88;  % Length of connecting rod1
    l3 = 1.3;   % Length of rocker1
    l4 = 1.82;  % Length of rocker2
    l5 = 2.65;  %Length of leg
    l6 = 2.22;  % Extention of the connecting rod
    l7 = 4.9;   % Length of extended leg

figure
axis square
axis([-7 1.5 -6 3]);
line([0 -2.6616],[0 -1.3],'LineWidth',3,'Color','y');
line([0 -2.6616],[0 0.6145],'LineWidth',3,'Color','y');
line([-2.6616 -2.6616],[-1.3 0.6145],'LineWidth',3,'Color','y');

N = size(t,2);
crank = [];
crod1 = [];
rock1 = [];
rock2 = [];
crod2 = [];
leg1 = [];
leg2 = [];
Apos = [];
Bpos = [];
Cpos = [];
Dpos = [];
Epos = [];
timedisplay = [];

phi1 = pcoordsall(3,1);
phi2 = pcoordsall(6,1);
phi3 = pcoordsall(9,1);
phi4 = pcoordsall(12,1);
phi5 = pcoordsall(15,1);
phi6 = pcoordsall(18,1);
phi7 = pcoordsall(21,1);

xO1 = 0;yO1 = 0;
xO2 = -2.6616;yO2 = -1.3;
xO3 = -2.6616;yO3 = 0.6145;
xA = l1*cos(phi1);yA = l1*sin(phi1);
xB = xA - l2*cos(phi2);yB = yA - l2*sin(phi2);
xC = -2.6616 + l4*cos(phi4);yC = 0.6145 + l4*sin(phi4);
xD = xB - l6*cos(phi6);yD = yB - l6*sin(phi6);
xE = xD + l7*sin(phi7);yE = yD - l7*cos(phi7);


% baseline1 = line([0 xO2],[0 yO2],'LineWidth',3,'Color','y');
% baseline2 = line([0 xO3],[0 yO3],'LineWidth',3,'Color','y');
% baseline3 = line([xO2 xO3],[yO2 yO3],'LineWidth',3,'Color','y');
O1pos = rectangle('Position',[xO1-0.1,yO1-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
O2pos = rectangle('Position',[xO2-0.1,yO2-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
O3pos = rectangle('Position',[xO3-0.1,yO3-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
crank = line([0 xA],[0 yA],'LineWidth',3,'Color','r');
crod1 = line([xA xB],[yA yB],'LineWidth',3,'Color','b');
rock1 = line([xO2 xB],[yO2 yB],'LineWidth',3,'Color','g');
rock2 = line([xO3 xC],[yO3 yC],'LineWidth',3,'Color','g');
crod2 = line([xD xB],[yD yB],'LineWidth',3,'Color','b');
leg1 = line([xC xD],[yC yD],'LineWidth',3,'Color','c');
leg2 = line([xC xD],[yC yD],'LineWidth',3,'Color','c');
Apos = rectangle('Position',[xA-0.1,yA-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
Bpos = rectangle('Position',[xB-0.1,yB-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
Cpos = rectangle('Position',[xC-0.1,yC-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
Dpos = rectangle('Position',[xD-0.1,yD-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
Epos = rectangle('Position',[xE-0.1,yE-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
O1pos = rectangle('Position',[xO1-0.1,yO1-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
O2pos = rectangle('Position',[xO2-0.1,yO2-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
O3pos = rectangle('Position',[xO3-0.1,yO3-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
timedisplay = text(1,1,num2str(t(1)));

for i = 1:10:N
    set(crank,'xdata',[0 xA],'ydata',[0 yA]);
    set(crod1,'xdata',[xA xB],'ydata',[yA yB]);
    set(rock1,'xdata',[xO2 xB],'ydata',[yO2 yB]);
    set(rock2,'xdata',[xO3 xC],'ydata',[yO3 yC]);
    set(crod2,'xdata',[xD xB],'ydata',[yD yB]);
    set(leg1,'xdata',[xC xD],'ydata',[yC yD]);
    set(leg2,'xdata',[xD xE],'ydata',[yD yE]);
    set(Apos,'Position',[xA-0.1,yA-0.1,0.2,0.2]);
    set(Bpos,'Position',[xB-0.1,yB-0.1,0.2,0.2]);
    set(Cpos,'Position',[xC-0.1,yC-0.1,0.2,0.2]);
    set(Dpos,'Position',[xD-0.1,yD-0.1,0.2,0.2]);
    set(Epos,'Position',[xE-0.1,yE-0.1,0.2,0.2]);
    set(timedisplay,'Position',[1 1],'string',num2str(t(i)));
    drawnow();
phi1 = pcoordsall(3,i);
phi2 = pcoordsall(6,i);
phi3 = pcoordsall(9,i);
phi4 = pcoordsall(12,i);
phi5 = pcoordsall(15,i);
phi6 = pcoordsall(18,i);
phi7 = pcoordsall(21,i);
xA = l1*cos(phi1);yA = l1*sin(phi1);
xB = xA - l2*cos(phi2);yB = yA - l2*sin(phi2);
xC = -2.6616 + l4*cos(phi4);yC = 0.6145 + l4*sin(phi4);
xD = xB - l6*cos(phi6);yD = yB - l6*sin(phi6);
xE = xD + l7*sin(phi7);yE = yD - l7*cos(phi7);

    pause(0.001);
end