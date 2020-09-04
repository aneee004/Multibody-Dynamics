clc
clear all
close all
load 4rkadata
l1 = 2.61;
l2 = 4.85;
l3 = 5.81;
l4 = 8.30;


figure
axis([-3 9 -3 7]);
line([0 l4],[0 0],'Linewidth',3,'Color','y');

N = size(t,2);
crack = [];
crod = [];
arm = [];
Apos = [];
Bpos = [];
Qpos = [];
Opos = [];
timedisplay = [];

phi1 = pcoordsall(3,1);	phi2 = pcoordsall(6,1); phi3 = pcoordsall(9,1);
xA = l1*cos(phi1);		yA = l1*sin(phi1);
xB = xA + l2*cos(phi2);	yB = yA + l2*sin(phi2);

crank = line([0 xA],[0 yA],'LineWidth',3,'Color','r');
crod = line([xA xB],[yA yB],'LineWidth',3,'Color','b');
arm = line([xB l4],[yB 0],'LineWidth',3,'Color','g');
Apos = rectangle('Position',[xA-0.1,yA-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
Bpos = rectangle('Position',[xB-0.1,yB-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
Qpos = rectangle('Position',[l4-0.1,-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
Opos = rectangle('Position',[-0.1,-0.1,0.2,0.2],'Curvature',[1,1],'FaceColor','k');
timedisplay = text(1,1,num2str(t(1)));

for i = 1:10:N
    set(crank,'xdata',[0 xA],'ydata',[0 yA]);
    set(crod,'xdata',[xA xB],'ydata',[yA yB]);
    set(arm,'xdata',[xB l4],'ydata',[yB 0]);
    set(Apos,'Position',[xA-0.1,yA-0.1,0.2,0.2]);
    set(Bpos,'Position',[xB-0.1,yB-0.1,0.2,0.2]);
    set(timedisplay,'Position',[1 1],'string',num2str(t(i)));
    drawnow();
    phi1 = pcoordsall(3,i);phi2 = pcoordsall(6,i);
    xA = l1*cos(phi1);		yA = l1*sin(phi1);
    xB = xA + l2*cos(phi2);	yB = yA + l2*sin(phi2);
    pause(0.001);
end
