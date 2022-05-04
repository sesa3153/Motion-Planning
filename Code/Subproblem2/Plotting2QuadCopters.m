%% House Cleaning
clc
clear all
close all
%% plotting
Path1 = load('test1.txt');
Path2 = load('test2.txt');


[Matx,Maty,Matz] = CreateVertices([5 0 0],1,10,1);
[Matx2,Maty2,Matz2] = CreateVertices([9 0 0],1,10,1);
[Matx3, Maty3,Matz3] = CreateVertices([0,5,0],30,1,6);
[Matx4, Maty4,Matz4] = CreateVertices([0,-5,0],30,1,6);
[Matx5, Maty5,Matz5] = CreateVertices([14 0 0],1,10,6);
[Matx6, Maty6,Matz6] = CreateVertices([0 0 3],28,10,1);
[Matx7, Maty7,Matz7] = CreateVertices([0 0 -3],28,10,1);

[X,Y,Z] = sphere;


figure
plot3(Path1(:,1),Path1(:,3),Path1(:,5),'r','Linewidth',1);
hold on
plot3(Path2(:,1),Path2(:,3),Path2(:,5),'g','Linewidth',1);
scatter3(Path1(end,1),Path1(end,3),Path1(end,5),'filled');
scatter3(Path2(end,1),Path2(end,3),Path2(end,5),'filled');
alpha(surf(X+7, Y,Z),0.1);
alpha(surf(X+7, Y-1,Z),0.1);
alpha(surf(X+12, Y,Z),0.1);
for i = 1:6
    alpha(fill3(Matx(i,:),Maty(i,:),Matz(i,:),'k'),.5);
    alpha(fill3(Matx2(i,:),Maty2(i,:),Matz2(i,:),'k'),.5);
end

xlabel('x');
ylabel('y');
zlabel('z');
legend('Quadcopter 1','Quadcopter 2');
xlim([0 13]);
ylim([-6.5 6.5]);
zlim([-6.5 6.5]);
grid minor