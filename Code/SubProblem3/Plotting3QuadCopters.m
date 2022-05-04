%% House Cleaning
clc;
close all;
clear all;
%% Workspace 1

% Loading Path for each quadcopter
Path1 = load('Path1demo.txt');
Path2 = load('Path2demo.txt');
Path3 = load('Path3demo.txt');

% Obstacles & boundaries
[Matx,Maty,Matz] = CreateVertices([5 0 0],1,10,1);
[Matx2,Maty2,Matz2] = CreateVertices([9 0 0],1,10,1);
[Matx3, Maty3,Matz3] = CreateVertices([0,5,0],30,1,6);
[Matx4, Maty4,Matz4] = CreateVertices([0,-5,0],30,1,6);
[Matx5, Maty5,Matz5] = CreateVertices([14 0 0],1,10,6);
[Matx6, Maty6,Matz6] = CreateVertices([0 0 3],28,10,1);
[Matx7, Maty7,Matz7] = CreateVertices([0 0 -3],28,10,1);

% Sphere Goal Region
[X,Y,Z] = sphere;

% Defining radius
r = 1;

% Creating a circle with a certain radius
X = r*X;
Y = r*Y;
Z = r*Z;


%% Anmation generation for workspace 1 solution

% videoName = 'Map1View2.avi';
% v = VideoWriter(videoName);
% open(v);
% 
% Path1 = flipud(Path1);
% Path2 = flipud(Path2);
% Path3 = flipud(Path3);
% 
%     figure('units','normalized','outerposition',[0 0 1 1])
%     hold on
%     alpha(surf(X+7, Y,Z),0.1);
%     alpha(surf(X+7, Y+2,Z),0.1);
%     alpha(surf(X+7, Y-2,Z),0.1);
%     alpha(surf(X+12,Y,Z),0.1);
%     for i = 1:6
%         alpha(fill3(Matx(i,:),Maty(i,:),Matz(i,:),'k'),.5);
%         alpha(fill3(Matx2(i,:),Maty2(i,:),Matz2(i,:),'k'),.5);
%     end
% for i = 1:length(Path1)
%     %%view([ -sin(2*pi*i/length(Path1)) cos(2*pi*i/length(Path1)) 0.5]); 
%     view([0 0 1]);
%     plot3(Path1(1:i,1),Path1(1:i,3),Path1(1:i,5),'r','LineWidth',1.25);
%     plot3(Path2(1:i,1),Path2(1:i,3),Path2(1:i,5),'g','LineWidth',1.25);
%     if ( i < length(Path3))
%         plot3(Path3(1:i,1),Path3(1:i,3),Path3(1:i,5),'b','LineWidth',1.25);
%     else
%         plot3(Path3(:,1),Path3(:,3),Path3(:,5),'b','LineWidth',1.25);
%     end
%     legend('QuadCopter1','QuadCopter2','QuadCopter3');
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
%     xlim([0 13]);
%     ylim([-6.5 6.5]);
%     zlim([-6.5 6.5]);
%     axis equal
%     grid minor
%     frame = getframe(gcf);
%     writeVideo(v,frame);
%     
% end
% 
% close(v);


%% Workspace 1 solution
figure
plot3(Path1(:,1),Path1(:,3),Path1(:,5),'r','LineWidth',1.25);
hold on
plot3(Path2(:,1),Path2(:,3),Path2(:,5),'g','LineWidth',1.25);
plot3(Path3(:,1),Path3(:,3),Path3(:,5),'b','LineWidth',1.25);
scatter3(Path1(end,1),Path1(end,3),Path1(end,5),'filled');
scatter3(Path2(end,1),Path2(end,3),Path2(end,5),'filled');
scatter3(Path3(end,1),Path3(end,3),Path3(end,5),'filled');
for i = 1:6
    alpha(fill3(Matx(i,:),Maty(i,:),Matz(i,:),'k'),.5);
    alpha(fill3(Matx2(i,:),Maty2(i,:),Matz2(i,:),'k'),.5);
end

alpha(surf(X+7, Y,Z),0.1);
alpha(surf(X+7, Y+2,Z),0.1);
alpha(surf(X+7, Y-2,Z),0.1);
alpha(surf(X+12,Y,Z),0.1);
xlabel('x');
ylabel('y');
zlabel('z');
xlim([0 13]);
ylim([-6.5 6.5]);
zlim([-6.5 6.5]);
legend('QuadCopter1','QuadCopter2','QuadCopter3');
grid minor

%% Benchmarking data processing for workspace1

data1 = load('BenchmarkMap1.txt');
Quad1Success = 0;
Quad1Psuccess = 0;

Quad2Success = 0;
Quad2Psuccess = 0;

Quad3Success = 0;
Quad3Psuccess = 0;

QuadAllsuccess = 0;

QuadAllPsuccess =0;

for i = 1:100
    
        % Number of successes/Partial successes for each quadcopter
        if (data1(i,4) == 1)
            Quad1Success = Quad1Success + 1;
        elseif (data1(i,4) == 0.5)
            Quad1Psuccess = Quad1Psuccess + 1;
        end
        
        if (data1(i,5) == 1)
            Quad2Success = Quad2Success + 1;
        elseif (data1(i,5) == 0.5)
            Quad2Psuccess = Quad2Psuccess + 1;
        end
        
        if (data1(i,6) == 1)
            Quad3Success = Quad3Success + 1;
        elseif (data1(i,6) == 0.5)
            Quad3Psuccess = Quad3Psuccess + 1;
        end
        % Number of times all 3 quadcopters achieve goals
        if (data1(i,4) == 1 && data1(i,5) == 1 && data1(i,6) == 1)
            QuadAllsuccess = QuadAllsuccess + 1;
        end
        % Number of times all 3 quadcopters achieve partial goals
        if (data1(i,4) > 0 && data1(i,5) > 0 && data1(i,6) > 0)
            QuadAllPsuccess = QuadAllPsuccess + 1;
        end
end

% Removing path lengths ==0 as those correspond to when solutions were not
% found
QuadLength1 = data1(:,1);
indices = find(QuadLength1==0);
QuadLength1(indices) = [];

QuadLength2 = data1(:,2);
indices = find(QuadLength2 ==0);
QuadLength2(indices) = [];

QuadLength3 = data1(:,3);
indices = find(QuadLength3 == 0);
QuadLength3(indices) = [];

C = [QuadLength1; QuadLength2; QuadLength3];

grp = [ones(length(QuadLength1),1); 2.*ones(length(QuadLength2),1); ...
    3.*ones(length(QuadLength3),1)];

% Plotting
figure
bar([Quad1Success Quad2Success Quad3Success]);
ylim([0 100]);
title('Rate of Success');
ylabel('Number of Successful Goal Completions (W1)')
grid on
saveas(gcf,'RoSW1.png');

figure
bar([Quad1Psuccess Quad2Psuccess Quad3Psuccess]);
ylim([0 100]);
title('Rate of Partial Success (W1)');
ylabel('Number of Partial Completions')
grid on
saveas(gcf,'RoPSW1.png');

figure
boxplot(C,grp);
grid on
title('Path Length (W1)');
ylabel('Length');
saveas(gcf,'PathLW1.png');

figure
boxplot(data1(:,end));
grid on
title('Planner Time (W1)');
saveas(gcf,'TimeW1.png');
%% Workspace 2 plot

% Loading Solutions for all quadcopters
Path1W2 = load('Path1W2demo2.txt');
Path2W2 = load('Path2W2demo2.txt');
Path3W2 = load('Path3W2demo2.txt');

% Defining goal region
[X,Y,Z] = sphere;

% Defining radius of the goal region
r = 1.5;

X = r*X;
Y = r*Y;
Z = r*Z;

% defining obstacle centers for plotting
x = [4.5 4.5 2 2];
y = [-2 2 4.5 -4.5];
z = [0 0 0 0];

% Creating obstacle surfaces
[Matx(:,:,1),Maty(:,:,1),Matz(:,:,1)] = CreateVertices([x(1) y(1) z(1)],6,1,6); 
[Matx(:,:,2),Maty(:,:,2),Matz(:,:,2)] = CreateVertices([x(2) y(2) z(2)],6,1,6); 
[Matx(:,:,3),Maty(:,:,3),Matz(:,:,3)] = CreateVertices([x(3) y(3) z(3)],1,6,6); 
[Matx(:,:,4),Maty(:,:,4),Matz(:,:,4)] = CreateVertices([x(4) y(4) z(4)],1,6,6); 

% Plotting
figure
plot3(Path1W2(:,1),Path1W2(:,3), Path1W2(:,5),'r','LineWidth',1);
hold on
plot3(Path2W2(:,1),Path2W2(:,3), Path2W2(:,5),'g','LineWidth',1);
plot3(Path3W2(:,1),Path3W2(:,3), Path3W2(:,5),'b','LineWidth',1);
for i = 1:4 
    for j = 1:6
        alpha(fill3(Matx(j,:,i),Maty(j,:,i),Matz(j,:,i),'k'),0.5);
        hold on;
    end
end
scatter3(Path1W2(end,1),Path1W2(end,3),Path1W2(end,5),'filled');
scatter3(Path2W2(end,1),Path2W2(end,3),Path2W2(end,5),'filled');
scatter3(Path3W2(end,1),Path3W2(end,3),Path3W2(end,5),'filled');
alpha(surf(X+6, Y,Z),0.1);
alpha(surf(X, Y+6,Z),0.1);
alpha(surf(X, Y-6,Z),0.1);
alpha(surf(X, Y,Z),0.1);
xlabel('x');
ylabel('y');
zlabel('z');
legend('Quadcopter 1','Quadcopter 2','Quadcopter 3');
axis equal
grid minor
%% Workspace 2 solution animation
% videoName = 'Map2.avi';
% v = VideoWriter(videoName);
% open(v);
% 
% Path1W2 = flipud(Path1W2);
% Path2W2 = flipud(Path2W2);
% Path3W2 = flipud(Path3W2);
% 
%     figure('units','normalized','outerposition',[0 0 1 1])
%     hold on
%     alpha(surf(X+6, Y,Z),0.05);
%     alpha(surf(X, Y+6,Z),0.05);
%     alpha(surf(X, Y-6,Z),0.05);
%     alpha(surf(X, Y,Z),0.05);
%     for i = 1:4 
%         for j = 1:6
%             alpha(fill3(Matx(j,:,i),Maty(j,:,i),Matz(j,:,i),'k'),0.4);
%             hold on;
%         end
%     end
% for i = 1:length(Path3W2)
%     view([ -sin(2*pi*i/length(Path3W2)) cos(2*pi*i/length(Path3W2)) 0.75]); 
% %     view([ 0 0 1]);
%     if ( i<length(Path1W2))
%         plot3(Path1W2(1:i,1),Path1W2(1:i,3),Path1W2(1:i,5),'r','LineWidth',1.25);
%     else
%         plot3(Path1W2(1:end,1),Path1W2(1:end,3),Path1W2(1:end,5),'r','LineWidth',1.25);
%     end
%     if (i <length(Path2W2))
%         plot3(Path2W2(1:i,1),Path2W2(1:i,3),Path2W2(1:i,5),'g','LineWidth',1.25);
%     else 
%         plot3(Path2W2(1:end,1),Path2W2(1:end,3),Path2W2(1:end,5),'g','LineWidth',1.25);
%     end
%     
%     if ( i < length(Path3W2))
%         plot3(Path3W2(1:i,1),Path3W2(1:i,3),Path3W2(1:i,5),'b','LineWidth',1.25);
%     else
%         plot3(Path3W2(:,1),Path3W2(:,3),Path3W2(:,5),'b','LineWidth',1.25);
%     end
%     
%     
%     legend('QuadCopter1','QuadCopter2','QuadCopter3');
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
%     xlim([0 13]);
%     axis equal
%     grid minor
%     frame = getframe(gcf);
%     writeVideo(v,frame);
%     
% end
% 
% close(v);
%% Benchmarking data for workspace2 (same descriptions for benchmarking data for workspace 1 apply)
data2 = load('BenchmarkMap2.txt');

Quad1Success = 0;
Quad1Psuccess = 0;

Quad2Success = 0;
Quad2Psuccess = 0;

Quad3Success = 0;
Quad3Psuccess = 0;

QuadAllsuccess2 = 0;
QuadAllPsuccess2 = 0;
for i = 1:length(data2)
        if (data2(i,4) == 1)
            Quad1Success = Quad1Success + 1;
        elseif (data2(i,4) == 0.5)
            Quad1Psuccess = Quad1Psuccess + 1;
        end
        
        if (data2(i,5) == 1)
            Quad2Success = Quad2Success + 1;
        elseif (data2(i,5) == 0.5)
            Quad2Psuccess = Quad2Psuccess + 1;
        end
        
        if (data2(i,6) == 1)
            Quad3Success = Quad3Success + 1;
        elseif (data2(i,6) == 0.5)
            Quad3Psuccess = Quad3Psuccess + 1;
        end
        
        if (data2(i,4) == 1 && data2(i,5) == 1 && data2(i,6) == 1)
            QuadAllsuccess2 = QuadAllsuccess2 + 1;
        end
        if (data2(i,4) > 0 && data2(i,5) > 0 && data2(i,6) > 0)
            QuadAllPsuccess2 = QuadAllPsuccess2 + 1;
        end
end

QuadLength1 = data2(:,1);
indices = find(QuadLength1==0);
QuadLength1(indices) = [];

QuadLength2 = data2(:,2);
indices = find(QuadLength2 ==0);
QuadLength2(indices) = [];

QuadLength3 = data2(:,3);
indices = find(QuadLength3 == 0);
QuadLength3(indices) = [];

C = [QuadLength1; QuadLength2; QuadLength3];

grp = [ones(length(QuadLength1),1); 2.*ones(length(QuadLength2),1); ...
    3.*ones(length(QuadLength3),1)];

figure
bar([Quad1Success Quad2Success Quad3Success]);
ylim([0 100]);
title('Rate of Success');
ylabel('Number of Successful Goal Completions (W2)')
grid on
saveas(gcf,'RoSW2.png');

figure
bar([Quad1Psuccess Quad2Psuccess Quad3Psuccess]);
ylim([0 100]);
title('Rate of Partial Success (W2)');
ylabel('Number of Partial Completions')
grid on
saveas(gcf,'RoPSW2.png');

figure
boxplot(C,grp);
grid on
title('Path Length (W2)');
ylabel('Length');
saveas(gcf,'PathLW2.png');

figure
boxplot(data2(:,end));
grid on
title('Planner Time (W2)');
saveas(gcf,'TimeW2.png');

%% Workspace 3

% Loading solutions for all quadcopters
Path1W3 = load('Path1W3demo.txt');
Path2W3 = load('Path2W3demo.txt');
Path3W3 = load('Path3W3demo.txt');

% defining the spherical goal region
[X,Y,Z] = sphere;

% defining the radius
r = 1;

X = r*X;
Y = r*Y;
Z = r*Z;

% defining obstacle centers
x = [6.5 6.5];
y = [-1.75 1.75];
z = [0 0];

% Creating Obstacle surfaces
[Matx(:,:,1),Maty(:,:,1),Matz(:,:,1)] = CreateVertices([x(1) y(1) z(1)],10,1,5); 
[Matx(:,:,2),Maty(:,:,2),Matz(:,:,2)] = CreateVertices([x(2) y(2) z(2)],10,1,5); 


% Plotting
figure
plot3(Path1W3(:,1),Path1W3(:,3), Path1W3(:,5),'r','LineWidth',1);
hold on
plot3(Path2W3(:,1),Path2W3(:,3), Path2W3(:,5),'g','LineWidth',1);
plot3(Path3W3(:,1),Path3W3(:,3), Path3W3(:,5),'b','LineWidth',1);
for i = 1:2 
    for j = 1:6
        alpha(fill3(Matx(j,:,i),Maty(j,:,i),Matz(j,:,i),'k'),0.5);
        hold on;
    end
end
alpha(surf(X+9, Y,Z),0.1);
alpha(surf(X, Y,Z),0.1);
scatter3(Path1W3(end,1), Path1W3(end,3), Path1W3(end,5),'filled');
scatter3(Path2W3(end,1), Path2W3(end,3), Path2W3(end,5),'filled');
scatter3(Path3W3(end,1), Path3W3(end,3), Path3W3(end,5),'filled');
xlabel('x');
ylabel('y');
zlabel('z');
legend('Quadcopter 1','Quadcopter 2','Quadcopter 3');
axis equal
grid minor
%% Benchmarking for workspace 3

data3 = load('BenchmarkMap3.txt');

Quad1Success = 0;
Quad1Psuccess = 0;

Quad2Success = 0;
Quad2Psuccess = 0;

Quad3Success = 0;
Quad3Psuccess = 0;

QuadAllsuccess3 = 0;
QuadAllPsuccess3 = 0;

for i = 1:length(data3)
        if (data3(i,4) == 1)
            Quad1Success = Quad1Success + 1;
        elseif (data3(i,4) == 0.5)
            Quad1Psuccess = Quad1Psuccess + 1;
        end
        
        if (data3(i,5) == 1)
            Quad2Success = Quad2Success + 1;
        elseif (data3(i,5) == 0.5)
            Quad2Psuccess = Quad2Psuccess + 1;
        end
        
        if (data3(i,6) == 1)
            Quad3Success = Quad3Success + 1;
        elseif (data3(i,6) == 0.5)
            Quad3Psuccess = Quad3Psuccess + 1;
        end
        
        if (data3(i,4) == 1 && data3(i,5) == 1 && data3(i,6) == 1)
            QuadAllsuccess3 = QuadAllsuccess3 + 1;
        end
        
        if (data3(i,4) > 0 && data3(i,5) > 0 && data3(i,6) > 0)
            QuadAllPsuccess3 = QuadAllPsuccess3 + 1;
        end
        
end

QuadLength1 = data3(:,1);
indices = find(QuadLength1==0);
QuadLength1(indices) = [];

QuadLength2 = data3(:,2);
indices = find(QuadLength2 ==0);
QuadLength2(indices) = [];

QuadLength3 = data3(:,3);
indices = find(QuadLength3 == 0);
QuadLength3(indices) = [];

C = [QuadLength1; QuadLength2; QuadLength3];

grp = [ones(length(QuadLength1),1); 2.*ones(length(QuadLength2),1); ...
    3.*ones(length(QuadLength3),1)];

figure
bar([Quad1Success Quad2Success Quad3Success]);
ylim([0 100]);
title('Rate of Success');
ylabel('Number of Successful Goal Completions (W3)')
grid on
saveas(gcf,'RoSW3.png');

figure
bar([Quad1Psuccess Quad2Psuccess Quad3Psuccess]);
ylim([0 100]);
title('Rate of Partial Success (W3)');
ylabel('Number of Partial Completions')
grid on
saveas(gcf,'RoPSW3.png');

figure
boxplot(C,grp);
grid on
title('Path Length (W3)');
ylabel('Length');
saveas(gcf,'PathLW3.png');

figure
boxplot(data3(:,end));
grid on
title('Planner Time (W3)');
saveas(gcf,'TimeW3.png');

%% Creating solution animation for workspace 3
% videoName = 'Map3View1.avi';
% v = VideoWriter(videoName);
% open(v);
% 
% Path1W3 = flipud(Path1W3);
% Path2W3 = flipud(Path2W3);
% Path3W3 = flipud(Path3W3);
% 
%     figure('units','normalized','outerposition',[0 0 1 1])
%     hold on
%     for i = 1:2 
%         for j = 1:6
%             alpha(fill3(Matx(j,:,i),Maty(j,:,i),Matz(j,:,i),'k'),0.5);
%             hold on;
%         end
%     end
%     alpha(surf(X+9, Y,Z),0.1);
%     alpha(surf(X, Y,Z),0.1);
% for i = 1:length(Path2W3)
%     view([ -sin(2*pi*i/length(Path3W3)) cos(2*pi*i/length(Path3W3)) 0.75]); 
% %     view([ 0 0 1]);
%     if ( i<length(Path1W3))
%         plot3(Path1W3(1:i,1),Path1W3(1:i,3),Path1W3(1:i,5),'r','LineWidth',1.25);
%     else
%         plot3(Path1W3(1:end,1),Path1W3(1:end,3),Path1W3(1:end,5),'r','LineWidth',1.25);
%     end
%     if (i <length(Path2W3))
%         plot3(Path2W3(1:i,1),Path2W3(1:i,3),Path2W3(1:i,5),'g','LineWidth',1.25);
%     else 
%         plot3(Path2W3(1:end,1),Path2W3(1:end,3),Path2W3(1:end,5),'g','LineWidth',1.25);
%     end
%     
%     if ( i < length(Path3W3))
%         plot3(Path3W3(1:i,1),Path3W3(1:i,3),Path3W3(1:i,5),'b','LineWidth',1.25);
%     else
%         plot3(Path3W3(:,1),Path3W3(:,3),Path3W3(:,5),'b','LineWidth',1.25);
%     end
%     
%     
%     legend('QuadCopter1','QuadCopter2','QuadCopter3');
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
%     xlim([0 13]);
%     axis equal
%     grid minor
%     frame = getframe(gcf);
%     writeVideo(v,frame);
%     
% end
% 
% close(v);