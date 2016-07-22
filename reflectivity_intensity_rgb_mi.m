clearvars
clc
close all
%%
scan1 = 1100;
scan2 = 1115;
scanName1 = sprintf('%s/ScanIntensity/Scan_for_MI_Intensity_%04d.mat','D:\data',scan1);
M1 = load(scanName1);
I1 = M1.points(2:size(M1.points,1),5);
R1 = M1.points(2:size(M1.points,1),4);
RGB1 = M1.points(2:size(M1.points,1),6:8);
pts1 = pointCloud(M1.points(2:size(M1.points,1),1:3));
pts1.Normal = pcnormals(pts1,15);
%plane1 = planefit3D(pts1);
[ptc1,~,plane1,p1,outliers1] = gp(pts1);
nptc1=pts1;
xlimit1=nptc1.XLimits(1,2)-nptc1.XLimits(1,1);
ylimit1=nptc1.YLimits(1,2)-nptc1.YLimits(1,1);
scanName2 = sprintf('%s/ScanIntensity/Scan_for_MI_Intensity_%04d.mat','D:\data',scan2);
M2 = load(scanName2);
I2 = M2.points(2:size(M2.points,1),5);
R2 = M2.points(2:size(M2.points,1),4);
RGB2 = M2.points(2:size(M2.points,1),6:8);
pts2 = pointCloud(M2.points(2:size(M2.points,1),1:3));
pts2.Normal = pcnormals(pts2,15);
[ptc2,~,plane2,p2,outliers2] = gp(pts2);
%plane2 = planefit3D(pts2);
nptc2=pts2;
xlimit2 = nptc2.XLimits(1,2)-nptc2.XLimits(1,1); 
ylimit2 = nptc2.YLimits(1,2)-nptc2.YLimits(1,1);
xlimit = roundn(max(xlimit1,xlimit2),1);
ylimit = roundn(max(ylimit1,ylimit2),1);
%%
[R12,eul1] = vector2rot(plane1,plane2);
t = R12*mean(pts1.Location)'-mean(pts2.Location)';
%%
close all
Ref1 = R1;
Ref2 = R2;
int1 = I1;
int2 = I2;
rgb1 = RGB1;
rgb2 = RGB2;
map = [1,1,1
    0.8 0.9 0
    0 0.8 0.8
    0.2, 0.1, 0.5
    0.1, 0.5, 0.8
    0.2, 0.7, 0.6
    0.8, 0.7, 0.3
    0.9, 1, 0
    1 0 0];
mymap = [1 1 1 ;1 1 0;1 0 0;0 1 1;1 0 1;  0 0 1;0 1 0;0 0 0];
[~,SB1] = grid2DR(nptc1.Location,xlimit,ylimit);
figure
imagesc(SB1)
colormap(map)
title('Z-Variance')
xlabel('Scan 1')
set(gca,'FontSize',25)
[~,SB1R] = grid2DR(nptc2.Location,xlimit,ylimit);
figure
imagesc(SB1R)
colormap(map)
title('Z-Variance')
xlabel('Scan 2')
set(gca,'FontSize',25)
[~,SN1] = grid_normalR(nptc1.Location,nptc1.Normal,xlimit,ylimit);
figure
imagesc(SN1)
colormap(map)
title('Normal-Variance')
xlabel('Scan 1')
set(gca,'FontSize',25)
[~,SN1R] = grid_normalR(nptc2.Location,nptc2.Normal,xlimit,ylimit);
figure
imagesc(SN1R)
colormap(map)
title('Normal-Variance')
xlabel('Scan 2')
set(gca,'FontSize',25)
[~,SRB1] = grid_reflectivityR(nptc1.Location,Ref1,xlimit,ylimit);
figure
imagesc(SRB1)
colormap(map)
title('Reflectivity-Variance')
xlabel('Scan 1')
set(gca,'FontSize',25)
[~,SRB1R] = grid_reflectivityR(nptc2.Location,Ref2,xlimit,ylimit);
figure
imagesc(SRB1R)
colormap(map)
title('Reflectivity-Variance')
xlabel('Scan 2')
set(gca,'FontSize',25)
[~,Sib1] = grid_intensityR(nptc1.Location,int1,xlimit,ylimit);
figure
imagesc(Sib1)
colormap(map)
title('Intenstiy-Variance')
xlabel('Scan 1')
set(gca,'FontSize',25)
[~,Sib1R] = grid_intensityR(nptc2.Location,int2,xlimit,ylimit);
figure
imagesc(Sib1R)
colormap(map)
title('Intensity-Variance')
xlabel('Scan 2')
set(gca,'FontSize',25)
[~,SCB1] = grid_rgbR(nptc1.Location,rgb1,xlimit,ylimit);
figure
imagesc(SCB1)
colormap(map)
title('Color-Variance')
xlabel('Scan 1')
set(gca,'FontSize',25)
[~,SCB1R] = grid_rgbR(nptc2.Location,rgb2,xlimit,ylimit);
figure
imagesc(SCB1R)
colormap(map)
title('Color-Variance')
xlabel('Scan 2')
set(gca,'FontSize',25)
%%
tic;
tf0=0;
ObjectiveFunction1 = @(tf)mi_all2(tf,SB1R,SN1R,SRB1R,Sib1R,SCB1R,nptc2,Ref2,int2,rgb2,xlimit,ylimit);
%tf = fminbnd(ObjectiveFunction1,-0.4,0.4);
[tf,fval1,exitFlag1,output1] = patternsearch(ObjectiveFunction1,tf0);
%%
x0=[0;0];
ObjectiveFunction2 = @(x)mi_all(x,tf,SB1,SN1,SRB1,Sib1,SCB1,nptc2,Ref2,int2,rgb2,xlimit,ylimit);
[x,fval2,exitFlag2,output2] = patternsearch(ObjectiveFunction2,x0);
toc;
%tf=x(3,1);
%%
TR = [eul1(1,3); eul1(1,2); tf];
eul = [tf eul1(1,2) eul1(1,3)];
rt = eul2rotm(eul);
TT = [x(1:2,1)' t(3,1)];
A = [rt,[0 0 0]';[TT 1]];
tform = affine3d(A);
pts22 = pctransform(pts2,tform);
%%
[pose,tfr] = load_pose('F:/',scan1,scan2);
TT1 = pose(1:3,1);
TR1 = pose(4:6,1);
pts23 = pctransform(pts2,tfr);
%%
[tform2,TT2,TR2,mse_profile] = gicp(pts1,pts2,'OverlapFraction',0.99);
%%
[mt,mr] = err(pose,TT',TR);
%%
figure
pcshowpair(pts1,pts22,'MarkerSize',10);
%set(gca,'color','black');
figure
pcshowpair(pts1,pts23);
figure
pcshowpair(pts1,pts2);
%%
figure
pcshow(pts1,'MarkerSize',10)
set(gca,'color','black');
% figure
% imagesc(SB1);
% title('Z - Variance Scan 1')
% figure
% imagesc(SB2);
% title('Z - Variance Scan 2')
% figure
% imagesc(SRB1);
% title('Reflectivity - Variance Scan 1')
% figure
% imagesc(SRB2);
% title('Reflectivity - Variance Scan 2')
% figure
% imagesc(Sib1);
% title('Intensity - Variance Scan 1')
% figure
% imagesc(Sib2);
% title('Intensity - Variance Scan 2')
% figure
% imagesc(SCB1);
% title('Colour - Variance Scan 1')
% figure
% imagesc(SCB2);
% title('Colour - Variance Scan 2')