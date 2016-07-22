clearvars
clc
close all
%%
file_path1=('F:\HP\experiments\ScanError\error_rotation_3');
i_max = 100;
j_max = 8;
mr = zeros(i_max,j_max);
mrg = zeros(i_max,j_max);
for j=1:j_max
for i=1:i_max;
l = 200+10*randi(90,1);
scan1 = l;
scan2 = l+2;
%%
ax=0;%(pi/36);
Tx=0;
ay=0;%(pi/36);
Ty=0;
az=(pi/36)*j;
Tz=0;
Rx=[1 0 0;0 cos(ax) sin(ax);0 -sin(ax) cos(ax)];
Ry=[cos(ay) 0 -sin(ay);0 1 0;sin(ay) 0 cos(ay)];
Rz=[cos(az) sin(az) 0;-sin(az) cos(az) 0;0 0 1];
R=Rz*Ry*Rx;
T=[Tx Ty Tz 1];
A=[R,[0 0 0]';T];
tfr=affine3d(A);
%%
scanName1 = sprintf('%s/ScanIntensity/Scan_for_MI_Intensity_%04d.mat','D:\data',scan1);
M1 = load(scanName1);
I1 = M1.points(2:size(M1.points,1),5);
R1 = M1.points(2:size(M1.points,1),4);
RGB1 = M1.points(2:size(M1.points,1),6:8);
pts1 = pointCloud(M1.points(2:size(M1.points,1),1:3));
pts1.Normal = pcnormals(pts1,20);
%[ptc1,~,plane1,p1,outliers1] = gp(pts1);
nptc1=pts1;
xlimit1=nptc1.XLimits(1,2)-nptc1.XLimits(1,1);
ylimit1=nptc1.YLimits(1,2)-nptc1.YLimits(1,1);
%%
tic
scanName2 = sprintf('%s/ScanIntensity/Scan_for_MI_Intensity_%04d.mat','D:\data',scan2);
M2 = load(scanName2);
I2 = M2.points(2:size(M2.points,1),5);
R2 = M2.points(2:size(M2.points,1),4);
RGB2 = M2.points(2:size(M2.points,1),6:8);
pts2 = pointCloud(M2.points(2:size(M2.points,1),1:3));
pts2.Normal = pcnormals(pts2,20);
pts2 = pctransform(pts2,tfr);
%[ptc2,~,plane2,p2,outliers2] = gp(pts2);
xlimit2 = pts2.XLimits(1,2)-pts2.XLimits(1,1);
ylimit2 = pts2.YLimits(1,2)-pts2.YLimits(1,1);
xlimit = roundn(max(xlimit1,xlimit2),1);
ylimit = roundn(max(ylimit1,ylimit2),1);
%%
Ref1 = R1;%(outliers1);
Ref2 = R2;%(outliers2);
int1 = I1;%(outliers1);
int2 = I2;%(outliers2);
rgb1 = RGB1;%(outliers1,:);
rgb2 = RGB2;%(outliers2,:);
%[~,SB1] = grid2D(pts1.Location,xlimit,ylimit);
[~,SB1R] = grid2DR(pts1.Location,xlimit,ylimit);
%[~,SN1] = grid_normal(pts1.Location,pts1.Normal,xlimit,ylimit);
[~,SN1R] = grid_normalR(pts1.Location,pts1.Normal,xlimit,ylimit);
%[~,SRB1] = grid_reflectivity(pts1.Location,Ref1,xlimit,ylimit);
[~,SRB1R] = grid_reflectivityR(pts1.Location,Ref1,xlimit,ylimit);
%[~,Sib1] = grid_intensity(pts1.Location,int1,xlimit,ylimit);
[~,Sib1R] = grid_intensityR(pts1.Location,int1,xlimit,ylimit);
%[~,SCB1] = grid_rgb(pts1.Location,rgb1,xlimit,ylimit);
[~,SCB1R] = grid_rgbR(pts1.Location,rgb1,xlimit,ylimit);
%%
% [R12,eul1] = find_rotation(plane1,plane2);
% t = R12*mean(pts1.Location)'-mean(pts2.Location)';
tic;
tf0=0;
ObjectiveFunction1 = @(tf)mi_all2(tf,SB1R,SN1R,SRB1R,Sib1R,SCB1R,pts2,Ref2,int2,rgb2,xlimit,ylimit);
[tf,fval1,exitFlag1,output1] = patternsearch(ObjectiveFunction1,tf0);
%%
% x0=[0;0];
% ObjectiveFunction2 = @(x)mi_all(x,tf,SB1,SN1,SRB1,Sib1,SCB1,nptc2,Ref2,int2,rgb2,xlimit,ylimit);
% [x,fval2,exitFlag2,output2] = patternsearch(ObjectiveFunction2,x0);
% x0=[0;0;0];
% ObjectiveFunction2 = @(x)mitest(x,SB1R,SN1R,SRB1R,Sib1R,SCB1R,nptc2,Ref2,int2,rgb2,xlimit,ylimit);
% [x,fval2,exitFlag2,output2] = patternsearch(ObjectiveFunction2,x0);
% tf=x(3,1);
toc;
%%
% TR = [eul1(1,3); eul1(1,2); tf];
% eul = [tf eul1(1,2) eul1(1,3)];
%rt = eul2rotm(eul);
%TT = [x(1:2,1)' t(3,1)];
% A = [rt,[0 0 0]';[TT 1]];
% tform = affine3d(A);
% pts22 = pctransform(pts2,tform);
%%
[pose,tfr] = load_pose('F:/',scan1,scan2);
[~,~,TR2,mse_profile] = gicp(pts1,pts2,'OverlapFraction',0.99);
%TT1 = pose(1:3,1);
TR1 = pose(4:6,1);
% pts23 = pctransform(pts2,tfr);
%%
% [~,mr(i,1)] = err(pose,TT',TR);
 mrg(i,j) = immse(az+TR1(3,1),TR2(1,1));
 mr(i,j) = immse((az+TR1(3,1)),tf);
end
end
file_name1=[file_path1 '.mat'];
save(file_name1,'mr','mrg');