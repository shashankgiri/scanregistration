clearvars -except l m
clc
close all
%%
file_path1=('F:\HP\experiments\ScanError\error_normal_2');
i_max = 120;
j_max = 1;
mrn = zeros(i_max,j_max);
mtn = zeros(i_max,j_max);
% mrg = zeros(i_max,j_max);
% mtg = zeros(i_max,j_max);
% l = zeros(i_max,j_max);
% m = zeros(i_max,j_max);
%for j=1:j_max
for i=1:i_max;
% l(i,1) = 200+10*randi(300,1);
% m(i,1) = l(i,1)+randi(10,1);
scan1 = l(i,1);
scan2 = m(i,1);
scanName1 = sprintf('%s/ScanIntensity/Scan_for_MI_Intensity_%04d.mat','D:\data',scan1);
M1 = load(scanName1);
% I1 = M1.points(2:size(M1.points,1),5);
% R1 = M1.points(2:size(M1.points,1),4);
% RGB1 = M1.points(2:size(M1.points,1),6:8);
pts1 = pointCloud(M1.points(2:size(M1.points,1),1:3));
pts1.Normal = pcnormals(pts1,15);
[ptc1,~,plane1,p1,outliers1] = gp(pts1);
nptc1=pts1;
xlimit1=nptc1.XLimits(1,2)-nptc1.XLimits(1,1);
ylimit1=nptc1.YLimits(1,2)-nptc1.YLimits(1,1);
scanName2 = sprintf('%s/ScanIntensity/Scan_for_MI_Intensity_%04d.mat','D:\data',scan2);
M2 = load(scanName2);
% I2 = M2.points(2:size(M2.points,1),5);
% R2 = M2.points(2:size(M2.points,1),4);
% RGB2 = M2.points(2:size(M2.points,1),6:8);
pts2 = pointCloud(M2.points(2:size(M2.points,1),1:3));
pts2.Normal = pcnormals(pts2,15);
[ptc2,~,plane2,p2,outliers2] = gp(pts2);
nptc2=pts2;
xlimit2 = nptc2.XLimits(1,2)-nptc2.XLimits(1,1); 
ylimit2 = nptc2.YLimits(1,2)-nptc2.YLimits(1,1);
xlimit = roundn(max(xlimit1,xlimit2),1);
ylimit = roundn(max(ylimit1,ylimit2),1);
%%
% Ref1 = R1;
% Ref2 = R2;
% int1 = I1;
% int2 = I2;
% rgb1 = RGB1;
% rgb2 = RGB2;
% [~,SB1] = grid2D(nptc1.Location,xlimit,ylimit);
% [~,SB1R] = grid2DR(nptc1.Location,xlimit,ylimit);
% [~,SN1] = grid_normal(nptc1.Location,nptc1.Normal,xlimit,ylimit);
[~,SN1R] = grid_normalR(nptc1.Location,nptc1.Normal,xlimit,ylimit);
% [~,SRB1] = grid_reflectivity(nptc1.Location,Ref1,xlimit,ylimit);
% [~,SRB1R] = grid_reflectivityR(nptc1.Location,Ref1,xlimit,ylimit);
% [~,Sib1] = grid_intensity(nptc1.Location,int1,xlimit,ylimit);
% [~,Sib1R] = grid_intensityR(nptc1.Location,int1,xlimit,ylimit);
% [~,SCB1] = grid_rgb(nptc1.Location,rgb1,xlimit,ylimit);
% [~,SCB1R] = grid_rgbR(nptc1.Location,rgb1,xlimit,ylimit);
%%
[R12,eul1] = find_rotation(plane1,plane2);
t = R12*mean(pts1.Location)'-mean(pts2.Location)';
%%
tic;
% tf0=0;
% ObjectiveFunction1 = @(tf)mi_all2(tf,SB1R,SN1R,SRB1R,Sib1R,SCB1R,nptc2,Ref2,int2,rgb2,xlimit,ylimit);
% [tf,fval1,exitFlag1,output1] = patternsearch(ObjectiveFunction1,tf0);
%%
% x0=[0;0];
% ObjectiveFunction2 = @(x)mi_all(x,tf,SB1,SN1,SRB1,Sib1,SCB1,nptc2,Ref2,int2,rgb2,xlimit,ylimit);
% [x,fval2,exitFlag2,output2] = patternsearch(ObjectiveFunction2,x0);
x0=[0;0;0];
ObjectiveFunction2 = @(x)mitest(x,SN1R,nptc2,xlimit,ylimit);%,SRB1R,SB1R,SCB1R,Ref2,rgb2,int2,Sib1R,
[x,fval2,exitFlag2,output2] = patternsearch(ObjectiveFunction2,x0);
tf=x(3,1);
toc;
%%
TR = [eul1(1,3); eul1(1,2); tf];
%eul = [tf eul1(1,2) eul1(1,3)];
%rt = eul2rotm(eul);
TT = [x(1:2,1)' t(3,1)];
%A = [rt,[0 0 0]';[TT 1]];
%tform = affine3d(A);
%pts22 = pctransform(pts2,tform);
%%
[pose,~] = load_pose('F:/',scan1,scan2);
TT1 = pose(1:3,1);
TR1 = pose(4:6,1);
%pts23 = pctransform(pts2,tfr);
%%
% tic;
% [~,TT2,TR2] = gicp(pts1,pts2,'OverlapFraction',0.99);
% toc;
%%
[mtn(i,1),mrn(i,1)] = err(pose,TT',TR);
%[mtg(i,1),mrg(i,1)] = err(pose,TT2',TR2');
end
%end
file_name1=[file_path1 '.mat'];
save(file_name1,'mtn','mrn');%,'mtg','mrg','l','m');