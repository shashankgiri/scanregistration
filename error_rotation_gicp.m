clearvars
clc
close all
%%
file_path1=('F:\HP\experiments\ScanError\error_rotation_gicp');
i_max = 100;
j_max = 6;
mrg = zeros(i_max,j_max);
%mt = zeros(i_max,j_max);
for j=1:j_max
for i=1:i_max;
l = 200+10*randi(300,1);
scan1 = l;
scan2 = l;
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
%%
[~,~,TR2,mse_profile] = gicp(pts1,pts2,'OverlapFraction',0.99);
 mrg(i,j) = immse(az,TR2(1,1));
end
end
file_name1=[file_path1 '.mat'];
save(file_name1,'mrg');