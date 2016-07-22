clearvars
clc
close all
%%
file_path1=('F:\HP\experiments\ScanError\error_gicp_15');
i_max = 150;
j_max = 1;
mrg = zeros(i_max,j_max);
mtg = zeros(i_max,j_max);
for i=1:i_max;
scan1 = 1000+i-1;%10*(j-1);
scan2 = 1000+i+14;%+10*(j-1);
scanName1 = sprintf('%s/ScanIntensity/Scan_for_MI_Intensity_%04d.mat','D:\data',scan1);
M1 = load(scanName1);
pts1 = pointCloud(M1.points(2:size(M1.points,1),1:3));
pts1.Normal = pcnormals(pts1,15);
scanName2 = sprintf('%s/ScanIntensity/Scan_for_MI_Intensity_%04d.mat','D:\data',scan2);
M2 = load(scanName2);
pts2 = pointCloud(M2.points(2:size(M2.points,1),1:3));
pts2.Normal = pcnormals(pts2,15);
%%
tic;
[tform2,TT2,TR2] = gicp(pts1,pts2,'OverlapFraction',0.99);
toc;
%%
[pose,tfr] = load_pose('F:/',scan1,scan2);
TT1 = pose(1:3,1);
TR1 = pose(4:6,1);
pts23 = pctransform(pts2,tfr);
%%
[mtg(i,1),mrg(i,1)] = err(pose,TT2',TR2');
end
file_name1=[file_path1 '.mat'];
save(file_name1,'mtg','mrg');