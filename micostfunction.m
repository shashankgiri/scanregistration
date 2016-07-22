clearvars
clc
close all
%%
scan1 = 300;
scan2 = 302;
scanName1 = sprintf('%s/ScanIntensity/Scan_for_MI_Intensity_%04d.mat','D:\data',scan1);
M1 = load(scanName1);
I1 = M1.points(2:size(M1.points,1),5);
R1 = M1.points(2:size(M1.points,1),4);
RGB1 = M1.points(2:size(M1.points,1),6:8);
pts1 = pointCloud(M1.points(2:size(M1.points,1),1:3));
pts1.Normal = pcnormals(pts1,20);
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
pts2.Normal = pcnormals(pts2,20);
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
Ref1 = R1;
Ref2 = R2;
int1 = I1;
int2 = I2;
rgb1 = RGB1;
rgb2 = RGB2;
%[~,SB1] = grid2D(nptc1.Location,xlimit,ylimit);
[~,SB1R] = grid2DR(nptc1.Location,xlimit,ylimit);
%[~,SN1] = grid_normal(nptc1.Location,nptc1.Normal,xlimit,ylimit);
[~,SN1R] = grid_normalR(nptc1.Location,nptc1.Normal,xlimit,ylimit);
%[~,SRB1] = grid_reflectivity(nptc1.Location,Ref1,xlimit,ylimit);
[~,SRB1R] = grid_reflectivityR(nptc1.Location,Ref1,xlimit,ylimit);
%[~,Sib1] = grid_intensity(nptc1.Location,int1,xlimit,ylimit);
[~,Sib1R] = grid_intensityR(nptc1.Location,int1,xlimit,ylimit);
%[~,SCB1] = grid_rgb(nptc1.Location,rgb1,xlimit,ylimit);
[~,SCB1R] = grid_rgbR(nptc1.Location,rgb1,xlimit,ylimit);
%%
MI = zeros(60,1);
x = -0.0011;
tf = -0.0097;
i=0;
for y=0.01:0.01:6
 i = i+1;
 MI(i,1) = mitest1(x,y,tf,SCB1R,SB1R,SN1R,SRB1R,int2,Sib1R,Ref2,rgb2,nptc2,xlimit,ylimit);
    
end
%%
MI = imgaussfilt(MI,3);
a=0.01:0.01:6;
figure
plot(a',MI);
xlabel('Translation(m)')
ylabel('Cost')
set(gca,'FontSize',20)
%%
c=gicp_cost(pts1,pts2,'OverlapFraction',0.99,'Method','gicp');%[tform,TT,TR,mse_profile,c,fun]