load('F:\HP\experiments\ScanError\error_rotation_3.mat');
%load('F:\HP\experiments\ScanError\error_rotation_gicp_2.mat');
mr=mr*180/pi;
for j=1:8
for i=1:100
if mr(i,j)>90
    A = mr(i,j);
    A = A/90;
    int = floor(A);
    mr(i,j) = (A-int)*90;
end
end
end
mr = mr*pi/180;
%%
r1 = mr(:,1);
r2 = mr(:,2);
r3 = mr(:,3);
r4 = mr(:,4);
r5 = mr(:,5);
r6 = mr(:,6);
r7 = mr(:,7);
r8 = mr(:,8);
l3 = find(r3<pi/12);
l4 = find(r4<pi/12);
l5 = find(r5<pi/12);
l6 = find(r6<pi/4);
l7 = find(r7<pi/3);
l8 = find(r8<pi/2);
err_rot(1,1) = mean(r1);
std_rot(1,1) = std(r1);
err_rot(2,1) = mean(r2);
std_rot(2,1) = std(r2);
err_rot(3,1) = mean(r3);%(l3));
std_rot(3,1) = std(r3);%(l3));
err_rot(4,1) = mean(r4);%(l4));
std_rot(4,1) = std(r4);%(l4));
err_rot(5,1) = mean(r5(l5));
std_rot(5,1) = std(r5(l5));
err_rot(6,1) = mean(r6(l6));
std_rot(6,1) = std(r6(l6));
err_rot(7,1) = mean(r7(l7));
std_rot(7,1) = std(r7(l7));
err_rot(8,1) = mean(r8(l8));
std_rot(8,1) = std(r8(l8));
err_rot = err_rot*180/pi;
err_rot_gicp = mean(mrg)*180/pi;
std_rot_gicp = std(mrg)*180/pi;
%%
figure
hold on
plot(5:5:20,err_rot(1:4,1));
plot(5:5:20,err_rot_gicp(1,1:4));
title('Rotation Error')
xlabel('Rotation between Scans (degrees)');
ylabel('Rotation Error (degrees)');
set(gca,'FontSize',20)
legend('Proposed Method','GICP');
figure
hold on
plot(5:5:20,std_rot(1:4,1)*180/pi);
plot(5:5:20,std_rot_gicp(1,1:4));
title('Standard Deviation of Rotation Error')
xlabel('Rotation between Scans (degrees)');
ylabel('Standard Deviation (degrees)');
set(gca,'FontSize',20)
legend('Proposed Method','GICP');