clearvars
clc
%%
i=2;
load('F:\HP\experiments\ScanError\error_gicp_15.mat');
t_error_gicp(i,1) = mean(mtg);
t_std_gicp(i,1) = std(mtg);
r_error_gicp(i,1) = mean(mrg);
r_std_gicp(i,1) = std(mrg);
%%
load('error')
load('error_gicp')
figure
hold on; 
plot(8/15:8/15:8,t_error);
plot(8/15:8/15:8,t_error_gicp);
title('Error')
xlabel('Scan Difference');
ylabel('Translation Error (m)');
legend('Proposed Method','GICP');
figure
hold on; 
plot(8/15:8/15:8,t_std);
plot(8/15:8/15:8,t_std_gicp);
title('Standard Deviation of Translation Error')
xlabel('Scan Difference');
ylabel('Standard Deviation (m)');
legend('Proposed Method','GICP');
figure
hold on; 
plot(r_error*180/pi);
plot(r_error_gicp*180/pi);
title('Error')
xlabel('Scan Difference');
ylabel('Rotation Error (degrees)');
legend('Proposed Method','GICP');
figure
hold on; 
plot(r_std*180/pi);
plot(r_std_gicp*180/pi);
title('Standard Deviation of Rotation Error')
xlabel('Scan Difference');
ylabel('Standard Deviation (degrees)');
legend('Proposed Method','GICP');