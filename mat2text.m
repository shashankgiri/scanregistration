for i=1:3891
scanName1 = sprintf('%s/ScanIntensity/Scan_for_MI_Intensity_%04d.mat','D:\data',i);
Data = load(scanName1);
scanName2 = sprintf('%s/Scan_for_MI_%04d.txt','D:\text_data',i);
file_path=(scanName2);
fid=fopen(file_path,'w');
fprintf(fid, '%f\n', Data.points(1,1));
fprintf(fid,'%f %f %f %f %f %f %f %f\n', Data.points(2:end,:)');
fclose(fid);
end