clc; close all; clear all; 
DataFolder = 'D:\Projects\Teaser_SPARK\Teaser'; 
cloud_src = pcread( fullfile(DataFolder, 'cloud_src.pcd') ); 
cloud_aft = pcread( fullfile(DataFolder, 'cloud_aft.pcd') ); 
cloud_tgt = pcread( fullfile(DataFolder, 'cloud_tgt.pcd') ); 
figure; set(gcf, 'Position', [50 50 800 600], 'color', 'w'); hold on; grid on; box on; axis equal; 
pcshow(cloud_tgt.Location, 'g', 'markersize', 50); 
pcshow(cloud_aft.Location, 'b', 'markersize', 50); 
% pcshow(cloud_src.Location, 'r', 'markersize', 50); 