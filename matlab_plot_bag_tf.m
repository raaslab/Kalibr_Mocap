%%

% got to terminal and play the bag
%cd ~/kalib_mocap$ rosbag play -r 0.1 2017-03-29-18-33-03.bag
% 
% % 
clear;
tftree = rostf;
% %tftree.AvailableFrames;
 pause(1);
temp = tftree.LastUpdateTime;
cell_tim_loctag = cell(1,3);
nowtime = rostime('now');
count = 1;
record_sec = 3000;
begin_time = tftree.LastUpdateTime;

%system('rosbag play -r 0.1 ~/kalib_mocap/2017-03-29-18-33-03.bag'); % this
%doen't work
% pause(10);
% 
% 
% while((nowtime.Sec - temp.Sec) <  30)
%     if(temp~=tftree.LastUpdateTime)
%         waitForTransform(tftree, 'local_origin', 'tag-3');
%         local_origin_2_tag = getTransform(tftree, 'local_origin', 'tag-3');
% 
%         local_origin_2_tag_trans = local_origin_2_tag.Transform.Translation;
%         quat_lo = local_origin_2_tag.Transform.Rotation;
%         local_origin_2_tag_angle = rad2deg(quat2eul([quat_lo.W quat_lo.X quat_lo.Y quat_lo.Z]));
%                
%         cell_tim_loctag(count, :) = {local_origin_2_tag.Header.Stamp local_origin_2_tag_trans, local_origin_2_tag_angle};
%         temp = tftree.LastUpdateTime;
%         count = count +1;
%     end
%     
%     nowtime = rostime('now');
% end
%%

% % plot 3d scatter
% 
% %scatter3(X,Y,Z) 
% %entries in cell_tim_loctag
% nempty_bool = cellfun(@isempty,cell_tim_loctag);
% nnz_entry = nnz(~nempty_bool(:,1));
% X_vec = zeros(nnz_entry, 1);
% Y_vec = zeros(nnz_entry, 1);
% Z_vec = zeros(nnz_entry, 1);
% figure;
% for i = 1:nnz_entry-1
%     x_temp = cell_tim_loctag{i, 2}.X;
%     X_vec(i) = x_temp;
%     Y_vec(i) = cell_tim_loctag{i, 2}.Y;
%     Z_vec(i) = cell_tim_loctag{i, 2}.Z;
%     scatter(X_vec(i),Y_vec(i)) ;
%     hold on;
% end
% 
% % Z_vec_mean = mean(Z_vec);
% 
% figure;
% plot(1:length(Z_vec), Z_vec)
% hold on;
% plot(1:length(X_vec), X_vec)
% plot(1:length(Y_vec), Y_vec)
% 
% figure;
% scatter3(X_vec,Y_vec,Z_vec) ;
% % 
% % figure;
%%

pause(10);


while((nowtime.Sec - temp.Sec) <  30)
    if(temp~=tftree.LastUpdateTime)
        waitForTransform(tftree, 'fcu', 'tag-3');
        fcu_2_tag = getTransform(tftree, 'fcu', 'tag-3');

        fcu_2_tag_trans = fcu_2_tag.Transform.Translation;
        quat_lo = fcu_2_tag.Transform.Rotation;
        fcu_2_tag_angle = rad2deg(quat2eul([quat_lo.W quat_lo.X quat_lo.Y quat_lo.Z]));
               
        cell_tim_loctag(count, :) = {fcu_2_tag.Header.Stamp fcu_2_tag_trans, fcu_2_tag_angle};
        temp = tftree.LastUpdateTime;
        count = count +1;
    end
    
    nowtime = rostime('now');
end
% 

% 
% waitForTransform(tftree, 'fcu', 'tag-3');
% fcu_2_tag = getTransform(tftree, 'fcu', 'tag-3');
% 
% fcu_2_tag_trans = fcu_2_tag.Transform.Translation;
% quat_f = fcu_2_tag.Transform.Rotation;
% fcu_2_tag_angle = rad2deg(quat2eul([quat_f.W quat_f.X quat_f.Y quat_f.Z]));


%%

% plot 3d scatter

%scatter3(X,Y,Z) 
%entries in cell_tim_loctag
nempty_bool = cellfun(@isempty,cell_tim_loctag);
nnz_entry = nnz(~nempty_bool(:,1));
X_vec = zeros(nnz_entry, 1);
Y_vec = zeros(nnz_entry, 1);
Z_vec = zeros(nnz_entry, 1);
figure;
for i = 1:nnz_entry-1
    x_temp = cell_tim_loctag{i, 2}.X;
    X_vec(i) = x_temp;
    Y_vec(i) = cell_tim_loctag{i, 2}.Y;
    Z_vec(i) = cell_tim_loctag{i, 2}.Z;
    scatter(X_vec(i),Y_vec(i)) ;
    hold on;
end

% Z_vec_mean = mean(Z_vec);

figure;
plot(1:length(Z_vec), Z_vec)
hold on;
plot(1:length(X_vec), X_vec)
plot(1:length(Y_vec), Y_vec)

figure;
scatter3(X_vec,Y_vec,Z_vec) ;
% 
% figure;


%%

