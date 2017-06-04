bagselect = rosbag('/media/ashishkb/Seagate Backup Plus Drive/Ashish_HP_Backup/16.04/ros_bags/2017-05-27-19-17-03_loc2tag_tf.bag');
% %%
bagselect_tf = select(bagselect, 'Topic', '/tf');
%%
start_msg = 10000;
end_msg = 40000;
alltfs = readMessages(bagselect_tf, 10000:40000);
%%
store_tag_tfs = cell(floor(bagselect_tf.NumMessages/10),1);
ind_store_tf = 1;
for i = 1:(end_msg-start_msg)
    if(length(alltfs{i}.Transforms.ChildFrameId) > 7) 
        if(isequal(alltfs{i}.Transforms.ChildFrameId(1:7), 'loc2tag'))
            store_tag_tfs{ind_store_tf} = alltfs{i};
            ind_store_tf = ind_store_tf + 1;
            if(ind_store_tf > length(store_tag_tfs)/2)
                %resize cell to double when half filled
                store_tag_tfs = [store_tag_tfs ;cell(floor(bagselect_tf.NumMessages/10),1)];
            end
        end
    end
end

%%
%currently we are using only 8 tags
Pose = struct('X', {}, 'Y', {}, 'Z', {});
tags = struct('tag_0',Pose,'tag_1',Pose,'tag_2',Pose, 'tag_3',Pose,'tag_4',Pose,'tag_5',Pose, 'tag_6',Pose,'tag_7',Pose,'tag_8',Pose);
tag_count = struct('tag_0',1,'tag_1',1,'tag_2',1, 'tag_3',1,'tag_4',1,'tag_5',1, 'tag_6',1,'tag_7',1,'tag_8',1);
%tags.tag1.pose{1}.X
nnz_store_tag_tfs = length(store_tag_tfs) - nnz(cellfun(@(x) isequal(x,[]), store_tag_tfs));
for i = 1:nnz_store_tag_tfs
    
    cur_tag = str2num(store_tag_tfs{i}.Transforms.ChildFrameId(9:end));
    if(cur_tag <= 8)
        cur_tag_char = sprintf('tag_%d',cur_tag);
        cur_count = tag_count.(cur_tag_char);
        cur_trans = store_tag_tfs{i}.Transforms.Transform.Translation;
        tags.(cur_tag_char)(cur_count).X = cur_trans.X;
        tags.(cur_tag_char)(cur_count).Y = cur_trans.Y;
        tags.(cur_tag_char)(cur_count).Z = cur_trans.Z;
        tag_count.(cur_tag_char) = tag_count.(cur_tag_char) + 1;
    end
    
end
%%

figure(1);

for i = 0:8
    cur_tag_char = sprintf('tag_%d',i);
    scatter([tags.(cur_tag_char).X]', [tags.(cur_tag_char).Y]');
    hold on;

end

axis equal;

%%

figure(123);

for i = [1, 8]
    cur_tag_char = sprintf('tag_%d',i);
    scatter([tags.(cur_tag_char).X]', [tags.(cur_tag_char).Y]');
    hold on;

end

axis equal;

%%

tag_1_mean_xy = [mean([tags.tag_1.X]') mean([tags.tag_1.Y]')];
tag_8_mean_xy = [mean([tags.tag_8.X]') mean([tags.tag_8.Y]')];
norm(tag_1_mean_xy - tag_8_mean_xy)
