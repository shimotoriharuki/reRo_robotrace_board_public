f1 = figure(1);
f2 = figure(2);
clf (f1, "reset")
clf (f2, "reset")
clc
clear



% ---データをロード---
first_run_distances = load('workingDirectory/first_run_distance.txt');
first_run_thetas = load('workingDirectory/first_run_theta.txt');
first_run_sideline_distances = load('workingDirectory/first_run_side.txt');
first_run_crossline_distances = load('workingDirectory/first_run_cross.txt');
% %first_run_total_distances = load('workingDirectory/first_run_total_distances.txt');
first_run_current_velocity = load('workingDirectory/first_run_current_velocity.txt');
first_run_target_velocity = load('workingDirectory/first_run_target_velocity.txt');

second_run_distances = load('workingDirectory/second_run_distance.txt');
second_run_thetas = load('workingDirectory/second_run_theta.txt');
second_run_sideline_distances = load('workingDirectory/second_run_side.txt');
second_run_crossline_distances = load('workingDirectory/second_run_cross.txt');
%second_run_total_distances = load('workingDirectory/second_run_total_distances.txt');
second_run_current_velocity = load('workingDirectory/second_run_current_velocity.txt');
second_run_target_velocity = load('workingDirectory/second_run_target_velocity.txt');

third_run_distances = load('workingDirectory/third_run_distance.txt');
third_run_thetas = load('workingDirectory/third_run_theta.txt');
third_run_sideline_distances = load('workingDirectory/third_run_side.txt');
third_run_crossline_distances = load('workingDirectory/third_run_cross.txt');
%third_run_total_distances = load('workingDirectory/third_run_total_distances.txt');
third_run_current_velocity = load('workingDirectory/third_run_current_velocity.txt');
third_run_target_velocity = load('workingDirectory/third_run_target_velocity.txt');

fourth_run_distances = load('workingDirectory/fourth_run_distance.txt');
fourth_run_thetas = load('workingDirectory/fourth_run_theta.txt');
fourth_run_sideline_distances = load('workingDirectory/fourth_run_side.txt');
fourth_run_crossline_distances = load('workingDirectory/fourth_run_cross.txt');
%fourth_run_total_distances = load('workingDirectory/fourth_run_total_distances.txt');
fourth_run_current_velocity = load('workingDirectory/fourth_run_current_velocity.txt');
fourth_run_target_velocity = load('workingDirectory/fourth_run_target_velocity.txt');

fifth_run_distances = load('workingDirectory/fifth_run_distance.txt');
fifth_run_thetas = load('workingDirectory/fifth_run_theta.txt');
fifth_run_sideline_distances = load('workingDirectory/fifth_run_side.txt');
fifth_run_crossline_distances = load('workingDirectory/fifth_run_cross.txt');
%fifth_run_total_distances = load('workingDirectory/fifth_run_total_distances.txt');
fifth_run_current_velocity = load('workingDirectory/fifth_run_current_velocity.txt');
fifth_run_target_velocity = load('workingDirectory/fifth_run_target_velocity.txt');

% degub_data1 = load('workingDirectory/translation_ratio.txt');
% degub_data2 = load('workingDirectory/rotation_ratio.txt');
% degub_data3 = load('workingDirectory/current_velocity.txt');

% current_velocity = load('workingDirectory/current_velocity.txt');
% target_velocity = load('workingDirectory/target_velocity.txt');


% --- データが有るところだけ抽出---
first_run_distances = nonzeros(first_run_distances); %mm
first_run_thetas = first_run_thetas(1 : size(first_run_distances)); %rad
first_run_sideline_distances = nonzeros(first_run_sideline_distances); %mm
first_run_crossline_distances = nonzeros(first_run_crossline_distances); %mm 
%first_run_total_distances = nonzeros(first_run_total_distances); %mm
first_run_current_velocity = nonzeros(first_run_current_velocity); %m/s
first_run_target_velocity = first_run_target_velocity(1 : size(first_run_current_velocity)); %m/s
first_run_thetas = first_run_thetas * 1.015;


second_run_distances = nonzeros(second_run_distances); %mm
second_run_thetas = second_run_thetas(1 : size(second_run_distances)); %rad
second_run_sideline_distances = nonzeros(second_run_sideline_distances); %mm
second_run_crossline_distances = nonzeros(second_run_crossline_distances); %mm 
%second_run_total_distances = nonzeros(second_run_total_distances); %mm
second_run_current_velocity = nonzeros(second_run_current_velocity); %m/s
second_run_target_velocity = second_run_target_velocity(1 : size(second_run_current_velocity)); %m/s
second_run_thetas = second_run_thetas * 1.015;


third_run_distances = nonzeros(third_run_distances); %mm
third_run_thetas = third_run_thetas(1 : size(third_run_distances)); %rad
third_run_sideline_distances = nonzeros(third_run_sideline_distances); %mm
third_run_crossline_distances = nonzeros(third_run_crossline_distances); %mm 
%third_run_total_distances = nonzeros(third_run_total_distances); %mm
third_run_current_velocity = nonzeros(third_run_current_velocity); %m/s
third_run_target_velocity = third_run_target_velocity(1 : size(third_run_current_velocity)); %m/s
third_run_thetas = third_run_thetas * 1.015;

fourth_run_distances = nonzeros(fourth_run_distances); %mm
fourth_run_thetas = fourth_run_thetas(1 : size(fourth_run_distances)); %rad
fourth_run_sideline_distances = nonzeros(fourth_run_sideline_distances); %mm
fourth_run_crossline_distances = nonzeros(fourth_run_crossline_distances); %mm 
%fourth_run_total_distances = nonzeros(fourth_run_total_distances); %mm
fourth_run_current_velocity = nonzeros(fourth_run_current_velocity); %m/s
fourth_run_target_velocity = fourth_run_target_velocity(1 : size(fourth_run_current_velocity)); %m/s
fourth_run_thetas = fourth_run_thetas * 1.015;

fifth_run_distances = nonzeros(fifth_run_distances); %mm
fifth_run_thetas = fifth_run_thetas(1 : size(fifth_run_distances)); %rad
fifth_run_sideline_distances = nonzeros(fifth_run_sideline_distances); %mm
fifth_run_crossline_distances = nonzeros(fifth_run_crossline_distances); %mm 
%fifth_run_total_distances = nonzeros(fifth_run_total_distances); %mm
fifth_run_current_velocity = nonzeros(fifth_run_current_velocity); %m/s
fifth_run_target_velocity = fifth_run_target_velocity(1 : size(fifth_run_current_velocity)); %m/s
fifth_run_thetas = fifth_run_thetas * 1.015;

% degub_data1 = nonzeros(degub_data1); 
% degub_data2 = degub_data2(1:length(degub_data1));
% current_velocity = nonzeros(current_velocity); 
% target_velocity = target_velocity(1:length(current_velocity));

% ---コースの形状をプロット---
figure(1);
% subplot(2, 1, 1);
plotCourseInformation(first_run_distances, first_run_thetas, first_run_sideline_distances, first_run_crossline_distances);
title('1走目')

figure(2)
subplot(2, 2, 1);
plotCourseInformation(second_run_distances, second_run_thetas, second_run_sideline_distances, second_run_crossline_distances);
title('2走目')

subplot(2, 2, 2);
plotCourseInformation(third_run_distances, third_run_thetas, third_run_sideline_distances, third_run_crossline_distances);
title('3走目')

subplot(2, 2, 3);
plotCourseInformation(fourth_run_distances, fourth_run_thetas, fourth_run_sideline_distances, fourth_run_crossline_distances);
title('4走目')

subplot(2, 2, 4);
plotCourseInformation(fifth_run_distances, fifth_run_thetas, fifth_run_sideline_distances, fifth_run_crossline_distances);
title('5走目')

% axis equal

figure(3);
subplot(2, 1, 1);
plotRadius(first_run_distances, first_run_thetas);
title('radius')

subplot(2, 1, 2);
plotdTheta(first_run_distances, first_run_thetas);
title('dtheta')


figure(4)
subplot(2, 2, 1);
plotVelocityTable(first_run_distances, first_run_thetas, 6.0, 2.5, 1000, 'linear');
title('Second Velocity tabele')

subplot(2, 2, 2);
plotVelocityTable(first_run_distances, first_run_thetas, 6.0, 2.5, 1000, 'sigmoid');
title('Third Velocity tabele')

subplot(2, 2, 3);
plotVelocityTable(first_run_distances, first_run_thetas, 6.0, 2.5, 1000, 'sigmoid');
title('Fourth Velocity tabele')

subplot(2, 2, 4);
plotVelocityTable(first_run_distances, first_run_thetas, 6.5, 2.5, 1000, 'sigmoid');
title('Fifth Velocity tabele')


%{
figure(5)
subplot(2, 2, 1);
plotTotalDistanceDiff(first_run_total_distances, second_run_total_distances);
title('トータル距離の違い 1st vs 2nd')
subplot(2, 2, 2);
plotTotalDistanceDiff(first_run_total_distances, third_run_total_distances);
title('トータル距離の違い 1st vs 3rd')
subplot(2, 2, 3);
plotTotalDistanceDiff(first_run_total_distances, fourth_run_total_distances);
title('トータル距離の違い 1st vs 4th')
subplot(2, 2, 4);
plotTotalDistanceDiff(first_run_total_distances, fifth_run_total_distances);
title('トータル距離の違い 1st vs 5th')
%}

% figure(6)
% plot(1:length(degub_data1), degub_data1, 'LineWidth', 3)
% hold on
% plot(1:length(degub_data2), degub_data2)
% hold off
% legend('translation', 'rotaion')
% title('汎用デバッグデータ')
% 
% total_ratio_l = degub_data1 + degub_data2;
% total_ratio_r = degub_data1 - degub_data2;
% 
% figure(7)
% plot(1:length(total_ratio_l), total_ratio_l, 'LineWidth', 3)
% legend('total ratio l')
% 
% hold on
% plot(1:length(total_ratio_r), total_ratio_r)
% title('soiya')
% legend('tota ratio r')
% hold off

figure(8)
plot(1:length(first_run_current_velocity), first_run_current_velocity)
hold on
plot(1:length(first_run_target_velocity), first_run_target_velocity)
hold off
legend('1st 速度追従')


figure(9)
subplot(2, 2, 1);
plot(1:length(second_run_current_velocity), second_run_current_velocity)
hold on
plot(1:length(second_run_target_velocity), second_run_target_velocity)
hold off
legend('2nd 速度追従')

subplot(2, 2, 2);
plot(1:length(third_run_current_velocity), third_run_current_velocity)
hold on
plot(1:length(third_run_target_velocity), third_run_target_velocity)
hold off
legend('3rd 速度追従')

subplot(2, 2, 3);
plot(1:length(fourth_run_current_velocity), fourth_run_current_velocity)
hold on
plot(1:length(fourth_run_target_velocity), fourth_run_target_velocity)
hold off
legend('4th 速度追従')

subplot(2, 2, 4);
plot(1:length(fifth_run_current_velocity), fifth_run_current_velocity)
hold on
plot(1:length(fifth_run_target_velocity), fifth_run_target_velocity)
hold off
legend('5th 速度追従')


function plotCourseInformation(distances, thetas, sidelines, crosslines)
    x = 0;
    y = 0;
    th = 0;
    total_distance = 0;

    robot_positions = [zeros(size(distances)), zeros(size(distances))];
    sensor_positions = [zeros(size(distances)), zeros(size(distances))];
    sideline_positions = [zeros(size(sidelines)), zeros(size(sidelines))];
    crossline_positions = [zeros(size(crosslines)), zeros(size(crosslines))];
    sideline_idx = 1;
    crossline_idx = 1;

    pivot_lenght = 110; %mm

    % コース情報をプロット
    for i = 1:size(distances)
        % 位置を計算
        x = x + (distances(i)) * cos(th + thetas(i)/2);
        y = y + (distances(i)) * sin(th + thetas(i)/2);
    
        th = th + thetas(i);
        robot_positions(i, :) = [x, y];
        sensor_positions(i, :) = [x + pivot_lenght * cos(th), y + pivot_lenght * sin(th)];


        % サイドラインの位置を計算
        if isempty(sidelines) == 0
            if total_distance + 10 >= sidelines(sideline_idx) && sidelines(sideline_idx) >= total_distance - 10 
                y_sideline = y + 100 * cos(th);
                sideline_positions(sideline_idx, :) = [x, y_sideline];
                sideline_idx = sideline_idx + 1;
    
                if sideline_idx > length(sidelines(:, 1))
                    sideline_idx = length(sidelines(:, 1));
                end
            end
        end

        % クロスラインの位置を計算
        if isempty(crosslines) == 0
            if total_distance + 10 >= crosslines(crossline_idx) && crosslines(crossline_idx) >= total_distance - 10
                
                crossline_positions(crossline_idx, :) = [x, y];
                crossline_idx = crossline_idx + 1;
    
                if crossline_idx > length(crosslines(:, 1))
                    crossline_idx = length(crosslines(:, 1));
                end
            end
        end

        total_distance = total_distance + distances(i);
    
    end
    hold on
    scatter(robot_positions(:, 1), robot_positions(:, 2), 6); % ロボットの中心の走行経路をプロット
    scatter(sensor_positions(:, 1), sensor_positions(:, 2), 6); % センサーの走行経路をプロット


    crossline_nums = 1 : length(crossline_positions(:, 1));
    scatter(crossline_positions(: ,1), crossline_positions(:, 2), 300, "red", "x", "LineWidth", 2); % クロスラインをプロット
    text(crossline_positions(: ,1) + 50, crossline_positions(:, 2) + 50, string(crossline_nums), "Color", 'r');
    
    sideline_nums = 1 : length(sideline_positions(:, 1));
    scatter(sideline_positions(: ,1), sideline_positions(:, 2), 300, "magenta", "*", "LineWidth", 2) % サイドラインをプロット
    text(sideline_positions(: ,1) + 50, sideline_positions(:, 2) + 50, string(sideline_nums), "Color", "m");

    hold off
end

function plotRadius(distances, thetas)
    thetas(thetas == 0) = 0.00001;

    t = 1 : length(distances);
    radius = abs(distances ./ thetas);
    radius(radius >= 2000) = 2000;
    
    plot(t, radius);
    ylim([0, 2100])

end

function plotdTheta(distances, thetas)
    thetas(thetas == 0) = 0.00001;

    t = 1 : length(distances);
    dtheta = abs(thetas ./ distances);
    
    
    plot(t, dtheta);
%     ylim([0, 5500])

end 

function plotVelocityTable(distance, theta, max_velo, min_velo, straight_radius, function_type)
    theta(theta == 0) = 0.00001;
    radius = abs(distance ./ theta);
    radius(radius >= straight_radius) = straight_radius;

    if strcmp(function_type, 'linear')
        velocity_table = radius .* ((max_velo - min_velo) / straight_radius) + min_velo;
    elseif strcmp(function_type, 'quadratic')
        velocity_table = 1e-3 .* radius .* radius .* ((max_velo - min_velo) / straight_radius) + min_velo;
    elseif strcmp(function_type, 'sigmoid')
        facter = straight_radius/12;
        gain = 1/facter;
        velocity_table = (1 ./ (1 + exp(-gain*radius+6))) * (max_velo-min_velo) + min_velo;
    end


    plot(1 : length(velocity_table), velocity_table);
end

function plotTotalDistanceDiff(data1, data2)
    plot(1:length(data1), data1, 1:length(data2), data2)
end