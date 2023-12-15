% |-----------------Single Extended Object Tracking---------------------|
% |----Using Gaussian Mixture Probability Hypothesis Density Filter-----|
% |----------------and Partition Labeling Algorithm---------------------|
% =======================================================================
% |-----------------MATLAB Driving Toolbox Simulation-------------------|
% =======================================================================
% |------------------------Table of contents:---------------------------|
% |1. Simulation setting                                                |
% |2. Generate data                                                     |
% |3. Memory Allocation                                                 |
% |4. Retrive ground-truths and measurements from data structure        |
% |5. Generate clutters and measurement's noises                        |
% |6. Predict and update measurements                                   |
% |7. Plot estimation, ground-truths, evaluation metrics                |
% =======================================================================
% |--|--------------------SIMULATION TUTORIAL---------------------------|
% =======================================================================
% |1.|-----------If you want to use different scenario.-----------------|
% |  |Create a Driving Scenario using MATLAB                            |
% |  |Export Scenario Function, save it under 'xxx.m'                   |
% |  |Replace old Scenario function in 2. Generate data with 'xxx'      |
% |2.|---------If you want to improve estimator's accuracy--------------|
% |  |Increase simulation's duration                                    |
% |  |Set waypoint for target and self vehicle to straight waypoint     |
% |  |Decrease clutters distribution cardinality (lambda_c)             |
% |  |Increase detection probability                                    |
% |  |Decrease noise amplification parameters                           |
% |3.|-------------If you want simulation to be realistic---------------|
% |  |(Reverse the options in tutorial 2)                               |
% |4.|--------------If you want simulation to run faster----------------|
% |  |Decrease duration                                                 |
% |  |Set all of plotting options to FALSE                              |
% |  |Set doSaveMP4 option to FALSE                                     |
% |5.|--------------------If you want to save video---------------------|
% |  |Set doSaveMP4                                                     |
% =======================================================================

clc, clear, close all
%% Simulation setting

SimulationID = 4;

doSaveMP4 = false;

doPlotOSPA = true;

doPlotScenario = true;
doPlotSimpleScenario = false;
doPlot_ExtendedSimpleScenario = true;

doPlotScenario_Estimates = true;
doPlotScenario_Clutters = true;

doCompareMEMEKF = 1;

isDetectionNoisy = true;
isDetectionProbability = true;
isDetectionEllipticalNoisy = true;

detections_probability = 0.9;
detections_noise_amplifier = 1;

lambda_r_p = 10;
lambda_c = 40;

%% Generate data
switch(SimulationID)
    case 1
        [DTB_Data, DTB_Scenario, DTB_Sensor] = DTB_Same_Direction;
    case 2
        [DTB_Data, DTB_Scenario, DTB_Sensor] = DTB_Same_Direction2;
    case 3
        [DTB_Data, DTB_Scenario, DTB_Sensor] = DTB_Opposite_Direction;
    case 4
        [DTB_Data, DTB_Scenario, DTB_Sensor] = DTB_Opposite_Direction2;
    otherwise
        disp("Incorrect simulation's ID.");

end
model = gen_model;

DTB_Time = vertcat(DTB_Data.Time);
DTB_SamplingTime = DTB_Time(2) - DTB_Time(1);
duration = round((DTB_Time(end) - DTB_Time(1)) / DTB_SamplingTime + 1);

%% Memory Allocation
z = cell(duration, 1);
d_p = cell(duration, 1);
c = cell(duration, 1);

gt_self_dynamic = zeros(4, duration);
gt_self_orient = zeros(1, duration);
gt_self_shape = zeros(3, duration);
gt_target_dynamic = zeros(4, duration);
gt_target_orient = zeros(1, duration);
gt_target_shape = zeros(3, duration);
isInSensorRange = zeros(1, duration);

num_targets = zeros(duration, 1);
num_detections = zeros(duration, 1);
num_clutters = zeros(duration, 1);
num_reflection_points = zeros(duration, 1);

%% Retrieve ground-truth
for i = 1:duration
    overhang_rear_calibrate = [DTB_Scenario.Actors(1).Length/2 - DTB_Scenario.Actors(1,1).RearOverhang; 0];
    gt_self_dynamic(:, i) = [DTB_Data(i).ActorPoses(1).Position(1:2)' + overhang_rear_calibrate; DTB_Data(i).ActorPoses(1).Velocity(1:2)'];
    gt_self_orient(i) = DTB_Data(i).ActorPoses(1).Yaw /180 * pi;
    gt_self_shape(:, i) = [gt_self_orient(i); DTB_Scenario.Actors(1).Length/2; DTB_Scenario.Actors(1).Width/2];

    overhang_rear_calibrate = [DTB_Scenario.Actors(2).Length/2 - DTB_Scenario.Actors(1,2).RearOverhang; 0];
    gt_target_dynamic(:, i) = [DTB_Data(i).ActorPoses(2).Position(1:2)' + overhang_rear_calibrate; DTB_Data(i).ActorPoses(2).Velocity(1:2)'];
    gt_target_orient(i) = DTB_Data(i).ActorPoses(2).Yaw / 180 * pi;
    gt_target_shape(:, i) = [gt_target_orient(i); DTB_Scenario.Actors(2).Length/2; DTB_Scenario.Actors(2).Width/2];

    distance = sqrt((gt_self_dynamic(1, i) - gt_target_dynamic(1, i)) * (gt_self_dynamic(1, i) - gt_target_dynamic(1, i)) + ...
                    (gt_self_dynamic(2, i) - gt_target_dynamic(2, i)) * (gt_self_dynamic(2, i) - gt_target_dynamic(2, i)));
    isInSensorRange(i) = (distance <= 50);
end

%% Retrieve measurements
for i = 1:duration
    num_reflection_points(i) = size(DTB_Data(i).ObjectDetections, 1);
    reflection_points = zeros(2, num_reflection_points(i));
    detections = [];

    for j = 1:num_reflection_points(i)
        reflection_points(:, j) = DTB_Data(i).ObjectDetections{j, 1}.Measurement(1:2) - overhang_rear_calibrate;
    end

    meas_pos_sensor = reflection_points;
    meas_conv_angle = gt_self_orient(i);
    meas_conv_angle_matrix = [cos(meas_conv_angle) -sin(meas_conv_angle); sin(meas_conv_angle) cos(meas_conv_angle)];
    meas_pos_pole = meas_conv_angle_matrix * meas_pos_sensor + repmat(gt_self_dynamic(1:2, i), 1, num_reflection_points(i));
    reflection_points = meas_pos_pole;
    
    if isDetectionNoisy
        if isDetectionEllipticalNoisy && (num_reflection_points(i) ~= 0) && isInSensorRange(i)
            num_reflection_points(i) = poissrnd(lambda_r_p) + 1;
            for j = 1:num_reflection_points(i)
                multiplicative_noise = -1 + 2.* rand(1, 2);
                while norm(multiplicative_noise) > 1
                    multiplicative_noise = -1 + 2.* rand(1, 2);
                end
    
                h(j, :) = multiplicative_noise;
                reflection_points(:, j) = gt_target_dynamic(1:2, i) + ...
                                    h(j, 1) * gt_target_shape(2, i) * [cos(gt_target_shape(1, i)); sin(gt_target_shape(1, i))] + ...
                                    h(j, 2) * gt_target_shape(3, i) * [-sin(gt_target_shape(1, i)); cos(gt_target_shape(1, i))] + ...
                                    detections_noise_amplifier * mvnrnd([0; 0], [1 0; 0 1], 1)';
            end
        else
            reflection_points = reflection_points + detections_noise_amplifier * mvnrnd([0; 0], [1 0; 0 1], num_reflection_points(i))';
        end
    end

    if isDetectionProbability
        for j = 1:num_reflection_points(i)
            if rand(1) <= detections_probability
                detections(:, end + 1) = reflection_points(:, j);
            end
        end
    else
        detections = reflection_points;
    end

    num_detections(i) = size(detections, 2);
    d_p{i} = detections;
end

%% Generate clutters
clutter_region = [min(gt_self_dynamic(1, :)) - 2, max(gt_self_dynamic(1, :)) + 30;
                min(gt_self_dynamic(2, :)) - 30, max(gt_self_dynamic(2, :)) + 40];
pdf_c = 1 / prod(clutter_region(:, 2) - clutter_region(:, 1));

for i = 1:duration
    num_clutters(i) = poissrnd(lambda_c);

    clutters = [unifrnd(clutter_region(1, 1), clutter_region(1, 2), 1, num_clutters(i)); 
            unifrnd(clutter_region(2, 1), clutter_region(2, 2), 1, num_clutters(i))];
    c{i} = clutters;
    z{i} = [d_p{i} c{i}];
end

%% Prior
w_update = cell(duration, 1);
m_update = cell(duration, 1);
P_update = cell(duration, 1);

w_update{1} = .01;
m_update{1} = [0; 20; 2; 0];
P_update{1} = diag([10 10 30 0]);

est = cell(duration, 1);
exec_time = zeros(duration, 1);
isDetected = zeros(duration, 1);

elim_threshold = 1e-5;
merge_threshold = 4;
L_max = 100;
d_threshold = 3;

r = repmat(m_update{1}, 1, duration);
p = repmat([-pi/8; 5; 2], 1, duration);
Cr = diag([10 10 15 0]);
Cp = diag([pi/10 1.25 0.8]);

H = model.H;
Ar = model.F;
Ap = eye(3);

Ch = diag([1/4 1/4]);
Cv = diag([20 8]);
Cwr = diag([10 10 1 1]);
Cwp = diag([.05 .001 .001]);

if doCompareMEMEKF
    r_memekf = repmat(m_update{1}, 1, duration);
    p_memekf = repmat([-pi/8; 5; 2], 1, duration);
    Cr_memekf = diag([10 10 15 0]);
    Cp_memekf = diag([pi/10 1.25 0.8]);
    
    H_memekf = model.H;
    Ar_memekf = model.F;
    Ap_memekf = eye(3);
    
    Ch_memekf = diag([1/4 1/4]);
    Cv_memekf = diag([20 8]);
    Cwr_memekf = diag([10 10 1 1]);
    Cwp_memekf = diag([.05 .001 .001]);
end

for k = 2:duration
    exec_time_start = tic;
%% Predict
    w_birth = 0.01;
    m_birth = [0; 0; 3; 0];
    P_birth = diag([10 10 20 30]);

    [m_predict, P_predict] = predict_KF(model, m_update{k-1}, P_update{k-1});
    w_predict = model.P_S * w_update{k-1};

    w_predict = cat(1, w_birth, w_predict);
    m_predict = cat(2, m_birth, m_predict);
    P_predict = cat(3, P_birth, P_predict);

%% Update
    w_update{k} = model.P_MD * w_predict;
    m_update{k} = m_predict;
    P_update{k} = P_predict;
    
    [likelihood] = cal_likelihood(z{k}, model, m_predict, P_predict);

    if size(z{k}, 2) ~= 0
        [m_temp, P_temp] = update_KF(z{k}, model, m_predict, P_predict);

        for i = 1:size(z{k}, 2)
            w_temp = model.P_D * w_predict .* likelihood(:, i);
            w_temp = w_temp ./ (num_clutters(k) * pdf_c + sum(w_temp));

            w_update{k} = cat(1, w_update{k}, w_temp);
            m_update{k} = cat(2, m_update{k}, m_temp(:, :, i));
            P_update{k} = cat(3, P_update{k}, P_temp);
        end
    end

    [w_update{k}, m_update{k}, P_update{k}] = gaus_prune(w_update{k}, m_update{k}, P_update{k}, elim_threshold);
    [w_update{k}, m_update{k}, P_update{k}] = gaus_merge(w_update{k}, m_update{k}, P_update{k}, merge_threshold);
    [w_update{k}, m_update{k}, P_update{k}] = gaus_cap(w_update{k}, m_update{k}, P_update{k}, L_max);

    num_targets(k) = round(sum(w_update{k}));
    w_copy = w_update{k};
    indices = [];

    for i = 1:num_targets(k)
        [~, maxIndex] = max(w_copy);
        if w_copy(maxIndex) >= 0.5
            indices(i) = maxIndex;
        end
        w_copy(maxIndex) = -inf;
    end

    for i = 1:size(indices, 2)
        est{k} = [est{k} m_update{k}(1:2, i)];
    end

%% Partition labeling
    est{k} = partition_labeling(d_threshold, est{k});
    num_targets(k) = size(est{k}, 2);
    
    if ~isempty(est{k})
        if ~isDetected(k)
            isDetected(k:end) = 1;
            r(:, k) = [gt_target_dynamic(1:2, k) + [0; 10]; 2; 0];
            Cr = diag([10 10 15 0]);
        end
        [r(:,k), p(:,k), Cr, Cp] = measurement_update(est{k}, H, r(:,k), p(:,k), Cr, Cp, Ch, Cv);
        
        [r(:,k+1), p(:,k+1), Cr, Cp] = time_update(r(:,k), p(:,k), Cr, Cp, Ar, Ap, Cwr, Cwp);
    end
    
    exec_time(k) = toc(exec_time_start);

    disp(['Time step ', num2str(k), ...
        ': measurements = ', num2str(size(z{k}, 2)), ...
        ', estimations = ', num2str(num_targets(k)), ...
        ', detections = ', num2str(num_detections(k)), ...
        ', reflection points = ', num2str(num_reflection_points(k)), ...
        ', clutters = ', num2str(num_clutters(k)), ...
        ', execution time = ', num2str(exec_time(k)) , 's']);
    
%% Update MEM-EKF only (if enable)
    if ~isempty(z{k})
        if ~isDetected(k)
            r_memekf(:, k) = [gt_target_dynamic(1:2, k) + [0; 10]; 2; 0];
            Cr_memekf = diag([10 10 15 0]);
        end
        [r_memekf(:,k), p_memekf(:,k), Cr_memekf, Cp_memekf] = measurement_update(z{k}, H_memekf, r_memekf(:,k), ...
                                                        p_memekf(:,k), Cr_memekf, Cp_memekf, Ch_memekf, Cv_memekf);
        
        [r_memekf(:,k+1), p_memekf(:,k+1), Cr_memekf, Cp_memekf] = time_update(r_memekf(:,k), p_memekf(:,k), ...
                                            Cr_memekf, Cp_memekf, Ar_memekf, Ap_memekf, Cwr_memekf, Cwp_memekf);
    end
end

%% Plot pre-setup
r_range = 40;
r_fov = DTB_Sensor.FieldOfView(1) * pi / 180;
r_coverage = cell(duration, 1);

for i = 1:duration
    r_pos =  gt_self_dynamic(1:2,i) + [1; 0];
    if r_fov < 2 * pi
        r_coverage{i} = cat(2, r_coverage{i}, r_pos);
    end
    for j = -r_fov/2 + gt_self_shape(1, i) :r_fov/36: r_fov/2 + gt_self_shape(1, i)
        r_coverage{i} = cat(2, r_coverage{i}, [r_pos(1) + r_range * cos(j); r_pos(2) + r_range * sin(j)]);
    end
    if r_fov < 2 * pi
        r_coverage{i} = cat(2, r_coverage{i}, r_pos);
    end
end

%% Plot and visualize
if doSaveMP4
    videoFile = VideoWriter('simu_DrivingToolbox_SEOT_GM_PHD');
    videoFile.FrameRate = 30;
    open(videoFile);
end

if doPlotScenario
    plot(DTB_Scenario);
    set(gcf, 'Position', [0 0 900 1000]);
    pause(2);

    while advance(DTB_Scenario)
        time_step = round(DTB_Scenario.SimulationTime / DTB_SamplingTime) + 1;
        t = time_step;

        if t <= duration
            hold on;
            plot([0 0], [-40 40], 'k.', 'MarkerSize', 1);
            r_fill = fill(r_coverage{t}(1, :), r_coverage{t}(2, :), 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'r');

            if mod(t, 3) == 0
                if doPlotScenario_Clutters
                    if isInSensorRange(t) && ~isempty(d_p{t})
                        d_p_plot = plot(d_p{t}(1, :), d_p{t}(2, :), 'k.', 'MarkerSize', 8);
                    end
                    c_plot = plot(c{t}(1, :), c{t}(2, :), 'k.', 'MarkerSize', 8);
                end
                if doPlotScenario_Estimates && isDetected(t)
                    est_center_plot = plot(r(1, t), r(2, t), 'r.', 'MarkerSize', 10);
                    est_plot = plot_extent([r(1:2, t); p(:, t)], '-', 'r', 1.5);
                end
            end
            
        end
        if doSaveMP4
            currentFrame = getframe(gcf);
            writeVideo(videoFile, currentFrame);
        end

        pause(.02);
        if t < duration
            if mod(t, 3) == 0
                delete(c_plot);
            end
            delete(r_fill);
        end
    end

    title("Scenario Simulation");
    if doPlotScenario_Clutters && doPlotScenario_Estimates
        legend([est_plot, d_p_plot], 'Estimations', 'Measurements', 'Location', 'east');
    elseif doPlotScenario_Clutters
        legend([z_plot], 'Measurements', 'Location', 'east');
    elseif doPlotScenario_Estimates
        legend([est_plot], 'Estimation', 'Location', 'east');
    end
end

if doPlotSimpleScenario
    figure (2);
    hold on;

    gt_self_plot = plot(gt_self_dynamic(2, :), gt_self_dynamic(1, :), '.b-', 'LineWidth', 1.5, 'MarkerSize', 15);
    gt_target_plot = plot(gt_target_dynamic(2, :), gt_target_dynamic(1, :), '.r-', 'LineWidth', 1.5, 'MarkerSize', 15);
    for i = 1:duration
        meas_plot = plot(z{i}(2, :), z{i}(1, :), 'k.', 'MarkerSize', 5);
    end
    
    axis equal;
    set(gca, 'XDir', 'reverse');
    xlabel("Position Y");
    ylabel("Position X");
    title("Simple Sensor FOV");
    legend([meas_plot, gt_self_plot, gt_target_plot], 'Measurements', 'Self Vehicle', 'Target Vehicle');
end

if doPlot_ExtendedSimpleScenario
    figure (3);
    hold on;
    set(gcf, 'Position', [0 0 900 1000]);

    for i = 1:duration
        if mod(i, 3) == 0
            gt_self_center_plot = plot(gt_self_dynamic(2, i), gt_self_dynamic(1, i), 'b.', 'MarkerSize', 15);
            gt_self_plot = plot_extent([gt_self_dynamic(2, i); gt_self_dynamic(1, i); -(gt_self_shape(1, i) + pi/2); gt_self_shape(2:3, i)]', '-', 'b', 3);
            
            if ~(isDetected(i) && ~isInSensorRange(i))
                gt_target_center_plot = plot(gt_target_dynamic(2, i), gt_target_dynamic(1, i), 'k.', 'MarkerSize', 15);
                gt_target_plot = plot_extent([gt_target_dynamic(2, i); gt_target_dynamic(1, i);  ...
                                                        -(gt_target_shape(1, i)+ pi/2); gt_target_shape(2:3, i)]', '-', 'k', 3);
            end

            if isDetected(i)
                est_center_plot = plot(r(2, i), r(1, i), 'r.', 'MarkerSize', 15);
                est_plot = plot_extent([r(2, i); r(1, i); -(p(1, i) + pi/2); p(2:3, i)], '-', 'r', 1.5);
            end
            
            if isInSensorRange(i) && ~isempty(d_p{i})
                d_p_plot = plot(d_p{i}(2, :), d_p{i}(1, :), 'r*', 'MarkerSize', 3);
            end

            c_plot = plot(c{i}(2, :), c{i}(1, :), 'k.', 'MarkerSize', 4);
        end
    end
    
    axis equal;
    set(gca, 'XDir', 'reverse');
    xlabel("Position Y");
    ylabel("Position X");
    title("Extended Estimation Using GM-PHD Filter");
    legend([est_plot, gt_self_plot, gt_target_plot, c_plot, d_p_plot], 'Estimations', 'Self Vehicle', 'Target Vehicle', ...
                                                    'Clutters', 'Measurements', 'Location', 'southeast');
end

if doSaveMP4
    close(videoFile);
end
%% Evaluation
disp(['-------------------------------------------------Total runtime: ', num2str(sum(exec_time, 1)), ' (s)-----------------------------------------------']);
if doPlotOSPA
    ospa = zeros(duration, 1);
    ospa_cutoff = 15;
    ospa_order = 1;
    for i = 2:duration
        [gt_target_ospa_mat, est_target_ospa_mat] = get_uniform_points_boundary([gt_target_dynamic(1:2, i); gt_target_shape(:, i)]', ...
                                                                            [r(1:2, i); p(:, i)]', 50);
        ospa(i) = ospa_dist(gt_target_ospa_mat, est_target_ospa_mat, ospa_cutoff, ospa_order);
    end

    figure (4);
    hold on;
    ospa_plot = plot(2:duration, ospa(2:end), 'b', 'LineWidth', 1.5);
    xlabel("Time step");
    ylim([0, 18]);
    ylabel("Distance (in m)");
    title("OSPA Evaluation Metric for SEOT GM-PHD");
    legend(ospa_plot, 'Accuracy of thesis estimator', 'Location', 'northeast');

    if doCompareMEMEKF
        ospa_memekf = zeros(duration, 1);
        for i = 2:duration
            [gt_target_ospa_mat_memekf, est_target_ospa_mat_memekf] = get_uniform_points_boundary([gt_target_dynamic(1:2, i); ...
                                                        gt_target_shape(:, i)]', [r_memekf(1:2, i); p_memekf(:, i)]', 50);
            ospa_memekf(i) = ospa_dist(gt_target_ospa_mat_memekf, est_target_ospa_mat_memekf, ospa_cutoff, ospa_order);
        end

        ospa_plot_memekf = plot(2:duration, ospa_memekf(2:end), 'r', 'LineWidth', 1.5);
        legend([ospa_plot, ospa_plot_memekf], 'Accuracy of thesis estimator', 'Accuracy of normal MEM-EKF*', 'Location', 'northeast');
    end
end