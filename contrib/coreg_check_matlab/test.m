%% Load the data
if ispc
    data_path = fullfile(getenv('APPDATA'), '.psmoveapi');
else
    data_path = fullfile(getenv('HOME'), '.psmoveapi');
end
poses = readtable(fullfile(data_path, 'output.txt'));
camera_pose = csvread(fullfile(data_path, 'output_camerapose.csv'));
camera_4x4 = mat4fromvec7(camera_pose);
camera_invxform = [camera_4x4(1:3,1:3)', -camera_4x4(1:3,1:3)'*camera_4x4(1:3,4); 0 0 0 1];

n = size(poses, 1);

%% Calculate the transforms
A = nan(3*n, 15);
b = nan(3*n, 1);
for p = 1:n
    dk2vec = [...
        poses.dk2_px(p) poses.dk2_py(p) poses.dk2_pz(p) ...
        poses.dk2_ow(p) poses.dk2_ox(p) poses.dk2_oy(p) poses.dk2_oz(p)];
    dk2mat = mat4fromvec7(dk2vec);
    dk2mat = camera_invxform * dk2mat;
    RMi = dk2mat(1:3, 1:3)';
    
    psmovevec = [...
        poses.psm_px(p) poses.psm_py(p) poses.psm_pz(p) ...
        poses.psm_ow(p) poses.psm_ox(p) poses.psm_oy(p) poses.psm_oz(p)];
    psmovemat = mat4fromvec7(psmovevec);
    
    A((p-1)*3 + (1:3), :) = [RMi * psmovemat(1,4), RMi * psmovemat(2,4), RMi * psmovemat(3,4), RMi, -eye(3)];
    b((p-1)*3 + (1:3)) = RMi * dk2mat(1:3, 4);
end

x = A\b;

%x = pinv(A)*b;

%[Q,R,P] = qr(A);
%x = R\(R'\(A'*b));

globalxfm = reshape(x(1:12), 3, 4);
localxfm = [1 0 0 x(12); 0 1 0 x(13); 0 0 1 x(14); 0 0 0 1];

clear p h RMi Ti x

%% Plot the result
dk2_xyz = [poses.dk2_px, poses.dk2_py, poses.dk2_pz]';
%dk2_xyz = camera_invxform(1:3,:) * [dk2_xyz; ones(1, n)];
psm_xyz = [poses.psm_px, poses.psm_py,  poses.psm_pz]';
lims = [min([psm_xyz'; dk2_xyz']); max([psm_xyz'; dk2_xyz'])];
subplot(2,2,1)
plot3(dk2_xyz(1,:), dk2_xyz(3,:), dk2_xyz(2,:), 'k',...
    psm_xyz(1,:), psm_xyz(3,:), psm_xyz(2,:), 'm',...
    'LineWidth', 3)
xlabel('X')
ylabel('Z')
zlabel('Y')
xlim([lims(1,1) lims(2,1)]);
ylim([lims(1,3) lims(2,3)]);
zlim([lims(1,2) lims(2,2)]);
legend('DK2', 'PSMove', 'Location', 'North')
legend('boxoff')
set(gca, 'FontSize', 14)
set(gca, 'Color', 'none')
%set(gca, 'CameraViewAngle', 8.7)

subplot(2,2,2)
hist(sqrt(sum((dk2_xyz - psm_xyz).^2)));
xlabel('Eucl. Distance (cm)')
ylabel('Count')
set(gca, 'Color', 'none')
set(gca, 'FontSize', 14)
title('Before Coreg.')
box off

% Put both dk2 and psm in dk2_cam reference frame.
tdk2_xyz = camera_invxform * [dk2_xyz; ones(1, n)];
tdk2_xyz = tdk2_xyz(1:3, :);
tpsm_xyz = [globalxfm; 0 0 0 1] * [psm_xyz; ones(1, n)];
tpsm_xyz = tpsm_xyz(1:3, :);
% They could both be put in dk2_native reference frame by left multiplying
% with camera_4x4.
subplot(2,2,3)
plot3(tdk2_xyz(1,:), tdk2_xyz(3,:), tdk2_xyz(2,:), 'k',...
    tpsm_xyz(1,:), tpsm_xyz(3,:), tpsm_xyz(2,:), 'm',...
    'LineWidth', 3)
xlabel('X')
ylabel('Z')
zlabel('Y')
lims = [min([tpsm_xyz'; tdk2_xyz']); max([tpsm_xyz'; tdk2_xyz'])];
xlim([lims(1,1) lims(2,1)]);
ylim([lims(1,3) lims(2,3)]);
zlim([lims(1,2) lims(2,2)]);
set(gca, 'FontSize', 14)
set(gca, 'Color', 'none')
%set(gca, 'CameraViewAngle', 8.7)

subplot(2,2,4)
hist(sqrt(sum((tdk2_xyz - tpsm_xyz(1:3,:)).^2)));
xlabel('Eucl. Distance (cm)')
ylabel('Count')
title('After Coreg.')
set(gca, 'Color', 'none')
set(gca, 'FontSize', 14)
box off