% Inverse Dynamics for 2R Manipulator with Cycloidal Trajectory
clc; clear; close all;

% ---- Parameters ----
m1 = 1; m2 = 1;       % masses (kg)
l1 = 1; l2 = 1;       % link lengths (m)
r1 = l1/2; r2 = l2/2; % COM distances (uniform rods)
I1 = (1/12)*m1*l1^2;  % inertia about COM
I2 = (1/12)*m2*l2^2;
g = 9.81;

% ---- Cycloidal trajectory settings ----
theta1I = pi/3;   theta1F = pi/10;   % start around 45 deg
theta2I = -pi/4; theta2F = -pi/14;

timeI = 0; timeF = 2; steps = 200;
T = timeF - timeI;
timeArray = linspace(timeI,timeF,steps);

A1 = (theta1F - theta1I)/T;
A2 = (theta2F - theta2I)/T;

% ---- Storage ----
theta1Array = zeros(steps,1);
theta2Array = zeros(steps,1);
theta1DotArray = zeros(steps,1);
theta2DotArray = zeros(steps,1);
theta1DDotArray = zeros(steps,1);
theta2DDotArray = zeros(steps,1);
tauArray = zeros(steps,2);

% ---- Cycloidal joint trajectories + inverse dynamics ----
for k = 1:steps
    t = timeArray(k);
    delT = t - timeI;

    % Joint 1 cycloidal
    th1   = theta1I + A1*(delT - (T/(2*pi))*sin((2*pi/T)*delT));
    th1d  = A1*(1 - cos((2*pi/T)*delT));
    th1dd = (2*pi*A1/T)*sin((2*pi/T)*delT);

    % Joint 2 cycloidal
    th2   = theta2I + A2*(delT - (T/(2*pi))*sin((2*pi/T)*delT));
    th2d  = A2*(1 - cos((2*pi/T)*delT));
    th2dd = (2*pi*A2/T)*sin((2*pi/T)*delT);

    % ---- Dynamics ----
    % Inertia matrix
    M11 = I1+I2 + m1*r1^2 + m2*(l1^2+r2^2) + 2*m2*l1*r2*cos(th2);
    M12 = I2 + m2*r2^2 + m2*l1*r2*cos(th2);
    M21 = M12;
    M22 = I2 + m2*r2^2;
    M = [M11 M12; M21 M22];

    % Coriolis / Centrifugal
    h  = -m2*l1*r2*sin(th2);
    C1 = h*(2*th1d*th2d + th2d^2);
    C2 = h*(-th1d^2);
    C  = [C1; C2];

    % Gravity
    G1 = (m1*r1 + m2*l1)*g*cos(th1) + m2*r2*g*cos(th1+th2);
    G2 = m2*r2*g*cos(th1+th2);
    G  = [G1; G2];

    % Joint torques
    tau = M*[th1dd; th2dd] + C + G;

    % Store
    theta1Array(k)      = th1;
    theta2Array(k)      = th2;
    theta1DotArray(k)   = th1d;
    theta2DotArray(k)   = th2d;
    theta1DDotArray(k)  = th1dd;
    theta2DDotArray(k)  = th2dd;
    tauArray(k,:)       = tau.';
end

%% ---------- Build back-and-forth trajectory for animation ONLY ----------

% --- SMALL “CONTROLLER-LIKE” ERROR ADDED FOR VISUALS ---
% Make the animation look a LITTLE off compared to ideal inverse dynamics
noise1 = 0.04 * sin(4*pi*timeArray/T);          % ~2–3° oscillation
noise2 = -0.03 * sin(3*pi*timeArray/T + 0.7);   % ~2° oscillation

theta1Vis = theta1Array + noise1;
theta2Vis = theta2Array + noise2;

%% ---------- One-way trajectory for animation ONLY ----------
theta1Anim = theta1Array;    % only forward motion
theta2Anim = theta2Array;
stepsAnim  = length(theta1Anim);


%% ---- 3D Animation: vertical plane, grounded, back-and-forth, darkening marks ----
figure('Color','w'); hold on;

wall_x = 1.75;   % wall position in X
baseZ  = 0;      % ground level (Z=0 is ground)
baseX  = 0;      % base at origin in X

% Wall plane (vertical in X-Z, thickness in Y)
[z_wall, y_wall] = meshgrid(-2:0.1:2, -0.2:0.1:0.2);
x_wall = wall_x * ones(size(z_wall));

% Polishing intensity (for darkening marks)
z_bins = linspace(-2,2,200);
polish_intensity = zeros(size(z_bins));

% 3D axis setup
axis equal;
axis([-2 2 -0.5 0.5 -2 2]);   % [Xmin Xmax Ymin Ymax Zmin Zmax]
xlabel('X (towards wall)');
ylabel('Y (sideways)'); 
zlabel('Z (vertical)');
title('3D Polishing Robot (Back-and-forth, Darkening Marks)');
view(40,25);
grid on;

for k = 1:stepsAnim
    th1 = theta1Anim(k);
    th2 = theta2Anim(k);

    % ---- Forward kinematics in X-Z plane (Y = 0), base at (0,0,0) ----
    x1 = baseX + l1*cos(th1);    y1 = 0;    z1 = baseZ + l1*sin(th1);
    x2 = x1 + l2*cos(th1 + th2);
    z2 = z1 + l2*sin(th1 + th2);
    y2 = 0;

    % ---- Clamp at wall (no bypass) & update polishing intensity ----
    touching = false;
    if x2 > wall_x
        touching = true;
        x2 = wall_x;

        % increase intensity at closest z-bin
        [~, idx] = min(abs(z_bins - z2));
        polish_intensity(idx) = polish_intensity(idx) + 1;
    end

    % ---- Clear and redraw scene ----
    cla;

    % Wall
    surf(x_wall, y_wall, z_wall, ...
         'FaceColor',[0.9 0.9 0.9], ...
         'EdgeColor','none', 'FaceAlpha',0.6);
    hold on;

    % Darkening polishing marks on wall
    for i = 1:length(z_bins)
        if polish_intensity(i) > 0
            c = max(0, 1 - 0.1*polish_intensity(i));  % darker each pass
            plot3(wall_x, 0.02, z_bins(i), '.', ...
                  'Color',[c 1 1], 'MarkerSize', 30);
        end
    end

    % Base block at origin
    % Front face
    patch([baseX baseX baseX+0.3 baseX+0.3], ...
          [-0.1 -0.1 -0.1 -0.1], ...
          [baseZ baseZ+0.2 baseZ+0.2 baseZ], ...
          [0.35 0.35 0.35],'EdgeColor','none');
    % Top plate
    patch([baseX baseX baseX+0.3 baseX+0.3], ...
          [-0.1 0.1 0.1 -0.1], ...
          [baseZ baseZ baseZ baseZ], ...
          [0.4 0.4 0.4],'EdgeColor','none');

    % ---- Smooth link 1 ----
    tL = linspace(0,1,20);
    xL1 = baseX + tL*(x1 - baseX);
    yL1 = zeros(size(tL));
    zL1 = baseZ + tL*(z1 - baseZ);
    plot3(xL1, yL1, zL1, 'LineWidth',7, 'Color',[0.15 0.45 1]);

    % ---- Smooth link 2 ----
    xL2 = x1 + tL*(x2 - x1);
    yL2 = zeros(size(tL));
    zL2 = z1 + tL*(z2 - z1);
    plot3(xL2, yL2, zL2, 'LineWidth',7, 'Color',[0.25 0.25 0.25]);

    % ---- Tool ----
    if touching
        plot3(x2, y2, z2, 'o', 'MarkerSize',12, ...
              'MarkerFaceColor','m','MarkerEdgeColor','k');
    else
        plot3(x2, y2, z2, 'o', 'MarkerSize',12, ...
              'MarkerFaceColor','r','MarkerEdgeColor','k');
    end

    % Axes & view each frame
    axis([-2 2 -0.5 0.5 -2 2]);
    view(40,25);
    grid on;

    drawnow;
end
