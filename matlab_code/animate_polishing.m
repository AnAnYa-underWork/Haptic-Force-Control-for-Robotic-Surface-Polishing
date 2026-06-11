%loading simulink data
t  = out.t_sim;
q1 = out.q1_sim;
q2 = out.q2_sim;
x_data = out.x_sim;
z_data = out.z_sim;
F_data = out.F_sim;


% Robot parameters
l1 = 1;
l2 = 1;
wall_x = 1.75;


z_bins = linspace(-2,2,300);       
polish_intensity = zeros(size(z_bins));

%Animation
figure('Color','w'); hold on;
axis equal
axis([-2 2 -0.5 0.5 -2 2])
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on
view(40,25)
title('Simulink-Driven 3D Polishing Animation')

% Wall surface mesh
[z_wall, y_wall] = meshgrid(-2:0.1:2, -0.2:0.1:0.2);
x_wall = wall_x * ones(size(z_wall));
for k = 1:3:length(t)
    cla;

    
    th1 = q1(k);
    th2 = q2(k);

    x1 = l1*cos(th1);      y1 = 0;   z1 = l1*sin(th1);
    x2 = x1 + l2*cos(th1+th2);
    z2 = z1 + l2*sin(th1+th2);

    touching = false;
    Fn = F_data(k);

    if x2 >= wall_x
        touching = true;
        x2 = wall_x;

        % Force-based darkening
        [~, idx] = min(abs(z_bins - z2));
        polish_intensity(idx) = polish_intensity(idx) + 0.02*Fn;
    end

  
    surf(x_wall, y_wall, z_wall, 'FaceAlpha',0.6, ...
         'EdgeColor','none', 'FaceColor',[0.95 0.95 0.95]);

    for i = 1:length(z_bins)
        if polish_intensity(i) > 0
            darkness = min(polish_intensity(i), 1.0);
            color = [1 - 0.8*darkness, 1 - 0.8*darkness, 1 - darkness];
            plot3(wall_x, 0.03, z_bins(i), '.', ...
                  'Color',color, 'MarkerSize',28);
        end
    end

   
    tL = linspace(0,1,20);

    plot3(tL*x1, zeros(1,20), tL*z1, ...
        'LineWidth',7, 'Color',[0.1 0.4 1]);

    plot3(x1 + tL*(x2-x1), zeros(1,20), z1 + tL*(z2-z1), ...
        'LineWidth',7, 'Color',[0.2 0.2 0.2]);


    if touching
        plot3(x2,0,z2,'o','MarkerSize',12,'MarkerFaceColor','m','MarkerEdgeColor','k');
    else
        plot3(x2,0,z2,'o','MarkerSize',12,'MarkerFaceColor','r','MarkerEdgeColor','k');
    end

    drawnow;
end
