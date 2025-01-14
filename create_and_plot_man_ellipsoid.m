function create_and_plot_man_ellipsoid (W, Pe, colour)
    % Generate points for the ellipsoid
    [U, D] = eig(W);  % Eigen-decomposition of W
    
    % Create the ellipsoid points
    [ellipsoid_x, ellipsoid_y, ellipsoid_z] = ellipsoid(0, 0, 0, sqrt(D(1,1)), ...
        sqrt(D(2,2)), sqrt(D(3,3)));
    
    % Transform the ellipsoid based on eigenvectors
    ellipsoid_points = [ellipsoid_x(:), ellipsoid_y(:), ellipsoid_z(:)] * U';
    ellipsoid_x_transformed = reshape(ellipsoid_points(:, 1), size(ellipsoid_x));
    ellipsoid_y_transformed = reshape(ellipsoid_points(:, 2), size(ellipsoid_y));
    ellipsoid_z_transformed = reshape(ellipsoid_points(:, 3), size(ellipsoid_z));
    
    % Translate the ellipsoid to the end-effector position
    ellipsoid_x_transformed = ellipsoid_x_transformed + Pe(1);
    ellipsoid_y_transformed = ellipsoid_y_transformed + Pe(2);
    ellipsoid_z_transformed = ellipsoid_z_transformed + Pe(3);
    
    % Plot the manipulability ellipsoid
    surf(ellipsoid_x_transformed, ellipsoid_y_transformed, ellipsoid_z_transformed, ...
        'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', colour);
    
end




