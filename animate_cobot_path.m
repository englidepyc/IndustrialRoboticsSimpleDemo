function animate_cobot_path(video_name, positions)

% Assicuriamoci che l'ingresso abbia le dimensioni 3xN
assert(size(positions, 1) == 3, 'Positions matrix must have 3 rows for x, y, z coordinates.');

% Creiamo il nostro robot, e ne prendiamo i gradi di libertà.
Robot = make_robot();
dof = Robot.n;

% Pre-allochiamo la matrice che immagazzinerà il percorso nello spazio dei
% giunti, per ragioni di performance.
num_points = size(positions, 2);
path = zeros(num_points, dof);

% Calcoliamo i vettori tangenti a ciascun punto del percorso, tramite le
% differenze finite che approssimano le derivate.
tangents = zeros(3, num_points);
for k = 1:num_points
    if k == 1
        t = positions(:, 2) - positions(:, 1);
    elseif k == num_points
        t = positions(:, end) - positions(:, end-1);
    else
        t = positions(:, k+1) - positions(:, k-1);
    end
    tangents(:, k) = t / norm(t);
end

% Calcoliamo la normale al percorso in ciascun punto, in maniera analoga.
normals = zeros(3, num_points);
for k = 1:num_points
    if k == 1
        n = tangents(:, k+1) - tangents(:, k);
    elseif k == num_points
        n = tangents(:, k) - tangents(:, k-1);
    else
        n = tangents(:, k+1) - tangents(:, k-1);
    end
    if n == 0
        % Nei segmenti retti, n è nullo e va gestito in qualche modo.
        % Scegliamo di mantenere la normale precedente, oppure nel caso in
        % cui non esista di allinearci arbitrariamente l'asse y del
        % basamento.
        if k == 1
            normals(:, k) = [0, 1, 0];
        else
            normals(:, k) = normals(:, k - 1);
        end
    else
        normals(:, k) = n / norm(n);
    end
end

for k = 1:num_points
    t = tangents(:, k); % Tangente.
    n = normals(:, k); % Normale.
    b = cross(t, n);  % Binormale.
    b = b / norm(b);  % Normalizziamo la binormale.
    n = cross(b, t);  % Per stabilità numerica, ricalcoliamo la normale.
    
    % Costruiamo matrice di rotazione.
    R = [t, n, b];
    
    % Combiniamo rotazione e traslazione in una matrice omogenea.
    target_T = [R, positions(:, k); 0, 0, 0, 1];
    
    % Risolviamo la cinematica inversa.
    next_q = Robot.ikine6s(target_T, 'mask', [1 1 1 1 1 1]);
    
    % Immagazziniamo la configurazione ottenuta.
    path(k, :) = next_q;
end

% Plottiamo il robot che segue il percorso dato, salvandolo come file e
% plottando anche la traccia dell'end-effector.
f = gcf;
f.WindowState = 'fullscreen';
drawnow;
Robot.plot(path, ...
    'fps', 60, ...
    'movie', 'Cobot_' + video_name + '.mp4', ...
    'trail', 'r' ...
);
close all;
beep;