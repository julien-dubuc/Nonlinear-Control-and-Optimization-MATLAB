%% PLANIFICATION A* + EXPORT DES WAYPOINTS POUR SIMULINK
% Convention : carte(i,j) -> i = coordonnée X (lignes), j = coordonnée Y (colonnes)
%              1 case = res mètres ; le centre de la case (i,j) est en (i*res, j*res)
%              Le robot va de la GAUCHE (X petit) vers la DROITE (X grand).

%% 1. Environnement : arène 2 m x 2 m avec 2 obstacles espacés au centre
N   = 20;                 % la grille fait N x N cases
res = 0.1;                % taille d'une case (m)

%            i_min i_max  j_min j_max
obstacles = [  9    12     3     8 ;      % obstacle du bas
               9    12    13    18 ];     % obstacle du haut  (passage entre les deux)

depart  = [2  6];         % case de départ  (à gauche)
arrivee = [19 14];        % case d'arrivée  (à droite)

carte = zeros(N, N);
for k = 1:size(obstacles, 1)
    carte(obstacles(k,1):obstacles(k,2), obstacles(k,3):obstacles(k,4)) = 1;
end

% Marge de sécurité : on grossit chaque obstacle d'une case, pour que le
% robot (qui a une taille et qui coupe un peu les virages) ne les frôle pas.
carte_gonflee = conv2(carte, ones(3), 'same') > 0;

%% 2. A*
chemin_brut = mon_astar(carte_gonflee, depart, arrivee);
if isempty(chemin_brut)
    error('A* : aucun chemin trouvé entre le départ et l''arrivée.');
end

%% 3. Ne garder que les virages (waypoints utiles)
pas    = diff(chemin_brut, 1, 1);               % déplacement entre 2 cases successives
virage = any(diff(pas, 1, 1) ~= 0, 2);          % la direction change au point suivant
idx    = [1; find(virage) + 1; size(chemin_brut, 1)];
chemin_filtre = chemin_brut(idx, :);

%% 4. Export vers Simulink (en mètres)
chemins_X_workspace = chemin_filtre(:, 1) * res;
chemins_Y_workspace = chemin_filtre(:, 2) * res;
% Pensez à régler la position initiale des intégrateurs x et y de Simulink à
% (chemins_X_workspace(1), chemins_Y_workspace(1)).

%% 4bis. Les murs vus par le capteur de distance (variable "murs", utilisée par l'estimateur)
% Un mur = segment [a_x a_y b_x b_y], orienté pour que l'espace LIBRE soit
% à sa GAUCHE. Alors d = det(u, P-a) est positif quand le robot est du bon côté
% (et négatif s'il est derrière le mur : on ne le "voit" pas).
%   - arène : sens trigonométrique (anti-horaire)
%   - obstacles : sens horaire
xmin = 0.5 * res;   xmax = (N + 0.5) * res;     % bords des cases de la grille
ymin = xmin;        ymax = xmax;

murs = [xmin ymin  xmax ymin;                   % bas
        xmax ymin  xmax ymax;                   % droite
        xmax ymax  xmin ymax;                   % haut
        xmin ymax  xmin ymin];                  % gauche
for k = 1:size(obstacles, 1)
    x1 = (obstacles(k,1) - 0.5) * res;   x2 = (obstacles(k,2) + 0.5) * res;
    y1 = (obstacles(k,3) - 0.5) * res;   y2 = (obstacles(k,4) + 0.5) * res;
    murs = [murs;
            x1 y1  x1 y2;                       % côté gauche
            x1 y2  x2 y2;                       % côté haut
            x2 y2  x2 y1;                       % côté droit
            x2 y1  x1 y1];                      % côté bas
end

%% 5. Affichage (X en horizontal, comme le XY Graph de Simulink)
figure('Name', 'A*');
axe = res * (1:N);
imagesc(axe, axe, (carte_gonflee + carte)');    % 0 libre, 1 marge, 2 obstacle
set(gca, 'YDir', 'normal'); axis equal tight; hold on; grid on;
colormap([1 1 1; 0.85 0.85 0.85; 0.3 0.3 0.3]);
plot(chemin_brut(:,1)*res,   chemin_brut(:,2)*res,   'b-o', 'LineWidth', 1.5);
plot(chemins_X_workspace, chemins_Y_workspace, 'r*', 'MarkerSize', 10);
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('A* : %d points bruts, %d waypoints gardés', ...
      size(chemin_brut,1), size(chemin_filtre,1)));

%% =========================================================================
%  FONCTION A*
%  G = coût déjà parcouru, H = distance à vol d'oiseau jusqu'à l'arrivée,
%  F = G + H : on développe toujours la case ouverte qui a le plus petit F.
%  =========================================================================
function chemin = mon_astar(carte, depart, arrivee)
    [nl, nc] = size(carte);
    G      = inf(nl, nc);
    F      = inf(nl, nc);
    parent = zeros(nl, nc);                 % index de la case d'où l'on vient
    ouvert = false(nl, nc);                 % cases à explorer
    ferme  = false(nl, nc);                 % cases déjà explorées

    H = @(i, j) hypot(arrivee(1) - i, arrivee(2) - j);

    G(depart(1), depart(2)) = 0;
    F(depart(1), depart(2)) = H(depart(1), depart(2));
    ouvert(depart(1), depart(2)) = true;

    % 8 voisins : [di dj coût]  (les diagonales coûtent sqrt(2))
    voisins = [-1  0 1;   1  0 1;   0 -1 1;   0  1 1;
               -1 -1 sqrt(2);  -1  1 sqrt(2);   1 -1 sqrt(2);   1  1 sqrt(2)];

    trouve = false;
    while any(ouvert(:))
        % case ouverte de plus petit F
        Fo = F;  Fo(~ouvert) = inf;
        [~, idx] = min(Fo(:));
        [ci, cj] = ind2sub([nl nc], idx);

        if ci == arrivee(1) && cj == arrivee(2)
            trouve = true;
            break;
        end
        ouvert(ci, cj) = false;
        ferme(ci, cj)  = true;

        for k = 1:8
            ni = ci + voisins(k, 1);
            nj = cj + voisins(k, 2);

            if ni < 1 || ni > nl || nj < 1 || nj > nc, continue; end   % hors grille
            if carte(ni, nj) == 1 || ferme(ni, nj),    continue; end   % obstacle / déjà vu

            % une diagonale ne doit pas couper le coin d'un obstacle
            if voisins(k,1) ~= 0 && voisins(k,2) ~= 0 && ...
               (carte(ci, nj) == 1 || carte(ni, cj) == 1)
                continue;
            end

            g = G(ci, cj) + voisins(k, 3);
            if g < G(ni, nj)                       % meilleur chemin vers cette case
                G(ni, nj)      = g;
                F(ni, nj)      = g + H(ni, nj);
                parent(ni, nj) = idx;
                ouvert(ni, nj) = true;
            end
        end
    end

    % Reconstruction du chemin en remontant les parents
    chemin = [];
    if ~trouve, return; end
    k = sub2ind([nl nc], arrivee(1), arrivee(2));
    k_dep = sub2ind([nl nc], depart(1), depart(2));
    while k ~= k_dep
        [i, j] = ind2sub([nl nc], k);
        chemin = [i j; chemin];
        k = parent(k);
    end
    chemin = [depart; chemin];
end