% --- 1. DÉFINITION DE L'ENVIRONNEMENT COMPLEXE ---
taille_grille = 30;
carte = zeros(taille_grille, taille_grille);

% Placement de multiples obstacles ("blocs pas bons")
carte(5:10, 5:15) = 1;   % Obstacle 1 (Mur horizontal)
carte(15:25, 8:10) = 1;  % Obstacle 2 (Mur vertical)
carte(12:14, 18:28) = 1; % Obstacle 3 (Mur horizontal droit)
carte(20:28, 18:20) = 1; % Obstacle 4 (Mur vertical bas)
carte(2:8, 22:25) = 1;   % Obstacle 5 (Bloc isolé)

% Nouveaux points de départ et d'arrivée
depart = [2, 2];
arrivee = [28, 28];

% --- 2. EXÉCUTION DE L'ALGORITHME A* ---
chemin_brut = mon_astar(carte, depart, arrivee);

% Affichage du résultat avec une carte colorée
figure; 
% Création d'une palette de couleurs : [Vert clair (libre), Rouge brique (obstacle)]
map_couleurs = [0.85 0.95 0.85; 0.8 0.2 0.2];
imagesc(carte); 
colormap(map_couleurs); 
hold on;

% Tracé du chemin complet calculé par A*
plot(chemin_brut(:,2), chemin_brut(:,1), 'b-o', 'LineWidth', 2, 'MarkerSize', 4);

% Marquage du départ et de l'arrivée
plot(depart(2), depart(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(arrivee(2), arrivee(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
title('Trajectoire A* optimisée sur carte complexe'); 
grid on;

% --- 3. FILTRAGE DES WAYPOINTS ET EXPORT POUR SIMULINK ---
% Initialisation avec le point de départ
chemin_filtre = chemin_brut(1, :);

% Parcours des points intermédiaires pour détecter les virages
if size(chemin_brut, 1) > 2
    for i = 2:(size(chemin_brut, 1) - 1)
        % Vecteur d'entrée dans le point i
        dx1 = chemin_brut(i, 1) - chemin_brut(i-1, 1);
        dy1 = chemin_brut(i, 2) - chemin_brut(i-1, 2);
        
        % Vecteur de sortie du point i
        dx2 = chemin_brut(i+1, 1) - chemin_brut(i, 1);
        dy2 = chemin_brut(i+1, 2) - chemin_brut(i, 2);
        
        % Si la direction change (les vecteurs sont différents), c'est un virage
        if (dx1 ~= dx2) || (dy1 ~= dy2)
            chemin_filtre = [chemin_filtre; chemin_brut(i, :)];
        end
    end
end
% Ajout du point d'arrivée final
if size(chemin_brut, 1) >= 2
    chemin_filtre = [chemin_filtre; chemin_brut(end, :)];
end

% Affichage des points d'inflexion conservés (waypoints) en jaune
plot(chemin_filtre(:,2), chemin_filtre(:,1), 'y*', 'MarkerSize', 10, 'LineWidth', 1.5);

% Mise à l'échelle (1 case = 0.1 mètre)
resolution = 0.1; 
chemins_X_workspace = chemin_filtre(:, 1) * resolution;
chemins_Y_workspace = chemin_filtre(:, 2) * resolution;

disp(['Chemin filtré avec succès : ' num2str(size(chemin_filtre, 1)) ' points conservés.']);
disp('Variables exportées vers le Workspace. Lancez Simulink.');

% =========================================================================
% FONCTION A* 
% =========================================================================
function chemin = mon_astar(carte, depart, arrivee)
    [lignes, colonnes] = size(carte);
    G = inf(lignes, colonnes); 
    F = inf(lignes, colonnes); 
    parent_x = zeros(lignes, colonnes);
    parent_y = zeros(lignes, colonnes);
    
    ouvert = false(lignes, colonnes);
    ferme = false(lignes, colonnes);
    
    x_dep = depart(1); y_dep = depart(2);
    x_arr = arrivee(1); y_arr = arrivee(2);
    
    G(x_dep, y_dep) = 0;
    F(x_dep, y_dep) = sqrt((x_arr - x_dep)^2 + (y_arr - y_dep)^2); 
    ouvert(x_dep, y_dep) = true;
    
    dx = [-1, 1, 0, 0, -1, -1, 1, 1];
    dy = [0, 0, -1, 1, -1, 1, -1, 1];
    couts = [1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2)];
    
    arrivee_atteinte = false;
    
    while any(ouvert(:))
        F_ouvert = F;
        F_ouvert(~ouvert) = inf; 
        [~, idx] = min(F_ouvert(:));
        [cx, cy] = ind2sub([lignes, colonnes], idx); 
        
        if cx == x_arr && cy == y_arr
            arrivee_atteinte = true;
            break;
        end
        
        ouvert(cx, cy) = false;
        ferme(cx, cy) = true;
        
        for i = 1:8
            nx = cx + dx(i);
            ny = cy + dy(i);
            
            if nx > 0 && nx <= lignes && ny > 0 && ny <= colonnes
                if carte(nx, ny) == 0 && ~ferme(nx, ny)
                    nouveau_g = G(cx, cy) + couts(i);
                    if nouveau_g < G(nx, ny)
                        parent_x(nx, ny) = cx;
                        parent_y(nx, ny) = cy;
                        G(nx, ny) = nouveau_g;
                        H = sqrt((x_arr - nx)^2 + (y_arr - ny)^2);
                        F(nx, ny) = nouveau_g + H;
                        ouvert(nx, ny) = true; 
                    end
                end
            end
        end
    end
    
    chemin = [];
    if arrivee_atteinte
        cx = x_arr; cy = y_arr;
        while cx ~= x_dep || cy ~= y_dep
            chemin = [[cx, cy]; chemin]; 
            px = parent_x(cx, cy);
            py = parent_y(cx, cy);
            cx = px; cy = py;
        end
        chemin = [[x_dep, y_dep]; chemin]; 
    end
end