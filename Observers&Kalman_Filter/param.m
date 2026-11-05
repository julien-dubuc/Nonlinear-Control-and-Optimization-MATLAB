% --- Condition initiale "vraie" du bateau ---
x0     = 0.5;
y0     = 0.5;
alpha0 = 0;              % cap initial (rad)
X0 = [x0; y0; alpha0];

% --- Entrée u(t) = vitesse angulaire ---
% Exemple simple : sinusoïde lente
u_amp  = 0.3;            % rad/s
u_freq = 0.1;            % Hz
u_bias = 0.0;

% --- Bruit de mesure (sur beta_A, beta_B) ---
noise_on  = true;        % true/false
sigma_meas = 0.01;       % rad (écart-type)

% --- Bruit de processus pour l'EKF (Q) ---
q_pos   = 1e-3;
q_alpha = 1e-2;
Q_ekf   = diag([q_pos, q_pos, q_alpha]);

% --- Bruit de mesure pour l'EKF (R) ---
R_ekf   = diag([sigma_meas^2, sigma_meas^2]);

% --- Condition initiale de l'EKF (volontairement décalée) ---
Xhat0 = [0.3; 0.7; 0.2];   % estimation initiale
P0    = diag([0.5^2, 0.5^2, (pi/4)^2]);
