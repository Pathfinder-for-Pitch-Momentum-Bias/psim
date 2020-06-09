clc;

addpath('../');

global const

% initialize constants
config();
m = 3.7; % Mass (kg)
% J_min = 1.25e-4; % Min impulse (Ns) 5ms ontime
J_min = 0;
% J_max = 2.5e-2; % Max impulse (Ns) 1s ontime
J_max = 2.5;
max_dv = J_max / m;
min_dv = J_min / m;
p = 1.0e-5;
d = 1.0e-2;
energy_gain = 5.0e-5;
h_gain = 2.0e-3;
thrust_noise_ratio = 0;
dt_fire_min = 5 * 60; % [s] minimum time between firings

% time
tmax = 80 * 24 * 60 * 60; % [s]
dt = 10; % [s]
t = 0 : dt : tmax; % [s]
N = length(t);

% initial orbital elements
a  = 6793137.0;  % Semimajor axis                        (m)
e  = 0.001;      % Eccentricity                          (unitless)
I  = 45*pi/180;  % Inclination angle                     (rad)
O  = 0.0;        % Right ascension of the ascending node (rad)
o  = 0.0;        % Argument of perigee                   (rad)
nu = 0*pi/180;   % True anamoly                          (rad)
[   r1,...  % Position (m)   [eci]
    v1,...  % Velocity (m/s) [eci]
] = utl_orb2rv(a * (1 - e*e), e, I, O, o, nu, const.mu);

n = utl_orbrate(a);      % [rad / s] Orbital rate
w_hill = [0.0; 0.0; n];

% dispenser dynamics
energy_max = 12.5 + 0.2 * 12.5; % [J]
energy_min = 12.5 - 0.2 * 12.5; % [J]
v_max = sqrt(2 * energy_max / m); % [m/s]
v_min = sqrt(2 * energy_min / m); % [m/s]
v_rel = v_max - v_min;

% Add initial velocity difference
r_init = r1;
v_init = v1;
r2 = r1;
v2 = v1; % in the vhat direction
v2 = v1 + v_rel * (v2 / norm(v2));

% Allow spacecraft to drift apart
[~, r1, v1, r2, v2] = drift_phase(r1, v1, r2, v2, t_drift);

% add measurement noise (zero for now)
r1_measure = r1; % +0.1*randn(3,1);
r2_measure = r2; % +0.1*randn(3,1);
v1_measure = v1; % +0.001*randn(3,1);
v2_measure = v2; % +0.001*randn(3,1);

% initialize arrays
X = zeros(6, N);
deltaenergies = zeros(N, 1);
u_vec = zeros(N, 1);
r_eci = zeros(6, N);
dv_vec = zeros(3, N);
dv_norm = zeros(N, 1);
r_fire = [];
dv_des = zeros(N, 1);
de_point_mass = zeros(N, 1);
energy1_vec = zeros(N, 1);
energy1_j1_vec = zeros(N, 1);
energy2_vec = zeros(N, 1);
energy2_j1_vec = zeros(N, 1);
dh_vec = zeros(3, N);
h1_vec = zeros(3, N);
h2_vec = zeros(3, N);
dh_angle = zeros(N, 1);
dv_p_vec = zeros(N, 1);
dv_d_vec = zeros(N, 1);
dv_E_vec = zeros(N, 1);
dv_h_vec = zeros(N, 1);

% Calculate initial state
Q_eci_hill = utl_eci2hill(r1, v1);
r_hill = Q_eci_hill * (r2 - r1);
v_hill = Q_eci_hill * (v2 - v1) - cross(w_hill, r_hill);
X(:, 1) = [r_hill; v_hill];
r_eci(:, 1) = [r1; r2];

% ECI to ECEF conversion
[quat_ecef_eci, ~] = env_earth_attitude(t(1));

% calculate energy
r1_ecef = utl_rotateframe(quat_ecef_eci, r1);
r2_ecef = utl_rotateframe(quat_ecef_eci, r2);
[~,potential,~] = env_gravity(t(1), r1_ecef);
energy1 = 0.5 * dot(v1, v1) - potential;
[~,potential,~] = env_gravity(t(1), r2_ecef);
energy2= 0.5*dot(v2,v2)-potential;
deltaenergies(1)=energy2-energy1;
energy1_vec(1) = energy1;
energy2_vec(1) = energy2;

% calculate orbital angular momentum
h1 = cross(r1, v1);
h2 = cross(r2, v2);
dh = h1 - h2;
h1hat = h1 / norm(h1);
h2hat = h2 / norm(h2);
dh_angle(1) = acos(dot(h1hat, h2hat));
dh_vec(:, 1) = dh;
h1_vec(:, 1) = h1;
h2_vec(:, 1) = h2;

t_fire = -dt_fire_min; % set so we fire on the first orbit
tsteps_in_docking_range = 0;

for i = 1 : N - 1
    
    if ~mod(i, 10000)
        fprintf('progress: step %d / %d, run %d / %d\n', i, N - 1, ii, N_runs);
    end
    
    % check if we're in docking range
    if norm(X(1 : 3, i)) < 10 && norm(X(4:6, i)) < 0.1
        tsteps_in_docking_range = tsteps_in_docking_range + 1;
        if tsteps_in_docking_range > 6 * 60 % more than 1 hour in docking range
            break;
        end
    else
        tsteps_in_docking_range = 0;
    end

    % calculate follower orbital elements
    a = -const.mu / (2 * energy2);
    n = utl_orbrate(a);
    M = n * t(i);
    M = mod(M, 2 * pi);
    
    % calculate eccentric anomaly
    del = 1;
    E = M / (1 - e);
    
    if E > sqrt(6 * (1 - e) / e)
        E = (6 * M / e) ^ (1/3);
    end
    
    while del > eps(2 * pi)
        
        E = E - (M - E + e * sin(E)) / (e * cos(E) - 1);
        del = max(abs(M - (E - e * sin(E))));
        
    end
    
    % check if follower is at a firing point
    if mod(E, node_angle) < 0.01 && t(i) - t_fire > dt_fire_min
%     if (abs(E) < 0.01 || abs(E - angles(ii)) < 0.01) && t(i) - t_fire > dt_fire_min
        
        % record firing time and position
        t_fire = t(i);
        r_fire = [r_fire, [r1; r2]];

        % Hill frame PD controller
        pterm = p * r_hill(2);
        dterm = -d * v_hill(2);
        dv_p = pterm * v2 / norm(v2);
        dv_d = dterm * v2 / norm(v2);
        
        % energy controller
        energy_term = -energy_gain * (energy2 - energy1);
        dv_energy = energy_term * v2 / norm(v2);

        % h controller
        r2hat = r2 / norm(r2);

        % define the direction of our impulse, always in the h2 direction
        Jhat_plane = h2hat;

        % project h1 onto the plane formed by h2 and (r2 x h2)
        h1proj = h1 - dot(h1, r2hat) * r2hat;
        
        % calculate the angle between h1proj and h2 (this is what we are
        % driving to zero with this burn)
        theta = atan( dot(h1proj, cross(r2, h2)) / dot(h1proj, h2) );
        
        % scale the impulse delivered by the angle theta
        J_plane = theta * Jhat_plane;
        
        % scale dv by h_gain
        dv_plane = h_gain * J_plane / m;
        
        % total dv to be applied from all controllers
        dv = dv_p + dv_d + dv_energy + dv_plane;
        
        dv_p_vec(i) = norm(dv_p);
        dv_d_vec(i) = norm(dv_d);
        dv_E_vec(i) = norm(dv_energy);
        dv_h_vec(i) = norm(dv_plane);
        dv_des(i) = norm(dv);
        
        % thruster saturation
        if norm(dv) > max_dv
            dv = max_dv * dv / norm(dv);
        end
        
        if norm(dv) < min_dv
            dv = min_dv * dv / norm(dv);
        end
        
        % apply dv
        v2 = v2 + dv;
        dv_vec(:, i) = dv;
        dv_norm(i) = norm(dv);
        
    end

    % simulate dynamics
    y = utl_ode4(@(t, y) frhs(t, y, quat_ecef_eci), [0.0, dt], [r1; v1; r2; v2]);
    r1 = y(end, 1:3)';
    v1 = y(end, 4:6)';
    r2 = y(end, 7:9)';
    v2 = y(end, 10:12)';
    
    % Calculate new state
    w_hill = [0.0; 0.0; n];
    Q_eci_hill = utl_eci2hill(r1, v1);
    r_hill = Q_eci_hill * (r2 - r1);
    v_hill = Q_eci_hill * (v2 - v1) - cross(w_hill, r_hill);
    X(:, i + 1) = [r_hill; v_hill];
    r_eci(:, i + 1) = [r1; r2];
    [quat_ecef_eci, ~] = env_earth_attitude(t(i + 1));
    
    % calculate energies
    r1_ecef = utl_rotateframe(quat_ecef_eci, r1);
    r2_ecef = utl_rotateframe(quat_ecef_eci, r2);
    [~,potential,~]= env_gravity(t(i + 1), r1_ecef);
    energy1= 0.5*dot(v1,v1)-potential;
    [~,potential,~]= env_gravity(t(i + 1), r2_ecef);
    energy2= 0.5*dot(v2,v2)-potential;
    deltaenergies(i + 1)=energy2-energy1;
    energy1_vec(i + 1) = energy1;
    energy2_vec(i + 1) = energy2;

    % calculate orbital angular momentum
    h1 = cross(r1, v1);
    h2 = cross(r2, v2);
    h1hat = h1 / norm(h1);
    h2hat = h2 / norm(h2);
    dh_angle(i + 1) = acos(dot(h1hat, h2hat));
    dh = h1 - h2;
    dh_vec(:, i + 1) = dh;
    h1_vec(:, i + 1) = h1;
    h2_vec(:, i + 1) = h2;
    
end

function dy = frhs(t, y, quat_ecef_eci)

% global const

quat_eci_ecef = utl_quat_conj(quat_ecef_eci);

dy = zeros(12, 1);

r_ecef = utl_rotateframe(quat_ecef_eci, y(1:3, 1));
[g_ecef, ~, ~] = env_gravity(0, r_ecef);

% [gx, gy, gz] = gravityzonal(y(1:3)', 'Earth', 4, 'Error');
dy(1:3, 1) = y(4:6, 1);
dy(4:6, 1) = utl_rotateframe(quat_eci_ecef, g_ecef);
% dy(4:6, 1) = [gx; gy; gz];
% dy(4:6, 1) = -const.mu * y(1:3, 1) / norm(y(1:3, 1))^3;

r_ecef = utl_rotateframe(quat_ecef_eci, y(7:9, 1));
[g_ecef, ~, ~] = env_gravity(0, r_ecef);

% [gx, gy, gz] = gravityzonal(y(7:9)', 'Earth', 4, 'Error');
dy(7:9, 1) = y(10:12, 1);
dy(10:12, 1) = utl_rotateframe(quat_eci_ecef, g_ecef);
% dy(10:12, 1) = [gx; gy; gz];
% dy(10:12, 1) = -const.mu * y(7:9, 1) / norm(y(7:9, 1))^3;

end