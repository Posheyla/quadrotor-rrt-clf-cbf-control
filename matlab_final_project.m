%% =============== SETUP ======================
addpath('C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab')

client = RemoteAPIClient();
sim    = client.require('sim');

% === robots in CoppeliaSim ===
quad_qp = sim.getObject('/Quadcopter_QP');          % QP model
if isempty(quad_qp); error('Object /Quadcopter_QP not found'); end

quad_pd = sim.getObject('/Quadcopter_PD');          % PD model
if isempty(quad_pd); error('Object /Quadcopter_PD not found'); end

% === general params ===
m   = sim.getShapeMass(quad_qp);
g   = 9.81;
e3  = [0 0 1];

alpha_T = 0.5;

% Inertia
J   = diag([6.667e-05 0.00753 0.00753]); % from CoppeliaSim

% PD position
Kp_pos = diag([1.2 1.2 1.4]);  
Kd_pos = diag([0.9 0.9 2.2]);   

% PD orientation
Kp_ang = diag([4.0 4.0 0.3]);  
Kd_ang = diag([0.4 0.4 0.3]);

%========= Obstacles =============
%big walls
obs_list_walls = [
    -1.975  4.875  1.2
     1.975  4.875  1.2
     4.875  3.1    1.2
     4.875 -3.1    1.2
     1.950 -4.875  1.2
    -2.025 -4.875  1.2
    -4.9   -3.1    1.2
    -4.9    3.1    1.2
];
%small objects
obs_list = [
    -3.725  3.650  0.165    % plant
    -0.625  4.500  0.985    % bookshelf
     2.750  2.725  0.7      % table
     2.125  2.700  0.45     % chair
     3.325  2.700  0.45     % chair
     2.725 -0.175  1.2      % circular wall
     1.700 -3.100  0.5      % box
     3.975 -4.000  0.165    % plant
    -2.875 -0.875  1.2      %small wall
    -2.875 -1.825  1.2      %small wall
    -1.926 -2.350  1.2      %small wall
    -1.050 -1.925  1.2      %small wall
    -0.625 -0.460  1.2      %small wall
     0.095  0.075  1.2      %small wall
     0.100  1.025  1.2      %small wall
    -0.350  1.450  1.2      %small wall
    -1.350  1.450  1.2      %small wall
    -2.325  1.450  1.2      %small wall
    -2.875  1.025  1.2      %small wall
    -2.875  0.075  1.2      %small wall
];

% radius for CBF
r_safe_walls   = 0.8;
r_safe_objects = 0.8;

%% =============== RRT* 3D IN (x,y,z) =================

z_start = 0.5;
z_end   = 0.3;

start_xyz = [-4.7, -0.15, z_start];
goal_xyz  = [ 4.5, -0.125, z_end];

x_limits = [-5.5, 5.5];
y_limits = [-5.5, 5.5];
z_limits = [0.2, 1.8];   

margin_plan   = 0.0;
r_plan_walls  = r_safe_walls   + margin_plan;
r_plan_objs   = r_safe_objects + margin_plan;

maxIter    = 3000;
step_size  = 0.6;
goal_radius= 0.6;
near_radius= 1.0;

% obstacles in 3D for planning 
obst_xyz = [obs_list; obs_list_walls];
obsta_r  = [ r_plan_objs*ones(size(obs_list,1),1); ...
             r_plan_walls*ones(size(obs_list_walls,1),1) ];

[path_xyz, ~] = rrt_star_3d(start_xyz, goal_xyz, ...
                            obst_xyz, obsta_r, ...
                            x_limits, y_limits, z_limits, ...
                            maxIter, step_size, goal_radius, near_radius);

if isempty(path_xyz)
    error('RRT* 3D could not find a path');
end

% For xy plots
path_xy = path_xyz(:,1:2);

%% =============== WAYPOINTS 3D  =================
Nw       = size(path_xyz,1);
waypoint = path_xyz;   % [x y z] directamente desde RRT* 3D

%% =============== SPLINES FOR REFERENCE =================
simTime = 28; %simulation time
t_ways  = linspace(0, simTime, Nw);

ppx  = spline(t_ways, waypoint(:,1)');
ppy  = spline(t_ways, waypoint(:,2)');
ppz  = spline(t_ways, waypoint(:,3)');

ppvx = fnder(ppx);
ppvy = fnder(ppy);
ppvz = fnder(ppz);

% second derivatives (for a_ref inCLF 3D)
ppax = fnder(ppvx);
ppay = fnder(ppvy);
ppaz = fnder(ppvz);

yaw_ref = 0;

dt        = 0.05;
time_vec  = 0:dt:simTime;
numSteps  = numel(time_vec)-1;
tsol      = time_vec(:);

%% =============== PARAMS CLF–CBF–QP =================
Qp    = diag([1 1 1]);
Qv    = diag([0.5 0.5 0.5]);
c_clf = 0.8;
rho   = 10;

k0_cbf = 6.0; %to better see CBF behavior
k1_cbf = 4.5; %to better see CBF behavior

H       = blkdiag(2*eye(3), 2*rho);
qp_opts = optimoptions('quadprog','Display','off');

F_lim   = 3.0;   % use acc_lim in QP

% Params for ODE
params.m   = m;
params.g   = g;
params.alpha_T = alpha_T;
params.J   = J;
params.Kp_pos = Kp_pos;
params.Kd_pos = Kd_pos;
params.Kp_ang = Kp_ang;
params.Kd_ang = Kd_ang;
params.obs_list       = obs_list;
params.obs_list_walls = obs_list_walls;
params.r_safe_objects = r_safe_objects;
params.r_safe_walls   = r_safe_walls;
params.Qp = Qp; params.Qv = Qv;
params.c_clf = c_clf;
params.k0_cbf = k0_cbf;
params.k1_cbf = k1_cbf;
params.rho    = rho;
params.H      = H;
params.qp_opts = qp_opts;
params.e3      = e3;
params.F_lim   = F_lim;
params.yaw_ref = yaw_ref;
params.ppx  = ppx;  params.ppy  = ppy;  params.ppz  = ppz;
params.ppvx = ppvx; params.ppvy = ppvy; params.ppvz = ppvz;
params.ppax = ppax; params.ppay = ppay; params.ppaz = ppaz;
params.t_ways = t_ways;

%% =============== INITIAL STATE =================
p0 = local_cell_to_vec(sim.getObjectPosition(quad_qp, -1));
Rm0 = sim.getObjectMatrix(quad_qp,-1);
Rm0 = reshape(local_cell_to_vec(Rm0),4,3)';
R0 = Rm0(:,1:3);

v0 = [0 0 0];
w0 = [0 0 0];

x0 = [p0(:); v0(:); R0(:); w0(:)];
nX = numel(x0);

%% =============== (RK4) QP =================
xsol = zeros(numSteps+1, nX);
tsol = time_vec(:);

x    = x0;
xsol(1,:) = x0.';

% === CONTROL LOGS ===
u_pd_log   = zeros(numSteps+1,3);
u_star_log = zeros(numSteps+1,3);

% value at t = tsol(1) with the initial state
[u_star_1, u_pd_1, ~, ~, ~, ~] = clf_cbf_qp_thrust_3D(tsol(1), x0, params);

% u_star_1 = [ux, uy, T]  -> Convert T to acceleration in z
a_qp_1 = [u_star_1(1:2), -g + u_star_1(3)/m];

u_star_log(1,:) = a_qp_1;   %effective accelerations [ax_QP, ay_QP, az_QP]
u_pd_log(1,:)   = u_pd_1;   % [ax_PD, ay_PD, az_PD]

for k = 1:numSteps
    t = tsol(k);

    k1 = quad_dynamics_xyCBF(t,          x,             params);
    k2 = quad_dynamics_xyCBF(t + dt/2.0, x + dt/2.0*k1, params);
    k3 = quad_dynamics_xyCBF(t + dt/2.0, x + dt/2.0*k2, params);
    k4 = quad_dynamics_xyCBF(t + dt,     x + dt   *k3,  params);

    x  = x + dt/6.0*(k1 + 2*k2 + 2*k3 + k4);

    xsol(k+1,:) = x.';

    % === CONTROL LOGS AT t = tsol(k+1) ===
    t_ctrl   = tsol(k+1);
    x_now    = xsol(k+1,:).';
    [u_star_k, u_pd_k, ~, ~, ~, ~] = clf_cbf_qp_thrust_3D(t_ctrl, x_now, params);

    % u_star_k = [ux, uy, T] -> convert T to az
    a_qp_k = [u_star_k(1:2), -g + u_star_k(3)/m];

    u_star_log(k+1,:) = a_qp_k;   % effective QP accelerations
    u_pd_log(k+1,:)   = u_pd_k;   % PD base accelerations
end

p_log = xsol(:,1:3);
v_log = xsol(:,4:6);

%% =============== (RK4) ONLY PD =================
xsol_pd = zeros(numSteps+1, nX);
x_pd    = x0;
xsol_pd(1,:) = x0.';

for k = 1:numSteps
    t = tsol(k);

    k1 = quad_dynamics_PD(t,          x_pd,             params);
    k2 = quad_dynamics_PD(t + dt/2.0, x_pd + dt/2.0*k1, params);
    k3 = quad_dynamics_PD(t + dt/2.0, x_pd + dt/2.0*k2, params);
    k4 = quad_dynamics_PD(t + dt,     x_pd + dt   *k3,  params);

    x_pd  = x_pd + dt/6.0*(k1 + 2*k2 + 2*k3 + k4);

    xsol_pd(k+1,:) = x_pd.';
end

p_log_pd = xsol_pd(:,1:3);

%% =============== REFERENCE CALCULATION AND ERRORS =================
pref_log = zeros(size(p_log));
e_log    = zeros(size(p_log));
e_log_pd = zeros(size(p_log));

for k = 1:length(tsol)
    t_eval = min(max(tsol(k), t_ways(1)), t_ways(end));
    pref_log(k,:) = [ppval(ppx, t_eval), ...
                     ppval(ppy, t_eval), ...
                     ppval(ppz, t_eval)];
    e_log(k,:)    = pref_log(k,:) - p_log(k,:);
    e_log_pd(k,:) = pref_log(k,:) - p_log_pd(k,:);
end

t_log = tsol;

%% =============== SEND TRAJECTORIES TO COPPELIA =================
sim.setStepping(true);
sim.startSimulation();

for k = 1:length(tsol)
    % ---- QP ----
    p_qp  = p_log(k,:);  
    R_qp  = reshape(xsol(k,7:15),3,3);
    [roll_qp, pitch_qp, yaw_qp] = R_to_rpy(R_qp);
    
    sim.setObjectPosition(quad_qp, -1, p_qp);
    sim.setObjectOrientation(quad_qp, -1, [roll_qp, pitch_qp, yaw_qp]);

    % ---- PD ----
    p_pd  = p_log_pd(k,:);
    R_pd  = reshape(xsol_pd(k,7:15),3,3);
    [roll_pd, pitch_pd, yaw_pd] = R_to_rpy(R_pd);

    sim.setObjectPosition(quad_pd, -1, p_pd);
    sim.setObjectOrientation(quad_pd, -1, [roll_pd, pitch_pd, yaw_pd]);

    sim.step();
end

sim.stopSimulation();

%% =============== PLOTS: TRACKING QP vs PD =================

figure;
subplot(3,1,1);
plot(t_log, pref_log(:,1),'k--','LineWidth',1.2); hold on;
plot(t_log, p_log(:,1),'b','LineWidth',1.2);
plot(t_log, p_log_pd(:,1),'r:','LineWidth',1.2);
grid on; ylabel('x [m]');
title('X tracking performance');
legend('ref','QP','PD');

subplot(3,1,2);
plot(t_log, pref_log(:,2),'k--','LineWidth',1.2); hold on;
plot(t_log, p_log(:,2),'b','LineWidth',1.2);
plot(t_log, p_log_pd(:,2),'r:','LineWidth',1.2);
grid on; ylabel('y [m]');
title('Y tracking performance');

subplot(3,1,3);
plot(t_log, pref_log(:,3),'k--','LineWidth',1.2); hold on;
plot(t_log, p_log(:,3),'b','LineWidth',1.2);
plot(t_log, p_log_pd(:,3),'r:','LineWidth',1.2);
grid on; ylabel('z [m]'); xlabel('t [s]');
title('Z tracking performance');

%% x-y trajectory with obstacles QP vs PD)
figure; hold on; grid on; axis equal
for j = 1:size(obs_list,1)
    viscircles(obs_list(j,1:2), r_safe_objects, 'LineStyle','--','Color',[1 0 0]);
end
for j = 1:size(obs_list_walls,1)
    viscircles(obs_list_walls(j,1:2), r_safe_walls, 'LineStyle',':','Color',[1 0 0]);
end
plot(path_xy(:,1), path_xy(:,2),'k--','LineWidth',1.5,'DisplayName','RRT* 3D projection');
plot(p_log(:,1),  p_log(:,2),  'b-', 'LineWidth',1.5,'DisplayName','QP traj');
plot(p_log_pd(:,1),p_log_pd(:,2),'r:', 'LineWidth',1.5,'DisplayName','PD traj');
legend show
xlabel('x[m]'); ylabel('y[m]');
title('Trajectories with obstacles (xy projection)');

%% minimum distance to obstacles (QP vs PD) in XY
num = size(p_log,1);
d_min_qp = zeros(num,1);
d_min_pd = zeros(num,1);
for k = 1:num
    pk_xy   = p_log(k,1:2);
    pk_xy_pd= p_log_pd(k,1:2);
    d_all   = vecnorm(obs_list(:,1:2)-pk_xy,2,2);
    d_all_w = vecnorm(obs_list_walls(:,1:2)-pk_xy,2,2);
    d_min_qp(k) = min([d_all; d_all_w]);

    d_all_pd   = vecnorm(obs_list(:,1:2)-pk_xy_pd,2,2);
    d_all_w_pd = vecnorm(obs_list_walls(:,1:2)-pk_xy_pd,2,2);
    d_min_pd(k) = min([d_all_pd; d_all_w_pd]);
end
min_dist_qp = min(d_min_qp);
min_dist_pd = min(d_min_pd);
viol_qp = sum(d_min_qp < r_safe_objects);
viol_pd = sum(d_min_pd < r_safe_objects);

figure; hold on; grid on;
plot(t_log, d_min_qp, 'b-', 'LineWidth',1.2,'DisplayName','QP');
plot(t_log, d_min_pd, 'r:', 'LineWidth',1.2,'DisplayName','PD');
yline(r_safe_objects,'k--', 'DisplayName','r_{safe}');
legend show
xlabel('t[s]'); ylabel('distance to an obstacle [m]');
title('Min distance to obstacles: QP vs PD');

%% norm error QP vs PD
e_norm_qp = vecnorm(e_log,2,2);
e_norm_pd = vecnorm(e_log_pd,2,2);
figure; hold on; grid on;
plot(t_log, e_norm_qp, 'b', 'LineWidth',1.2,'DisplayName','QP');
plot(t_log, e_norm_pd, 'r:', 'LineWidth',1.2,'DisplayName','PD');
legend show
xlabel('t[s]'); ylabel('||e_p||[m]');
title('Norm error position: QP vs PD');

%% RMSE
rmse_qp   = sqrt(mean(e_log.^2,1));
rmse_pd   = sqrt(mean(e_log_pd.^2,1));
rmse_tot_qp = norm(rmse_qp);
rmse_tot_pd = norm(rmse_pd);

fprintf('RMSE QP [x y z] = [%g  %g  %g], total = %g\n', rmse_qp,   rmse_tot_qp);
fprintf('RMSE PD [x y z] = [%g  %g  %g], total = %g\n', rmse_pd,   rmse_tot_pd);
fprintf('Min dist QP = %g m (violations = %d)\n', min_dist_qp, viol_qp);
fprintf('Min dist PD = %g m (violations = %d)\n', min_dist_pd, viol_pd);

%% === DIFFERENCE BETWEEN QP AND PD BASE  ===
figure;
plot(t_log, vecnorm(u_star_log - u_pd_log,2,2),'LineWidth',1.2);
grid on;
xlabel('t [s]');
ylabel('||u^* - u_{PD}||');
title('Difference between CLF-CBF-QP control and PD base');

figure;
subplot(3,1,1);
plot(t_log, u_pd_log(:,1),'k--','LineWidth',1.2); hold on;
plot(t_log, u_star_log(:,1),'b','LineWidth',1.2);
grid on; ylabel('u_x'); legend('PD base','QP');

subplot(3,1,2);
plot(t_log, u_pd_log(:,2),'k--','LineWidth',1.2); hold on;
plot(t_log, u_star_log(:,2),'b','LineWidth',1.2);
grid on; ylabel('u_y'); legend('PD base','QP');

subplot(3,1,3);
plot(t_log, u_pd_log(:,3),'k--','LineWidth',1.2); hold on;
plot(t_log, u_star_log(:,3),'b','LineWidth',1.2);
grid on; ylabel('u_z'); xlabel('t [s]'); legend('PD base','QP');
title('Base PD vs QP Control Components');


%% =============== AUXILIARY FUNCTIONS =================

function dx = quad_dynamics_xyCBF(t,x,P)
% x = [p(1:3); v(4:6); R(7:15); w(16:18)]

    p = x(1:3).';
    v = x(4:6).';
    R = reshape(x(7:15),3,3);
    w = x(16:18).';

    % === Translational control: ux, uy and thrust T (CLF 3D + CBF in xy) ===
    [u_star, ~, ~, ~, ~, ~] = clf_cbf_qp_thrust_3D(t,x,P);
    ux = u_star(1);
    uy = u_star(2);
    T  = u_star(3);   % thrust total

    %   vdot = [ux; uy; -g + T/m]
    dp = v(:);
    dv = [ux;
          uy;
         -P.g + T/P.m];

    % === Orientation dynamics: Angular PD ===
    J       = P.J;
    Kp_ang  = P.Kp_ang;
    Kd_ang  = P.Kd_ang;
    yaw_ref = P.yaw_ref;
    sat = @(vv,lim) max(min(vv,lim),-lim);

    [roll, pitch, yaw] = R_to_rpy(R);
    yaw_err = wrapToPi(yaw_ref - yaw);   

    e_ang = [-roll, -pitch, yaw_err]; 
    e_w   = -w;

    Tau_lim_xy   = 2.0;   
    Tau_lim_yaw  = 0.3; 
    Tau_des_body  = (Kp_ang*e_ang.' + Kd_ang*e_w.').';
    Tau_des_body(1:2) = sat(Tau_des_body(1:2), Tau_lim_xy);
    Tau_des_body(3)   = sat(Tau_des_body(3),   Tau_lim_yaw);

    tau = Tau_des_body(:);

    w_vec = w(:);
    dR = R*skew(w_vec);
    dw = J \ (tau - cross(w_vec, J*w_vec));

    dx = [dp; dv; dR(:); dw];
end


function dx = quad_dynamics_PD(t,x,P)
% Dynamics using ONLY the PD position controller (without CBF or CLF)
% x = [p(1:3); v(4:6); R(7:15); w(16:18)]

    p = x(1:3).';
    v = x(4:6).';
    R = reshape(x(7:15),3,3);
    w = x(16:18).';

    % Params
    Kp_pos = P.Kp_pos;
    Kd_pos = P.Kd_pos;
    Kp_ang = P.Kp_ang;
    Kd_ang = P.Kd_ang;
    yaw_ref = P.yaw_ref;
    ppx  = P.ppx;  ppy  = P.ppy;  ppz  = P.ppz;
    ppvx = P.ppvx; ppvy = P.ppvy; ppvz = P.ppvz;
    t_ways = P.t_ways;
    J = P.J;

    sat = @(vv,lim) max(min(vv,lim),-lim);

    % Reference
    t_eval = min(max(t, t_ways(1)), t_ways(end));
    p_ref = [ppval(ppx, t_eval), ...
             ppval(ppy, t_eval), ...
             ppval(ppz, t_eval)];
    v_ref = [ppval(ppvx, t_eval), ...
             ppval(ppvy, t_eval), ...
             ppval(ppvz, t_eval)];

    ep = p_ref - p;
    ev = v_ref - v;

    % Translational PD control
    u_pd = (Kp_pos*ep.' + Kd_pos*ev.').';   % [ux, uy, uz]

    dp = v(:);
    dv = u_pd(:);

    % Angular PD orientation dynamics
    [roll, pitch, yaw] = R_to_rpy(R);
    yaw_err = wrapToPi(yaw_ref - yaw);   

    e_ang = [-roll, -pitch, yaw_err]; 
    e_w   = -w;

    Tau_lim_xy   = 2.0;   
    Tau_lim_yaw  = 0.3; 
    Tau_des_body  = (Kp_ang*e_ang.' + Kd_ang*e_w.').';
    Tau_des_body(1:2) = sat(Tau_des_body(1:2), Tau_lim_xy);
    Tau_des_body(3)   = sat(Tau_des_body(3),   Tau_lim_yaw);

    tau = Tau_des_body(:);

    w_vec = w(:);
    dR = R*skew(w_vec);
    dw = J \ (tau - cross(w_vec, J*w_vec));

    dx = [dp; dv; dR(:); dw];
end

% Control CLF–CBF–QP:
function [u_star, u_pd, A_clf, b_clf, A_cbf, b_cbf] = clf_cbf_qp_thrust_3D(t,x,P)
    %   - u = [ux, uy, T]
    %   - 3D CLF on (x,y,z) using V(ep,ev) with dynamics vdot = f_v + G_v u
    %   - CBF (HOCBF) in the xy plane to avoid obstacles

    % x = [p(1:3); v(4:6); R(7:15); w(16:18)]
    p = x(1:3).';
    v = x(4:6).';

    % Params
    Kp_pos = P.Kp_pos;
    Kd_pos = P.Kd_pos;
    Qp = P.Qp;
    Qv = P.Qv;
    obs_list       = P.obs_list;
    obs_list_walls = P.obs_list_walls;
    r_safe_objects = P.r_safe_objects;
    r_safe_walls   = P.r_safe_walls;
    c_clf = P.c_clf;
    k0_cbf = P.k0_cbf; 
    k1_cbf = P.k1_cbf;
    rho    = P.rho;
    qp_opts = P.qp_opts;
    ppx  = P.ppx;  ppy  = P.ppy;  ppz  = P.ppz;
    ppvx = P.ppvx; ppvy = P.ppvy; ppvz = P.ppvz;
    ppax = P.ppax; ppay = P.ppay; ppaz = P.ppaz;
    t_ways = P.t_ways;
    m = P.m;
    g = P.g;
    alpha_T = P.alpha_T;

    % Reference
    t_eval = min(max(t, t_ways(1)), t_ways(end));
    p_ref = [ppval(ppx, t_eval), ...
             ppval(ppy, t_eval), ...
             ppval(ppz, t_eval)];
    v_ref = [ppval(ppvx, t_eval), ...
             ppval(ppvy, t_eval), ...
             ppval(ppvz, t_eval)];
    a_ref = [ppval(ppax, t_eval), ...
             ppval(ppay, t_eval), ...
             ppval(ppaz, t_eval)];

    ep = p_ref - p;      % 1x3
    ev = v_ref - v;      % 1x3

    % PD base control (desired acceleration in xyz)
    u_pd = (Kp_pos*ep.' + Kd_pos*ev.').';   % [a_x_PD, a_y_PD, a_z_PD]

    ux_nom = u_pd(1);
    uy_nom = u_pd(2);
    T_nom  = m*(u_pd(3) + g);   % map a_z_PD -> thrust nominal

    % ===== CLF 3D vdot = f_v + G_v u =====
    % vdot = [0;0;-g] + [1 0 0; 0 1 0; 0 0 1/m]*[ux;uy;T]
    f_v = [0;0;-g];
    G_v = [1 0 0;
           0 1 0;
           0 0 1/m];

    ep_col = ep(:);
    ev_col = ev(:);

    V   = ep_col.'*Qp*ep_col + ev_col.'*Qv*ev_col;
    LfV = 2*ep_col.'*Qp*ev_col + 2*ev_col.'*Qv*(a_ref(:) - f_v);
    LgV = -2*(ev_col.'*Qv*G_v);   % 1x3

    % CLF: LfV + LgV * u <= -c_clf V + delta
    A_clf = [LgV, -1];            % [ux uy T delta]
    b_clf = -c_clf*V - LfV;       

    % ===== CBFs ON XY (HOCBF 2º orden) =====
    A_cbf = zeros(0,4);
    b_cbf = zeros(0,1);

    v_xy = v(1:2);

    % walls
    for j = 1:size(obs_list_walls,1)
        p_obs_xy = obs_list_walls(j,1:2);
        r_safe   = r_safe_walls;

        p_rel_xy = p(1:2) - p_obs_xy;
        h            = p_rel_xy*p_rel_xy.' - r_safe^2;
        hdot         = 2*p_rel_xy*v_xy.';
        hddot_const  = 2*(v_xy*v_xy.');
        Lg2h_xy      = 2*p_rel_xy;

        rhs_cbf = -hddot_const - k1_cbf*hdot - k0_cbf*h;

        Lg2h  = [Lg2h_xy 0];   % coef in [ux, uy, T]
        A_cbf = [A_cbf; -Lg2h 0];
        b_cbf = [b_cbf; -rhs_cbf(:)];
    end

    % objets
    for j = 1:size(obs_list,1)
        p_obs_xy = obs_list(j,1:2);
        r_safe   = r_safe_objects;

        p_rel_xy = p(1:2) - p_obs_xy;
        h            = p_rel_xy*p_rel_xy.' - r_safe^2;
        hdot         = 2*p_rel_xy*v_xy.';
        hddot_const  = 2*(v_xy*v_xy.');
        Lg2h_xy      = 2*p_rel_xy;

        rhs_cbf = -hddot_const - k1_cbf*hdot - k0_cbf*h;

        Lg2h  = [Lg2h_xy 0];
        A_cbf = [A_cbf; -Lg2h 0];
        b_cbf = [b_cbf; -rhs_cbf(:)];
    end

    % ===== QP =====
    H = diag([2, 2, 2*alpha_T, 2*rho]);
    f = [-2*ux_nom;
         -2*uy_nom;
         -2*alpha_T*T_nom;
          0];

    A  = [A_clf; A_cbf; 0 0 0 -1];   % last: delta >= 0 -> -delta <= 0
    b_ = [b_clf; b_cbf(:); 0];

    acc_lim_xy = 2.5;
    T_min      = 0;
    T_max      = 2*m*g;

    lb = [-acc_lim_xy; -acc_lim_xy; T_min; 0];
    ub = [ acc_lim_xy;  acc_lim_xy; T_max; Inf];

    try
        z_star = quadprog(H,f,A,b_,[],[],lb,ub,[],qp_opts);
        u_star = z_star(1:3).';
    catch
        u_star = [ux_nom, uy_nom, T_nom];
    end
end
%% ---------- RRT* 3D ----------
function [path, nodes] = rrt_star_3d(start_xyz, goal_xyz, ...
                                     obst_xyz, obst_r, ...
                                     x_limits, y_limits, z_limits, ...
                                     maxIter, step_size, goal_radius, near_radius)

    nodes(1).pos    = start_xyz;
    nodes(1).parent = 0;
    nodes(1).cost   = 0;

    for k = 2:maxIter
        % random sampling in the 3D cube
        xr = [ ...
            x_limits(1) + rand()*(x_limits(2)-x_limits(1)), ...
            y_limits(1) + rand()*(y_limits(2)-y_limits(1)), ...
            z_limits(1) + rand()*(z_limits(2)-z_limits(1)) ];

        % goal bias
        if rand() < 0.1
            xr = goal_xyz;
        end

        pos_all = reshape([nodes.pos],3,[])';
        [~, idx_near] = min(sum((pos_all - xr).^2,2));
        x_near = nodes(idx_near).pos;

        dir = xr - x_near;
        d   = norm(dir);
        if d < 1e-6, continue; end
        x_new = x_near + step_size * dir/d;

        if ~isCollisionFree3D(x_near, x_new, obst_xyz, obst_r)
            continue;
        end

        new.cost   = nodes(idx_near).cost + norm(x_new - x_near);
        new.pos    = x_new;
        new.parent = idx_near;

        pos_all = reshape([nodes.pos],3,[])';
        dists   = sqrt(sum((pos_all - x_new).^2,2));
        neigh_idx = find(dists < near_radius);

        for j = neigh_idx'
            c_new = nodes(j).cost + norm(nodes(j).pos - x_new);
            if c_new < new.cost && ...
               isCollisionFree3D(nodes(j).pos, x_new, obst_xyz, obst_r)
                new.cost   = c_new;
                new.parent = j;
            end
        end

        nodes(end+1) = new;
        new_idx = numel(nodes);

        for j = neigh_idx'
            c_old = nodes(j).cost;
            c_through_new = new.cost + norm(nodes(j).pos - x_new);
            if c_through_new < c_old && ...
               isCollisionFree3D(nodes(j).pos, x_new, obst_xyz, obst_r)
                nodes(j).parent = new_idx;
                nodes(j).cost   = c_through_new;
            end
        end

        if norm(x_new - goal_xyz) < goal_radius
            path = backtrackPath3D(nodes, new_idx);
            return;
        end
    end

    path = [];
end

function ok = isCollisionFree3D(p1, p2, obst_xyz, obst_r)
    ok = true;
    for i = 1:size(obst_xyz,1)
        c  = obst_xyz(i,:);
        r  = obst_r(i);

        v  = p2 - p1;
        w  = c  - p1;

        if norm(v) < 1e-6
            d = norm(w);
        else
            t    = max(0, min(1, dot(w,v)/dot(v,v)));
            proj = p1 + t*v;
            d    = norm(c - proj);
        end

        if d < r
            ok = false;
            return;
        end
    end
end

function path = backtrackPath3D(nodes, idx)
    pts = [];
    while idx ~= 0
        pts = [nodes(idx).pos; pts]; 
        idx = nodes(idx).parent;
    end
    path = pts;
end

function [roll, pitch, yaw] = R_to_rpy(R)
    pitch = -asin( max(-1,min(1,R(3,1))) );
    roll  = atan2(R(3,2), R(3,3));
    yaw   = atan2(R(2,1), R(1,1));
end
%% ---------- OTHERS ----------

%convert coppeliasim to matlab format
function v = local_cell_to_vec(c) 
    if iscell(c) 
        if numel(c)==1 
            v = double(c{1}); 
        else 
            v = cellfun(@double,c);
        end 
    else 
        v = double(c); 
    end 
    v = v(:).'; 
end

%return a w= [w1,w2,w3]
function S = skew(w)
    S = [   0   -w(3)  w(2);
          w(3)   0   -w(1);
         -w(2)  w(1)   0 ];
end
%limit to -pi and +pi
function ang = wrapToPi(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end
