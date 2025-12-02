%% IK + FK test for 3-DOF arm with downward claw tip

clear; clc;

% Link lengths (mm)
L1 = 125;
L2 = 130;
CLAW_LENGTH = 130;

% Desired TIP target (same as before)
x_des = -150;    % mm
y_des = 0;    % mm
z_des = 60;   % mm

% ---- Inverse Kinematics ----
[th1, th2, th3, reachable] = ik3dof_tip(x_des, y_des, z_des, L1, L2, CLAW_LENGTH);

if ~reachable
    error('Target is NOT reachable with this arm.');
end

fprintf('IK solution (radians): theta1 = %.4f, theta2 = %.4f, theta3 = %.4f\n', th1, th2, th3);
fprintf('IK solution (degrees): theta1 = %.2f, theta2 = %.2f, theta3 = %.2f\n', ...
        rad2deg(th1), rad2deg(th2), rad2deg(th3));

% ---- Forward Kinematics (check) ----
[x_chk, y_chk, z_chk] = tipPosition(th1, th2, th3, L1, L2, CLAW_LENGTH);

fprintf('\nDesired TIP:   (%.2f, %.2f, %.2f) mm\n', x_des,  y_des,  z_des);
fprintf('FK TIP result: (%.2f, %.2f, %.2f) mm\n', x_chk,  y_chk,  z_chk);

err = norm([x_des - x_chk, y_des - y_chk, z_des - z_chk]);
fprintf('Position error norm: %.6f mm\n', err);


%% ============================================================
%  LOCAL FUNCTION: Inverse Kinematics
%% ============================================================
function [theta1, theta2, theta3, reachable] = ik3dof_tip(x_tip, y_tip, z_tip, L1, L2, CLAW_LENGTH)

    reachable = true;

    % 1. Convert TIP to wrist position (claw pointing DOWN)
    wx = x_tip;
    wy = y_tip;
    wz = z_tip + CLAW_LENGTH;

    % 2. Base angle
    theta1 = atan2(wy, wx);

    % 3. Planar radius and height
    r  = hypot(wx, wy);
    rp = r;
    zp = wz;

    % 4. Reachability
    dist2    = rp^2 + zp^2;
    maxReach = (L1 + L2);
    minReach = abs(L1 - L2);

    if dist2 > maxReach^2 || dist2 < minReach^2
        reachable = false;
        theta1 = NaN; theta2 = NaN; theta3 = NaN;
        return;
    end

    % 5. Elbow (law of cosines)
    D = (dist2 - L1^2 - L2^2) / (2 * L1 * L2);
    D = max(min(D, 1.0), -1.0);

    % Elbow-down solution
    theta3 = atan2(-sqrt(1 - D^2), D);

    % 6. Shoulder angle
    phi  = atan2(zp, rp);
    beta = atan2(L2 * sin(theta3), L1 + L2 * cos(theta3));
    theta2 = phi - beta;
end


%% ============================================================
%  LOCAL FUNCTION: Forward Kinematics (TIP position)
%% ============================================================
function [x_tip, y_tip, z_tip] = tipPosition(theta1, theta2, theta3, L1, L2, CLAW_LENGTH)

    r_w =  L1*cos(theta2) + L2*cos(theta2 + theta3);
    z_w =  L1*sin(theta2) + L2*sin(theta2 + theta3);

    x_w = r_w * cos(theta1);
    y_w = r_w * sin(theta1);

    x_tip = x_w;
    y_tip = y_w;
    z_tip = z_w - CLAW_LENGTH;
end


%% Utility: radians to degrees
function d = rad2deg(r)
    d = r * 180/pi;
end
