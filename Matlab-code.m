%% Parameter Identification for Stick-Slip Piezoelectric Motor (Duty Ratio Variations)
% This script identifies the mechanical and friction parameters of a 
% 2-DOF stick-slip piezoelectric actuator using experimental datasets.

clc;
clear variables;
close all;

% Set text interpreters to LaTeX for professional academic plots
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

%% 1. Initialization & Fixed Settings
dt = 0.001;          
stop_time = 15;      
slope_time = [1, 3]; % Time window for steady-state velocity evaluation

fixed_volt = 140;    % Fixed Voltage [V]
fixed_freq = 3;      % Fixed Frequency [Hz]

% Define Duty Ratios for Training and Validation
train_drs = [0.97, 0.98, 0.99]; 
val_drs   = [0.975, 0.985, 0.995];

%% 2. System Identification (Optimization Setup)
% x = [m_p, c_p, k, m_s, c_s, mu*Nor, alpha_1, alpha_2]
x0 = [1.2, 77, 140, 1.2, 80, 0.007, 2, 0.5];
lb = [0.1,  10,  50, 0.1,  10, 0.0001, -Inf, -Inf];
ub = [5.0, 300, 500, 5.0, 300, 0.5000,  Inf,  Inf];
step_vector = [0.05, 0.5, 0.5, 0.01, 0.5, 1e-5, 0.01, 0.02];

w_disp = 1;  
w_slope = 1; 

options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'interior-point', ...         
    'MaxIterations', 150, ...                  
    'MaxFunctionEvaluations', 10000, ...        
    'OptimalityTolerance', 1e-6, ...            
    'StepTolerance', 1e-12, ...                  
    'FiniteDifferenceStepSize', step_vector, ... 
    'FiniteDifferenceType', 'central');

% Objective function wrapper
cost_function = @(x) calc_multi_mae_combined(x, dt, fixed_freq, fixed_volt, train_drs, stop_time, w_disp, w_slope, slope_time);

fprintf('Starting parameter optimization based on Duty Ratios...\n');
[x_opt, total_fval] = fmincon(cost_function, x0, [], [], [], [], lb, ub, [], options);

fprintf('\n--- Optimization Finished ---\n');
disp('Identified Parameters [m_p, c_p, k, m_s, c_s, mu*Nor, alpha_1, alpha_2]:');
disp(x_opt);

%% 3. Plot Training Results
n_samples = round(stop_time/dt);

for dr = train_drs
    filename = sprintf('DR%gf%dV%d.mat', dr * 100, fixed_freq, fixed_volt);
    train_data = load(filename); 
    
    n_pts = min(n_samples, length(train_data.t));
    t_train = train_data.t(1:n_pts);
    x2_exp = movmean(train_data.outx2(1:n_pts), 4); 
    
    y_sim = rk4_solver_inline(t_train, [0;0;0;0], dt, x_opt, fixed_volt, fixed_freq, dr);
    x2_sim = y_sim(:,3) * 1e6;
    
    figure('Name', sprintf('Training Result: DR = %g%%', dr*100), 'Color', 'w');
    ax = axes('Parent', gcf);
    hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on'); grid minor;
    ax.GridLineStyle = ':'; ax.GridAlpha = 0.5; ax.LineWidth = 1.1;
    
    plot(t_train, x2_exp, 'r', 'LineWidth', 1.5, 'DisplayName', 'Experimental'); 
    plot(t_train, x2_sim, 'b--', 'LineWidth', 1.2, 'DisplayName', 'Identified Model');
    
    title(sprintf('Training Fit at DR = %g\\%%', dr*100), 'Interpreter', 'latex', 'FontSize', 12);
    xlabel('Time $t \ [\mathrm{s}]$', 'Interpreter', 'latex', 'FontSize', 11); 
    ylabel('Position $x_2 \ [\mu\mathrm{m}]$', 'Interpreter', 'latex', 'FontSize', 11);
    legend('Location', 'best', 'Interpreter', 'latex');
end

%% 4. Validation Loop
fprintf('\n--- Running Validation on Unseen Duty Ratios ---\n');
for dr = val_drs
    val_file = sprintf('DR%gf%dV%d.mat', dr * 100, fixed_freq, fixed_volt);
    val_data = load(val_file);
    
    n_v = min(n_samples, length(val_data.t));
    t_val = val_data.t(1:n_v);
    x2_exp_val = movmean(val_data.outx2(1:n_v), 4);
    
    y_sim_val = rk4_solver_inline(t_val, [0;0;0;0], dt, x_opt, fixed_volt, fixed_freq, dr);
    x2_sim_val = y_sim_val(:,3) * 1e6;
    
    T_total_val = t_val(end) - t_val(1);
    val_mae_disp = (dt / T_total_val) * sum(abs(x2_sim_val - x2_exp_val));
    
    % Evaluate mean velocity in the specified slope window
    idx_slope = (t_val >= slope_time(1)) & (t_val <= slope_time(2));
    p_exp = polyfit(t_val(idx_slope), x2_exp_val(idx_slope), 1);
    p_sim = polyfit(t_val(idx_slope), x2_sim_val(idx_slope), 1);
    
    v_exp_val = p_exp(1); 
    v_sim_val = p_sim(1); 
    val_mae_slope = abs(v_sim_val - v_exp_val);
    
    fprintf('Validation DR=%g%% | Disp MAE: %7.4f | Slope Error: %7.4f | Exp Vel: %7.4f | Sim Vel: %7.4f\n', ...
            dr*100, val_mae_disp, val_mae_slope, v_exp_val, v_sim_val);
    
    figure('Name', sprintf('Validation DR = %g%%', dr*100), 'Color', 'w');
    ax = axes('Parent', gcf);
    hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on'); grid minor;
    ax.GridLineStyle = ':'; ax.GridAlpha = 0.5; ax.LineWidth = 1.1;
    
    plot(t_val, x2_exp_val, 'r', 'LineWidth', 1.5, 'DisplayName', 'Experimental'); 
    plot(t_val, x2_sim_val, 'b--', 'LineWidth', 1.2, 'DisplayName', 'Identified Model');
    
    title(sprintf('Validation on DR = %g\\%%', dr*100), 'Interpreter', 'latex', 'FontSize', 12);
    xlabel('Time $t \ [\mathrm{s}]$', 'Interpreter', 'latex', 'FontSize', 11);
    ylabel('Position $x_2 \ [\mu\mathrm{m}]$', 'Interpreter', 'latex', 'FontSize', 11);
    legend('Location', 'best', 'Interpreter', 'latex');
end

%% 5. Helper Functions
function total_err = calc_multi_mae_combined(x, dt, freq, volt, train_drs, stop_time, w_disp, w_slope, slope_time)
    total_err = 0;
    n_samples = round(stop_time/dt);
    
    for dr = train_drs
        filename = sprintf('DR%gf%dV%d.mat', dr * 100, freq, volt);
        temp_data = load(filename); 
        
        n = min(n_samples, length(temp_data.t));
        t_exp = temp_data.t(1:n);
        x2_exp = movmean(temp_data.outx2(1:n), 4); 
        
        y_sim = rk4_solver_inline(t_exp, [0;0;0;0], dt, x, volt, freq, dr);
        x2_sim = y_sim(:,3) * 1e6; 
        
        T_total = t_exp(end) - t_exp(1);
        mae_disp = (dt / T_total) * sum(abs(x2_sim - x2_exp));
        
        idx_slope = (t_exp >= slope_time(1)) & (t_exp <= slope_time(2));
        p_exp = polyfit(t_exp(idx_slope), x2_exp(idx_slope), 1);
        p_sim = polyfit(t_exp(idx_slope), x2_sim(idx_slope), 1);
        mae_slope = abs(p_sim(1) - p_exp(1));
        
        total_err = total_err + (w_disp * mae_disp) + (w_slope * mae_slope);
    end
end

function y = rk4_solver_inline(t, y0, dt, x, amp, freq, duty_ratio)
    N = length(t);
    y = zeros(N, 4);
    y(1,:) = y0';
    
    x1 = x(1); x2 = x(2); x3 = x(3);
    x4 = x(4); x5 = x(5); x6 = x(6);
    alpha1 = x(7); alpha2 = x(8);
    
    for i = 1:(N-1)
        ti = t(i); yi = y(i,:)';
        
        V1 = (amp / 2) * (sawtooth((2 * pi * freq) * ti, duty_ratio) + 1);
        Fp1 = alpha1 * V1 + alpha2 * (V1^2);
        ff1 = x6 * tanh(yi(2) - yi(4));
        k1 = [yi(2); (Fp1 - x2*yi(2) - x3*yi(1) - ff1)/x1; yi(4); (ff1 - x5*yi(4))/x4];
        
        yi_k2 = yi + (dt / 2) * k1;
        V2 = (amp / 2) * (sawtooth((2 * pi * freq) * (ti + (dt / 2)), duty_ratio) + 1);
        Fp2 = alpha1 * V2 + alpha2 * (V2^2);
        ff2 = x6 * tanh(yi_k2(2) - yi_k2(4));
        k2 = [yi_k2(2); (Fp2 - x2*yi_k2(2) - x3*yi_k2(1) - ff2)/x1; yi_k2(4); (ff2 - x5*yi_k2(4))/x4];
        
        yi_k3 = yi + (dt / 2) * k2;
        ff3 = x6 * tanh(yi_k3(2) - yi_k3(4));
        k3 = [yi_k3(2); (Fp2 - x2*yi_k3(2) - x3*yi_k3(1) - ff3)/x1; yi_k3(4); (ff3 - x5*yi_k3(4))/x4];
        
        yi_k4 = yi + dt * k3;
        V4 = (amp / 2) * (sawtooth((2 * pi * freq) * (ti + dt), duty_ratio) + 1);
        Fp4 = alpha1 * V4 + alpha2 * (V4^2);
        ff4 = x6 * tanh(yi_k4(2) - yi_k4(4));
        k4 = [yi_k4(2); (Fp4 - x2*yi_k4(2) - x3*yi_k4(1) - ff4)/x1; yi_k4(4); (ff4 - x5*yi_k4(4))/x4];
        
        y(i+1,:) = (yi + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4))';
    end
end
