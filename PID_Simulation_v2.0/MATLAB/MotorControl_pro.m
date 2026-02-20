clc; clear; close all;

%% =========================================================
%                 仿真设置
% ==========================================================

dt = 0.00199;          % 采样时间
T_end = 10;            
N = floor(T_end/dt);

mode = 8;   % 推荐3或7测试前馈

% 1  速度阶跃
% 2  速度斜坡
% 3  速度正弦
% 4  角度单环阶跃
% 5  角度单环正弦
% 6  串级阶跃
% 7  串级正弦
% 8  串级 + 扰动
% 9  开环


%% =========================================================
%                 电机参数
% ==========================================================

J  = 0.047;
b  = 0.03;
R  = 1.8;
L  = 0.00578;
Kt = 0.741;
Ke = 0.7164;
T_c = 0.5;


%% =========================================================
%                 前馈系数
% ==========================================================

a2 = (L*J)/Kt;
a1 = (L*b + R*J)/Kt;
a0 = (R*b)/Kt + Ke;


%% =========================================================
%                 PID参数
% ==========================================================

Kp_vel = 5;
Ki_vel = 40;
Kd_vel = 0.01;

Kp_ang = 30;
Ki_ang = 0;
Kd_ang = 0.27;


%% =========================================================
%                 状态变量
% ==========================================================

theta = 0;
omega = 0;
i = 0;

vel_integral = 0;
vel_last_fdb = 0;

ang_integral = 0;
ang_last_fdb = 0;

vel_filter = 0;
alpha = 0.3;

velRef_last = 0;
velRef_last2 = 0;


%% =========================================================
%                 数据记录
% ==========================================================

time = zeros(N,1);
Angle = zeros(N,1);
Velocity = zeros(N,1);
Input = zeros(N,1);
AngleRef_log = zeros(N,1);
VelocityRef_log = zeros(N,1);


%% =========================================================
%                 主循环
% ==========================================================

for k = 1:N
    
    t = (k-1)*dt;
    
    %% -------- 参考信号 --------
    
    AngleRef = 0;
    VelocityRef = 0;
    
    omega_vel = 36;
    omega_ang = 4.8;
    omega_cas = 5.5;
    
    switch mode
        
        case 1
            VelocityRef = 10;
            
        case 2
            VelocityRef = t;
            
        case 3
            VelocityRef = 10*sin(omega_vel*t);
            
        case 4
            AngleRef = 2*pi;
            
        case 5
            AngleRef = 2*pi*sin(omega_ang*t);
            
        case 6
            AngleRef = 2*pi;
            
        case 7
            AngleRef = 2*pi*sin(omega_cas*t);
            
        case 8
            AngleRef = 0;
            
        case 9
            % 开环
    end
    
    AngleRef_log(k) = AngleRef;
    VelocityRef_log(k) = VelocityRef;
    
    
    %% -------- 测量滤波 --------
    
    vel_filter = (1-alpha)*vel_filter + alpha*omega;
    Velocity_meas = vel_filter;
    Angle_meas = theta;
    
    
    %% -------- 角度PID --------
    
    error_ang = AngleRef - Angle_meas;
    der_ang = -(Angle_meas - ang_last_fdb)/dt;
    new_int_ang = ang_integral + error_ang*dt;
    
    u_ang = Kp_ang*error_ang + ...
            Ki_ang*new_int_ang + ...
            Kd_ang*der_ang;
    
    u_ang = max(min(u_ang,20),-20);
    
    ang_integral = new_int_ang;
    ang_last_fdb = Angle_meas;
    
    
    %% -------- 速度PID --------
    
    error_vel = VelocityRef - Velocity_meas;
    der_vel = -(Velocity_meas - vel_last_fdb)/dt;
    new_int_vel = vel_integral + error_vel*dt;
    
    u_vel = Kp_vel*error_vel + ...
            Ki_vel*new_int_vel + ...
            Kd_vel*der_vel;
    
    u_vel = max(min(u_vel,24),-24);
    
    vel_integral = new_int_vel;
    vel_last_fdb = Velocity_meas;
    
    
    %% -------- 前馈计算 --------
    
    dVelRef = (VelocityRef - velRef_last)/dt;
    ddVelRef = (VelocityRef - 2*velRef_last + velRef_last2)/(dt^2);
    
    u_ff = a2*ddVelRef + a1*dVelRef + a0*VelocityRef;
    
    velRef_last2 = velRef_last;
    velRef_last = VelocityRef;
    
    
    %% -------- 控制结构 --------
    
    switch mode
        
        case {1,2,3}
            u = u_vel + u_ff;
            
        case {4,5}
            u = u_ang;
            
        case {6,7,8}
            VelocityRef = u_ang;
            u = u_vel + u_ff;
            
        case 9
            u = 24;
    end
    
    
    %% -------- 扰动 --------
    
    if mode == 8
        if t > 0.5 && t < 0.7
            u = u + 10;
        end
    end
    
    
    %% -------- 输入限幅 --------
    
    u = max(min(u,24),-24);
    
    
    %% -------- 电机模型 --------
    
    if omega > 0
        T_fric = T_c;
    elseif omega < 0
        T_fric = -T_c;
    else
        T_fric = 0;
    end
    
    di = (u - R*i - Ke*omega)/L;
    domega = (Kt*i - b*omega - T_fric)/J;
    dtheta = omega;
    
    i = i + di*dt;
    omega = omega + domega*dt;
    theta = theta + dtheta*dt;
    
    
    %% -------- 记录 --------
    
    time(k) = t;
    Angle(k) = theta;
    Velocity(k) = omega;
    Input(k) = u;
    
end


%% =========================================================
%                 画图
% ==========================================================

figure;

subplot(3,1,1)
plot(time,Velocity,'b'); hold on;
plot(time,VelocityRef_log,'r--');
ylabel('Velocity (rad/s)')
legend('Velocity','VelocityRef')

subplot(3,1,2)
plot(time,Angle,'b'); hold on;
plot(time,AngleRef_log,'r--');
ylabel('Angle (rad)')
legend('Angle','AngleRef')

subplot(3,1,3)
plot(time,Input)
ylabel('Input (V)')
xlabel('Time (s)')