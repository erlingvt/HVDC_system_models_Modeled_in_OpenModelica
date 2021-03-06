Ts = 1e-6;         
    S_base = 20000;
    V_base = 400;
    L_pu = 0.15;
    C_pu = 0.88 / 1.015;
    R_pu = 0.01;
    V_DC_base = V_base * 2;
    V_DC_step = 0.1*V_DC_base;
    V_DC_step_time = 0.4;
    P_step_time = 10;
    tau_PWM = 1 / 10000;
    i_q_reference = 0;
    direction_V = 1;
    direction_P = direction_V * (-1);
    I_base = 2 * S_base / (3 * V_base);
    omega_base = 50 * 2 * pi;
    Z_base = V_base / I_base;
    R = R_pu * Z_base;
    L = L_pu * Z_base / omega_base;
   
    I_DC_base = S_base / V_DC_base;
    Z_DC_base = V_DC_base / I_DC_base;
    C = 1 / (C_pu * omega_base * Z_DC_base);
    i_load = I_DC_base;
    V_DC_start = V_DC_base;
  %Tuning
    f = 6;
    tau_eq = tau_PWM * f;
    K_i = R_pu / tau_eq;
    K_p = L_pu / (omega_base * tau_eq);
    tau = (3 + 2 * sqrt(2)) * tau_eq;
    K_CRCP = 1 / (C_pu * omega_base * sqrt(tau * tau_eq));
    K_CRCI = K_CRCP / tau;