model HVDC_base_test
Real V_DC_Global_pu;
Real w_sum_z_Global_pu;
Real P_in_Global_pu;
protected
constant Real pi = 2 * Modelica.Math.asin(1.0);
 //MMC parameters
  parameter Real S_n = 1000e6;
  parameter Real V_n = 313.5e3 "line-to-line RMS voltage";
  parameter Real R_a_pu = 0.005;
  parameter Real R_f_pu = 0.00285;
  parameter Real R_g_pu = 0.0001 "Only used in extensive grid model";
  parameter Real L_a_pu = 0.08;
  parameter Real L_f_pu = 0.16428;
  parameter Real L_g_pu = 0.1121;
  parameter Real C_o_pu = 0.0887;
  parameter Real C_eq_pu = 0.8;
  parameter Real C_DC_pu = 0.051637;
  parameter Real f_n = 50;
  //Inputs
  parameter Real V_DC_step = V_DC_base * 0.1;
  parameter Real V_DC_step_time = 0.3;
  parameter Real P_step_time = 0.5;
  parameter Real tau_PWM = 1 / 10000;
  parameter Real i_q_reference = 0;
  parameter Real direction_V = 1 "1 => to power controller -1 is from power controller";
  //Per unit system:
  parameter Real direction_P = direction_V * (-1);
  parameter Real S_base = S_n;
  parameter Real w_base = 2 * V_DC_base ^ 2 * (1 / 2) * C_eq;
  parameter Real V_base = sqrt(2 / 3) * V_n;
  parameter Real I_base = sqrt(2 / 3) * S_n / V_n;
  parameter Real omega_base = 50 * 2 * pi;
  parameter Real Z_base = V_base / I_base;
  parameter Real L_base = Z_base / omega_base;
  parameter Real C_base = 1 / (omega_base * Z_base);
  parameter Real V_DC_base = V_base * 2;
  parameter Real I_DC_base = S_base / V_DC_base;
  parameter Real Z_DC_base = V_DC_base / I_DC_base;
  parameter Real R_v_pu = R_a_pu / 2 + R_f_pu;
  parameter Real L_v_pu = L_a_pu / 2 + L_f_pu;
  //Physical values
  parameter Real R_a = R_a_pu * Z_base;
  parameter Real L_a = L_a_pu * L_base;
  parameter Real L_f = L_f_pu * L_base;
  parameter Real R_f = R_f_pu * Z_base;
  parameter Real C_o = C_o_pu * C_base;
  parameter Real L_g = L_g_pu * L_base;
  parameter Real R_g = R_g_pu * Z_base;
  parameter Real C_eq = C_eq_pu * C_base;
  parameter Real C_DC = C_DC_pu * C_base;
  parameter Real R_v = R_v_pu * Z_base;
  parameter Real L_v = L_v_pu * L_base;
  //Tuning
  parameter Real k_cc_p = L_v_pu / (omega_base * tau_eq);
  //2.6010 "current controller proportional gain";
  parameter Real k_cc_i = R_v_pu / tau_eq;
  parameter Real feed = 1;
  //21.400 "current controller integral gain";
  parameter Real k_circ_cc_p = 0.1114 "circulating current controller proportional gain";
  parameter Real k_circ_cc_i = 2.1875 "circulating current controller integral gain";
  parameter Real tau_eq_circ_cc = tau_PWM * 23;
  parameter Real k_p_cc_2 = L_a_pu / (omega_base * tau_PWM * 23) "This seems strange";
  parameter Real k_i_cc_2 = R_a_pu / (tau_PWM * 23) "This seems strange";
  parameter Real tau_eq_cc = k_cc_i / R_a_pu;
  parameter Real k_p_ccz = 0.1114;
  parameter Real k_i_ccz = 2.1875;
  parameter Real tau_test_1 = L_a_pu / (omega_base * R_a_pu);
  parameter Real tau_test_2 = k_cc_p / k_cc_i;
  parameter Real k_i_w_sum_z = 10;
  parameter Real k_p_w_sum_z = 10;
  parameter Real k_p_w_sum_dq = 2;
  parameter Real k_i_w_sum_dq = 2;
  parameter Real k_p_w_del_dq = 2;
  //del = delta
  parameter Real k_p_w_del_z_dq = 0.2;
  parameter Real k_p_p_ac = 1;
  parameter Real k_i_p_ac = 50;
  //Tuning
  parameter Real K_i = R_v_pu / tau_eq;
  parameter Real K_p = L_v_pu / (omega_base * tau_eq);
  parameter Real tau_eq = tau_PWM * f;
  parameter Real f = 2.5;
  parameter Real tau = (3 + 2 * sqrt(2)) * tau_eq;
  parameter Real K_CRCP = 4 * C_DC_pu / (omega_base * sqrt(tau * tau_eq));
  parameter Real K_CRCI = K_CRCP / tau;
  parameter Real tau_e = tau_PWM * 2.5;
  parameter Real PhaseMarginConstant = 3 + 2 * sqrt(2);
  parameter Real K_vcp = 8 / 3 * C_DC_pu / (omega_base * sqrt(PhaseMarginConstant) * tau_e);
  parameter Real K_vci = (1/15)* 8 / 3 * C_DC_pu / (omega_base * sqrt(PhaseMarginConstant) ^ 3 * tau_e ^ 2);
  //facator of 3 is to include the cable capcitance in the model
    //factor of (1/10) to reduce oscilation
    //factor of (8/3) from the PU-definition.
  HVDC_base hVDC_base1(S_base = 1000e6)  annotation(
    Placement(visible = true, transformation(origin = {0, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.SignalRouting.PIN2INOUT pin2inout1 annotation(
    Placement(visible = true, transformation(origin = {44, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.SignalRouting.PIN2INOUT pin2inout2 annotation(
    Placement(visible = true, transformation(origin = {-70, 8}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 1)  annotation(
    Placement(visible = true, transformation(origin = {2, 76}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const1(k = 0) annotation(
    Placement(visible = true, transformation(origin = {0, 38}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
equation
V_DC_Global_pu = hVDC_base1.V_DC_local_pu;
w_sum_z_Global_pu = hVDC_base1.w_sum_z_local_pu;
P_in_Global_pu = hVDC_base1.P_in_local_pu;
  connect(const1.y, pin2inout1.vi) annotation(
    Line(points = {{0, 28}, {0, 28}, {0, 12}, {26, 12}, {26, 8}, {34, 8}, {34, 8}}, color = {0, 0, 127}));
  connect(const1.y, pin2inout2.vi) annotation(
    Line(points = {{0, 28}, {-20, 28}, {-20, 8}, {-60, 8}, {-60, 8}}, color = {0, 0, 127}));
  connect(const.y, pin2inout1.vr) annotation(
    Line(points = {{2, 64}, {2, 64}, {2, 60}, {28, 60}, {28, 16}, {34, 16}, {34, 16}}, color = {0, 0, 127}));
  connect(const.y, pin2inout2.vr) annotation(
    Line(points = {{2, 64}, {0, 64}, {0, 58}, {-44, 58}, {-44, 16}, {-60, 16}, {-60, 16}}, color = {0, 0, 127}));
  connect(pin2inout2.p, hVDC_base1.pwPin1) annotation(
    Line(points = {{-60, 0}, {-12, 0}, {-12, 0}, {-10, 0}}));
  connect(hVDC_base1.pwPin2, pin2inout1.p) annotation(
    Line(points = {{12, 0}, {34, 0}}));

annotation(
    uses(Modelica(version = "3.2.2")));end HVDC_base_test;
