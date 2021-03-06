package MMC
  model SystemParameters
    //MMC parameters
    constant Real pi = 2 * Modelica.Math.asin(1.0);
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
    //multiplier to model the capacitance of the cable
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
    parameter Real K_vci = 1 / 15 * 8 / 3 * C_DC_pu / (omega_base * sqrt(PhaseMarginConstant) ^ 3 * tau_e ^ 2);
    parameter Real K_p_E = 15;
    parameter Real K_i_E = 5000000;
    //facator of 3 is to include the cable capcitance in the model
    //factor of (1/10) to reduce oscilation
    //factor of (8/3) from the PU-definition.
  end SystemParameters;

  model MMC_zero
    extends SystemParameters;
    Modelica.Blocks.Interfaces.RealInput i_d_ref annotation(
      Placement(visible = true, transformation(origin = {-348, -78}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-322, 250}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput i_q_ref annotation(
      Placement(visible = true, transformation(origin = {-348, -106}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-320, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput w_sum_z_ref annotation(
      Placement(visible = true, transformation(origin = {-350, 234}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-322, -262}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput w_sum_z annotation(
      Placement(visible = true, transformation(origin = {310, 280}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {310, 280}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput i_sum_z annotation(
      Placement(visible = true, transformation(origin = {310, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {310, 122}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput i_q_mes annotation(
      Placement(visible = true, transformation(origin = {310, -270}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {310, -280}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.CurrentController currentController1 annotation(
      Placement(visible = true, transformation(origin = {-166, -32}, extent = {{-76, -76}, {76, 76}}, rotation = 0)));
    Modelica.Blocks.Sources.Step v_grid_d(height = 0, offset = V_base) annotation(
      Placement(visible = true, transformation(origin = {-316, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step v_grid_q(height = 0, offset = V_base * 0) annotation(
      Placement(visible = true, transformation(origin = {-318, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.Energy_Controller energy_Controller1 annotation(
      Placement(visible = true, transformation(origin = {-171, 165}, extent = {{-85, -85}, {85, 85}}, rotation = 0)));
    MMC.SystemEquations control1 annotation(
      Placement(visible = true, transformation(origin = {98.4, -8}, extent = {{-111.6, -186}, {111.6, 186}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step_vdc(height = 0, offset = V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {-26, -178}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step1(height = 0, offset = V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {-374, 92}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-281, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {-328, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-296, -20}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-298, -42}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain7(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-324, 130}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain8(k = 1 / w_base) annotation(
      Placement(visible = true, transformation(origin = {-326, 180}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain9(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-28, 165}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain10(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-20, 3}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain11(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-22, -69}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput i_d_mes annotation(
      Placement(visible = true, transformation(origin = {310, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {310, -120}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(gain7.y, energy_Controller1.i_circ_z) annotation(
      Line(points = {{-316, 130}, {-282, 130}, {-282, 98}, {-272, 98}}, color = {0, 0, 127}));
    connect(i_sum_z, gain7.u) annotation(
      Line(points = {{310, 110}, {352, 110}, {352, 308}, {-380, 308}, {-380, 132}, {-334, 132}, {-334, 130}}, color = {0, 0, 127}));
    connect(gain8.y, energy_Controller1.w_sum_z_pu) annotation(
      Line(points = {{-318, 180}, {-284, 180}, {-284, 148}, {-272, 148}}, color = {0, 0, 127}));
    connect(gain4.y, energy_Controller1.v_DC_pu) annotation(
      Line(points = {{-316, 90}, {-300, 90}, {-300, 190}, {-272, 190}, {-272, 190}}, color = {0, 0, 127}));
    connect(energy_Controller1.u_circ_z_pu, gain9.u) annotation(
      Line(points = {{-68, 200}, {-40, 200}, {-40, 166}, {-38, 166}}, color = {0, 0, 127}));
    connect(currentController1.v_q_conv_pu, gain11.u) annotation(
      Line(points = {{-84, -56}, {-32, -56}, {-32, -68}, {-32, -68}}, color = {0, 0, 127}));
    connect(currentController1.v_d_conv_pu, gain10.u) annotation(
      Line(points = {{-82, -10}, {-30, -10}, {-30, 4}, {-30, 4}}, color = {0, 0, 127}));
    connect(w_sum_z_ref, energy_Controller1.w_sum_z_ref) annotation(
      Line(points = {{-350, 234}, {-272, 234}, {-272, 234}, {-272, 234}}, color = {0, 0, 127}));
    connect(v_grid_q.y, currentController1.v_q_grid_pu) annotation(
      Line(points = {{-306, 8}, {-254, 8}, {-254, 16}, {-250, 16}}, color = {0, 0, 127}));
    connect(gain3.y, currentController1.v_d_grid_pu) annotation(
      Line(points = {{-270, 36}, {-254, 36}, {-254, 36}, {-250, 36}}, color = {0, 0, 127}));
    connect(v_grid_d.y, gain3.u) annotation(
      Line(points = {{-304, 38}, {-292, 38}, {-292, 36}}, color = {0, 0, 127}));
    connect(i_q_ref, currentController1.i_q_ref) annotation(
      Line(points = {{-348, -106}, {-254, -106}, {-254, -100}, {-250, -100}}, color = {0, 0, 127}));
    connect(gain6.y, currentController1.i_q_mes_pu) annotation(
      Line(points = {{-290, -42}, {-254, -42}, {-254, -42}, {-250, -42}}, color = {0, 0, 127}));
    connect(gain5.y, currentController1.i_d_mes_pu) annotation(
      Line(points = {{-288, -20}, {-254, -20}, {-254, -20}, {-250, -20}}, color = {0, 0, 127}));
    connect(gain5.u, i_d_mes) annotation(
      Line(points = {{-306, -20}, {-390, -20}, {-390, -312}, {322, -312}, {322, -112}, {306, -112}, {306, -110}, {310, -110}}, color = {0, 0, 127}));
    connect(control1.i_d_mes, i_d_mes) annotation(
      Line(points = {{192, -64}, {302, -64}, {302, -110}, {310, -110}}, color = {0, 0, 127}));
    connect(i_d_ref, currentController1.i_d_ref) annotation(
      Line(points = {{-348, -78}, {-252, -78}, {-252, -80}, {-250, -80}}, color = {0, 0, 127}));
    connect(step_vdc.y, control1.v_DC) annotation(
      Line(points = {{-15, -178}, {-2.5, -178}, {-2.5, -176}, {10, -176}}, color = {0, 0, 127}));
    connect(gain11.y, control1.v_q_conv) annotation(
      Line(points = {{-14, -68}, {14, -68}, {14, -64}, {10, -64}}, color = {0, 0, 127}));
    connect(gain10.y, control1.v_d_conv) annotation(
      Line(points = {{-12, 4}, {4, 4}, {4, 48}, {10, 48}}, color = {0, 0, 127}));
    connect(gain9.y, control1.u_circ_z) annotation(
      Line(points = {{-20, 164}, {2, 164}, {2, 160}, {10, 160}}, color = {0, 0, 127}));
    connect(gain8.u, w_sum_z) annotation(
      Line(points = {{-336, 180}, {-372, 180}, {-372, 302}, {326, 302}, {326, 282}, {310, 282}, {310, 280}}, color = {0, 0, 127}));
    connect(i_q_mes, gain6.u) annotation(
      Line(points = {{310, -270}, {312, -270}, {312, -306}, {-382, -306}, {-382, -42}, {-308, -42}, {-308, -42}}, color = {0, 0, 127}));
    connect(step1.y, gain4.u) annotation(
      Line(points = {{-362, 92}, {-342, 92}, {-342, 90}, {-340, 90}}, color = {0, 0, 127}));
    connect(control1.w_z_sum, w_sum_z) annotation(
      Line(points = {{192, 160}, {304, 160}, {304, 280}, {310, 280}}, color = {0, 0, 127}));
    connect(control1.i_circ_z, i_sum_z) annotation(
      Line(points = {{192, 52}, {300, 52}, {300, 110}, {310, 110}}, color = {0, 0, 127}));
    connect(control1.i_q_mes, i_q_mes) annotation(
      Line(points = {{192, -176}, {302, -176}, {302, -270}, {310, -270}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-300, 300}, {300, -300}}), Text(origin = {0, 5}, fillPattern = FillPattern.Solid, extent = {{-292, 293}, {292, -293}}, textString = "MMC_Zero")}, coordinateSystem(extent = {{-300, -300}, {300, 300}})),
      Diagram(coordinateSystem(extent = {{-300, -300}, {300, 300}})),
      __OpenModelica_commandLineOptions = "");
  end MMC_zero;

  model MMC_zero_sequence
    MMC.CurrentController currentController1 annotation(
      Placement(visible = true, transformation(origin = {-139, 91}, extent = {{-31, 31}, {31, -31}}, rotation = 0)));
    MMC.Energy_equation energy_equation1 annotation(
      Placement(visible = true, transformation(origin = {-62, 206}, extent = {{-48, -48}, {48, 48}}, rotation = 0)));
    MMC.Energy_Controller energy_Controller1(u_circ_z_pu(fixed = true, start = 1)) annotation(
      Placement(visible = true, transformation(origin = {167, -15}, extent = {{-37, -37}, {37, 37}}, rotation = 0)));
    MMC.VoltageController voltageController1 annotation(
      Placement(visible = true, transformation(origin = {-55, 273}, extent = {{-33, 33}, {33, -33}}, rotation = 180)));
  protected
    extends SystemParameters;
    Modelica.Blocks.Sources.Ramp ramp1(duration = 0.1, height = 0, offset = 1, startTime = 2) annotation(
      Placement(visible = true, transformation(origin = {163, 33}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin a annotation(
      Placement(visible = true, transformation(origin = {-400, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-402, 326}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin b annotation(
      Placement(visible = true, transformation(origin = {-400, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-398, 108}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin c annotation(
      Placement(visible = true, transformation(origin = {-400, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-392, -136}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R_f) annotation(
      Placement(visible = true, transformation(origin = {-230, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor2(R = R_f) annotation(
      Placement(visible = true, transformation(origin = {-230, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor3(R = R_f) annotation(
      Placement(visible = true, transformation(origin = {-230, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1 annotation(
      Placement(visible = true, transformation(origin = {-280, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor2 annotation(
      Placement(visible = true, transformation(origin = {-300, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor3 annotation(
      Placement(visible = true, transformation(origin = {-320, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {-340, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {-360, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {-380, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L_f) annotation(
      Placement(visible = true, transformation(origin = {-192, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor2(L = L_f) annotation(
      Placement(visible = true, transformation(origin = {-192, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor3(L = L_f) annotation(
      Placement(visible = true, transformation(origin = {-192, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.DQ_w VoltageTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w CurrentTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 94}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w_inv dQ_w_inv1 annotation(
      Placement(visible = true, transformation(origin = {-70, 92}, extent = {{-20, 20}, {20, -20}}, rotation = 0)));
    Modelica.Blocks.Sources.Step i_q_pu_ref(height = 0, offset = 0) annotation(
      Placement(visible = true, transformation(origin = {-180, 116}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 108}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 92}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain7(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 76}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_a annotation(
      Placement(visible = true, transformation(origin = {-22, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_b annotation(
      Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_c annotation(
      Placement(visible = true, transformation(origin = {18, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {50, 1}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain8(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 86}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain9(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 62}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 70}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 78}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 94}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 102}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage Two_v_circ annotation(
      Placement(visible = true, transformation(origin = {278, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain10(k = 2 * V_base) annotation(
      Placement(visible = true, transformation(origin = {248, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = 2 * C_DC, v(fixed = true, start = V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {374, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor2(C = 2 * C_DC, v(fixed = true, start = -V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {374, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Basic.Inductor inductor4(L = 2 / 3 * L_a) annotation(
      Placement(visible = true, transformation(origin = {292, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor4(R = 2 / 3 * R_a) annotation(
      Placement(visible = true, transformation(origin = {318, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
      Placement(visible = true, transformation(origin = {390, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {408, 190}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {400, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor1 annotation(
      Placement(visible = true, transformation(origin = {344, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain11(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {234, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Math.Gain gain12(k = 1 / I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {8, 299}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor4 annotation(
      Placement(visible = true, transformation(origin = {348, 60}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step V_DC_ref(height = 0.1, offset = 1, startTime = 3) annotation(
      Placement(visible = true, transformation(origin = {10, 246}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor5 annotation(
      Placement(visible = true, transformation(origin = {278, 28}, extent = {{10, 10}, {-10, -10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain13(k = 1 / 3) annotation(
      Placement(visible = true, transformation(origin = {-137, 191}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain14(k = 1 / (3 * I_base)) annotation(
      Placement(visible = true, transformation(origin = {100, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain15(k = I_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 212}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain16(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 220}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain17(k = 1 / w_base) annotation(
      Placement(visible = true, transformation(origin = {12, 206}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain18(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-137, 202}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain19(k = -1) annotation(
      Placement(visible = true, transformation(origin = {-182, 106}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
  equation
    connect(CurrentTransform.d, gain15.u) annotation(
      Line(points = {{-218, 90}, {-190, 90}, {-190, 212}, {-142, 212}, {-142, 212}}, color = {0, 0, 127}));
    connect(voltageController1.v_grid_d, VoltageTransform.d) annotation(
      Line(points = {{-72, 312}, {-74, 312}, {-74, 322}, {-200, 322}, {-200, 66}, {-218, 66}, {-218, 66}}, color = {0, 0, 127}));
    connect(currentController1.v_d_conv_pu, gain16.u) annotation(
      Line(points = {{-104, 82}, {-100, 82}, {-100, 158}, {-168, 158}, {-168, 220}, {-142, 220}, {-142, 220}}, color = {0, 0, 127}));
    connect(ramp1.y, energy_Controller1.w_sum_z_ref) annotation(
      Line(points = {{158, 34}, {120, 34}, {120, 14}, {122, 14}}, color = {0, 0, 127}));
    connect(resistor4.n, capacitor1.p) annotation(
      Line(points = {{328, 60}, {334, 60}, {334, 40}, {374, 40}, {374, 30}, {374, 30}}, color = {0, 0, 255}));
    connect(Two_v_circ.n, capacitor2.p) annotation(
      Line(points = {{278, -10}, {278, -52}, {372, -52}, {372, -30}}, color = {0, 0, 255}));
    connect(capacitor2.p, m_V_DC_2) annotation(
      Line(points = {{372, -30}, {372, -52}, {400, -52}}, color = {0, 0, 255}));
    connect(voltageSensor1.n, capacitor2.p) annotation(
      Line(points = {{344, -10}, {344, -10}, {344, -30}, {374, -30}, {374, -30}}, color = {0, 0, 255}));
    connect(capacitor1.n, capacitor2.n) annotation(
      Line(points = {{374, 10}, {374, -10}}, color = {0, 0, 255}));
    connect(ground2.p, capacitor1.n) annotation(
      Line(points = {{378, 0}, {374, 0}, {374, 10}}, color = {0, 0, 255}));
    connect(voltageSensor1.p, capacitor1.p) annotation(
      Line(points = {{342, 10}, {342, 10}, {342, 30}, {372, 30}, {372, 30}}, color = {0, 0, 255}));
    connect(gain11.u, voltageSensor1.v) annotation(
      Line(points = {{246, 42}, {332, 42}, {332, 0}}, color = {0, 0, 127}));
    connect(currentSensor4.p, V_DC_2) annotation(
      Line(points = {{358, 60}, {400, 60}}, color = {0, 0, 255}));
    connect(resistor4.n, currentSensor4.n) annotation(
      Line(points = {{330, 60}, {338, 60}}, color = {0, 0, 255}));
    connect(gain12.u, currentSensor4.i) annotation(
      Line(points = {{20, 300}, {348, 300}, {348, 70}}, color = {0, 0, 127}));
    connect(voltageController1.I_DC_pu, gain12.y) annotation(
      Line(points = {{-24, 300}, {-14, 300}, {-14, 298}, {-3, 298}}, color = {0, 0, 127}));
    connect(gain19.u, voltageController1.i_d_ref_pu) annotation(
      Line(points = {{-184, 106}, {-188, 106}, {-188, 272}, {-92, 272}, {-92, 274}}, color = {0, 0, 127}));
    connect(currentController1.i_d_ref, gain19.y) annotation(
      Line(points = {{-174, 106}, {-180, 106}, {-180, 106}, {-180, 106}}, color = {0, 0, 127}));
    connect(converterVoltage_c.n, ground1.p) annotation(
      Line(points = {{28, -20}, {40, -20}, {40, 0}}, color = {0, 0, 255}));
    connect(converterVoltage_b.n, ground1.p) annotation(
      Line(points = {{8, 0}, {40, 0}}, color = {0, 0, 255}));
    connect(converterVoltage_a.n, ground1.p) annotation(
      Line(points = {{-12, 20}, {40, 20}, {40, 0}}, color = {0, 0, 255}));
    connect(gain6.y, converterVoltage_b.v) annotation(
      Line(points = {{-27.6, 92}, {-4, 92}, {-4, 7}}, color = {0, 0, 127}));
    connect(inductor2.n, converterVoltage_b.p) annotation(
      Line(points = {{-182, 0}, {-14, 0}}, color = {0, 0, 255}));
    connect(gain18.u, energy_Controller1.u_circ_z_pu) annotation(
      Line(points = {{-142, 202}, {-162, 202}, {-162, 166}, {70, 166}, {70, -64}, {224, -64}, {224, 0}, {212, 0}, {212, 0}}, color = {0, 0, 127}));
    connect(gain13.u, currentSensor5.i) annotation(
      Line(points = {{-142, 191}, {-158, 191}, {-158, 174}, {78, 174}, {78, -78}, {268, -78}, {268, 28}}, color = {0, 0, 127}));
    connect(gain13.y, energy_equation1.i_circ_z) annotation(
      Line(points = {{-130.5, 191}, {-120.5, 191}, {-120.5, 192}, {-114, 192}}, color = {0, 0, 127}));
    connect(energy_equation1.v_circ_z, gain18.y) annotation(
      Line(points = {{-114, 202}, {-132, 202}, {-132, 202}, {-132, 202}}, color = {0, 0, 127}));
    connect(gain17.y, energy_Controller1.w_sum_z_pu) annotation(
      Line(points = {{16, 206}, {96, 206}, {96, -22}, {122, -22}, {122, -22}}, color = {0, 0, 127}));
    connect(energy_equation1.w_sum_z, gain17.u) annotation(
      Line(points = {{-10, 206}, {8, 206}, {8, 206}, {8, 206}}, color = {0, 0, 127}));
    connect(gain15.y, energy_equation1.i_grid_d) annotation(
      Line(points = {{-134, 212}, {-116, 212}, {-116, 210}, {-114, 210}}, color = {0, 0, 127}));
    connect(gain16.y, energy_equation1.v_conv_d) annotation(
      Line(points = {{-134, 220}, {-118, 220}, {-118, 220}, {-114, 220}}, color = {0, 0, 127}));
    connect(gain14.u, currentSensor5.i) annotation(
      Line(points = {{88, -46}, {78, -46}, {78, -78}, {268, -78}, {268, 28}, {268, 28}}, color = {0, 0, 127}));
    connect(gain14.y, energy_Controller1.i_circ_z) annotation(
      Line(points = {{112, -46}, {118, -46}, {118, -44}, {122, -44}}, color = {0, 0, 127}));
    connect(gain9.u, potentialSensor1.phi) annotation(
      Line(points = {{-266.8, 62}, {-277.8, 62}, {-277.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.a, gain9.y) annotation(
      Line(points = {{-240, 62}, {-256, 62}}, color = {0, 0, 127}));
    connect(gain1.u, potentialSensor2.phi) annotation(
      Line(points = {{-266.8, 70}, {-297.8, 70}, {-297.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.b, gain1.y) annotation(
      Line(points = {{-242, 70}, {-258, 70}}, color = {0, 0, 127}));
    connect(gain2.u, potentialSensor3.phi) annotation(
      Line(points = {{-266.8, 78}, {-317.8, 78}, {-317.8, 51}}, color = {0, 0, 127}));
    connect(gain2.y, VoltageTransform.c) annotation(
      Line(points = {{-257.6, 78}, {-241.6, 78}}, color = {0, 0, 127}));
    connect(gain8.u, currentSensor1.i) annotation(
      Line(points = {{-266.8, 86}, {-337.8, 86}, {-337.8, 30}}, color = {0, 0, 127}));
    connect(CurrentTransform.a, gain8.y) annotation(
      Line(points = {{-240, 86}, {-256, 86}}, color = {0, 0, 127}));
    connect(gain3.u, currentSensor2.i) annotation(
      Line(points = {{-266.8, 94}, {-357.8, 94}, {-357.8, 10}}, color = {0, 0, 127}));
    connect(CurrentTransform.b, gain3.y) annotation(
      Line(points = {{-240, 94}, {-256, 94}}, color = {0, 0, 127}));
    connect(gain4.u, currentSensor3.i) annotation(
      Line(points = {{-266.8, 102}, {-377.8, 102}, {-377.8, -10}}, color = {0, 0, 127}));
    connect(CurrentTransform.c, gain4.y) annotation(
      Line(points = {{-240, 102}, {-256, 102}}, color = {0, 0, 127}));
    connect(inductor4.p, currentSensor5.p) annotation(
      Line(points = {{282, 60}, {278, 60}, {278, 36}, {278, 36}}, color = {0, 0, 255}));
    connect(currentSensor5.n, Two_v_circ.p) annotation(
      Line(points = {{278, 16}, {278, 16}, {278, 10}, {278, 10}}, color = {0, 0, 255}));
    connect(energy_Controller1.u_circ_z_pu, gain10.u) annotation(
      Line(points = {{211.4, -0.2}, {226.4, -0.2}, {226.4, 0}, {238, 0}}, color = {0, 0, 127}));
    connect(gain10.y, Two_v_circ.v) annotation(
      Line(points = {{261, 0}, {269, 0}}, color = {0, 0, 127}));
    connect(gain11.y, energy_Controller1.v_DC_pu) annotation(
      Line(points = {{223, 42}, {106, 42}, {106, -4}, {122, -4}}, color = {0, 0, 127}));
    connect(voltageController1.V_DC_mes_pu, gain11.y) annotation(
      Line(points = {{-18, 274}, {198, 274}, {198, 42}, {223, 42}}, color = {0, 0, 127}));
    connect(resistor4.p, inductor4.n) annotation(
      Line(points = {{310, 60}, {302, 60}}, color = {0, 0, 255}));
    connect(voltageController1.V_DC_ref, V_DC_ref.y) annotation(
      Line(points = {{-18, 246}, {1, 246}}, color = {0, 0, 127}));
    connect(gain7.y, converterVoltage_a.v) annotation(
      Line(points = {{-27.6, 76}, {-24, 76}, {-24, 27}}, color = {0, 0, 127}));
    connect(inductor1.n, converterVoltage_a.p) annotation(
      Line(points = {{-182, 20}, {-34, 20}}, color = {0, 0, 255}));
    connect(gain5.y, converterVoltage_c.v) annotation(
      Line(points = {{-27.6, 108}, {16, 108}, {16, -13}}, color = {0, 0, 127}));
    connect(inductor3.n, converterVoltage_c.p) annotation(
      Line(points = {{-182, -20}, {6, -20}}, color = {0, 0, 255}));
    connect(gain7.u, dQ_w_inv1.a) annotation(
      Line(points = {{-35, 76}, {-46, 76}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.b, gain6.u) annotation(
      Line(points = {{-46, 92}, {-35, 92}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.c, gain5.u) annotation(
      Line(points = {{-46, 108}, {-35, 108}}, color = {0, 0, 127}));
    connect(currentController1.v_q_conv_pu, dQ_w_inv1.q) annotation(
      Line(points = {{-105.21, 100.61}, {-81.21, 100.61}, {-81.21, 102}, {-92, 102}}, color = {0, 0, 127}));
    connect(currentController1.v_d_conv_pu, dQ_w_inv1.d) annotation(
      Line(points = {{-104.9, 81.7}, {-91.9, 81.7}, {-91.9, 82}, {-92, 82}}, color = {0, 0, 127}));
    connect(currentController1.i_q_ref, i_q_pu_ref.y) annotation(
      Line(points = {{-173.1, 115.8}, {-177.1, 115.8}}, color = {0, 0, 127}));
    connect(CurrentTransform.d, currentController1.i_d_mes_pu) annotation(
      Line(points = {{-216, 90}, {-172, 90}, {-172, 84}}, color = {0, 0, 127}));
    connect(CurrentTransform.q, currentController1.i_q_mes_pu) annotation(
      Line(points = {{-216, 100}, {-174, 100}, {-174, 98}, {-172, 98}}, color = {0, 0, 127}));
    connect(VoltageTransform.q, currentController1.v_q_grid_pu) annotation(
      Line(points = {{-216, 76}, {-172, 76}, {-172, 74}}, color = {0, 0, 127}));
    connect(VoltageTransform.d, currentController1.v_d_grid_pu) annotation(
      Line(points = {{-216, 66}, {-174, 66}, {-174, 64}, {-172, 64}}, color = {0, 0, 127}));
    connect(potentialSensor1.p, a) annotation(
      Line(points = {{-280, 30}, {-280, 20}, {-398, 20}}, color = {0, 0, 255}));
    connect(a, currentSensor1.n) annotation(
      Line(points = {{-398, 20}, {-349, 20}}, color = {0, 0, 255}));
    connect(resistor1.n, inductor1.p) annotation(
      Line(points = {{-218, 20}, {-198, 20}, {-198, 20}, {-198, 20}}, color = {0, 0, 255}));
    connect(currentSensor1.p, resistor1.p) annotation(
      Line(points = {{-330, 20}, {-241, 20}}, color = {0, 0, 255}));
    connect(potentialSensor2.p, b) annotation(
      Line(points = {{-298, 30}, {-298, 0}, {-396, 0}}, color = {0, 0, 255}));
    connect(potentialSensor3.p, c) annotation(
      Line(points = {{-318, 30}, {-318, -20}, {-396, -20}}, color = {0, 0, 255}));
    connect(b, currentSensor2.n) annotation(
      Line(points = {{-398, 0}, {-371, 0}}, color = {0, 0, 255}));
    connect(c, currentSensor3.n) annotation(
      Line(points = {{-398, -20}, {-391, -20}}, color = {0, 0, 255}));
    connect(resistor2.n, inductor2.p) annotation(
      Line(points = {{-218, 0}, {-198, 0}, {-198, 0}, {-198, 0}}, color = {0, 0, 255}));
    connect(currentSensor2.p, resistor2.p) annotation(
      Line(points = {{-350, 0}, {-239, 0}}, color = {0, 0, 255}));
    connect(resistor3.n, inductor3.p) annotation(
      Line(points = {{-218, -20}, {-198, -20}, {-198, -20}, {-198, -20}}, color = {0, 0, 255}));
    connect(currentSensor3.p, resistor3.p) annotation(
      Line(points = {{-370, -20}, {-239, -20}}, color = {0, 0, 255}));
    annotation(
      Diagram(coordinateSystem(extent = {{-400, -200}, {400, 400}})),
      Icon(coordinateSystem(extent = {{-400, -200}, {400, 400}}, initialScale = 0.1), graphics = {Rectangle(origin = {0, -1}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-400, 401}, {400, -201}}), Text(origin = {-44, 132}, fillPattern = FillPattern.Solid, extent = {{-236, 114}, {290, -166}}, textString = "MMC_V")}),
      __OpenModelica_commandLineOptions = "");
  end MMC_zero_sequence;

  model PowerController
    Modelica.Blocks.Math.MultiProduct P_pu(nu = 2) annotation(
      Placement(visible = true, transformation(origin = {-82, 20}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
    Modelica.Blocks.Interfaces.RealInput P_ref annotation(
      Placement(visible = true, transformation(origin = {-122, 2}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-111, -81}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput V_DC_mes_pu annotation(
      Placement(visible = true, transformation(origin = {-122, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -4.44089e-16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput I_DC_pu annotation(
      Placement(visible = true, transformation(origin = {-124, 88}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_grid_d annotation(
      Placement(visible = true, transformation(origin = {-113, 53}, extent = {{-11, -11}, {11, 11}}, rotation = 0), iconTransformation(origin = {50, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-82, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction1(a = {1, 0}, b = {K_vcp, K_vci * 10}) annotation(
      Placement(visible = true, transformation(origin = {-44, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback2 annotation(
      Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.ff_voltage_controller ff_voltage_controller1 annotation(
      Placement(visible = true, transformation(origin = {-12, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput i_d_ref_pu annotation(
      Placement(visible = true, transformation(origin = {106, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = feed) annotation(
      Placement(visible = true, transformation(origin = {-12, 24}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    extends SystemParameters;
  equation
    connect(ff_voltage_controller1.V_d, v_grid_d) annotation(
      Line(points = {{-24, 48}, {-106, 48}, {-106, 54}, {-112, 54}}, color = {0, 0, 127}));
    connect(I_DC_pu, P_pu.u[2]) annotation(
      Line(points = {{-124, 88}, {-82, 88}, {-82, 26}, {-82, 26}}, color = {0, 0, 127}));
    connect(P_pu.y, feedback1.u2) annotation(
      Line(points = {{-82, 12}, {-82, 12}, {-82, 8}, {-82, 8}}, color = {0, 0, 127}));
    connect(V_DC_mes_pu, P_pu.u[1]) annotation(
      Line(points = {{-122, 30}, {-84, 30}, {-84, 26}, {-82, 26}}, color = {0, 0, 127}));
    connect(gain1.y, feedback2.u2) annotation(
      Line(points = {{-12, 12}, {-8, 12}, {-8, 8}, {-10, 8}}, color = {0, 0, 127}));
    connect(ff_voltage_controller1.ff, gain1.u) annotation(
      Line(points = {{-12, 42}, {-12, 42}, {-12, 36}, {-12, 36}}, color = {0, 0, 127}));
    connect(transferFunction1.y, feedback2.u1) annotation(
      Line(points = {{-31, 0}, {-18, 0}}, color = {0, 0, 127}));
    connect(feedback1.y, transferFunction1.u) annotation(
      Line(points = {{-72, 0}, {-54, 0}}, color = {0, 0, 127}));
    connect(P_ref, feedback1.u1) annotation(
      Line(points = {{-122, 0}, {-90, 0}}, color = {0, 0, 127}));
    connect(I_DC_pu, ff_voltage_controller1.i_DC) annotation(
      Line(points = {{-124, 88}, {8, 88}, {8, 52}, {0, 52}}, color = {0, 0, 127}));
    connect(ff_voltage_controller1.V_DC, V_DC_mes_pu) annotation(
      Line(points = {{-11, 63}, {-11, 78}, {-70, 78}, {-70, 30}, {-120, 30}}, color = {0, 0, 127}));
    connect(feedback2.y, i_d_ref_pu) annotation(
      Line(points = {{0, 0}, {100, 0}, {100, 2}, {106, 2}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, 0}, fillPattern = FillPattern.Solid, extent = {{-99, 100}, {99, -100}}, textString = "PowerController")}, coordinateSystem(initialScale = 0.1)));
  end PowerController;

  class CurrentController
    //extends CurrentTunerParameter;
    Modelica.Blocks.Interfaces.RealInput v_q_grid_pu annotation(
      Placement(visible = true, transformation(origin = {-80, -86}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput i_d_mes_pu annotation(
      Placement(visible = true, transformation(origin = {-80, 28}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput i_q_mes_pu annotation(
      Placement(visible = true, transformation(origin = {-80, -28}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput v_d_conv_pu(start = 1) annotation(
      Placement(visible = true, transformation(origin = {84, 55}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput v_q_conv_pu annotation(
      Placement(visible = true, transformation(origin = {84, -57}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {109, -31}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput i_q_ref annotation(
      Placement(visible = true, transformation(origin = {-80, -58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput i_d_ref annotation(
      Placement(visible = true, transformation(origin = {-80, 55}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-40, 55}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback2 annotation(
      Placement(visible = true, transformation(origin = {-40, -58}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_d_grid_pu annotation(
      Placement(visible = true, transformation(origin = {-80, 83}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    extends SystemParameters;
    // parameter Real w = 2 * pi * 50;
    //parameter Real pi = 2 * Modelica.Math.asin(1.0);
    Modelica.Blocks.Math.Gain gain1(k = L_v_pu) annotation(
      Placement(visible = true, transformation(origin = {-30, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = -L_v_pu) annotation(
      Placement(visible = true, transformation(origin = {-30, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.CurrentAdder currentAdder1 annotation(
      Placement(visible = true, transformation(origin = {48, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.CurrentAdder currentAdder2 annotation(
      Placement(visible = true, transformation(origin = {48, -58}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
    Modelica.Blocks.Continuous.PI PI(T = k_cc_p / k_cc_i, k = k_cc_p) annotation(
      Placement(visible = true, transformation(origin = {2, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.PI PI1(T = k_cc_p / k_cc_i, k = k_cc_p) annotation(
      Placement(visible = true, transformation(origin = {2, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(currentAdder1.y, v_d_conv_pu) annotation(
      Line(points = {{59, 55}, {82, 55}}, color = {0, 0, 127}));
    connect(currentAdder2.y, v_q_conv_pu) annotation(
      Line(points = {{59, -57}, {82, -57}}, color = {0, 0, 127}));
    connect(v_d_grid_pu, currentAdder1.u) annotation(
      Line(points = {{-80, 84}, {48, 84}, {48, 68}}, color = {0, 0, 127}));
    connect(gain2.y, currentAdder1.u2) annotation(
      Line(points = {{-18, -16}, {0, -16}, {0, 20}, {48, 20}, {48, 45}}, color = {0, 0, 127}));
    connect(PI.y, currentAdder1.u1) annotation(
      Line(points = {{14, 56}, {34, 56}, {34, 54}, {36, 54}}, color = {0, 0, 127}));
    connect(v_q_grid_pu, currentAdder2.u) annotation(
      Line(points = {{-80, -86}, {48, -86}, {48, -71}}, color = {0, 0, 127}));
    connect(gain1.y, currentAdder2.u2) annotation(
      Line(points = {{-19, 14}, {48, 14}, {48, -47}}, color = {0, 0, 127}));
    connect(PI1.y, currentAdder2.u1) annotation(
      Line(points = {{13, -58}, {24.5, -58}, {24.5, -59}, {36, -59}}, color = {0, 0, 127}));
    connect(feedback2.y, PI1.u) annotation(
      Line(points = {{-30, -58}, {-12, -58}, {-12, -59}, {-10, -59}}, color = {0, 0, 127}));
    connect(feedback1.y, PI.u) annotation(
      Line(points = {{-30, 56}, {-12, 56}, {-12, 56}, {-10, 56}}, color = {0, 0, 127}));
    connect(i_d_mes_pu, gain1.u) annotation(
      Line(points = {{-80, 26}, {-72, 26}, {-72, 16}, {-42, 16}}, color = {0, 0, 127}));
    connect(i_d_mes_pu, feedback1.u2) annotation(
      Line(points = {{-80, 28}, {-40, 28}, {-40, 47}}, color = {0, 0, 127}));
    connect(i_d_ref, feedback1.u1) annotation(
      Line(points = {{-80, 55}, {-48, 55}}, color = {0, 0, 127}));
    connect(i_q_mes_pu, gain2.u) annotation(
      Line(points = {{-80, -28}, {-70, -28}, {-70, -16}, {-42, -16}}, color = {0, 0, 127}));
    connect(i_q_mes_pu, feedback2.u2) annotation(
      Line(points = {{-80, -30}, {-40, -30}, {-40, -52}}, color = {0, 0, 127}));
    connect(i_q_ref, feedback2.u1) annotation(
      Line(points = {{-80, -58}, {-48, -58}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {23, -52}, fillPattern = FillPattern.Solid, extent = {{-111, 158}, {57, -62}}, textString = "Current controller")}, coordinateSystem(initialScale = 0.1)),
      uses(Modelica(version = "3.2.2")));
  end CurrentController;


  model Energy_Controller
    Modelica.Blocks.Interfaces.RealInput w_sum_z_ref annotation(
      Placement(visible = true, transformation(origin = {-85, 10}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput w_sum_z_pu annotation(
      Placement(visible = true, transformation(origin = {-85, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput u_circ_z_pu(start = 1) annotation(
      Placement(visible = true, transformation(origin = {118, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput i_circ_z_pu annotation(
      Placement(visible = true, transformation(origin = {-85, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_DC_pu annotation(
      Placement(visible = true, transformation(origin = {-85, -76}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  protected
    extends SystemParameters;
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-53, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback2 annotation(
      Placement(visible = true, transformation(origin = {-3, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Add add1 annotation(
      Placement(visible = true, transformation(origin = {62, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain half(k = 1 / 2) annotation(
      Placement(visible = true, transformation(origin = {10, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.PI PI(T = k_p_w_sum_z / k_i_w_sum_z, k = k_p_w_sum_z) annotation(
      Placement(visible = true, transformation(origin = {-27, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.PI PI1(T = k_circ_cc_p / k_circ_cc_i, k = -k_circ_cc_p) annotation(
      Placement(visible = true, transformation(origin = {22, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(PI1.y, add1.u1) annotation(
      Line(points = {{34, 10}, {76, 10}, {76, 16}, {76, 16}}, color = {0, 0, 127}));
    connect(add1.y, u_circ_z_pu) annotation(
      Line(points = {{74, 10}, {110, 10}, {110, 10}, {118, 10}}, color = {0, 0, 127}));
    connect(w_sum_z_ref, feedback1.u1) annotation(
      Line(points = {{-85, 10}, {-61, 10}}, color = {0, 0, 127}));
    connect(w_sum_z_pu, feedback1.u2) annotation(
      Line(points = {{-85, -20}, {-53, -20}, {-53, 2}}, color = {0, 0, 127}));
    connect(feedback1.y, PI.u) annotation(
      Line(points = {{-44, 8}, {-39, 8}}, color = {0, 0, 127}));
    connect(i_circ_z_pu, feedback2.u2) annotation(
      Line(points = {{-85, -50}, {-3, -50}, {-3, 2}}, color = {0, 0, 127}));
    connect(PI.y, feedback2.u1) annotation(
      Line(points = {{-16, 10}, {-11, 10}}, color = {0, 0, 127}));
    connect(feedback2.y, PI1.u) annotation(
      Line(points = {{6, 8}, {10, 8}}, color = {0, 0, 127}));
    connect(half.y, add1.u2) annotation(
      Line(points = {{21, -76}, {48, -76}, {48, 4}, {50, 4}}, color = {0, 0, 127}));
    connect(v_DC_pu, half.u) annotation(
      Line(points = {{-85, -76}, {-2, -76}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, 3}, fillPattern = FillPattern.Solid, extent = {{-97, 97}, {97, -97}}, textString = "Energy Controller")}));
  end Energy_Controller;

  model VoltageController
    Modelica.Blocks.Interfaces.RealInput V_DC_ref annotation(
      Placement(visible = true, transformation(origin = {-122, 2}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-111, -81}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput V_DC_mes_pu annotation(
      Placement(visible = true, transformation(origin = {-115, 37}, extent = {{-13, -13}, {13, 13}}, rotation = 0), iconTransformation(origin = {-110, -4.44089e-16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput I_DC_pu annotation(
      Placement(visible = true, transformation(origin = {-126, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_grid_d annotation(
      Placement(visible = true, transformation(origin = {-116, 56}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {50, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-82, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction1(a = {1, 0}, b = {K_vcp, K_vci}) annotation(
      Placement(visible = true, transformation(origin = {-44, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback2 annotation(
      Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.ff_voltage_controller ff_voltage_controller1 annotation(
      Placement(visible = true, transformation(origin = {-12, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput i_d_ref_pu annotation(
      Placement(visible = true, transformation(origin = {106, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    extends SystemParameters;
  equation
    connect(ff_voltage_controller1.ff, feedback2.u2) annotation(
      Line(points = {{-12, 42}, {-10, 42}, {-10, 8}, {-10, 8}}, color = {0, 0, 127}));
    connect(ff_voltage_controller1.V_d, v_grid_d) annotation(
      Line(points = {{-24, 48}, {-96, 48}, {-96, 56}, {-116, 56}, {-116, 56}}, color = {0, 0, 127}));
    connect(I_DC_pu, ff_voltage_controller1.i_DC) annotation(
      Line(points = {{-126, 100}, {8, 100}, {8, 52}, {0, 52}}, color = {0, 0, 127}));
    connect(ff_voltage_controller1.V_DC, V_DC_mes_pu) annotation(
      Line(points = {{-11, 63}, {-11, 78}, {-70, 78}, {-70, 37}, {-115, 37}}, color = {0, 0, 127}));
    connect(feedback1.u2, V_DC_mes_pu) annotation(
      Line(points = {{-82, 8}, {-82, 37}, {-115, 37}}, color = {0, 0, 127}));
    connect(transferFunction1.y, feedback2.u1) annotation(
      Line(points = {{-31, 0}, {-18, 0}}, color = {0, 0, 127}));
    connect(feedback1.y, transferFunction1.u) annotation(
      Line(points = {{-72, 0}, {-54, 0}}, color = {0, 0, 127}));
    connect(V_DC_ref, feedback1.u1) annotation(
      Line(points = {{-122, 0}, {-90, 0}}, color = {0, 0, 127}));
    connect(feedback2.y, i_d_ref_pu) annotation(
      Line(points = {{0, 0}, {100, 0}, {100, 2}, {106, 2}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, 0}, fillPattern = FillPattern.Solid, extent = {{-99, 100}, {99, -100}}, textString = "VoltageController")}));
  end VoltageController;

  model Energy_equation
    Modelica.Blocks.Interfaces.RealInput v_conv_d annotation(
      Placement(visible = true, transformation(origin = {-108, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput i_grid_d annotation(
      Placement(visible = true, transformation(origin = {-98, 10}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_circ_z annotation(
      Placement(visible = true, transformation(origin = {-88, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput i_circ_z annotation(
      Placement(visible = true, transformation(origin = {-78, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput w_sum_z(start = w_base) annotation(
      Placement(visible = true, transformation(origin = {104, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    extends SystemParameters;
  equation
    der(w_sum_z) = (-1 / 2) * (i_grid_d * v_conv_d) + 2 * v_circ_z * i_circ_z;
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 40}, {100, -40}}), Text(origin = {3, 1}, fillPattern = FillPattern.Solid, extent = {{-95, 27}, {83, -25}}, textString = "Energy Equation")}, coordinateSystem(initialScale = 0.1)));
  end Energy_equation;

  model CirculatingCurrentEquation
    Modelica.Blocks.Interfaces.RealInput V_DC annotation(
      Placement(visible = true, transformation(origin = {-108, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_circ annotation(
      Placement(visible = true, transformation(origin = {-130, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -22}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput i_circ annotation(
      Placement(visible = true, transformation(origin = {126, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {119, -1}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
  protected
    extends SystemParameters;
  equation
    L_a * der(i_circ) * 2 / 3 = (-R_a * i_circ) * 2 / 3 + V_DC - v_circ;
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 40}, {100, -38}}), Text(origin = {-22, 16}, fillPattern = FillPattern.Solid, extent = {{-60, 10}, {100, -38}}, textString = "CirculatingCurrentEquation")}, coordinateSystem(initialScale = 0.1)));
  end CirculatingCurrentEquation;

  /*
                                                                                                                                                                  AC voltages are called e_
                                                                                                                                                                  converter voltages are called v_
                                                                                                                                                                  */

  model SystemEquations
    //Parameters
    extends SystemParameters;
    parameter Real v_d_grid = 2.7189e5 "AC-grid voltage d-component";
    //Perfect dq transformation is assumed.
    parameter Real v_q_grid = 0 "AC-grid voltage q-component";
    //Inputs
    Modelica.Blocks.Interfaces.RealInput u_circ_z annotation(
      Placement(visible = true, transformation(origin = {-78, -16}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-47, 90}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_d_conv annotation(
      Placement(visible = true, transformation(origin = {-68, 26}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-47, 30}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_q_conv annotation(
      Placement(visible = true, transformation(origin = {-72, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-47, -30}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_DC annotation(
      Placement(visible = true, transformation(origin = {-62, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-47, -90}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
    //Outputs
    Modelica.Blocks.Interfaces.RealOutput w_z_sum(start = w_base * 1.3) annotation(
      Placement(visible = true, transformation(origin = {6, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput i_circ_z(start = 0) annotation(
      Placement(visible = true, transformation(origin = {16, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput i_d_mes(start = 0) annotation(
      Placement(visible = true, transformation(origin = {26, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput i_q_mes(start = 0) annotation(
      Placement(visible = true, transformation(origin = {36, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
//Defining fluxes
//Describing relationship between input and output
    der(L_v * i_d_mes) = (-R_v * i_d_mes) + v_d_conv - v_d_grid + omega_base * L_v * i_q_mes;
    der(L_v * i_q_mes) = (-R_v * i_q_mes) + v_q_conv - v_q_grid - omega_base * L_v * i_d_mes;
    der(L_a * i_circ_z) = (-R_a * i_circ_z) + v_DC / 2 - u_circ_z;
    der(w_z_sum) = (-1 / 2) * (v_d_conv * i_d_mes + v_q_conv * i_q_mes) + 2 * u_circ_z * i_circ_z;
    annotation(
      Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-38, 98}, {40, -100}}), Text(fillPattern = FillPattern.Solid, extent = {{-36, 94}, {36, -94}}, textString = "f(x)")}, coordinateSystem(extent = {{-60, -100}, {60, 100}}, initialScale = 0.1)),
      Diagram(coordinateSystem(extent = {{-60, -100}, {60, 100}})),
      __OpenModelica_commandLineOptions = "");
  end SystemEquations;

  model MMC_Zero_test
    Real i_sum_z_pu;
    Real w_sum_z_pu;
    Real i_d_mes_pu;
    Real i_q_mes_pu;
  protected
    extends SystemParameters;
    MMC.MMC_zero MMC_zero1 annotation(
      Placement(visible = true, transformation(origin = {4, -2}, extent = {{-62, -62}, {62, 62}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step1(height = -1 * 0.95, offset = -1 * 0.85, startTime = 0.3) annotation(
      Placement(visible = true, transformation(origin = {-88, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step2(height = 0.2, offset = 0, startTime = 0.7) annotation(
      Placement(visible = true, transformation(origin = {-88, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step3(height = 0.05, offset = 1.3, startTime = 5) annotation(
      Placement(visible = true, transformation(origin = {-90, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    i_sum_z_pu = MMC_zero1.i_sum_z / I_base;
    w_sum_z_pu = MMC_zero1.w_sum_z / w_base;
    i_d_mes_pu = MMC_zero1.i_d_mes / I_base;
    i_q_mes_pu = MMC_zero1.i_q_mes / I_base;
    connect(step3.y, MMC_zero1.w_sum_z_ref) annotation(
      Line(points = {{-78, -56}, {-64, -56}, {-64, -56}, {-62, -56}}, color = {0, 0, 127}));
    connect(step2.y, MMC_zero1.i_q_ref) annotation(
      Line(points = {{-76, -2}, {-64, -2}, {-64, -2}, {-62, -2}}, color = {0, 0, 127}));
    connect(step1.y, MMC_zero1.i_d_ref) annotation(
      Line(points = {{-76, 50}, {-62, 50}, {-62, 50}, {-62, 50}}, color = {0, 0, 127}));
  end MMC_Zero_test;

  model Controller_test
    extends SystemParameters;
    MMC.SystemEquations control1 annotation(
      Placement(visible = true, transformation(origin = {4.8, -4}, extent = {{-34.8, -58}, {34.8, 58}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step1(height = 0, offset = 3.2e5) annotation(
      Placement(visible = true, transformation(origin = {-56, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step2(height = 0, offset = 2.7e5) annotation(
      Placement(visible = true, transformation(origin = {-56, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step3(height = 3e4, offset = -6e4, startTime = 0.4) annotation(
      Placement(visible = true, transformation(origin = {-54, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step4(height = 0, offset = 640000, startTime = 0) annotation(
      Placement(visible = true, transformation(origin = {-54, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(step4.y, control1.v_DC) annotation(
      Line(points = {{-45, -56}, {-22, -56}}, color = {0, 0, 127}));
    connect(step3.y, control1.v_q_conv) annotation(
      Line(points = {{-44, -20}, {-24, -20}, {-24, -22}, {-22, -22}}, color = {0, 0, 127}));
    connect(step1.y, control1.u_circ_z) annotation(
      Line(points = {{-47, 48}, {-22, 48}}, color = {0, 0, 127}));
    connect(step2.y, control1.v_d_conv) annotation(
      Line(points = {{-44, 12}, {-26, 12}, {-26, 14}, {-22, 14}}, color = {0, 0, 127}));
  end Controller_test;

  model MMC_zero_sequence_test
    Grid grid1 annotation(
      Placement(visible = true, transformation(origin = {-96, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.MMC_zero_sequence_V mMC_zero_sequence_V1 annotation(
      Placement(visible = true, transformation(origin = {-37.2478, 47.2478}, extent = {{-18.7522, -9.37608}, {18.7522, 18.7522}}, rotation = 0)));
    MMC.MMC_zero_sequence_V mMC_zero_sequence_V2 annotation(
      Placement(visible = true, transformation(origin = {-59.2183, -16.7817}, extent = {{-20.7817, -10.3908}, {20.7817, 20.7817}}, rotation = 0)));
    MMC.Grid grid2 annotation(
      Placement(visible = true, transformation(origin = {-96, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.MMC_zero_sequence_P mMC_zero_sequence_P1 annotation(
      Placement(visible = true, transformation(origin = {71.0591, 44.9409}, extent = {{25.0591, -12.5296}, {-25.0591, 25.0591}}, rotation = 0)));
    MMC.Grid grid3 annotation(
      Placement(visible = true, transformation(origin = {130, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Cable.HVDC_Cable hVDC_Cable1 annotation(
      Placement(visible = true, transformation(origin = {18, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Cable.HVDC_Cable hVDC_Cable2(l = 20) annotation(
      Placement(visible = true, transformation(origin = {-20, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    extends SystemParameters;
  equation
    connect(mMC_zero_sequence_P1.c, grid3.C) annotation(
      Line(points = {{96, 33}, {120.617, 33}, {120.617, 30.6856}, {129.617, 30.6856}, {129.617, 42.6855}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_P1.b, grid3.B) annotation(
      Line(points = {{96, 48}, {119.993, 48}, {119.993, 47.9716}, {129.993, 47.9716}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_P1.a, grid3.A) annotation(
      Line(points = {{96, 62}, {128.243, 62}, {128.243, 53.6288}, {130.243, 53.6288}}, color = {0, 0, 255}));
    connect(hVDC_Cable1.m_V_DC_2_out, mMC_zero_sequence_P1.m_V_DC_2) annotation(
      Line(points = {{28, 42}, {46, 42}}, color = {0, 0, 255}));
    connect(hVDC_Cable1.V_DC_2_out, mMC_zero_sequence_P1.V_DC_2) annotation(
      Line(points = {{28, 54}, {27, 54}, {27, 53}, {45, 53}}, color = {0, 0, 255}));
    connect(grid1.C, mMC_zero_sequence_V1.c) annotation(
      Line(points = {{-96, 44}, {-90, 44}, {-90, 36}, {-56, 36}}, color = {0, 0, 255}));
    connect(grid1.B, mMC_zero_sequence_V1.b) annotation(
      Line(points = {{-96, 50}, {-90, 50}, {-90, 47}, {-56, 47}}, color = {0, 0, 255}));
    connect(grid1.A, mMC_zero_sequence_V1.a) annotation(
      Line(points = {{-96, 56}, {-96, 57}, {-56, 57}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V1.m_V_DC_2, hVDC_Cable1.m_V_DC_2_in) annotation(
      Line(points = {{-18, 39}, {-5, 39}, {-5, 42}, {8, 42}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V1.V_DC_2, hVDC_Cable1.V_DC_2_in) annotation(
      Line(points = {{-18, 51}, {8, 51}, {8, 54}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V2.a, grid2.A) annotation(
      Line(points = {{-80, -5}, {-96, -5}, {-96, -12}}, color = {0, 0, 255}));
    connect(grid2.B, mMC_zero_sequence_V2.b) annotation(
      Line(points = {{-96, -18}, {-90, -18}, {-90, -16}, {-80, -16}}, color = {0, 0, 255}));
    connect(grid2.C, mMC_zero_sequence_V2.c) annotation(
      Line(points = {{-96, -24}, {-90, -24}, {-90, -29}, {-80, -29}}, color = {0, 0, 255}));
    connect(hVDC_Cable2.m_V_DC_2_out, hVDC_Cable1.m_V_DC_2_in) annotation(
      Line(points = {{-10, -24}, {4, -24}, {4, 42}, {8, 42}, {8, 42}}, color = {0, 0, 255}));
    connect(hVDC_Cable2.V_DC_2_out, hVDC_Cable1.V_DC_2_in) annotation(
      Line(points = {{-10, -12}, {-6, -12}, {-6, 54}, {8, 54}, {8, 54}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V2.V_DC_2, hVDC_Cable2.V_DC_2_in) annotation(
      Line(points = {{-38, -12}, {-30, -12}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V2.m_V_DC_2, hVDC_Cable2.m_V_DC_2_in) annotation(
      Line(points = {{-38, -24}, {-30, -24}}, color = {0, 0, 255}));
  end MMC_zero_sequence_test;

  class CurrentAdder
    Modelica.Blocks.Interfaces.RealInput u annotation(
      Placement(visible = true, transformation(origin = {-120, 68}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Interfaces.RealInput u1 annotation(
      Placement(visible = true, transformation(origin = {-110, 78}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-122, -2}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput u2 annotation(
      Placement(visible = true, transformation(origin = {-100, 88}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(visible = true, transformation(origin = {114, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {114, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    y = u + u1 + u2;
    annotation(
      Icon(coordinateSystem(initialScale = 0.1), graphics = {Ellipse(origin = {1, -1}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-101, 99}, {101, -99}}, endAngle = 360), Line(origin = {0, -5}, points = {{0, 65}, {0, -65}, {0, -65}}, thickness = 0.5), Line(origin = {0.64, -1.64}, points = {{-73, 0}, {73, 0}, {73, 0}}, thickness = 0.5)}),
      uses(Modelica(version = "3.2.2")));
  end CurrentAdder;

  model DQ_w
    parameter Real pi = 2 * Modelica.Math.asin(1.0);
    parameter Real w = 2 * pi * 50;
    Modelica.Blocks.Interfaces.RealInput a annotation(
      Placement(visible = true, transformation(origin = {-116, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput b annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput c annotation(
      Placement(visible = true, transformation(origin = {-126, -74}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput d annotation(
      Placement(visible = true, transformation(origin = {120, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput q annotation(
      Placement(visible = true, transformation(origin = {120, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    d = 2 / 3 * (a * cos(w * time) + b * cos(w * time - 2 * pi / 3) + c * cos(w * time - 4 * pi / 3));
    q = 2 / 3 * ((-a * sin(w * time)) - b * sin(w * time - 2 * pi / 3) - c * sin(w * time - 4 * pi / 3));
    annotation(
      Icon(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(origin = {0, -1}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 101}, {100, -99}}), Text(origin = {-60, 82}, fillPattern = FillPattern.Solid, extent = {{-34, 34}, {74, -90}}, textString = "123"), Line(points = {{-100, -100}, {100, 100}, {100, 100}}), Text(origin = {16, -37}, fillPattern = FillPattern.Solid, extent = {{-24, -35}, {54, 43}}, textString = "dq_w")}),
      uses(Modelica(version = "3.2.2")),
      Diagram);
  end DQ_w;

  model DQ_w_inv
    parameter Real pi = 2 * Modelica.Math.asin(1.0);
    parameter Real w = 2 * pi * 50;
    Modelica.Blocks.Interfaces.RealInput d annotation(
      Placement(visible = true, transformation(origin = {-160, 72}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput q annotation(
      Placement(visible = true, transformation(origin = {-150, 82}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput a annotation(
      Placement(visible = true, transformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput b annotation(
      Placement(visible = true, transformation(origin = {120, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput c annotation(
      Placement(visible = true, transformation(origin = {130, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    a = d * cos(w * time) - q * sin(w * time);
    b = d * cos(w * time - 2 * pi / 3) - q * sin(w * time - 2 * pi / 3);
    c = d * cos(w * time - 4 * pi / 3) - q * sin(w * time - 4 * pi / 3);
    annotation(
      Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(points = {{-100, -100}, {100, 100}, {100, 100}}), Text(origin = {-38, 68}, fillPattern = FillPattern.Solid, extent = {{28, 74}, {-46, -90}}, textString = "dq_w"), Text(origin = {74, -87}, fillPattern = FillPattern.Solid, extent = {{2, -1}, {-92, 75}}, textString = "123")}, coordinateSystem(initialScale = 0.1)),
      uses(Modelica(version = "3.2.2")));
  end DQ_w_inv;

  model ff_voltage_controller
  //Real k(start = 0);
    //Real a;
    Modelica.Blocks.Interfaces.RealInput i_DC annotation(
      Placement(visible = true, transformation(origin = {-108, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {98, -2}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
    Modelica.Blocks.Interfaces.RealInput V_DC annotation(
      Placement(visible = true, transformation(origin = {-112, 52}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-10, 108}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Interfaces.RealInput V_d annotation(
      Placement(visible = true, transformation(origin = {-114, -58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-114, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput ff annotation(
      Placement(visible = true, transformation(origin = {-2, -114}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-8, -100}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  equation
//when {time > 10} then
//  k := 1;
// end when;
//  a := (delay(w_sum_z_pu, 0.0001) - delay(w_sum_z_pu, 0.0002)) / 0.0001 / V_d;
    ff = i_DC * V_DC / V_d;
// - a * k;
    annotation(
      Icon(graphics = {Ellipse(origin = {-10, -1}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-86, 87}, {86, -87}}, endAngle = 360), Line(origin = {-6, 0}, points = {{-78, 0}, {78, 0}, {78, 0}}), Text(origin = {0, 6}, fillPattern = FillPattern.Solid, extent = {{-60, 30}, {32, -2}}, textString = "i_DC*V_DC"), Text(origin = {6, -26}, fillPattern = FillPattern.Solid, extent = {{-60, 30}, {24, -2}}, textString = "v_d_conv")}, coordinateSystem(initialScale = 0.1)),
      uses(Modelica(version = "3.2.2")));
  end ff_voltage_controller;

  model DCside
    Real P;
    Real V_DC;
    Real P_pu;
    Real V_DC_pu;
  protected
    parameter Real load_current = I_DC_base * direction_V;
    extends SystemParameters;
    parameter Real start_time = P_step_time;
    constant Real pi = 2 * Modelica.Math.asin(1.0);
    constant Real w = 2 * pi * 50;
    parameter Real step = load_current * 0.1;
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {-76, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 50}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {-76, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -52}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent1 annotation(
      Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Sources.Step step1(height = step * direction_V, offset = load_current * direction_V, startTime = 2) annotation(
      Placement(visible = true, transformation(origin = {74, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
    connect(signalCurrent1.i, step1.y) annotation(
      Line(points = {{8, 0}, {63, 0}}, color = {0, 0, 127}));
    connect(V_DC_2, signalCurrent1.p) annotation(
      Line(points = {{-76, 62}, {0, 62}, {0, 10}, {0, 10}}, color = {0, 0, 255}));
    connect(m_V_DC_2, signalCurrent1.n) annotation(
      Line(points = {{-76, -60}, {0, -60}, {0, -10}, {0, -10}}, color = {0, 0, 255}));
    P_pu = P / S_base;
    V_DC_pu = V_DC / V_DC_base;
    P = V_DC_2.i * V_DC;
    V_DC = V_DC_2.v - m_V_DC_2.v;
    annotation(
      Diagram(coordinateSystem(initialScale = 0.1)),
      Icon(graphics = {Line(origin = {0, 1}, points = {{-4, 99}, {-4, -99}, {4, -99}, {4, 99}, {-4, 99}, {-4, 99}}, color = {255, 0, 0}, thickness = 3.5)}));
  end DCside;

  model Grid
    Real P_pu;
    Real I_a_pu;
  protected
    extends SystemParameters;
    parameter Real v_amp = V_base;
    constant Real pi = 2 * Modelica.Math.asin(1.0);
    constant Real w = 2 * pi * 50;
    parameter Real phi1 = pi / 2;
    parameter Real phi2 = (-2 * pi / 3) + pi / 2;
    parameter Real phi3 = (-4 * pi / 3) + pi / 2;
    Modelica.Electrical.Analog.Sources.SineVoltage PhaseA(V = v_amp, freqHz = 50, phase(displayUnit = "rad") = phi1) annotation(
      Placement(visible = true, transformation(origin = {40, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SineVoltage PhaseB(V = v_amp, freqHz = 50, phase(displayUnit = "rad") = phi2) annotation(
      Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SineVoltage PhaseC(V = v_amp, freqHz = 50, phase(displayUnit = "rad") = phi3) annotation(
      Placement(visible = true, transformation(origin = {40, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Interfaces.Pin A annotation(
      Placement(visible = true, transformation(origin = {0, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-2, 64}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin B annotation(
      Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-2, 2}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin C annotation(
      Placement(visible = true, transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-2, -58}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
  equation
    connect(B, PhaseB.p) annotation(
      Line(points = {{0, 0}, {30, 0}, {30, 0}, {30, 0}}, color = {0, 0, 255}));
    connect(PhaseB.n, ground1.p) annotation(
      Line(points = {{50, 0}, {70, 0}, {70, 0}, {70, 0}}, color = {0, 0, 255}));
    connect(PhaseA.n, ground1.p) annotation(
      Line(points = {{50, 60}, {70, 60}, {70, 0}, {70, 0}}, color = {0, 0, 255}));
    connect(A, PhaseA.p) annotation(
      Line(points = {{0, 62}, {30, 62}, {30, 60}, {30, 60}}, color = {0, 0, 255}));
    I_a_pu = PhaseA.i / I_base;
    P_pu = (PhaseA.v * PhaseA.i + PhaseB.v * PhaseB.i + PhaseC.v * PhaseC.i) / S_base;
    connect(C, PhaseC.p) annotation(
      Line(points = {{0, -60}, {30, -60}, {30, -60}, {30, -60}}, color = {0, 0, 255}));
    connect(PhaseC.n, ground1.p) annotation(
      Line(points = {{50, -60}, {70, -60}, {70, 0}, {70, 0}}, color = {0, 0, 255}));
    annotation(
      Diagram(coordinateSystem(initialScale = 0.1)),
      Icon(graphics = {Line(origin = {0, 1}, points = {{-4, 99}, {-4, -99}, {4, -99}, {4, 99}, {-4, 99}, {-4, 99}}, color = {255, 0, 0}, thickness = 3.5)}));
  end Grid;

  model MMC_zero_sequence_V
    MMC.CurrentController currentController1 annotation(
      Placement(visible = true, transformation(origin = {-139, 91}, extent = {{-31, 31}, {31, -31}}, rotation = 0)));
    MMC.Energy_equation energy_equation1 annotation(
      Placement(visible = true, transformation(origin = {-62, 206}, extent = {{-48, -48}, {48, 48}}, rotation = 0)));
    MMC.Energy_Controller energy_Controller1(u_circ_z_pu(fixed = true, start = 1)) annotation(
      Placement(visible = true, transformation(origin = {167, -15}, extent = {{-37, -37}, {37, 37}}, rotation = 0)));
    MMC.VoltageController voltageController1 annotation(
      Placement(visible = true, transformation(origin = {-55, 273}, extent = {{-33, 33}, {33, -33}}, rotation = 180)));
    Modelica.Blocks.Sources.Step w_sum_z_ref(height = 0.1, offset = 1, startTime = 3) annotation(
      Placement(visible = true, transformation(origin = {160, 30}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
  protected
    extends SystemParameters;
    Modelica.Electrical.Analog.Interfaces.Pin a annotation(
      Placement(visible = true, transformation(origin = {-400, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-402, 326}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin b annotation(
      Placement(visible = true, transformation(origin = {-400, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-398, 108}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin c annotation(
      Placement(visible = true, transformation(origin = {-400, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-392, -136}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R_v) annotation(
      Placement(visible = true, transformation(origin = {-230, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor2(R = R_v) annotation(
      Placement(visible = true, transformation(origin = {-230, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor3(R = R_v) annotation(
      Placement(visible = true, transformation(origin = {-230, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1 annotation(
      Placement(visible = true, transformation(origin = {-280, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor2 annotation(
      Placement(visible = true, transformation(origin = {-300, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor3 annotation(
      Placement(visible = true, transformation(origin = {-320, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {-340, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {-360, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {-380, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L_v, i(fixed = false, start = I_base)) annotation(
      Placement(visible = true, transformation(origin = {-192, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor2(L = L_v, i(fixed = false, start = -0.7 * I_base)) annotation(
      Placement(visible = true, transformation(origin = {-192, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor3(L = L_v, i(fixed = false, start = -0.7 * I_base)) annotation(
      Placement(visible = true, transformation(origin = {-192, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.DQ_w VoltageTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w CurrentTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 94}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w_inv dQ_w_inv1 annotation(
      Placement(visible = true, transformation(origin = {-70, 92}, extent = {{-20, 20}, {20, -20}}, rotation = 0)));
    Modelica.Blocks.Sources.Step i_q_pu_ref(height = 0, offset = 0) annotation(
      Placement(visible = true, transformation(origin = {-180, 116}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 108}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 92}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain7(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 76}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_a annotation(
      Placement(visible = true, transformation(origin = {-22, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_b annotation(
      Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_c annotation(
      Placement(visible = true, transformation(origin = {18, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {50, 1}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain8(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 86}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain9(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 62}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 70}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 78}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 94}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 102}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage Two_v_circ annotation(
      Placement(visible = true, transformation(origin = {278, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain10(k = 2 * V_base) annotation(
      Placement(visible = true, transformation(origin = {248, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = 2 * C_DC, v(fixed = true, start = V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {350, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor2(C = 2 * C_DC, v(fixed = true, start = -V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {350, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Basic.Inductor inductor4(L = 2 / 3 * L_a) annotation(
      Placement(visible = true, transformation(origin = {292, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor4(R = 2 / 3 * R_a) annotation(
      Placement(visible = true, transformation(origin = {318, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
      Placement(visible = true, transformation(origin = {366, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {408, 190}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {400, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor1 annotation(
      Placement(visible = true, transformation(origin = {320, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain11(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {234, 41}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Math.Gain gain12(k = 1 / I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {8, 299}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor4 annotation(
      Placement(visible = true, transformation(origin = {370, 60}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step V_DC_ref(height = 0.1, offset = 1, startTime = 4) annotation(
      Placement(visible = true, transformation(origin = {10, 246}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor5 annotation(
      Placement(visible = true, transformation(origin = {278, 28}, extent = {{10, 10}, {-10, -10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain13(k = 1 / 3) annotation(
      Placement(visible = true, transformation(origin = {-137, 191}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain14(k = 1 / (3 * I_base)) annotation(
      Placement(visible = true, transformation(origin = {100, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain15(k = I_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 212}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain16(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 220}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain17(k = 1 / w_base) annotation(
      Placement(visible = true, transformation(origin = {12, 206}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain18(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-137, 202}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain19(k = -1) annotation(
      Placement(visible = true, transformation(origin = {-182, 106}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
  equation
    connect(currentController1.i_d_ref, gain19.y) annotation(
      Line(points = {{-174, 106}, {-180, 106}}, color = {0, 0, 127}));
    connect(gain19.u, voltageController1.i_d_ref_pu) annotation(
      Line(points = {{-184, 106}, {-188, 106}, {-188, 272}, {-92, 272}, {-92, 274}}, color = {0, 0, 127}));
    connect(inductor3.n, converterVoltage_c.p) annotation(
      Line(points = {{-182, -20}, {6, -20}}, color = {0, 0, 255}));
    connect(gain5.y, converterVoltage_c.v) annotation(
      Line(points = {{-27.6, 108}, {16, 108}, {16, -13}}, color = {0, 0, 127}));
    connect(converterVoltage_c.n, ground1.p) annotation(
      Line(points = {{28, -20}, {40, -20}, {40, 0}}, color = {0, 0, 255}));
    connect(w_sum_z_ref.y, energy_Controller1.w_sum_z_ref) annotation(
      Line(points = {{156, 30}, {111, 30}, {111, 16}, {121, 16}, {121, 14}}, color = {0, 0, 127}));
    connect(energy_Controller1.i_circ_z_pu, gain14.y) annotation(
      Line(points = {{122, -44}, {110, -44}, {110, -46}, {112, -46}}, color = {0, 0, 127}));
    connect(voltageController1.V_DC_mes_pu, gain11.y) annotation(
      Line(points = {{-18, 274}, {198, 274}, {198, 42}, {223, 42}}, color = {0, 0, 127}));
    connect(gain11.u, voltageSensor1.v) annotation(
      Line(points = {{246, 42}, {312, 42}, {312, 0}}, color = {0, 0, 127}));
    connect(gain11.y, energy_Controller1.v_DC_pu) annotation(
      Line(points = {{223, 42}, {106, 42}, {106, -4}, {122, -4}}, color = {0, 0, 127}));
    connect(gain14.u, currentSensor5.i) annotation(
      Line(points = {{88, -46}, {78, -46}, {78, -78}, {268, -78}, {268, 28}, {268, 28}}, color = {0, 0, 127}));
    connect(gain18.u, energy_Controller1.u_circ_z_pu) annotation(
      Line(points = {{-142, 202}, {-162, 202}, {-162, 166}, {70, 166}, {70, -64}, {224, -64}, {224, 0}, {212, 0}, {212, 0}}, color = {0, 0, 127}));
    connect(gain17.y, energy_Controller1.w_sum_z_pu) annotation(
      Line(points = {{16, 206}, {96, 206}, {96, -22}, {122, -22}, {122, -22}}, color = {0, 0, 127}));
    connect(energy_Controller1.u_circ_z_pu, gain10.u) annotation(
      Line(points = {{211.4, -0.2}, {226.4, -0.2}, {226.4, 0}, {238, 0}}, color = {0, 0, 127}));
    connect(gain10.y, Two_v_circ.v) annotation(
      Line(points = {{261, 0}, {269, 0}}, color = {0, 0, 127}));
    connect(gain13.u, currentSensor5.i) annotation(
      Line(points = {{-142, 191}, {-158, 191}, {-158, 174}, {78, 174}, {78, -78}, {268, -78}, {268, 28}}, color = {0, 0, 127}));
    connect(inductor4.p, currentSensor5.p) annotation(
      Line(points = {{282, 60}, {278, 60}, {278, 36}, {278, 36}}, color = {0, 0, 255}));
    connect(currentSensor5.n, Two_v_circ.p) annotation(
      Line(points = {{278, 16}, {278, 16}, {278, 10}, {278, 10}}, color = {0, 0, 255}));
    connect(resistor4.p, inductor4.n) annotation(
      Line(points = {{310, 60}, {302, 60}}, color = {0, 0, 255}));
    connect(Two_v_circ.n, capacitor2.p) annotation(
      Line(points = {{278, -10}, {278, -52}, {352, -52}, {352, -30}}, color = {0, 0, 255}));
    connect(resistor4.n, capacitor1.p) annotation(
      Line(points = {{330, 60}, {350, 60}, {350, 30}}, color = {0, 0, 255}));
    connect(resistor4.n, currentSensor4.n) annotation(
      Line(points = {{330, 60}, {360, 60}}, color = {0, 0, 255}));
    connect(voltageSensor1.p, capacitor1.p) annotation(
      Line(points = {{320, 10}, {320, 10}, {320, 30}, {350, 30}, {350, 30}}, color = {0, 0, 255}));
    connect(voltageSensor1.n, capacitor2.p) annotation(
      Line(points = {{322, -10}, {322, -10}, {322, -30}, {352, -30}, {352, -30}}, color = {0, 0, 255}));
    connect(capacitor1.n, capacitor2.n) annotation(
      Line(points = {{352, 10}, {352, -10}}, color = {0, 0, 255}));
    connect(capacitor2.p, m_V_DC_2) annotation(
      Line(points = {{352, -30}, {352, -52}, {400, -52}}, color = {0, 0, 255}));
    connect(ground2.p, capacitor1.n) annotation(
      Line(points = {{354, 0}, {350, 0}, {350, 10}}, color = {0, 0, 255}));
    connect(gain12.u, currentSensor4.i) annotation(
      Line(points = {{20, 300}, {368, 300}, {368, 70}, {370, 70}}, color = {0, 0, 127}));
    connect(currentSensor4.p, V_DC_2) annotation(
      Line(points = {{380, 60}, {402, 60}, {402, 60}, {400, 60}}, color = {0, 0, 255}));
    connect(currentController1.v_d_conv_pu, gain16.u) annotation(
      Line(points = {{-104, 82}, {-100, 82}, {-100, 158}, {-168, 158}, {-168, 220}, {-142, 220}, {-142, 220}}, color = {0, 0, 127}));
    connect(gain16.y, energy_equation1.v_conv_d) annotation(
      Line(points = {{-134, 220}, {-118, 220}, {-118, 220}, {-114, 220}}, color = {0, 0, 127}));
    connect(CurrentTransform.d, gain15.u) annotation(
      Line(points = {{-218, 90}, {-190, 90}, {-190, 212}, {-142, 212}, {-142, 212}}, color = {0, 0, 127}));
    connect(gain15.y, energy_equation1.i_grid_d) annotation(
      Line(points = {{-134, 212}, {-116, 212}, {-116, 210}, {-114, 210}}, color = {0, 0, 127}));
    connect(gain13.y, energy_equation1.i_circ_z) annotation(
      Line(points = {{-130.5, 191}, {-120.5, 191}, {-120.5, 192}, {-114, 192}}, color = {0, 0, 127}));
    connect(energy_equation1.v_circ_z, gain18.y) annotation(
      Line(points = {{-114, 202}, {-132, 202}, {-132, 202}, {-132, 202}}, color = {0, 0, 127}));
    connect(energy_equation1.w_sum_z, gain17.u) annotation(
      Line(points = {{-10, 206}, {8, 206}, {8, 206}, {8, 206}}, color = {0, 0, 127}));
    connect(voltageController1.V_DC_ref, V_DC_ref.y) annotation(
      Line(points = {{-18, 246}, {1, 246}}, color = {0, 0, 127}));
    connect(voltageController1.I_DC_pu, gain12.y) annotation(
      Line(points = {{-24, 300}, {-14, 300}, {-14, 298}, {-3, 298}}, color = {0, 0, 127}));
    connect(voltageController1.v_grid_d, VoltageTransform.d) annotation(
      Line(points = {{-72, 312}, {-74, 312}, {-74, 322}, {-200, 322}, {-200, 66}, {-218, 66}, {-218, 66}}, color = {0, 0, 127}));
    connect(converterVoltage_b.n, ground1.p) annotation(
      Line(points = {{8, 0}, {40, 0}}, color = {0, 0, 255}));
    connect(converterVoltage_a.n, ground1.p) annotation(
      Line(points = {{-12, 20}, {40, 20}, {40, 0}}, color = {0, 0, 255}));
    connect(gain6.y, converterVoltage_b.v) annotation(
      Line(points = {{-27.6, 92}, {-4, 92}, {-4, 7}}, color = {0, 0, 127}));
    connect(inductor2.n, converterVoltage_b.p) annotation(
      Line(points = {{-182, 0}, {-14, 0}}, color = {0, 0, 255}));
    connect(gain9.u, potentialSensor1.phi) annotation(
      Line(points = {{-266.8, 62}, {-277.8, 62}, {-277.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.a, gain9.y) annotation(
      Line(points = {{-240, 62}, {-256, 62}}, color = {0, 0, 127}));
    connect(gain1.u, potentialSensor2.phi) annotation(
      Line(points = {{-266.8, 70}, {-297.8, 70}, {-297.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.b, gain1.y) annotation(
      Line(points = {{-242, 70}, {-258, 70}}, color = {0, 0, 127}));
    connect(gain2.u, potentialSensor3.phi) annotation(
      Line(points = {{-266.8, 78}, {-317.8, 78}, {-317.8, 51}}, color = {0, 0, 127}));
    connect(gain2.y, VoltageTransform.c) annotation(
      Line(points = {{-257.6, 78}, {-241.6, 78}}, color = {0, 0, 127}));
    connect(gain8.u, currentSensor1.i) annotation(
      Line(points = {{-266.8, 86}, {-337.8, 86}, {-337.8, 30}}, color = {0, 0, 127}));
    connect(CurrentTransform.a, gain8.y) annotation(
      Line(points = {{-240, 86}, {-256, 86}}, color = {0, 0, 127}));
    connect(gain3.u, currentSensor2.i) annotation(
      Line(points = {{-266.8, 94}, {-357.8, 94}, {-357.8, 10}}, color = {0, 0, 127}));
    connect(CurrentTransform.b, gain3.y) annotation(
      Line(points = {{-240, 94}, {-256, 94}}, color = {0, 0, 127}));
    connect(gain4.u, currentSensor3.i) annotation(
      Line(points = {{-266.8, 102}, {-377.8, 102}, {-377.8, -10}}, color = {0, 0, 127}));
    connect(CurrentTransform.c, gain4.y) annotation(
      Line(points = {{-240, 102}, {-256, 102}}, color = {0, 0, 127}));
    connect(gain7.y, converterVoltage_a.v) annotation(
      Line(points = {{-27.6, 76}, {-24, 76}, {-24, 27}}, color = {0, 0, 127}));
    connect(inductor1.n, converterVoltage_a.p) annotation(
      Line(points = {{-182, 20}, {-34, 20}}, color = {0, 0, 255}));
    connect(gain7.u, dQ_w_inv1.a) annotation(
      Line(points = {{-35, 76}, {-46, 76}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.b, gain6.u) annotation(
      Line(points = {{-46, 92}, {-35, 92}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.c, gain5.u) annotation(
      Line(points = {{-46, 108}, {-35, 108}}, color = {0, 0, 127}));
    connect(currentController1.v_q_conv_pu, dQ_w_inv1.q) annotation(
      Line(points = {{-105.21, 100.61}, {-81.21, 100.61}, {-81.21, 102}, {-92, 102}}, color = {0, 0, 127}));
    connect(currentController1.v_d_conv_pu, dQ_w_inv1.d) annotation(
      Line(points = {{-104.9, 81.7}, {-91.9, 81.7}, {-91.9, 82}, {-92, 82}}, color = {0, 0, 127}));
    connect(currentController1.i_q_ref, i_q_pu_ref.y) annotation(
      Line(points = {{-173.1, 115.8}, {-177.1, 115.8}}, color = {0, 0, 127}));
    connect(CurrentTransform.d, currentController1.i_d_mes_pu) annotation(
      Line(points = {{-216, 90}, {-172, 90}, {-172, 84}}, color = {0, 0, 127}));
    connect(CurrentTransform.q, currentController1.i_q_mes_pu) annotation(
      Line(points = {{-216, 100}, {-174, 100}, {-174, 98}, {-172, 98}}, color = {0, 0, 127}));
    connect(VoltageTransform.q, currentController1.v_q_grid_pu) annotation(
      Line(points = {{-216, 76}, {-172, 76}, {-172, 74}}, color = {0, 0, 127}));
    connect(VoltageTransform.d, currentController1.v_d_grid_pu) annotation(
      Line(points = {{-216, 66}, {-174, 66}, {-174, 64}, {-172, 64}}, color = {0, 0, 127}));
    connect(potentialSensor1.p, a) annotation(
      Line(points = {{-280, 30}, {-280, 20}, {-398, 20}}, color = {0, 0, 255}));
    connect(a, currentSensor1.n) annotation(
      Line(points = {{-398, 20}, {-349, 20}}, color = {0, 0, 255}));
    connect(resistor1.n, inductor1.p) annotation(
      Line(points = {{-218, 20}, {-198, 20}, {-198, 20}, {-198, 20}}, color = {0, 0, 255}));
    connect(currentSensor1.p, resistor1.p) annotation(
      Line(points = {{-330, 20}, {-241, 20}}, color = {0, 0, 255}));
    connect(potentialSensor2.p, b) annotation(
      Line(points = {{-298, 30}, {-298, 0}, {-396, 0}}, color = {0, 0, 255}));
    connect(potentialSensor3.p, c) annotation(
      Line(points = {{-318, 30}, {-318, -20}, {-396, -20}}, color = {0, 0, 255}));
    connect(b, currentSensor2.n) annotation(
      Line(points = {{-398, 0}, {-371, 0}}, color = {0, 0, 255}));
    connect(c, currentSensor3.n) annotation(
      Line(points = {{-398, -20}, {-391, -20}}, color = {0, 0, 255}));
    connect(resistor2.n, inductor2.p) annotation(
      Line(points = {{-218, 0}, {-198, 0}, {-198, 0}, {-198, 0}}, color = {0, 0, 255}));
    connect(currentSensor2.p, resistor2.p) annotation(
      Line(points = {{-350, 0}, {-239, 0}}, color = {0, 0, 255}));
    connect(resistor3.n, inductor3.p) annotation(
      Line(points = {{-218, -20}, {-198, -20}, {-198, -20}, {-198, -20}}, color = {0, 0, 255}));
    connect(currentSensor3.p, resistor3.p) annotation(
      Line(points = {{-370, -20}, {-239, -20}}, color = {0, 0, 255}));
    annotation(
      Diagram(coordinateSystem(extent = {{-400, -200}, {400, 400}})),
      Icon(coordinateSystem(extent = {{-400, -200}, {400, 400}}, initialScale = 0.1), graphics = {Rectangle(origin = {0, -1}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-400, 401}, {400, -201}}), Text(origin = {-44, 132}, fillPattern = FillPattern.Solid, extent = {{-236, 114}, {290, -166}}, textString = "MMC_V")}),
      __OpenModelica_commandLineOptions = "");
  end MMC_zero_sequence_V;

  model MMC_zero_sequence_P
    Real P_AC_pu;
    Real P_refrence;
    Real V_local_pu;
    Real Eng;
    Real Eng_tot(start = 0);
    MMC.CurrentController currentController1 annotation(
      Placement(visible = true, transformation(origin = {-139, 91}, extent = {{-31, 31}, {31, -31}}, rotation = 0)));
    MMC.Energy_equation energy_equation1 annotation(
      Placement(visible = true, transformation(origin = {-62, 206}, extent = {{-48, -48}, {48, 48}}, rotation = 0)));
    MMC.Energy_Controller energy_Controller1(u_circ_z_pu(fixed = true, start = 1)) annotation(
      Placement(visible = true, transformation(origin = {167, -15}, extent = {{-37, -37}, {37, 37}}, rotation = 0)));
    MMC.PowerController powerController1 annotation(
      Placement(visible = true, transformation(origin = {-74, 284}, extent = {{48, -48}, {-48, 48}}, rotation = 0)));
    Modelica.Blocks.Sources.Exponentials exponentials1(fallTimeConst = 10, offset = 0.855, outMax = -0.033, riseTime = 1, riseTimeConst = 0.01, startTime = 1.1) annotation(
      Placement(visible = true, transformation(origin = {162, 32}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
  protected
    extends SystemParameters;
    Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
      Placement(visible = true, transformation(origin = {410, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = C_DC * 2, v(fixed = true, start = V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {386, 26}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor2(C = C_DC * 2, v(fixed = true, start = -V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {386, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Add add1 annotation(
      Placement(visible = true, transformation(origin = {14, 244}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin a annotation(
      Placement(visible = true, transformation(origin = {-400, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-402, 326}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin b annotation(
      Placement(visible = true, transformation(origin = {-400, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-398, 108}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin c annotation(
      Placement(visible = true, transformation(origin = {-400, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-392, -136}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R_v) annotation(
      Placement(visible = true, transformation(origin = {-230, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor2(R = R_v) annotation(
      Placement(visible = true, transformation(origin = {-230, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor3(R = R_v) annotation(
      Placement(visible = true, transformation(origin = {-230, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1 annotation(
      Placement(visible = true, transformation(origin = {-280, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor2 annotation(
      Placement(visible = true, transformation(origin = {-300, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor3 annotation(
      Placement(visible = true, transformation(origin = {-320, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {-340, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {-360, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {-380, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L_v) annotation(
      Placement(visible = true, transformation(origin = {-192, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor2(L = L_v) annotation(
      Placement(visible = true, transformation(origin = {-192, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor3(L = L_v) annotation(
      Placement(visible = true, transformation(origin = {-192, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.DQ_w VoltageTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w CurrentTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 94}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w_inv dQ_w_inv1 annotation(
      Placement(visible = true, transformation(origin = {-70, 92}, extent = {{-20, 20}, {20, -20}}, rotation = 0)));
    Modelica.Blocks.Sources.Step i_q_pu_ref(height = 0, offset = 0) annotation(
      Placement(visible = true, transformation(origin = {-180, 116}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 108}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 92}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain7(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 76}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_a annotation(
      Placement(visible = true, transformation(origin = {-22, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_b annotation(
      Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_c annotation(
      Placement(visible = true, transformation(origin = {18, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {50, 1}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain8(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 86}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain9(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 62}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 70}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 78}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 94}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 102}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage Two_v_circ annotation(
      Placement(visible = true, transformation(origin = {278, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain10(k = 2 * V_base) annotation(
      Placement(visible = true, transformation(origin = {248, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor4(L = 2 / 3 * L_a) annotation(
      Placement(visible = true, transformation(origin = {292, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor4(R = 2 / 3 * R_a) annotation(
      Placement(visible = true, transformation(origin = {318, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {408, 190}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {400, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor1 annotation(
      Placement(visible = true, transformation(origin = {332, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain11(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {234, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Math.Gain gain12(k = 1 / I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {126, 323}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor4 annotation(
      Placement(visible = true, transformation(origin = {370, 60}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step P_ref(height = 0.1 * direction_V, offset = 0.7 * direction_V, startTime = 1) annotation(
      Placement(visible = true, transformation(origin = {49, 229}, extent = {{9, -9}, {-9, 9}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor5 annotation(
      Placement(visible = true, transformation(origin = {278, 28}, extent = {{10, 10}, {-10, -10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain13(k = 1 / 3) annotation(
      Placement(visible = true, transformation(origin = {-137, 191}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain14(k = 1 / (3 * I_base)) annotation(
      Placement(visible = true, transformation(origin = {100, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain15(k = I_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 212}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain16(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 220}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain17(k = 1 / w_base) annotation(
      Placement(visible = true, transformation(origin = {12, 206}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain18(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-137, 202}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain19(k = 1) annotation(
      Placement(visible = true, transformation(origin = {-182, 106}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step1(height = 0.1 * direction_V, offset = 0, startTime = 1.1) annotation(
      Placement(visible = true, transformation(origin = {53, 253}, extent = {{9, -9}, {-9, 9}}, rotation = 0)));
  equation
    connect(exponentials1.y, energy_Controller1.w_sum_z_ref) annotation(
      Line(points = {{155, 32}, {112, 32}, {112, 12}, {120, 12}, {120, 14}, {122, 14}}, color = {0, 0, 127}));
    V_local_pu = (V_DC_2.v - m_V_DC_2.v) / V_DC_base;
    connect(gain14.y, energy_Controller1.i_circ_z_pu) annotation(
      Line(points = {{112, -46}, {118, -46}, {118, -44}, {122, -44}}, color = {0, 0, 127}));
    P_AC_pu = -(converterVoltage_a.v * converterVoltage_a.i + converterVoltage_b.v * converterVoltage_b.i + converterVoltage_c.v * converterVoltage_c.i) / S_base;
    if time > 1.1 then
      P_refrence = 0.8977;
    elseif time > 1 then
      P_refrence = 0.79723;
    else
      P_refrence = 0.697676;
    end if;
    if time > 1 then
      Eng = P_refrence - P_AC_pu;
    else
      Eng = 0;
    end if;
    der(Eng_tot) = Eng;
    connect(add1.u2, P_ref.y) annotation(
      Line(points = {{26, 238}, {39, 238}, {39, 229}}, color = {0, 0, 127}));
    connect(add1.y, powerController1.P_ref) annotation(
      Line(points = {{2, 244}, {-16, 244}, {-16, 246}, {-20, 246}}, color = {0, 0, 127}));
    connect(step1.y, add1.u1) annotation(
      Line(points = {{44, 254}, {28, 254}, {28, 250}, {26, 250}}, color = {0, 0, 127}));
    connect(resistor4.n, capacitor1.p) annotation(
      Line(points = {{328, 60}, {350, 60}, {350, 38}, {386, 38}, {386, 36}}, color = {0, 0, 255}));
    connect(Two_v_circ.n, m_V_DC_2) annotation(
      Line(points = {{278, -10}, {278, -76}, {400, -76}}, color = {0, 0, 255}));
    connect(capacitor2.p, m_V_DC_2) annotation(
      Line(points = {{388, -40}, {388, -76}, {400, -76}}, color = {0, 0, 255}));
    connect(capacitor1.n, ground2.p) annotation(
      Line(points = {{386, 16}, {386, 0}, {400, 0}}, color = {0, 0, 255}));
    connect(capacitor1.n, capacitor2.n) annotation(
      Line(points = {{386, 16}, {386, 16}, {386, -20}, {386, -20}}, color = {0, 0, 255}));
    connect(Two_v_circ.n, voltageSensor1.n) annotation(
      Line(points = {{278, -10}, {278, -76}, {334, -76}, {334, -10}}, color = {0, 0, 255}));
    connect(voltageSensor1.p, resistor4.n) annotation(
      Line(points = {{334, 10}, {334, 60}, {328, 60}}, color = {0, 0, 255}));
    connect(gain11.u, voltageSensor1.v) annotation(
      Line(points = {{246, 42}, {324, 42}, {324, 0}}, color = {0, 0, 127}));
    connect(currentController1.i_d_ref, gain19.y) annotation(
      Line(points = {{-174, 106}, {-180, 106}}, color = {0, 0, 127}));
    connect(powerController1.i_d_ref_pu, gain19.u) annotation(
      Line(points = {{-126, 284}, {-186, 284}, {-186, 106}, {-184, 106}}, color = {0, 0, 127}));
    connect(powerController1.V_DC_mes_pu, gain11.y) annotation(
      Line(points = {{-22, 284}, {210, 284}, {210, 44}, {224, 44}, {224, 44}}, color = {0, 0, 127}));
    connect(powerController1.v_grid_d, VoltageTransform.d) annotation(
      Line(points = {{-98, 342}, {-98, 342}, {-98, 362}, {-200, 362}, {-200, 66}, {-218, 66}, {-218, 66}}, color = {0, 0, 127}));
    connect(powerController1.I_DC_pu, gain12.y) annotation(
      Line(points = {{-22, 322}, {-4, 322}, {-4, 323}, {113, 323}}, color = {0, 0, 127}));
    connect(gain12.u, currentSensor4.i) annotation(
      Line(points = {{136, 323}, {368, 323}, {368, 70}, {370, 70}}, color = {0, 0, 127}));
    connect(converterVoltage_c.n, ground1.p) annotation(
      Line(points = {{28, -20}, {40, -20}, {40, 0}}, color = {0, 0, 255}));
    connect(converterVoltage_b.n, ground1.p) annotation(
      Line(points = {{8, 0}, {40, 0}}, color = {0, 0, 255}));
    connect(converterVoltage_a.n, ground1.p) annotation(
      Line(points = {{-12, 20}, {40, 20}, {40, 0}}, color = {0, 0, 255}));
    connect(currentController1.v_d_conv_pu, gain16.u) annotation(
      Line(points = {{-104, 82}, {-100, 82}, {-100, 158}, {-168, 158}, {-168, 220}, {-142, 220}, {-142, 220}}, color = {0, 0, 127}));
    connect(gain6.y, converterVoltage_b.v) annotation(
      Line(points = {{-27.6, 92}, {-4, 92}, {-4, 7}}, color = {0, 0, 127}));
    connect(inductor2.n, converterVoltage_b.p) annotation(
      Line(points = {{-182, 0}, {-14, 0}}, color = {0, 0, 255}));
    connect(gain18.u, energy_Controller1.u_circ_z_pu) annotation(
      Line(points = {{-142, 202}, {-162, 202}, {-162, 166}, {70, 166}, {70, -64}, {224, -64}, {224, 0}, {212, 0}, {212, 0}}, color = {0, 0, 127}));
    connect(gain13.u, currentSensor5.i) annotation(
      Line(points = {{-142, 191}, {-158, 191}, {-158, 174}, {78, 174}, {78, -78}, {268, -78}, {268, 28}}, color = {0, 0, 127}));
    connect(gain13.y, energy_equation1.i_circ_z) annotation(
      Line(points = {{-130.5, 191}, {-120.5, 191}, {-120.5, 192}, {-114, 192}}, color = {0, 0, 127}));
    connect(energy_equation1.v_circ_z, gain18.y) annotation(
      Line(points = {{-114, 202}, {-132, 202}, {-132, 202}, {-132, 202}}, color = {0, 0, 127}));
    connect(gain17.y, energy_Controller1.w_sum_z_pu) annotation(
      Line(points = {{16, 206}, {96, 206}, {96, -22}, {122, -22}, {122, -22}}, color = {0, 0, 127}));
    connect(energy_equation1.w_sum_z, gain17.u) annotation(
      Line(points = {{-10, 206}, {8, 206}, {8, 206}, {8, 206}}, color = {0, 0, 127}));
    connect(CurrentTransform.d, gain15.u) annotation(
      Line(points = {{-218, 90}, {-190, 90}, {-190, 212}, {-142, 212}, {-142, 212}}, color = {0, 0, 127}));
    connect(gain15.y, energy_equation1.i_grid_d) annotation(
      Line(points = {{-134, 212}, {-116, 212}, {-116, 210}, {-114, 210}}, color = {0, 0, 127}));
    connect(gain16.y, energy_equation1.v_conv_d) annotation(
      Line(points = {{-134, 220}, {-118, 220}, {-118, 220}, {-114, 220}}, color = {0, 0, 127}));
    connect(gain14.u, currentSensor5.i) annotation(
      Line(points = {{88, -46}, {78, -46}, {78, -78}, {268, -78}, {268, 28}, {268, 28}}, color = {0, 0, 127}));
    connect(gain9.u, potentialSensor1.phi) annotation(
      Line(points = {{-266.8, 62}, {-277.8, 62}, {-277.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.a, gain9.y) annotation(
      Line(points = {{-240, 62}, {-256, 62}}, color = {0, 0, 127}));
    connect(gain1.u, potentialSensor2.phi) annotation(
      Line(points = {{-266.8, 70}, {-297.8, 70}, {-297.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.b, gain1.y) annotation(
      Line(points = {{-242, 70}, {-258, 70}}, color = {0, 0, 127}));
    connect(gain2.u, potentialSensor3.phi) annotation(
      Line(points = {{-266.8, 78}, {-317.8, 78}, {-317.8, 51}}, color = {0, 0, 127}));
    connect(gain2.y, VoltageTransform.c) annotation(
      Line(points = {{-257.6, 78}, {-241.6, 78}}, color = {0, 0, 127}));
    connect(gain8.u, currentSensor1.i) annotation(
      Line(points = {{-266.8, 86}, {-337.8, 86}, {-337.8, 30}}, color = {0, 0, 127}));
    connect(CurrentTransform.a, gain8.y) annotation(
      Line(points = {{-240, 86}, {-256, 86}}, color = {0, 0, 127}));
    connect(gain3.u, currentSensor2.i) annotation(
      Line(points = {{-266.8, 94}, {-357.8, 94}, {-357.8, 10}}, color = {0, 0, 127}));
    connect(CurrentTransform.b, gain3.y) annotation(
      Line(points = {{-240, 94}, {-256, 94}}, color = {0, 0, 127}));
    connect(gain4.u, currentSensor3.i) annotation(
      Line(points = {{-266.8, 102}, {-377.8, 102}, {-377.8, -10}}, color = {0, 0, 127}));
    connect(CurrentTransform.c, gain4.y) annotation(
      Line(points = {{-240, 102}, {-256, 102}}, color = {0, 0, 127}));
    connect(inductor4.p, currentSensor5.p) annotation(
      Line(points = {{282, 60}, {278, 60}, {278, 36}, {278, 36}}, color = {0, 0, 255}));
    connect(currentSensor5.n, Two_v_circ.p) annotation(
      Line(points = {{278, 16}, {278, 16}, {278, 10}, {278, 10}}, color = {0, 0, 255}));
    connect(energy_Controller1.u_circ_z_pu, gain10.u) annotation(
      Line(points = {{211.4, -0.2}, {226.4, -0.2}, {226.4, 0}, {238, 0}}, color = {0, 0, 127}));
    connect(gain10.y, Two_v_circ.v) annotation(
      Line(points = {{261, 0}, {269, 0}}, color = {0, 0, 127}));
    connect(gain11.y, energy_Controller1.v_DC_pu) annotation(
      Line(points = {{223, 42}, {106, 42}, {106, -4}, {122, -4}}, color = {0, 0, 127}));
    connect(resistor4.p, inductor4.n) annotation(
      Line(points = {{310, 60}, {302, 60}}, color = {0, 0, 255}));
    connect(resistor4.n, currentSensor4.n) annotation(
      Line(points = {{330, 60}, {360, 60}}, color = {0, 0, 255}));
    connect(currentSensor4.p, V_DC_2) annotation(
      Line(points = {{380, 60}, {402, 60}, {402, 60}, {400, 60}}, color = {0, 0, 255}));
    connect(gain7.y, converterVoltage_a.v) annotation(
      Line(points = {{-27.6, 76}, {-24, 76}, {-24, 27}}, color = {0, 0, 127}));
    connect(inductor1.n, converterVoltage_a.p) annotation(
      Line(points = {{-182, 20}, {-34, 20}}, color = {0, 0, 255}));
    connect(gain5.y, converterVoltage_c.v) annotation(
      Line(points = {{-27.6, 108}, {16, 108}, {16, -13}}, color = {0, 0, 127}));
    connect(inductor3.n, converterVoltage_c.p) annotation(
      Line(points = {{-182, -20}, {6, -20}}, color = {0, 0, 255}));
    connect(gain7.u, dQ_w_inv1.a) annotation(
      Line(points = {{-35, 76}, {-46, 76}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.b, gain6.u) annotation(
      Line(points = {{-46, 92}, {-35, 92}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.c, gain5.u) annotation(
      Line(points = {{-46, 108}, {-35, 108}}, color = {0, 0, 127}));
    connect(currentController1.v_q_conv_pu, dQ_w_inv1.q) annotation(
      Line(points = {{-105.21, 100.61}, {-81.21, 100.61}, {-81.21, 102}, {-92, 102}}, color = {0, 0, 127}));
    connect(currentController1.v_d_conv_pu, dQ_w_inv1.d) annotation(
      Line(points = {{-104.9, 81.7}, {-91.9, 81.7}, {-91.9, 82}, {-92, 82}}, color = {0, 0, 127}));
    connect(currentController1.i_q_ref, i_q_pu_ref.y) annotation(
      Line(points = {{-173.1, 115.8}, {-177.1, 115.8}}, color = {0, 0, 127}));
    connect(CurrentTransform.d, currentController1.i_d_mes_pu) annotation(
      Line(points = {{-216, 90}, {-172, 90}, {-172, 84}}, color = {0, 0, 127}));
    connect(CurrentTransform.q, currentController1.i_q_mes_pu) annotation(
      Line(points = {{-216, 100}, {-174, 100}, {-174, 98}, {-172, 98}}, color = {0, 0, 127}));
    connect(VoltageTransform.q, currentController1.v_q_grid_pu) annotation(
      Line(points = {{-216, 76}, {-172, 76}, {-172, 74}}, color = {0, 0, 127}));
    connect(VoltageTransform.d, currentController1.v_d_grid_pu) annotation(
      Line(points = {{-216, 66}, {-174, 66}, {-174, 64}, {-172, 64}}, color = {0, 0, 127}));
    connect(potentialSensor1.p, a) annotation(
      Line(points = {{-280, 30}, {-280, 20}, {-398, 20}}, color = {0, 0, 255}));
    connect(a, currentSensor1.n) annotation(
      Line(points = {{-398, 20}, {-349, 20}}, color = {0, 0, 255}));
    connect(resistor1.n, inductor1.p) annotation(
      Line(points = {{-218, 20}, {-198, 20}, {-198, 20}, {-198, 20}}, color = {0, 0, 255}));
    connect(currentSensor1.p, resistor1.p) annotation(
      Line(points = {{-330, 20}, {-241, 20}}, color = {0, 0, 255}));
    connect(potentialSensor2.p, b) annotation(
      Line(points = {{-298, 30}, {-298, 0}, {-396, 0}}, color = {0, 0, 255}));
    connect(potentialSensor3.p, c) annotation(
      Line(points = {{-318, 30}, {-318, -20}, {-396, -20}}, color = {0, 0, 255}));
    connect(b, currentSensor2.n) annotation(
      Line(points = {{-398, 0}, {-371, 0}}, color = {0, 0, 255}));
    connect(c, currentSensor3.n) annotation(
      Line(points = {{-398, -20}, {-391, -20}}, color = {0, 0, 255}));
    connect(resistor2.n, inductor2.p) annotation(
      Line(points = {{-218, 0}, {-198, 0}, {-198, 0}, {-198, 0}}, color = {0, 0, 255}));
    connect(currentSensor2.p, resistor2.p) annotation(
      Line(points = {{-350, 0}, {-239, 0}}, color = {0, 0, 255}));
    connect(resistor3.n, inductor3.p) annotation(
      Line(points = {{-218, -20}, {-198, -20}, {-198, -20}, {-198, -20}}, color = {0, 0, 255}));
    connect(currentSensor3.p, resistor3.p) annotation(
      Line(points = {{-370, -20}, {-239, -20}}, color = {0, 0, 255}));
    annotation(
      Diagram(coordinateSystem(extent = {{-400, -200}, {400, 400}})),
      Icon(coordinateSystem(extent = {{-400, -200}, {400, 400}}, initialScale = 0.1), graphics = {Rectangle(origin = {0, -1}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-400, 401}, {400, -201}}), Text(origin = {-44, 132}, fillPattern = FillPattern.Solid, extent = {{-236, 114}, {290, -166}}, textString = "MMC_P")}),
      __OpenModelica_commandLineOptions = "");
  end MMC_zero_sequence_P;

  model MMC_zero_sequence_test_single_cableless
    Real V_DC_upper_pu;
    Real V_DC_lower_pu;
    Grid grid1 annotation(
      Placement(visible = true, transformation(origin = {-96, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.MMC_zero_sequence_V mMC_zero_sequence_V1 annotation(
      Placement(visible = true, transformation(origin = {-44, 38}, extent = {{-40, -20}, {40, 40}}, rotation = 0)));
    MMC.MMC_zero_sequence_P mMC_zero_sequence_P1 annotation(
      Placement(visible = true, transformation(origin = {75.0591, 46.9409}, extent = {{25.0591, -12.5296}, {-25.0591, 25.0591}}, rotation = 0)));
    Grid grid3 annotation(
      Placement(visible = true, transformation(origin = {120, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    extends SystemParameters;
  equation
    V_DC_upper_pu = mMC_zero_sequence_V1.V_DC_2.v / V_DC_base;
    V_DC_lower_pu = mMC_zero_sequence_V1.m_V_DC_2.v / V_DC_base;
    connect(mMC_zero_sequence_V1.m_V_DC_2, mMC_zero_sequence_P1.m_V_DC_2) annotation(
      Line(points = {{-4, 32}, {50, 32}, {50, 44}, {50, 44}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V1.V_DC_2, mMC_zero_sequence_P1.V_DC_2) annotation(
      Line(points = {{-4, 56}, {50, 56}, {50, 56}, {50, 56}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_P1.c, grid3.C) annotation(
      Line(points = {{100, 34}, {120, 34}, {120, 44}, {120, 44}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_P1.b, grid3.B) annotation(
      Line(points = {{100, 50}, {120, 50}, {120, 50}, {120, 50}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_P1.a, grid3.A) annotation(
      Line(points = {{100, 64}, {120, 64}, {120, 56}, {120, 56}}, color = {0, 0, 255}));
    connect(grid1.C, mMC_zero_sequence_V1.c) annotation(
      Line(points = {{-96, 44}, {-96, 44}, {-96, 24}, {-84, 24}, {-84, 24}}, color = {0, 0, 255}));
    connect(grid1.B, mMC_zero_sequence_V1.b) annotation(
      Line(points = {{-96, 50}, {-84, 50}, {-84, 48}, {-84, 48}}, color = {0, 0, 255}));
    connect(grid1.A, mMC_zero_sequence_V1.a) annotation(
      Line(points = {{-96, 56}, {-96, 56}, {-96, 70}, {-84, 70}, {-84, 70}}, color = {0, 0, 255}));
  end MMC_zero_sequence_test_single_cableless;

  model MMC_zero_sequence_V_feed
    MMC.CurrentController currentController1 annotation(
      Placement(visible = true, transformation(origin = {-139, 91}, extent = {{-31, 31}, {31, -31}}, rotation = 0)));
    MMC.Energy_equation energy_equation1 annotation(
      Placement(visible = true, transformation(origin = {-62, 206}, extent = {{-48, -48}, {48, 48}}, rotation = 0)));
    MMC.Energy_Controller energy_Controller1(u_circ_z_pu(fixed = true, start = 1)) annotation(
      Placement(visible = true, transformation(origin = {152, -9}, extent = {{-22, -22}, {22, 22}}, rotation = 0)));
    MMC.VoltageController_feed voltageController_feed1 annotation(
      Placement(visible = true, transformation(origin = {-81, 287}, extent = {{51, -51}, {-51, 51}}, rotation = 0)));
  protected
    extends SystemParameters;
    Modelica.Electrical.Analog.Interfaces.Pin a annotation(
      Placement(visible = true, transformation(origin = {-400, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-402, 326}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin b annotation(
      Placement(visible = true, transformation(origin = {-400, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-398, 108}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin c annotation(
      Placement(visible = true, transformation(origin = {-400, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-392, -136}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R_f) annotation(
      Placement(visible = true, transformation(origin = {-230, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor2(R = R_f) annotation(
      Placement(visible = true, transformation(origin = {-230, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor3(R = R_f) annotation(
      Placement(visible = true, transformation(origin = {-230, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1 annotation(
      Placement(visible = true, transformation(origin = {-280, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor2 annotation(
      Placement(visible = true, transformation(origin = {-300, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor3 annotation(
      Placement(visible = true, transformation(origin = {-320, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {-340, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {-360, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {-380, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L_f) annotation(
      Placement(visible = true, transformation(origin = {-192, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor2(L = L_f) annotation(
      Placement(visible = true, transformation(origin = {-192, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor3(L = L_f) annotation(
      Placement(visible = true, transformation(origin = {-192, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.DQ_w VoltageTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w CurrentTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 94}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w_inv dQ_w_inv1 annotation(
      Placement(visible = true, transformation(origin = {-70, 92}, extent = {{-20, 20}, {20, -20}}, rotation = 0)));
    Modelica.Blocks.Sources.Step i_q_pu_ref(height = 0, offset = 0) annotation(
      Placement(visible = true, transformation(origin = {-180, 116}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 108}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 92}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain7(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 76}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_a annotation(
      Placement(visible = true, transformation(origin = {-22, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_b annotation(
      Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_c annotation(
      Placement(visible = true, transformation(origin = {18, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {50, 1}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain8(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 86}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain9(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 62}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 70}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 78}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 94}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 102}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Sources.Step w_sum_z_ref(height = 0.1, offset = 1, startTime = 3) annotation(
      Placement(visible = true, transformation(origin = {160, 30}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage Two_v_circ annotation(
      Placement(visible = true, transformation(origin = {278, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain10(k = 2 * V_base) annotation(
      Placement(visible = true, transformation(origin = {248, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = 2 * C_DC, v(fixed = true, start = V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {350, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor2(C = 2 * C_DC, v(fixed = true, start = -V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {350, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Basic.Inductor inductor4(L = 2 / 3 * L_a) annotation(
      Placement(visible = true, transformation(origin = {292, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor4(R = 2 / 3 * R_a) annotation(
      Placement(visible = true, transformation(origin = {318, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
      Placement(visible = true, transformation(origin = {366, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {408, 190}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {400, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor1 annotation(
      Placement(visible = true, transformation(origin = {320, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain11(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {234, 41}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Math.Gain gain12(k = 1 / I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {8, 325}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor4 annotation(
      Placement(visible = true, transformation(origin = {370, 60}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step V_DC_ref(height = 0.1, offset = 1, startTime = 4) annotation(
      Placement(visible = true, transformation(origin = {10, 246}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor5 annotation(
      Placement(visible = true, transformation(origin = {278, 28}, extent = {{10, 10}, {-10, -10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain13(k = 1 / 3) annotation(
      Placement(visible = true, transformation(origin = {-137, 191}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain14(k = 1 / (3 * I_base)) annotation(
      Placement(visible = true, transformation(origin = {100, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain15(k = I_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 212}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain16(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 220}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain17(k = 1 / w_base) annotation(
      Placement(visible = true, transformation(origin = {12, 206}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain18(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-137, 202}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain19(k = -1) annotation(
      Placement(visible = true, transformation(origin = {-182, 106}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
  equation
    connect(voltageController_feed1.v_grid_d, VoltageTransform.d) annotation(
      Line(points = {{-106, 348}, {-108, 348}, {-108, 356}, {-202, 356}, {-202, 66}, {-218, 66}, {-218, 66}}, color = {0, 0, 127}));
    connect(energy_Controller1.u_circ_z_pu, gain18.u) annotation(
      Line(points = {{178, 0}, {200, 0}, {200, -64}, {60, -64}, {60, 170}, {-162, 170}, {-162, 204}, {-142, 204}, {-142, 202}}, color = {0, 0, 127}));
    connect(energy_Controller1.u_circ_z_pu, gain10.u) annotation(
      Line(points = {{178, 0}, {234, 0}, {234, 0}, {236, 0}}, color = {0, 0, 127}));
    connect(gain17.y, energy_Controller1.w_sum_z_pu) annotation(
      Line(points = {{16, 206}, {96, 206}, {96, -14}, {126, -14}}, color = {0, 0, 127}));
    connect(gain11.y, energy_Controller1.v_DC_pu) annotation(
      Line(points = {{223, 42}, {106, 42}, {106, -3}, {126, -3}}, color = {0, 0, 127}));
    connect(w_sum_z_ref.y, energy_Controller1.w_sum_z_ref) annotation(
      Line(points = {{156, 30}, {111, 30}, {111, 8}, {126, 8}}, color = {0, 0, 127}));
    connect(gain14.y, energy_Controller1.i_circ_z_pu) annotation(
      Line(points = {{112, -46}, {120, -46}, {120, -28}, {126, -28}}, color = {0, 0, 127}));
    connect(voltageController_feed1.i_d_ref_pu, gain19.u) annotation(
      Line(points = {{-138, 288}, {-188, 288}, {-188, 106}, {-184, 106}, {-184, 106}}, color = {0, 0, 127}));
    connect(voltageController_feed1.V_DC_mes_pu, gain11.y) annotation(
      Line(points = {{-24, 288}, {200, 288}, {200, 42}, {224, 42}, {224, 42}}, color = {0, 0, 127}));
    connect(voltageController_feed1.w_sum_z_pu, gain17.y) annotation(
      Line(points = {{-24, 266}, {48, 266}, {48, 206}, {16, 206}, {16, 206}}, color = {0, 0, 127}));
    connect(voltageController_feed1.I_DC_pu, gain12.y) annotation(
      Line(points = {{-24, 328}, {-4, 328}, {-4, 326}, {-2, 326}}, color = {0, 0, 127}));
    connect(gain12.u, currentSensor4.i) annotation(
      Line(points = {{20, 325}, {368, 325}, {368, 70}, {370, 70}}, color = {0, 0, 127}));
    connect(voltageController_feed1.V_DC_ref, V_DC_ref.y) annotation(
      Line(points = {{-24, 246}, {-2, 246}, {-2, 246}, {0, 246}}, color = {0, 0, 127}));
    connect(gain11.u, voltageSensor1.v) annotation(
      Line(points = {{246, 42}, {312, 42}, {312, 0}}, color = {0, 0, 127}));
    connect(gain14.u, currentSensor5.i) annotation(
      Line(points = {{88, -46}, {78, -46}, {78, -78}, {268, -78}, {268, 28}, {268, 28}}, color = {0, 0, 127}));
    connect(gain10.y, Two_v_circ.v) annotation(
      Line(points = {{261, 0}, {269, 0}}, color = {0, 0, 127}));
    connect(gain13.u, currentSensor5.i) annotation(
      Line(points = {{-142, 191}, {-158, 191}, {-158, 174}, {78, 174}, {78, -78}, {268, -78}, {268, 28}}, color = {0, 0, 127}));
    connect(inductor4.p, currentSensor5.p) annotation(
      Line(points = {{282, 60}, {278, 60}, {278, 36}, {278, 36}}, color = {0, 0, 255}));
    connect(currentSensor5.n, Two_v_circ.p) annotation(
      Line(points = {{278, 16}, {278, 16}, {278, 10}, {278, 10}}, color = {0, 0, 255}));
    connect(resistor4.p, inductor4.n) annotation(
      Line(points = {{310, 60}, {302, 60}}, color = {0, 0, 255}));
    connect(Two_v_circ.n, capacitor2.p) annotation(
      Line(points = {{278, -10}, {278, -52}, {352, -52}, {352, -30}}, color = {0, 0, 255}));
    connect(resistor4.n, capacitor1.p) annotation(
      Line(points = {{330, 60}, {350, 60}, {350, 30}}, color = {0, 0, 255}));
    connect(resistor4.n, currentSensor4.n) annotation(
      Line(points = {{330, 60}, {360, 60}}, color = {0, 0, 255}));
    connect(voltageSensor1.p, capacitor1.p) annotation(
      Line(points = {{320, 10}, {320, 10}, {320, 30}, {350, 30}, {350, 30}}, color = {0, 0, 255}));
    connect(voltageSensor1.n, capacitor2.p) annotation(
      Line(points = {{322, -10}, {322, -10}, {322, -30}, {352, -30}, {352, -30}}, color = {0, 0, 255}));
    connect(capacitor1.n, capacitor2.n) annotation(
      Line(points = {{352, 10}, {352, -10}}, color = {0, 0, 255}));
    connect(capacitor2.p, m_V_DC_2) annotation(
      Line(points = {{352, -30}, {352, -52}, {400, -52}}, color = {0, 0, 255}));
    connect(ground2.p, capacitor1.n) annotation(
      Line(points = {{354, 0}, {350, 0}, {350, 10}}, color = {0, 0, 255}));
    connect(currentSensor4.p, V_DC_2) annotation(
      Line(points = {{380, 60}, {402, 60}, {402, 60}, {400, 60}}, color = {0, 0, 255}));
    connect(currentController1.v_d_conv_pu, gain16.u) annotation(
      Line(points = {{-104, 82}, {-100, 82}, {-100, 158}, {-168, 158}, {-168, 220}, {-142, 220}, {-142, 220}}, color = {0, 0, 127}));
    connect(gain16.y, energy_equation1.v_conv_d) annotation(
      Line(points = {{-134, 220}, {-118, 220}, {-118, 220}, {-114, 220}}, color = {0, 0, 127}));
    connect(CurrentTransform.d, gain15.u) annotation(
      Line(points = {{-218, 90}, {-190, 90}, {-190, 212}, {-142, 212}, {-142, 212}}, color = {0, 0, 127}));
    connect(gain15.y, energy_equation1.i_grid_d) annotation(
      Line(points = {{-134, 212}, {-116, 212}, {-116, 210}, {-114, 210}}, color = {0, 0, 127}));
    connect(gain13.y, energy_equation1.i_circ_z) annotation(
      Line(points = {{-130.5, 191}, {-120.5, 191}, {-120.5, 192}, {-114, 192}}, color = {0, 0, 127}));
    connect(energy_equation1.v_circ_z, gain18.y) annotation(
      Line(points = {{-114, 202}, {-132, 202}, {-132, 202}, {-132, 202}}, color = {0, 0, 127}));
    connect(energy_equation1.w_sum_z, gain17.u) annotation(
      Line(points = {{-10, 206}, {8, 206}, {8, 206}, {8, 206}}, color = {0, 0, 127}));
    connect(currentController1.i_d_ref, gain19.y) annotation(
      Line(points = {{-174, 106}, {-180, 106}, {-180, 106}, {-180, 106}}, color = {0, 0, 127}));
    connect(converterVoltage_c.n, ground1.p) annotation(
      Line(points = {{28, -20}, {40, -20}, {40, 0}}, color = {0, 0, 255}));
    connect(converterVoltage_b.n, ground1.p) annotation(
      Line(points = {{8, 0}, {40, 0}}, color = {0, 0, 255}));
    connect(converterVoltage_a.n, ground1.p) annotation(
      Line(points = {{-12, 20}, {40, 20}, {40, 0}}, color = {0, 0, 255}));
    connect(gain6.y, converterVoltage_b.v) annotation(
      Line(points = {{-27.6, 92}, {-4, 92}, {-4, 7}}, color = {0, 0, 127}));
    connect(inductor2.n, converterVoltage_b.p) annotation(
      Line(points = {{-182, 0}, {-14, 0}}, color = {0, 0, 255}));
    connect(gain9.u, potentialSensor1.phi) annotation(
      Line(points = {{-266.8, 62}, {-277.8, 62}, {-277.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.a, gain9.y) annotation(
      Line(points = {{-240, 62}, {-256, 62}}, color = {0, 0, 127}));
    connect(gain1.u, potentialSensor2.phi) annotation(
      Line(points = {{-266.8, 70}, {-297.8, 70}, {-297.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.b, gain1.y) annotation(
      Line(points = {{-242, 70}, {-258, 70}}, color = {0, 0, 127}));
    connect(gain2.u, potentialSensor3.phi) annotation(
      Line(points = {{-266.8, 78}, {-317.8, 78}, {-317.8, 51}}, color = {0, 0, 127}));
    connect(gain2.y, VoltageTransform.c) annotation(
      Line(points = {{-257.6, 78}, {-241.6, 78}}, color = {0, 0, 127}));
    connect(gain8.u, currentSensor1.i) annotation(
      Line(points = {{-266.8, 86}, {-337.8, 86}, {-337.8, 30}}, color = {0, 0, 127}));
    connect(CurrentTransform.a, gain8.y) annotation(
      Line(points = {{-240, 86}, {-256, 86}}, color = {0, 0, 127}));
    connect(gain3.u, currentSensor2.i) annotation(
      Line(points = {{-266.8, 94}, {-357.8, 94}, {-357.8, 10}}, color = {0, 0, 127}));
    connect(CurrentTransform.b, gain3.y) annotation(
      Line(points = {{-240, 94}, {-256, 94}}, color = {0, 0, 127}));
    connect(gain4.u, currentSensor3.i) annotation(
      Line(points = {{-266.8, 102}, {-377.8, 102}, {-377.8, -10}}, color = {0, 0, 127}));
    connect(CurrentTransform.c, gain4.y) annotation(
      Line(points = {{-240, 102}, {-256, 102}}, color = {0, 0, 127}));
    connect(gain7.y, converterVoltage_a.v) annotation(
      Line(points = {{-27.6, 76}, {-24, 76}, {-24, 27}}, color = {0, 0, 127}));
    connect(inductor1.n, converterVoltage_a.p) annotation(
      Line(points = {{-182, 20}, {-34, 20}}, color = {0, 0, 255}));
    connect(gain5.y, converterVoltage_c.v) annotation(
      Line(points = {{-27.6, 108}, {16, 108}, {16, -13}}, color = {0, 0, 127}));
    connect(inductor3.n, converterVoltage_c.p) annotation(
      Line(points = {{-182, -20}, {6, -20}}, color = {0, 0, 255}));
    connect(gain7.u, dQ_w_inv1.a) annotation(
      Line(points = {{-35, 76}, {-46, 76}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.b, gain6.u) annotation(
      Line(points = {{-46, 92}, {-35, 92}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.c, gain5.u) annotation(
      Line(points = {{-46, 108}, {-35, 108}}, color = {0, 0, 127}));
    connect(currentController1.v_q_conv_pu, dQ_w_inv1.q) annotation(
      Line(points = {{-105.21, 100.61}, {-81.21, 100.61}, {-81.21, 102}, {-92, 102}}, color = {0, 0, 127}));
    connect(currentController1.v_d_conv_pu, dQ_w_inv1.d) annotation(
      Line(points = {{-104.9, 81.7}, {-91.9, 81.7}, {-91.9, 82}, {-92, 82}}, color = {0, 0, 127}));
    connect(currentController1.i_q_ref, i_q_pu_ref.y) annotation(
      Line(points = {{-173.1, 115.8}, {-177.1, 115.8}}, color = {0, 0, 127}));
    connect(CurrentTransform.d, currentController1.i_d_mes_pu) annotation(
      Line(points = {{-216, 90}, {-172, 90}, {-172, 84}}, color = {0, 0, 127}));
    connect(CurrentTransform.q, currentController1.i_q_mes_pu) annotation(
      Line(points = {{-216, 100}, {-174, 100}, {-174, 98}, {-172, 98}}, color = {0, 0, 127}));
    connect(VoltageTransform.q, currentController1.v_q_grid_pu) annotation(
      Line(points = {{-216, 76}, {-172, 76}, {-172, 74}}, color = {0, 0, 127}));
    connect(VoltageTransform.d, currentController1.v_d_grid_pu) annotation(
      Line(points = {{-216, 66}, {-174, 66}, {-174, 64}, {-172, 64}}, color = {0, 0, 127}));
    connect(potentialSensor1.p, a) annotation(
      Line(points = {{-280, 30}, {-280, 20}, {-398, 20}}, color = {0, 0, 255}));
    connect(a, currentSensor1.n) annotation(
      Line(points = {{-398, 20}, {-349, 20}}, color = {0, 0, 255}));
    connect(resistor1.n, inductor1.p) annotation(
      Line(points = {{-218, 20}, {-198, 20}, {-198, 20}, {-198, 20}}, color = {0, 0, 255}));
    connect(currentSensor1.p, resistor1.p) annotation(
      Line(points = {{-330, 20}, {-241, 20}}, color = {0, 0, 255}));
    connect(potentialSensor2.p, b) annotation(
      Line(points = {{-298, 30}, {-298, 0}, {-396, 0}}, color = {0, 0, 255}));
    connect(potentialSensor3.p, c) annotation(
      Line(points = {{-318, 30}, {-318, -20}, {-396, -20}}, color = {0, 0, 255}));
    connect(b, currentSensor2.n) annotation(
      Line(points = {{-398, 0}, {-371, 0}}, color = {0, 0, 255}));
    connect(c, currentSensor3.n) annotation(
      Line(points = {{-398, -20}, {-391, -20}}, color = {0, 0, 255}));
    connect(resistor2.n, inductor2.p) annotation(
      Line(points = {{-218, 0}, {-198, 0}, {-198, 0}, {-198, 0}}, color = {0, 0, 255}));
    connect(currentSensor2.p, resistor2.p) annotation(
      Line(points = {{-350, 0}, {-239, 0}}, color = {0, 0, 255}));
    connect(resistor3.n, inductor3.p) annotation(
      Line(points = {{-218, -20}, {-198, -20}, {-198, -20}, {-198, -20}}, color = {0, 0, 255}));
    connect(currentSensor3.p, resistor3.p) annotation(
      Line(points = {{-370, -20}, {-239, -20}}, color = {0, 0, 255}));
    annotation(
      Diagram(coordinateSystem(extent = {{-400, -200}, {400, 400}})),
      Icon(coordinateSystem(extent = {{-400, -200}, {400, 400}}, initialScale = 0.1), graphics = {Rectangle(origin = {0, -1}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-400, 401}, {400, -201}}), Text(origin = {-44, 132}, fillPattern = FillPattern.Solid, extent = {{-236, 114}, {290, -166}}, textString = "MMC_V_feed")}),
      __OpenModelica_commandLineOptions = "");
  end MMC_zero_sequence_V_feed;

  model VoltageController_feed
    Modelica.Blocks.Interfaces.RealInput V_DC_ref annotation(
      Placement(visible = true, transformation(origin = {-122, -42}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-111, -81}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput V_DC_mes_pu annotation(
      Placement(visible = true, transformation(origin = {-115, 37}, extent = {{-13, -13}, {13, 13}}, rotation = 0), iconTransformation(origin = {-110, -4.44089e-16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput I_DC_pu annotation(
      Placement(visible = true, transformation(origin = {-126, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_grid_d annotation(
      Placement(visible = true, transformation(origin = {-116, 56}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {50, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-82, -42}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction1(a = {1, 0}, b = {K_vcp, K_vci}) annotation(
      Placement(visible = true, transformation(origin = {-44, -44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback2 annotation(
      Placement(visible = true, transformation(origin = {-10, -44}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.ff_voltage_controller ff_voltage_controller1 annotation(
      Placement(visible = true, transformation(origin = {-12, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput i_d_ref_pu annotation(
      Placement(visible = true, transformation(origin = {106, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback3 annotation(
      Placement(visible = true, transformation(origin = {42, -44}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput w_sum_z_pu annotation(
      Placement(visible = true, transformation(origin = {-115, 5}, extent = {{-13, -13}, {13, 13}}, rotation = 0), iconTransformation(origin = {-110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.Energy_feed energy_feed1 annotation(
      Placement(visible = true, transformation(origin = {20, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    extends SystemParameters;
  equation
    connect(feedback1.y, transferFunction1.u) annotation(
      Line(points = {{-73, -43}, {-64, -43}, {-64, -42}, {-55, -42}}, color = {0, 0, 127}));
    connect(V_DC_ref, feedback1.u1) annotation(
      Line(points = {{-122, -42}, {-106, -42}, {-106, -43}, {-90, -43}}, color = {0, 0, 127}));
    connect(feedback1.u2, V_DC_mes_pu) annotation(
      Line(points = {{-82, -35}, {-82, 37}, {-115, 37}}, color = {0, 0, 127}));
    connect(energy_feed1.Energy_feed, feedback3.u2) annotation(
      Line(points = {{32, 12}, {42, 12}, {42, -36}, {42, -36}}, color = {0, 0, 127}));
    connect(w_sum_z_pu, energy_feed1.w_sum_z_pu) annotation(
      Line(points = {{-114, 6}, {-6, 6}, {-6, 12}, {8, 12}}, color = {0, 0, 127}));
    connect(v_grid_d, energy_feed1.v_d_grid_pu) annotation(
      Line(points = {{-116, 56}, {-38, 56}, {-38, 14}, {8, 14}, {8, 17}}, color = {0, 0, 127}));
    connect(feedback3.y, i_d_ref_pu) annotation(
      Line(points = {{52, -44}, {98, -44}, {98, -42}, {106, -42}}, color = {0, 0, 127}));
    connect(feedback2.y, feedback3.u1) annotation(
      Line(points = {{0, -44}, {34, -44}, {34, -44}, {34, -44}}, color = {0, 0, 127}));
    connect(transferFunction1.y, feedback2.u1) annotation(
      Line(points = {{-33, -42}, {-20, -42}}, color = {0, 0, 127}));
    connect(ff_voltage_controller1.ff, feedback2.u2) annotation(
      Line(points = {{-12, 42}, {-10, 42}, {-10, -34}}, color = {0, 0, 127}));
    connect(ff_voltage_controller1.V_d, v_grid_d) annotation(
      Line(points = {{-24, 48}, {-96, 48}, {-96, 56}, {-116, 56}, {-116, 56}}, color = {0, 0, 127}));
    connect(I_DC_pu, ff_voltage_controller1.i_DC) annotation(
      Line(points = {{-126, 100}, {8, 100}, {8, 52}, {0, 52}}, color = {0, 0, 127}));
    connect(ff_voltage_controller1.V_DC, V_DC_mes_pu) annotation(
      Line(points = {{-11, 63}, {-11, 78}, {-70, 78}, {-70, 37}, {-115, 37}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, 0}, fillPattern = FillPattern.Solid, extent = {{-99, 100}, {99, -100}}, textString = "VoltageController")}));
  end VoltageController_feed;

  model Energy_feed
    Real Feed_forward;
  protected
    Real a;
    extends SystemParameters;
    Modelica.Blocks.Interfaces.RealInput w_sum_z_pu annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput Energy_feed annotation(
      Placement(visible = true, transformation(origin = {104, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_d_grid_pu annotation(
      Placement(visible = true, transformation(origin = {-120, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 52}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    Feed_forward = Energy_feed;
    when time > 0.9 then
      a = 2;
    end when;
    Energy_feed = (-(0 + a) * w_base / S_base * v_d_grid_pu) * der(w_sum_z_pu);
    annotation(
      Icon(graphics = {Rectangle(origin = {-1, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-99, 100}, {101, -100}}), Text(origin = {-46, 40}, fillPattern = FillPattern.Solid, extent = {{-12, 10}, {96, -84}}, textString = "Energy_feed")}));
  end Energy_feed;

  model Feed_test
    MMC.Grid grid1 annotation(
      Placement(visible = true, transformation(origin = {-84, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.MMC_zero_sequence_V_feed mMC_zero_sequence_V_feed1 annotation(
      Placement(visible = true, transformation(origin = {-58.3865, -55.1932}, extent = {{-17.6135, -8.80676}, {17.6135, 17.6135}}, rotation = 0)));
    MMC.Grid grid2 annotation(
      Placement(visible = true, transformation(origin = {-84, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.MMC_zero_sequence_V mMC_zero_sequence_V1 annotation(
      Placement(visible = true, transformation(origin = {-59, 13}, extent = {{-17, -8.5}, {17, 17}}, rotation = 0)));
    Cable.HVDC_Cable hVDC_Cable1 annotation(
      Placement(visible = true, transformation(origin = {-8, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Cable.HVDC_Cable hVDC_Cable2 annotation(
      Placement(visible = true, transformation(origin = {-6, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.DCside dCside1 annotation(
      Placement(visible = true, transformation(origin = {22, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.DCside dCside2 annotation(
      Placement(visible = true, transformation(origin = {26, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(hVDC_Cable1.m_V_DC_2_out, dCside1.m_V_DC_2) annotation(
      Line(points = {{2, 4}, {22, 4}, {22, 3}}, color = {0, 0, 255}));
    connect(hVDC_Cable1.V_DC_2_out, dCside1.V_DC_2) annotation(
      Line(points = {{2, 16}, {22, 16}, {22, 13}}, color = {0, 0, 255}));
    connect(hVDC_Cable2.m_V_DC_2_out, dCside2.m_V_DC_2) annotation(
      Line(points = {{4, -62}, {26, -62}, {26, -61}}, color = {0, 0, 255}));
    connect(hVDC_Cable2.V_DC_2_out, dCside2.V_DC_2) annotation(
      Line(points = {{4, -50}, {26, -50}, {26, -51}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V1.V_DC_2, hVDC_Cable1.V_DC_2_in) annotation(
      Line(points = {{-42, 16}, {-20, 16}, {-20, 16}, {-20, 16}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V1.m_V_DC_2, hVDC_Cable1.m_V_DC_2_in) annotation(
      Line(points = {{-42, 6}, {-20, 6}, {-20, 4}, {-20, 4}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V_feed1.m_V_DC_2, hVDC_Cable2.m_V_DC_2_in) annotation(
      Line(points = {{-40, -64}, {-14, -64}, {-14, -62}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V_feed1.V_DC_2, hVDC_Cable2.V_DC_2_in) annotation(
      Line(points = {{-40, -52}, {-14, -52}, {-14, -50}}, color = {0, 0, 255}));
    connect(grid1.A, mMC_zero_sequence_V_feed1.a) annotation(
      Line(points = {{-84.2, -51.6}, {-50.2, -51.6}, {-50.2, -46}, {-76, -46}}, color = {0, 0, 255}));
    connect(grid1.B, mMC_zero_sequence_V_feed1.b) annotation(
      Line(points = {{-84.2, -57.8}, {-67.2, -57.8}, {-67.2, -56}, {-76, -56}}, color = {0, 0, 255}));
    connect(grid1.C, mMC_zero_sequence_V_feed1.c) annotation(
      Line(points = {{-84.2, -63.8}, {-50.2, -63.8}, {-50.2, -67}, {-76, -67}}, color = {0, 0, 255}));
    connect(grid2.C, mMC_zero_sequence_V1.c) annotation(
      Line(points = {{-82.2, 6.2}, {-74.2, 6.2}, {-74.2, 1.2}}, color = {0, 0, 255}));
    connect(grid2.B, mMC_zero_sequence_V1.b) annotation(
      Line(points = {{-82.2, 12.2}, {-74.2, 12.2}}, color = {0, 0, 255}));
    connect(grid2.A, mMC_zero_sequence_V1.a) annotation(
      Line(points = {{-82.2, 18.4}, {-74.2, 18.4}, {-74.2, 21.4}}, color = {0, 0, 255}));
  end Feed_test;

  model MMC_zero_sequence_P_feed
    Real P_AC_pu;
    Real P_refrence;
    Real Eng;
    Real Eng_tot(start = 0);
    MMC.CurrentController currentController1 annotation(
      Placement(visible = true, transformation(origin = {-139, 91}, extent = {{-31, 31}, {31, -31}}, rotation = 0)));
    MMC.Energy_equation energy_equation1 annotation(
      Placement(visible = true, transformation(origin = {-62, 206}, extent = {{-48, -48}, {48, 48}}, rotation = 0)));
    MMC.Energy_Controller energy_Controller1(u_circ_z_pu(fixed = true, start = 1)) annotation(
      Placement(visible = true, transformation(origin = {167, -15}, extent = {{-37, -37}, {37, 37}}, rotation = 0)));
    Modelica.Blocks.Sources.Exponentials exponentials1(fallTimeConst = 10, offset = 1, outMax = -0.0225 / 0.8, riseTime = 0.05, riseTimeConst = 0.01, startTime = 1.1) annotation(
      Placement(visible = true, transformation(origin = {162, 32}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
    MMC.PowerController_feed powerController_feed1 annotation(
      Placement(visible = true, transformation(origin = {-69, 283}, extent = {{49, -49}, {-49, 49}}, rotation = 0)));
  protected
    extends SystemParameters;
    Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
      Placement(visible = true, transformation(origin = {410, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = C_DC * 2, v(fixed = true, start = V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {386, 26}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor2(C = C_DC * 2, v(fixed = true, start = -V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {386, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Add add1 annotation(
      Placement(visible = true, transformation(origin = {14, 244}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin a annotation(
      Placement(visible = true, transformation(origin = {-400, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-402, 326}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin b annotation(
      Placement(visible = true, transformation(origin = {-400, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-398, 108}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin c annotation(
      Placement(visible = true, transformation(origin = {-400, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-392, -136}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R_f) annotation(
      Placement(visible = true, transformation(origin = {-230, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor2(R = R_f) annotation(
      Placement(visible = true, transformation(origin = {-230, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor3(R = R_f) annotation(
      Placement(visible = true, transformation(origin = {-230, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1 annotation(
      Placement(visible = true, transformation(origin = {-280, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor2 annotation(
      Placement(visible = true, transformation(origin = {-300, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor3 annotation(
      Placement(visible = true, transformation(origin = {-320, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {-340, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {-360, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {-380, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L_f) annotation(
      Placement(visible = true, transformation(origin = {-192, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor2(L = L_f) annotation(
      Placement(visible = true, transformation(origin = {-192, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor3(L = L_f) annotation(
      Placement(visible = true, transformation(origin = {-192, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.DQ_w VoltageTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w CurrentTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 94}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w_inv dQ_w_inv1 annotation(
      Placement(visible = true, transformation(origin = {-70, 92}, extent = {{-20, 20}, {20, -20}}, rotation = 0)));
    Modelica.Blocks.Sources.Step i_q_pu_ref(height = 0, offset = 0) annotation(
      Placement(visible = true, transformation(origin = {-180, 116}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 108}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 92}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain7(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 76}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_a annotation(
      Placement(visible = true, transformation(origin = {-22, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_b annotation(
      Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_c annotation(
      Placement(visible = true, transformation(origin = {18, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {50, 1}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain8(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 86}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain9(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 62}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 70}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 78}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 94}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 102}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage Two_v_circ annotation(
      Placement(visible = true, transformation(origin = {278, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain10(k = 2 * V_base) annotation(
      Placement(visible = true, transformation(origin = {248, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor4(L = 2 / 3 * L_a) annotation(
      Placement(visible = true, transformation(origin = {292, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor4(R = 2 / 3 * R_a) annotation(
      Placement(visible = true, transformation(origin = {318, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {408, 190}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {400, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor1 annotation(
      Placement(visible = true, transformation(origin = {332, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain11(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {234, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Math.Gain gain12(k = 1 / I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {126, 323}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor4 annotation(
      Placement(visible = true, transformation(origin = {370, 60}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step P_ref(height = 0.1 * direction_V, offset = 0.7 * direction_V, startTime = 1) annotation(
      Placement(visible = true, transformation(origin = {49, 229}, extent = {{9, -9}, {-9, 9}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor5 annotation(
      Placement(visible = true, transformation(origin = {278, 28}, extent = {{10, 10}, {-10, -10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain13(k = 1 / 3) annotation(
      Placement(visible = true, transformation(origin = {-137, 191}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain14(k = 1 / (3 * I_base)) annotation(
      Placement(visible = true, transformation(origin = {100, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain15(k = I_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 212}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain16(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 220}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain17(k = 1 / w_base) annotation(
      Placement(visible = true, transformation(origin = {12, 206}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain18(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-137, 202}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain19(k = 1) annotation(
      Placement(visible = true, transformation(origin = {-182, 106}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step1(height = 0.1 * direction_V, offset = 0, startTime = 1.1) annotation(
      Placement(visible = true, transformation(origin = {49, 253}, extent = {{9, -9}, {-9, 9}}, rotation = 0)));
  equation
    connect(energy_Controller1.i_circ_z_pu, gain14.y) annotation(
      Line(points = {{122, -44}, {112, -44}, {112, -46}, {112, -46}}, color = {0, 0, 127}));
    connect(currentController1.i_d_ref, gain19.y) annotation(
      Line(points = {{-174, 106}, {-180, 106}}, color = {0, 0, 127}));
    connect(powerController_feed1.i_d_ref_pu, gain19.u) annotation(
      Line(points = {{-122, 284}, {-188, 284}, {-188, 106}, {-184, 106}, {-184, 106}}, color = {0, 0, 127}));
    connect(VoltageTransform.d, powerController_feed1.v_grid_d) annotation(
      Line(points = {{-218, 66}, {-200, 66}, {-200, 370}, {-98, 370}, {-98, 342}, {-94, 342}}, color = {0, 0, 127}));
    connect(powerController_feed1.V_DC_mes_pu, gain11.y) annotation(
      Line(points = {{-16, 284}, {200, 284}, {200, 44}, {224, 44}, {224, 44}}, color = {0, 0, 127}));
    connect(powerController_feed1.w_sum_z_pu, gain17.y) annotation(
      Line(points = {{-16, 264}, {80, 264}, {80, 206}, {16, 206}, {16, 206}}, color = {0, 0, 127}));
    connect(powerController_feed1.V_DC_ref, add1.y) annotation(
      Line(points = {{-14, 244}, {2, 244}, {2, 244}, {4, 244}}, color = {0, 0, 127}));
    connect(powerController_feed1.I_DC_pu, gain12.y) annotation(
      Line(points = {{-16, 322}, {116, 322}, {116, 324}, {116, 324}}, color = {0, 0, 127}));
    connect(step1.y, add1.u1) annotation(
      Line(points = {{41, 253}, {28, 253}, {28, 250}, {26, 250}}, color = {0, 0, 127}));
    P_AC_pu = -(converterVoltage_a.v * converterVoltage_a.i + converterVoltage_b.v * converterVoltage_b.i + converterVoltage_c.v * converterVoltage_c.i) / S_base;
    if time > 1.1 then
      P_refrence = 0.8977;
    elseif time > 1 then
      P_refrence = 0.79723;
    else
      P_refrence = 0.697676;
    end if;
    if time > 1 then
      Eng = P_refrence - P_AC_pu;
    else
      Eng = 0;
    end if;
    der(Eng_tot) = Eng;
    connect(add1.u2, P_ref.y) annotation(
      Line(points = {{26, 238}, {39, 238}, {39, 229}}, color = {0, 0, 127}));
    connect(exponentials1.y, energy_Controller1.w_sum_z_ref) annotation(
      Line(points = {{156, 32}, {112, 32}, {112, 12}, {120, 12}, {120, 14}, {122, 14}}, color = {0, 0, 127}));
    connect(resistor4.n, capacitor1.p) annotation(
      Line(points = {{328, 60}, {350, 60}, {350, 38}, {386, 38}, {386, 36}}, color = {0, 0, 255}));
    connect(Two_v_circ.n, m_V_DC_2) annotation(
      Line(points = {{278, -10}, {278, -76}, {400, -76}}, color = {0, 0, 255}));
    connect(capacitor2.p, m_V_DC_2) annotation(
      Line(points = {{388, -40}, {388, -76}, {400, -76}}, color = {0, 0, 255}));
    connect(capacitor1.n, ground2.p) annotation(
      Line(points = {{386, 16}, {386, 0}, {400, 0}}, color = {0, 0, 255}));
    connect(capacitor1.n, capacitor2.n) annotation(
      Line(points = {{386, 16}, {386, 16}, {386, -20}, {386, -20}}, color = {0, 0, 255}));
    connect(Two_v_circ.n, voltageSensor1.n) annotation(
      Line(points = {{278, -10}, {278, -76}, {334, -76}, {334, -10}}, color = {0, 0, 255}));
    connect(voltageSensor1.p, resistor4.n) annotation(
      Line(points = {{334, 10}, {334, 60}, {328, 60}}, color = {0, 0, 255}));
    connect(gain11.u, voltageSensor1.v) annotation(
      Line(points = {{246, 42}, {324, 42}, {324, 0}}, color = {0, 0, 127}));
    connect(gain12.u, currentSensor4.i) annotation(
      Line(points = {{136, 323}, {368, 323}, {368, 70}, {370, 70}}, color = {0, 0, 127}));
    connect(converterVoltage_c.n, ground1.p) annotation(
      Line(points = {{28, -20}, {40, -20}, {40, 0}}, color = {0, 0, 255}));
    connect(converterVoltage_b.n, ground1.p) annotation(
      Line(points = {{8, 0}, {40, 0}}, color = {0, 0, 255}));
    connect(converterVoltage_a.n, ground1.p) annotation(
      Line(points = {{-12, 20}, {40, 20}, {40, 0}}, color = {0, 0, 255}));
    connect(currentController1.v_d_conv_pu, gain16.u) annotation(
      Line(points = {{-104, 82}, {-100, 82}, {-100, 158}, {-168, 158}, {-168, 220}, {-142, 220}, {-142, 220}}, color = {0, 0, 127}));
    connect(gain6.y, converterVoltage_b.v) annotation(
      Line(points = {{-27.6, 92}, {-4, 92}, {-4, 7}}, color = {0, 0, 127}));
    connect(inductor2.n, converterVoltage_b.p) annotation(
      Line(points = {{-182, 0}, {-14, 0}}, color = {0, 0, 255}));
    connect(gain18.u, energy_Controller1.u_circ_z_pu) annotation(
      Line(points = {{-142, 202}, {-162, 202}, {-162, 166}, {70, 166}, {70, -64}, {224, -64}, {224, 0}, {212, 0}, {212, 0}}, color = {0, 0, 127}));
    connect(gain13.u, currentSensor5.i) annotation(
      Line(points = {{-142, 191}, {-158, 191}, {-158, 174}, {78, 174}, {78, -78}, {268, -78}, {268, 28}}, color = {0, 0, 127}));
    connect(gain13.y, energy_equation1.i_circ_z) annotation(
      Line(points = {{-130.5, 191}, {-120.5, 191}, {-120.5, 192}, {-114, 192}}, color = {0, 0, 127}));
    connect(energy_equation1.v_circ_z, gain18.y) annotation(
      Line(points = {{-114, 202}, {-132, 202}, {-132, 202}, {-132, 202}}, color = {0, 0, 127}));
    connect(gain17.y, energy_Controller1.w_sum_z_pu) annotation(
      Line(points = {{16, 206}, {96, 206}, {96, -22}, {122, -22}, {122, -22}}, color = {0, 0, 127}));
    connect(energy_equation1.w_sum_z, gain17.u) annotation(
      Line(points = {{-10, 206}, {8, 206}, {8, 206}, {8, 206}}, color = {0, 0, 127}));
    connect(CurrentTransform.d, gain15.u) annotation(
      Line(points = {{-218, 90}, {-190, 90}, {-190, 212}, {-142, 212}, {-142, 212}}, color = {0, 0, 127}));
    connect(gain15.y, energy_equation1.i_grid_d) annotation(
      Line(points = {{-134, 212}, {-116, 212}, {-116, 210}, {-114, 210}}, color = {0, 0, 127}));
    connect(gain16.y, energy_equation1.v_conv_d) annotation(
      Line(points = {{-134, 220}, {-118, 220}, {-118, 220}, {-114, 220}}, color = {0, 0, 127}));
    connect(gain14.u, currentSensor5.i) annotation(
      Line(points = {{88, -46}, {78, -46}, {78, -78}, {268, -78}, {268, 28}, {268, 28}}, color = {0, 0, 127}));
    connect(gain9.u, potentialSensor1.phi) annotation(
      Line(points = {{-266.8, 62}, {-277.8, 62}, {-277.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.a, gain9.y) annotation(
      Line(points = {{-240, 62}, {-256, 62}}, color = {0, 0, 127}));
    connect(gain1.u, potentialSensor2.phi) annotation(
      Line(points = {{-266.8, 70}, {-297.8, 70}, {-297.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.b, gain1.y) annotation(
      Line(points = {{-242, 70}, {-258, 70}}, color = {0, 0, 127}));
    connect(gain2.u, potentialSensor3.phi) annotation(
      Line(points = {{-266.8, 78}, {-317.8, 78}, {-317.8, 51}}, color = {0, 0, 127}));
    connect(gain2.y, VoltageTransform.c) annotation(
      Line(points = {{-257.6, 78}, {-241.6, 78}}, color = {0, 0, 127}));
    connect(gain8.u, currentSensor1.i) annotation(
      Line(points = {{-266.8, 86}, {-337.8, 86}, {-337.8, 30}}, color = {0, 0, 127}));
    connect(CurrentTransform.a, gain8.y) annotation(
      Line(points = {{-240, 86}, {-256, 86}}, color = {0, 0, 127}));
    connect(gain3.u, currentSensor2.i) annotation(
      Line(points = {{-266.8, 94}, {-357.8, 94}, {-357.8, 10}}, color = {0, 0, 127}));
    connect(CurrentTransform.b, gain3.y) annotation(
      Line(points = {{-240, 94}, {-256, 94}}, color = {0, 0, 127}));
    connect(gain4.u, currentSensor3.i) annotation(
      Line(points = {{-266.8, 102}, {-377.8, 102}, {-377.8, -10}}, color = {0, 0, 127}));
    connect(CurrentTransform.c, gain4.y) annotation(
      Line(points = {{-240, 102}, {-256, 102}}, color = {0, 0, 127}));
    connect(inductor4.p, currentSensor5.p) annotation(
      Line(points = {{282, 60}, {278, 60}, {278, 36}, {278, 36}}, color = {0, 0, 255}));
    connect(currentSensor5.n, Two_v_circ.p) annotation(
      Line(points = {{278, 16}, {278, 16}, {278, 10}, {278, 10}}, color = {0, 0, 255}));
    connect(energy_Controller1.u_circ_z_pu, gain10.u) annotation(
      Line(points = {{211.4, -0.2}, {226.4, -0.2}, {226.4, 0}, {238, 0}}, color = {0, 0, 127}));
    connect(gain10.y, Two_v_circ.v) annotation(
      Line(points = {{261, 0}, {269, 0}}, color = {0, 0, 127}));
    connect(gain11.y, energy_Controller1.v_DC_pu) annotation(
      Line(points = {{223, 42}, {106, 42}, {106, -4}, {122, -4}}, color = {0, 0, 127}));
    connect(resistor4.p, inductor4.n) annotation(
      Line(points = {{310, 60}, {302, 60}}, color = {0, 0, 255}));
    connect(resistor4.n, currentSensor4.n) annotation(
      Line(points = {{330, 60}, {360, 60}}, color = {0, 0, 255}));
    connect(currentSensor4.p, V_DC_2) annotation(
      Line(points = {{380, 60}, {402, 60}, {402, 60}, {400, 60}}, color = {0, 0, 255}));
    connect(gain7.y, converterVoltage_a.v) annotation(
      Line(points = {{-27.6, 76}, {-24, 76}, {-24, 27}}, color = {0, 0, 127}));
    connect(inductor1.n, converterVoltage_a.p) annotation(
      Line(points = {{-182, 20}, {-34, 20}}, color = {0, 0, 255}));
    connect(gain5.y, converterVoltage_c.v) annotation(
      Line(points = {{-27.6, 108}, {16, 108}, {16, -13}}, color = {0, 0, 127}));
    connect(inductor3.n, converterVoltage_c.p) annotation(
      Line(points = {{-182, -20}, {6, -20}}, color = {0, 0, 255}));
    connect(gain7.u, dQ_w_inv1.a) annotation(
      Line(points = {{-35, 76}, {-46, 76}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.b, gain6.u) annotation(
      Line(points = {{-46, 92}, {-35, 92}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.c, gain5.u) annotation(
      Line(points = {{-46, 108}, {-35, 108}}, color = {0, 0, 127}));
    connect(currentController1.v_q_conv_pu, dQ_w_inv1.q) annotation(
      Line(points = {{-105.21, 100.61}, {-81.21, 100.61}, {-81.21, 102}, {-92, 102}}, color = {0, 0, 127}));
    connect(currentController1.v_d_conv_pu, dQ_w_inv1.d) annotation(
      Line(points = {{-104.9, 81.7}, {-91.9, 81.7}, {-91.9, 82}, {-92, 82}}, color = {0, 0, 127}));
    connect(currentController1.i_q_ref, i_q_pu_ref.y) annotation(
      Line(points = {{-173.1, 115.8}, {-177.1, 115.8}}, color = {0, 0, 127}));
    connect(CurrentTransform.d, currentController1.i_d_mes_pu) annotation(
      Line(points = {{-216, 90}, {-172, 90}, {-172, 84}}, color = {0, 0, 127}));
    connect(CurrentTransform.q, currentController1.i_q_mes_pu) annotation(
      Line(points = {{-216, 100}, {-174, 100}, {-174, 98}, {-172, 98}}, color = {0, 0, 127}));
    connect(VoltageTransform.q, currentController1.v_q_grid_pu) annotation(
      Line(points = {{-216, 76}, {-172, 76}, {-172, 74}}, color = {0, 0, 127}));
    connect(VoltageTransform.d, currentController1.v_d_grid_pu) annotation(
      Line(points = {{-216, 66}, {-174, 66}, {-174, 64}, {-172, 64}}, color = {0, 0, 127}));
    connect(potentialSensor1.p, a) annotation(
      Line(points = {{-280, 30}, {-280, 20}, {-398, 20}}, color = {0, 0, 255}));
    connect(a, currentSensor1.n) annotation(
      Line(points = {{-398, 20}, {-349, 20}}, color = {0, 0, 255}));
    connect(resistor1.n, inductor1.p) annotation(
      Line(points = {{-218, 20}, {-198, 20}, {-198, 20}, {-198, 20}}, color = {0, 0, 255}));
    connect(currentSensor1.p, resistor1.p) annotation(
      Line(points = {{-330, 20}, {-241, 20}}, color = {0, 0, 255}));
    connect(potentialSensor2.p, b) annotation(
      Line(points = {{-298, 30}, {-298, 0}, {-396, 0}}, color = {0, 0, 255}));
    connect(potentialSensor3.p, c) annotation(
      Line(points = {{-318, 30}, {-318, -20}, {-396, -20}}, color = {0, 0, 255}));
    connect(b, currentSensor2.n) annotation(
      Line(points = {{-398, 0}, {-371, 0}}, color = {0, 0, 255}));
    connect(c, currentSensor3.n) annotation(
      Line(points = {{-398, -20}, {-391, -20}}, color = {0, 0, 255}));
    connect(resistor2.n, inductor2.p) annotation(
      Line(points = {{-218, 0}, {-198, 0}, {-198, 0}, {-198, 0}}, color = {0, 0, 255}));
    connect(currentSensor2.p, resistor2.p) annotation(
      Line(points = {{-350, 0}, {-239, 0}}, color = {0, 0, 255}));
    connect(resistor3.n, inductor3.p) annotation(
      Line(points = {{-218, -20}, {-198, -20}, {-198, -20}, {-198, -20}}, color = {0, 0, 255}));
    connect(currentSensor3.p, resistor3.p) annotation(
      Line(points = {{-370, -20}, {-239, -20}}, color = {0, 0, 255}));
    annotation(
      Diagram(coordinateSystem(extent = {{-400, -200}, {400, 400}})),
      Icon(coordinateSystem(extent = {{-400, -200}, {400, 400}}, initialScale = 0.1), graphics = {Rectangle(origin = {0, -1}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-400, 401}, {400, -201}}), Text(origin = {-44, 132}, fillPattern = FillPattern.Solid, extent = {{-236, 114}, {290, -166}}, textString = "MMC_P_feed")}),
      __OpenModelica_commandLineOptions = "");
  end MMC_zero_sequence_P_feed;

  model PowerController_feed
    Modelica.Blocks.Interfaces.RealInput V_DC_ref annotation(
      Placement(visible = true, transformation(origin = {-122, -42}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-111, -81}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput V_DC_mes_pu annotation(
      Placement(visible = true, transformation(origin = {-115, 37}, extent = {{-13, -13}, {13, 13}}, rotation = 0), iconTransformation(origin = {-110, -4.44089e-16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput I_DC_pu annotation(
      Placement(visible = true, transformation(origin = {-126, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_grid_d annotation(
      Placement(visible = true, transformation(origin = {-116, 56}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {50, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-82, -43}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction1(a = {1, 0}, b = {K_vcp, K_vci * 10}) annotation(
      Placement(visible = true, transformation(origin = {-44, -44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback2 annotation(
      Placement(visible = true, transformation(origin = {-10, -44}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.ff_voltage_controller ff_voltage_controller1 annotation(
      Placement(visible = true, transformation(origin = {-12, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput i_d_ref_pu annotation(
      Placement(visible = true, transformation(origin = {106, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback3 annotation(
      Placement(visible = true, transformation(origin = {42, -44}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput w_sum_z_pu annotation(
      Placement(visible = true, transformation(origin = {-115, 5}, extent = {{-13, -13}, {13, 13}}, rotation = 0), iconTransformation(origin = {-110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.Energy_feed energy_feed1 annotation(
      Placement(visible = true, transformation(origin = {20, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.MultiProduct multiProduct1(nu = 2) annotation(
      Placement(visible = true, transformation(origin = {-82, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain1(k = 1) annotation(
      Placement(visible = true, transformation(origin = {42, -12}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain2(k = 1) annotation(
      Placement(visible = true, transformation(origin = {-10, -14}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  protected
    extends SystemParameters;
  equation
    connect(gain2.y, feedback2.u2) annotation(
      Line(points = {{-10, -25}, {-10, -36}}, color = {0, 0, 127}));
    connect(ff_voltage_controller1.ff, gain2.u) annotation(
      Line(points = {{-12, 42}, {-12, 0}, {-10, 0}, {-10, -2}}, color = {0, 0, 127}));
    connect(gain1.y, feedback3.u2) annotation(
      Line(points = {{42, -24}, {42, -24}, {42, -36}, {42, -36}}, color = {0, 0, 127}));
    connect(energy_feed1.Energy_feed, gain1.u) annotation(
      Line(points = {{32, 12}, {44, 12}, {44, 0}, {44, 0}}, color = {0, 0, 127}));
    connect(feedback1.y, transferFunction1.u) annotation(
      Line(points = {{-73, -42}, {-55, -42}}, color = {0, 0, 127}));
    connect(V_DC_ref, feedback1.u1) annotation(
      Line(points = {{-122, -42}, {-90, -42}}, color = {0, 0, 127}));
    connect(multiProduct1.y, feedback1.u2) annotation(
      Line(points = {{-82, -22}, {-82, -22}, {-82, -36}, {-82, -36}}, color = {0, 0, 127}));
    connect(I_DC_pu, multiProduct1.u[1]) annotation(
      Line(points = {{-126, 100}, {-82, 100}, {-82, 0}, {-82, 0}}, color = {0, 0, 127}));
    connect(V_DC_mes_pu, multiProduct1.u[2]) annotation(
      Line(points = {{-114, 38}, {-84, 38}, {-84, 0}, {-82, 0}}, color = {0, 0, 127}));
    connect(w_sum_z_pu, energy_feed1.w_sum_z_pu) annotation(
      Line(points = {{-114, 6}, {-6, 6}, {-6, 12}, {8, 12}}, color = {0, 0, 127}));
    connect(v_grid_d, energy_feed1.v_d_grid_pu) annotation(
      Line(points = {{-116, 56}, {-38, 56}, {-38, 14}, {8, 14}, {8, 17}}, color = {0, 0, 127}));
    connect(feedback3.y, i_d_ref_pu) annotation(
      Line(points = {{52, -44}, {98, -44}, {98, -42}, {106, -42}}, color = {0, 0, 127}));
    connect(feedback2.y, feedback3.u1) annotation(
      Line(points = {{0, -44}, {34, -44}, {34, -44}, {34, -44}}, color = {0, 0, 127}));
    connect(transferFunction1.y, feedback2.u1) annotation(
      Line(points = {{-33, -42}, {-20, -42}}, color = {0, 0, 127}));
    connect(ff_voltage_controller1.V_d, v_grid_d) annotation(
      Line(points = {{-24, 48}, {-96, 48}, {-96, 56}, {-116, 56}, {-116, 56}}, color = {0, 0, 127}));
    connect(I_DC_pu, ff_voltage_controller1.i_DC) annotation(
      Line(points = {{-126, 100}, {8, 100}, {8, 52}, {0, 52}}, color = {0, 0, 127}));
    connect(ff_voltage_controller1.V_DC, V_DC_mes_pu) annotation(
      Line(points = {{-11, 63}, {-11, 78}, {-70, 78}, {-70, 37}, {-115, 37}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, 0}, fillPattern = FillPattern.Solid, extent = {{-99, 100}, {99, -100}}, textString = "PowerControllerFeed")}, coordinateSystem(initialScale = 0.1)));
  end PowerController_feed;

  model RandomeTests
    MMC.Grid grid1 annotation(
      Placement(visible = true, transformation(origin = {-70, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    DCside dCside1 annotation(
      Placement(visible = true, transformation(origin = {72, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC_zero_sequence_V mMC_zero_sequence_V1 annotation(
      Placement(visible = true, transformation(origin = {-2, -4}, extent = {{-40, -20}, {40, 40}}, rotation = 0)));
  equation
    connect(mMC_zero_sequence_V1.m_V_DC_2, dCside1.m_V_DC_2) annotation(
      Line(points = {{38, -10}, {72, -10}, {72, -6}, {72, -6}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V1.V_DC_2, dCside1.V_DC_2) annotation(
      Line(points = {{38, 16}, {72, 16}, {72, 6}, {72, 6}}, color = {0, 0, 255}));
    connect(grid1.C, mMC_zero_sequence_V1.c) annotation(
      Line(points = {{-70, -2}, {-70, -2}, {-70, -18}, {-42, -18}, {-42, -18}}, color = {0, 0, 255}));
    connect(grid1.B, mMC_zero_sequence_V1.b) annotation(
      Line(points = {{-70, 4}, {-42, 4}, {-42, 6}, {-42, 6}}, color = {0, 0, 255}));
    connect(grid1.A, mMC_zero_sequence_V1.a) annotation(
      Line(points = {{-70, 10}, {-70, 30}, {-42, 30}, {-42, 28}}, color = {0, 0, 255}));
  end RandomeTests;

  model MMC_zero_sequence_V_lon
    MMC.CurrentController currentController1 annotation(
      Placement(visible = true, transformation(origin = {-139, 91}, extent = {{-31, 31}, {31, -31}}, rotation = 0)));
    MMC.Energy_equation energy_equation1 annotation(
      Placement(visible = true, transformation(origin = {-62, 206}, extent = {{-48, -48}, {48, 48}}, rotation = 0)));
    MMC.Energy_Controller energy_Controller1(u_circ_z_pu(fixed = true, start = 1)) annotation(
      Placement(visible = true, transformation(origin = {167, -15}, extent = {{-37, -37}, {37, 37}}, rotation = 0)));
    MMC.VoltageController voltageController1 annotation(
      Placement(visible = true, transformation(origin = {-55, 273}, extent = {{-33, 33}, {33, -33}}, rotation = 180)));
    Modelica.Blocks.Interfaces.RealInput P_inn annotation(
      Placement(visible = true, transformation(origin = {179, 33}, extent = {{13, -13}, {-13, 13}}, rotation = 0), iconTransformation(origin = {424, 92}, extent = {{20, -20}, {-20, 20}}, rotation = 0)));
    MMC.EnergyReferenceController energyReferenceController1 annotation(
      Placement(visible = true, transformation(origin = {143, 34}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  protected
    extends SystemParameters;
    Modelica.Electrical.Analog.Interfaces.Pin a annotation(
      Placement(visible = true, transformation(origin = {-400, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-402, 326}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin b annotation(
      Placement(visible = true, transformation(origin = {-400, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-398, 108}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin c annotation(
      Placement(visible = true, transformation(origin = {-400, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-392, -136}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R_v) annotation(
      Placement(visible = true, transformation(origin = {-230, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor2(R = R_v) annotation(
      Placement(visible = true, transformation(origin = {-230, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor3(R = R_v) annotation(
      Placement(visible = true, transformation(origin = {-230, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1 annotation(
      Placement(visible = true, transformation(origin = {-280, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor2 annotation(
      Placement(visible = true, transformation(origin = {-300, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor3 annotation(
      Placement(visible = true, transformation(origin = {-320, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {-340, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {-360, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {-380, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L_v, i(fixed = false, start = I_base)) annotation(
      Placement(visible = true, transformation(origin = {-192, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor2(L = L_v, i(fixed = false, start = -0.7 * I_base)) annotation(
      Placement(visible = true, transformation(origin = {-192, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor3(L = L_v, i(fixed = false, start = -0.7 * I_base)) annotation(
      Placement(visible = true, transformation(origin = {-192, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.DQ_w VoltageTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w CurrentTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 94}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w_inv dQ_w_inv1 annotation(
      Placement(visible = true, transformation(origin = {-70, 92}, extent = {{-20, 20}, {20, -20}}, rotation = 0)));
    Modelica.Blocks.Sources.Step i_q_pu_ref(height = 0, offset = 0) annotation(
      Placement(visible = true, transformation(origin = {-180, 116}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 108}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 92}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain7(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 76}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_a annotation(
      Placement(visible = true, transformation(origin = {-22, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_b annotation(
      Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_c annotation(
      Placement(visible = true, transformation(origin = {18, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {50, 1}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain8(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 86}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain9(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 62}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 70}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 78}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 94}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 102}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage Two_v_circ annotation(
      Placement(visible = true, transformation(origin = {278, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain10(k = 2 * V_base) annotation(
      Placement(visible = true, transformation(origin = {248, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = 2 * C_DC, v(fixed = true, start = V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {350, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor2(C = 2 * C_DC, v(fixed = true, start = -V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {350, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Basic.Inductor inductor4(L = 2 / 3 * L_a) annotation(
      Placement(visible = true, transformation(origin = {292, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor4(R = 2 / 3 * R_a) annotation(
      Placement(visible = true, transformation(origin = {318, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
      Placement(visible = true, transformation(origin = {366, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {408, 190}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {400, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor1 annotation(
      Placement(visible = true, transformation(origin = {320, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain11(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {234, 41}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Math.Gain gain12(k = 1 / I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {8, 299}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor4 annotation(
      Placement(visible = true, transformation(origin = {370, 60}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step V_DC_ref(height = 0.1, offset = 1, startTime = 4) annotation(
      Placement(visible = true, transformation(origin = {10, 246}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor5 annotation(
      Placement(visible = true, transformation(origin = {278, 28}, extent = {{10, 10}, {-10, -10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain13(k = 1 / 3) annotation(
      Placement(visible = true, transformation(origin = {-137, 191}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain14(k = 1 / (3 * I_base)) annotation(
      Placement(visible = true, transformation(origin = {100, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain15(k = I_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 212}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain16(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 220}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain17(k = 1 / w_base) annotation(
      Placement(visible = true, transformation(origin = {12, 206}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain18(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-137, 202}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain19(k = -1) annotation(
      Placement(visible = true, transformation(origin = {-182, 106}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
  equation
    connect(energyReferenceController1.w_sum_ref, energy_Controller1.w_sum_z_ref) annotation(
      Line(points = {{137, 34}, {112, 34}, {112, 16}, {122, 16}, {122, 14}}, color = {0, 0, 127}));
    connect(energyReferenceController1.P_in, P_inn) annotation(
      Line(points = {{149, 34}, {167.5, 34}, {167.5, 33}, {173.25, 33}, {173.25, 33}, {179, 33}}, color = {0, 0, 127}));
    connect(energy_Controller1.i_circ_z_pu, gain14.y) annotation(
      Line(points = {{122, -44}, {110, -44}, {110, -46}, {112, -46}}, color = {0, 0, 127}));
    connect(voltageController1.V_DC_mes_pu, gain11.y) annotation(
      Line(points = {{-18, 274}, {198, 274}, {198, 42}, {223, 42}}, color = {0, 0, 127}));
    connect(gain11.u, voltageSensor1.v) annotation(
      Line(points = {{246, 42}, {312, 42}, {312, 0}}, color = {0, 0, 127}));
    connect(gain11.y, energy_Controller1.v_DC_pu) annotation(
      Line(points = {{223, 42}, {106, 42}, {106, -4}, {122, -4}}, color = {0, 0, 127}));
    connect(gain14.u, currentSensor5.i) annotation(
      Line(points = {{88, -46}, {78, -46}, {78, -78}, {268, -78}, {268, 28}, {268, 28}}, color = {0, 0, 127}));
    connect(gain18.u, energy_Controller1.u_circ_z_pu) annotation(
      Line(points = {{-142, 202}, {-162, 202}, {-162, 166}, {70, 166}, {70, -64}, {224, -64}, {224, 0}, {212, 0}, {212, 0}}, color = {0, 0, 127}));
    connect(gain17.y, energy_Controller1.w_sum_z_pu) annotation(
      Line(points = {{16, 206}, {96, 206}, {96, -22}, {122, -22}, {122, -22}}, color = {0, 0, 127}));
    connect(energy_Controller1.u_circ_z_pu, gain10.u) annotation(
      Line(points = {{211.4, -0.2}, {226.4, -0.2}, {226.4, 0}, {238, 0}}, color = {0, 0, 127}));
    connect(gain10.y, Two_v_circ.v) annotation(
      Line(points = {{261, 0}, {269, 0}}, color = {0, 0, 127}));
    connect(gain13.u, currentSensor5.i) annotation(
      Line(points = {{-142, 191}, {-158, 191}, {-158, 174}, {78, 174}, {78, -78}, {268, -78}, {268, 28}}, color = {0, 0, 127}));
    connect(inductor4.p, currentSensor5.p) annotation(
      Line(points = {{282, 60}, {278, 60}, {278, 36}, {278, 36}}, color = {0, 0, 255}));
    connect(currentSensor5.n, Two_v_circ.p) annotation(
      Line(points = {{278, 16}, {278, 16}, {278, 10}, {278, 10}}, color = {0, 0, 255}));
    connect(resistor4.p, inductor4.n) annotation(
      Line(points = {{310, 60}, {302, 60}}, color = {0, 0, 255}));
    connect(Two_v_circ.n, capacitor2.p) annotation(
      Line(points = {{278, -10}, {278, -52}, {352, -52}, {352, -30}}, color = {0, 0, 255}));
    connect(resistor4.n, capacitor1.p) annotation(
      Line(points = {{330, 60}, {350, 60}, {350, 30}}, color = {0, 0, 255}));
    connect(resistor4.n, currentSensor4.n) annotation(
      Line(points = {{330, 60}, {360, 60}}, color = {0, 0, 255}));
    connect(voltageSensor1.p, capacitor1.p) annotation(
      Line(points = {{320, 10}, {320, 10}, {320, 30}, {350, 30}, {350, 30}}, color = {0, 0, 255}));
    connect(voltageSensor1.n, capacitor2.p) annotation(
      Line(points = {{322, -10}, {322, -10}, {322, -30}, {352, -30}, {352, -30}}, color = {0, 0, 255}));
    connect(capacitor1.n, capacitor2.n) annotation(
      Line(points = {{352, 10}, {352, -10}}, color = {0, 0, 255}));
    connect(capacitor2.p, m_V_DC_2) annotation(
      Line(points = {{352, -30}, {352, -52}, {400, -52}}, color = {0, 0, 255}));
    connect(ground2.p, capacitor1.n) annotation(
      Line(points = {{354, 0}, {350, 0}, {350, 10}}, color = {0, 0, 255}));
    connect(gain12.u, currentSensor4.i) annotation(
      Line(points = {{20, 300}, {368, 300}, {368, 70}, {370, 70}}, color = {0, 0, 127}));
    connect(currentSensor4.p, V_DC_2) annotation(
      Line(points = {{380, 60}, {402, 60}, {402, 60}, {400, 60}}, color = {0, 0, 255}));
    connect(currentController1.v_d_conv_pu, gain16.u) annotation(
      Line(points = {{-104, 82}, {-100, 82}, {-100, 158}, {-168, 158}, {-168, 220}, {-142, 220}, {-142, 220}}, color = {0, 0, 127}));
    connect(gain16.y, energy_equation1.v_conv_d) annotation(
      Line(points = {{-134, 220}, {-118, 220}, {-118, 220}, {-114, 220}}, color = {0, 0, 127}));
    connect(CurrentTransform.d, gain15.u) annotation(
      Line(points = {{-218, 90}, {-190, 90}, {-190, 212}, {-142, 212}, {-142, 212}}, color = {0, 0, 127}));
    connect(gain15.y, energy_equation1.i_grid_d) annotation(
      Line(points = {{-134, 212}, {-116, 212}, {-116, 210}, {-114, 210}}, color = {0, 0, 127}));
    connect(gain13.y, energy_equation1.i_circ_z) annotation(
      Line(points = {{-130.5, 191}, {-120.5, 191}, {-120.5, 192}, {-114, 192}}, color = {0, 0, 127}));
    connect(energy_equation1.v_circ_z, gain18.y) annotation(
      Line(points = {{-114, 202}, {-132, 202}, {-132, 202}, {-132, 202}}, color = {0, 0, 127}));
    connect(energy_equation1.w_sum_z, gain17.u) annotation(
      Line(points = {{-10, 206}, {8, 206}, {8, 206}, {8, 206}}, color = {0, 0, 127}));
    connect(voltageController1.V_DC_ref, V_DC_ref.y) annotation(
      Line(points = {{-18, 246}, {1, 246}}, color = {0, 0, 127}));
    connect(voltageController1.I_DC_pu, gain12.y) annotation(
      Line(points = {{-24, 300}, {-14, 300}, {-14, 298}, {-3, 298}}, color = {0, 0, 127}));
    connect(gain19.u, voltageController1.i_d_ref_pu) annotation(
      Line(points = {{-184, 106}, {-188, 106}, {-188, 272}, {-92, 272}, {-92, 274}}, color = {0, 0, 127}));
    connect(voltageController1.v_grid_d, VoltageTransform.d) annotation(
      Line(points = {{-72, 312}, {-74, 312}, {-74, 322}, {-200, 322}, {-200, 66}, {-218, 66}, {-218, 66}}, color = {0, 0, 127}));
    connect(currentController1.i_d_ref, gain19.y) annotation(
      Line(points = {{-174, 106}, {-180, 106}, {-180, 106}, {-180, 106}}, color = {0, 0, 127}));
    connect(converterVoltage_c.n, ground1.p) annotation(
      Line(points = {{28, -20}, {40, -20}, {40, 0}}, color = {0, 0, 255}));
    connect(converterVoltage_b.n, ground1.p) annotation(
      Line(points = {{8, 0}, {40, 0}}, color = {0, 0, 255}));
    connect(converterVoltage_a.n, ground1.p) annotation(
      Line(points = {{-12, 20}, {40, 20}, {40, 0}}, color = {0, 0, 255}));
    connect(gain6.y, converterVoltage_b.v) annotation(
      Line(points = {{-27.6, 92}, {-4, 92}, {-4, 7}}, color = {0, 0, 127}));
    connect(inductor2.n, converterVoltage_b.p) annotation(
      Line(points = {{-182, 0}, {-14, 0}}, color = {0, 0, 255}));
    connect(gain9.u, potentialSensor1.phi) annotation(
      Line(points = {{-266.8, 62}, {-277.8, 62}, {-277.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.a, gain9.y) annotation(
      Line(points = {{-240, 62}, {-256, 62}}, color = {0, 0, 127}));
    connect(gain1.u, potentialSensor2.phi) annotation(
      Line(points = {{-266.8, 70}, {-297.8, 70}, {-297.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.b, gain1.y) annotation(
      Line(points = {{-242, 70}, {-258, 70}}, color = {0, 0, 127}));
    connect(gain2.u, potentialSensor3.phi) annotation(
      Line(points = {{-266.8, 78}, {-317.8, 78}, {-317.8, 51}}, color = {0, 0, 127}));
    connect(gain2.y, VoltageTransform.c) annotation(
      Line(points = {{-257.6, 78}, {-241.6, 78}}, color = {0, 0, 127}));
    connect(gain8.u, currentSensor1.i) annotation(
      Line(points = {{-266.8, 86}, {-337.8, 86}, {-337.8, 30}}, color = {0, 0, 127}));
    connect(CurrentTransform.a, gain8.y) annotation(
      Line(points = {{-240, 86}, {-256, 86}}, color = {0, 0, 127}));
    connect(gain3.u, currentSensor2.i) annotation(
      Line(points = {{-266.8, 94}, {-357.8, 94}, {-357.8, 10}}, color = {0, 0, 127}));
    connect(CurrentTransform.b, gain3.y) annotation(
      Line(points = {{-240, 94}, {-256, 94}}, color = {0, 0, 127}));
    connect(gain4.u, currentSensor3.i) annotation(
      Line(points = {{-266.8, 102}, {-377.8, 102}, {-377.8, -10}}, color = {0, 0, 127}));
    connect(CurrentTransform.c, gain4.y) annotation(
      Line(points = {{-240, 102}, {-256, 102}}, color = {0, 0, 127}));
    connect(gain7.y, converterVoltage_a.v) annotation(
      Line(points = {{-27.6, 76}, {-24, 76}, {-24, 27}}, color = {0, 0, 127}));
    connect(inductor1.n, converterVoltage_a.p) annotation(
      Line(points = {{-182, 20}, {-34, 20}}, color = {0, 0, 255}));
    connect(gain5.y, converterVoltage_c.v) annotation(
      Line(points = {{-27.6, 108}, {16, 108}, {16, -13}}, color = {0, 0, 127}));
    connect(inductor3.n, converterVoltage_c.p) annotation(
      Line(points = {{-182, -20}, {6, -20}}, color = {0, 0, 255}));
    connect(gain7.u, dQ_w_inv1.a) annotation(
      Line(points = {{-35, 76}, {-46, 76}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.b, gain6.u) annotation(
      Line(points = {{-46, 92}, {-35, 92}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.c, gain5.u) annotation(
      Line(points = {{-46, 108}, {-35, 108}}, color = {0, 0, 127}));
    connect(currentController1.v_q_conv_pu, dQ_w_inv1.q) annotation(
      Line(points = {{-105.21, 100.61}, {-81.21, 100.61}, {-81.21, 102}, {-92, 102}}, color = {0, 0, 127}));
    connect(currentController1.v_d_conv_pu, dQ_w_inv1.d) annotation(
      Line(points = {{-104.9, 81.7}, {-91.9, 81.7}, {-91.9, 82}, {-92, 82}}, color = {0, 0, 127}));
    connect(currentController1.i_q_ref, i_q_pu_ref.y) annotation(
      Line(points = {{-173.1, 115.8}, {-177.1, 115.8}}, color = {0, 0, 127}));
    connect(CurrentTransform.d, currentController1.i_d_mes_pu) annotation(
      Line(points = {{-216, 90}, {-172, 90}, {-172, 84}}, color = {0, 0, 127}));
    connect(CurrentTransform.q, currentController1.i_q_mes_pu) annotation(
      Line(points = {{-216, 100}, {-174, 100}, {-174, 98}, {-172, 98}}, color = {0, 0, 127}));
    connect(VoltageTransform.q, currentController1.v_q_grid_pu) annotation(
      Line(points = {{-216, 76}, {-172, 76}, {-172, 74}}, color = {0, 0, 127}));
    connect(VoltageTransform.d, currentController1.v_d_grid_pu) annotation(
      Line(points = {{-216, 66}, {-174, 66}, {-174, 64}, {-172, 64}}, color = {0, 0, 127}));
    connect(potentialSensor1.p, a) annotation(
      Line(points = {{-280, 30}, {-280, 20}, {-398, 20}}, color = {0, 0, 255}));
    connect(a, currentSensor1.n) annotation(
      Line(points = {{-398, 20}, {-349, 20}}, color = {0, 0, 255}));
    connect(resistor1.n, inductor1.p) annotation(
      Line(points = {{-218, 20}, {-198, 20}, {-198, 20}, {-198, 20}}, color = {0, 0, 255}));
    connect(currentSensor1.p, resistor1.p) annotation(
      Line(points = {{-330, 20}, {-241, 20}}, color = {0, 0, 255}));
    connect(potentialSensor2.p, b) annotation(
      Line(points = {{-298, 30}, {-298, 0}, {-396, 0}}, color = {0, 0, 255}));
    connect(potentialSensor3.p, c) annotation(
      Line(points = {{-318, 30}, {-318, -20}, {-396, -20}}, color = {0, 0, 255}));
    connect(b, currentSensor2.n) annotation(
      Line(points = {{-398, 0}, {-371, 0}}, color = {0, 0, 255}));
    connect(c, currentSensor3.n) annotation(
      Line(points = {{-398, -20}, {-391, -20}}, color = {0, 0, 255}));
    connect(resistor2.n, inductor2.p) annotation(
      Line(points = {{-218, 0}, {-198, 0}, {-198, 0}, {-198, 0}}, color = {0, 0, 255}));
    connect(currentSensor2.p, resistor2.p) annotation(
      Line(points = {{-350, 0}, {-239, 0}}, color = {0, 0, 255}));
    connect(resistor3.n, inductor3.p) annotation(
      Line(points = {{-218, -20}, {-198, -20}, {-198, -20}, {-198, -20}}, color = {0, 0, 255}));
    connect(currentSensor3.p, resistor3.p) annotation(
      Line(points = {{-370, -20}, {-239, -20}}, color = {0, 0, 255}));
    annotation(
      Diagram(coordinateSystem(extent = {{-400, -200}, {400, 400}})),
      Icon(coordinateSystem(extent = {{-400, -200}, {400, 400}}, initialScale = 0.1), graphics = {Rectangle(origin = {0, -1}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-400, 401}, {400, -201}}), Text(origin = {-44, 132}, fillPattern = FillPattern.Solid, extent = {{-236, 114}, {290, -166}}, textString = "MMC_V_l")}),
      __OpenModelica_commandLineOptions = "");
  end MMC_zero_sequence_V_lon;

  model EnergyReferenceController
    Real a = 1;
    Modelica.Blocks.Interfaces.RealInput P_in annotation(
      Placement(visible = true, transformation(origin = {-120, 8}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 8}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput w_sum_ref annotation(
      Placement(visible = true, transformation(origin = {110, -4}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant P_ref(k = S_base) annotation(
      Placement(visible = true, transformation(origin = {-106, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Division division1 annotation(
      Placement(visible = true, transformation(origin = {-74, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Division division2 annotation(
      Placement(visible = true, transformation(origin = {-46, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-30, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Add OutputOne annotation(
      Placement(visible = true, transformation(origin = {58, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.PI PI(T = K_i_E / K_p_E, k = K_p_E) annotation(
      Placement(visible = true, transformation(origin = {-4, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Nonlinear.Limiter limiter1(limitsAtInit = true, uMax = 0.3, uMin = -0.3) annotation(
      Placement(visible = true, transformation(origin = {26, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction1(a = {0.5, 1}, b = {1}) annotation(
      Placement(visible = true, transformation(origin = {92, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    extends SystemParameters;
  equation
    connect(P_in, division1.u1) annotation(
      Line(points = {{-120, 8}, {-86, 8}}, color = {0, 0, 127}));
    connect(transferFunction1.y, w_sum_ref) annotation(
      Line(points = {{103, -4}, {110, -4}}, color = {0, 0, 127}));
    connect(OutputOne.y, transferFunction1.u) annotation(
      Line(points = {{70, -4}, {80, -4}}, color = {0, 0, 127}));
    connect(PI.y, limiter1.u) annotation(
      Line(points = {{7, 2}, {13, 2}}, color = {0, 0, 127}));
    connect(limiter1.y, OutputOne.u1) annotation(
      Line(points = {{37, 2}, {45, 2}, {45, 2}, {45, 2}, {45, 2}, {45, 2}}, color = {0, 0, 127}));
    connect(feedback1.y, PI.u) annotation(
      Line(points = {{-21, 2}, {-16, 2}}, color = {0, 0, 127}));
    connect(division2.y, OutputOne.u2) annotation(
      Line(points = {{-35, -28}, {45, -28}, {45, -19}, {45, -19}, {45, -10}}, color = {0, 0, 127}));
    connect(division1.y, feedback1.u1) annotation(
      Line(points = {{-63, 2}, {-39, 2}}, color = {0, 0, 127}));
    connect(division2.y, feedback1.u2) annotation(
      Line(points = {{-35, -28}, {-31, -28}, {-31, -17}, {-31, -17}, {-31, -6}}, color = {0, 0, 127}));
    connect(P_ref.y, division2.u1) annotation(
      Line(points = {{-95, -28}, {-81, -28}, {-81, -22}, {-59, -22}, {-59, -22}, {-59, -22}, {-59, -22}}, color = {0, 0, 127}));
    connect(P_ref.y, division2.u2) annotation(
      Line(points = {{-95, -28}, {-81, -28}, {-81, -34}, {-59, -34}, {-59, -34}, {-59, -34}, {-59, -34}}, color = {0, 0, 127}));
    connect(P_ref.y, division1.u2) annotation(
      Line(points = {{-95, -28}, {-93, -28}, {-93, -5}, {-90, -5}, {-90, -5}, {-87, -5}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(origin = {0, -1}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 99}, {100, -99}})}));
  end EnergyReferenceController;

  model MMC_zero_sequence_P_controller
    Real P_AC_pu;
    Real P_refrence;
    Real Eng;
    Real Eng_tot(start = 0);
    MMC.CurrentController currentController1 annotation(
      Placement(visible = true, transformation(origin = {-139, 91}, extent = {{-31, 31}, {31, -31}}, rotation = 0)));
    MMC.Energy_equation energy_equation1 annotation(
      Placement(visible = true, transformation(origin = {-62, 206}, extent = {{-48, -48}, {48, 48}}, rotation = 0)));
    MMC.Energy_Controller energy_Controller1(u_circ_z_pu(fixed = true, start = 1)) annotation(
      Placement(visible = true, transformation(origin = {167, -15}, extent = {{-37, -37}, {37, 37}}, rotation = 0)));
    MMC.PowerController_two powerController_two1 annotation(
      Placement(visible = true, transformation(origin = {-8, 286}, extent = {{44, -44}, {-44, 44}}, rotation = 0)));
    MMC.EnergyReferenceController_two energyReferenceController_two1 annotation(
      Placement(visible = true, transformation(origin = {140, 56}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  protected
    extends SystemParameters;
    Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
      Placement(visible = true, transformation(origin = {410, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = C_DC * 2, v(fixed = true, start = V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {386, 26}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor2(C = C_DC * 2, v(fixed = true, start = -V_DC_base / 2)) annotation(
      Placement(visible = true, transformation(origin = {386, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Add add1 annotation(
      Placement(visible = true, transformation(origin = {140, 250}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin a annotation(
      Placement(visible = true, transformation(origin = {-400, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-402, 326}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin b annotation(
      Placement(visible = true, transformation(origin = {-400, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-398, 108}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin c annotation(
      Placement(visible = true, transformation(origin = {-400, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-392, -136}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R_v) annotation(
      Placement(visible = true, transformation(origin = {-230, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor2(R = R_v) annotation(
      Placement(visible = true, transformation(origin = {-230, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor3(R = R_v) annotation(
      Placement(visible = true, transformation(origin = {-230, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1 annotation(
      Placement(visible = true, transformation(origin = {-280, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor2 annotation(
      Placement(visible = true, transformation(origin = {-300, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor3 annotation(
      Placement(visible = true, transformation(origin = {-320, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {-340, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {-360, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {-380, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L_v) annotation(
      Placement(visible = true, transformation(origin = {-192, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor2(L = L_v) annotation(
      Placement(visible = true, transformation(origin = {-192, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor3(L = L_v) annotation(
      Placement(visible = true, transformation(origin = {-192, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.DQ_w VoltageTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w CurrentTransform annotation(
      Placement(visible = true, transformation(origin = {-230, 94}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.DQ_w_inv dQ_w_inv1 annotation(
      Placement(visible = true, transformation(origin = {-70, 92}, extent = {{-20, 20}, {20, -20}}, rotation = 0)));
    Modelica.Blocks.Sources.Step i_q_pu_ref(height = 0, offset = 0) annotation(
      Placement(visible = true, transformation(origin = {-180, 116}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 108}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 92}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain7(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-32, 76}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_a annotation(
      Placement(visible = true, transformation(origin = {-22, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_b annotation(
      Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage converterVoltage_c annotation(
      Placement(visible = true, transformation(origin = {18, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {50, 1}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain8(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 86}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain9(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 62}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 70}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 78}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 94}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-262, 102}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage Two_v_circ annotation(
      Placement(visible = true, transformation(origin = {278, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain10(k = 2 * V_base) annotation(
      Placement(visible = true, transformation(origin = {248, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor4(L = 2 / 3 * L_a) annotation(
      Placement(visible = true, transformation(origin = {292, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor4(R = 2 / 3 * R_a) annotation(
      Placement(visible = true, transformation(origin = {318, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {408, 190}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {400, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {400, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor1 annotation(
      Placement(visible = true, transformation(origin = {332, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain11(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {234, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Math.Gain gain12(k = 1 / I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {126, 323}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor4 annotation(
      Placement(visible = true, transformation(origin = {370, 60}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step P_ref(height = 0.1 * direction_V, offset = 0.7 * direction_V, startTime = 1) annotation(
      Placement(visible = true, transformation(origin = {175, 231}, extent = {{9, -9}, {-9, 9}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor5 annotation(
      Placement(visible = true, transformation(origin = {278, 28}, extent = {{10, 10}, {-10, -10}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain13(k = 1 / 3) annotation(
      Placement(visible = true, transformation(origin = {-137, 191}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain14(k = 1 / (3 * I_base)) annotation(
      Placement(visible = true, transformation(origin = {100, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain15(k = I_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 212}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain16(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-138, 220}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain17(k = 1 / w_base) annotation(
      Placement(visible = true, transformation(origin = {12, 206}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain18(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {-137, 202}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain19(k = 1) annotation(
      Placement(visible = true, transformation(origin = {-182, 106}, extent = {{-2, -2}, {2, 2}}, rotation = 0)));
    Modelica.Blocks.Sources.Step step1(height = 0.1 * direction_V, offset = 0, startTime = 1.1) annotation(
      Placement(visible = true, transformation(origin = {171, 269}, extent = {{9, -9}, {-9, 9}}, rotation = 0)));
  equation
    connect(powerController_two1.err, energyReferenceController_two1.P_in) annotation(
      Line(points = {{-12, 238}, {154, 238}, {154, 60}, {152, 60}}, color = {0, 0, 127}));
    connect(energyReferenceController_two1.w_sum_ref, energy_Controller1.w_sum_z_ref) annotation(
      Line(points = {{128, 56}, {110, 56}, {110, 10}, {122, 10}, {122, 14}}, color = {0, 0, 127}));
    connect(VoltageTransform.d, powerController_two1.v_grid_d) annotation(
      Line(points = {{-218, 66}, {-200, 66}, {-200, 382}, {-28, 382}, {-28, 338}, {-30, 338}}, color = {0, 0, 127}));
    connect(gain19.u, powerController_two1.i_d_ref_pu) annotation(
      Line(points = {{-184, 106}, {-186, 106}, {-186, 286}, {-56, 286}, {-56, 286}}, color = {0, 0, 127}));
    connect(gain11.y, powerController_two1.V_DC_mes_pu) annotation(
      Line(points = {{224, 44}, {210, 44}, {210, 286}, {40, 286}, {40, 286}}, color = {0, 0, 127}));
    connect(powerController_two1.P_ref, add1.y) annotation(
      Line(points = {{40, 250}, {130, 250}, {130, 250}, {130, 250}}, color = {0, 0, 127}));
    connect(powerController_two1.I_DC_pu, gain12.y) annotation(
      Line(points = {{40, 322}, {114, 322}, {114, 324}, {116, 324}}, color = {0, 0, 127}));
    connect(step1.y, add1.u1) annotation(
      Line(points = {{161, 269}, {152, 269}, {152, 256}}, color = {0, 0, 127}));
    connect(add1.u2, P_ref.y) annotation(
      Line(points = {{152, 244}, {165, 244}, {165, 231}}, color = {0, 0, 127}));
    connect(gain14.y, energy_Controller1.i_circ_z_pu) annotation(
      Line(points = {{112, -46}, {118, -46}, {118, -44}, {122, -44}}, color = {0, 0, 127}));
    P_AC_pu = -(converterVoltage_a.v * converterVoltage_a.i + converterVoltage_b.v * converterVoltage_b.i + converterVoltage_c.v * converterVoltage_c.i) / S_base;
    if time > 1.1 then
      P_refrence = 0.8977;
    elseif time > 1 then
      P_refrence = 0.79723;
    else
      P_refrence = 0.697676;
    end if;
    if time > 1 then
      Eng = P_refrence - P_AC_pu;
    else
      Eng = 0;
    end if;
    der(Eng_tot) = Eng;
    connect(resistor4.n, capacitor1.p) annotation(
      Line(points = {{328, 60}, {350, 60}, {350, 38}, {386, 38}, {386, 36}}, color = {0, 0, 255}));
    connect(Two_v_circ.n, m_V_DC_2) annotation(
      Line(points = {{278, -10}, {278, -76}, {400, -76}}, color = {0, 0, 255}));
    connect(capacitor2.p, m_V_DC_2) annotation(
      Line(points = {{388, -40}, {388, -76}, {400, -76}}, color = {0, 0, 255}));
    connect(capacitor1.n, ground2.p) annotation(
      Line(points = {{386, 16}, {386, 0}, {400, 0}}, color = {0, 0, 255}));
    connect(capacitor1.n, capacitor2.n) annotation(
      Line(points = {{386, 16}, {386, 16}, {386, -20}, {386, -20}}, color = {0, 0, 255}));
    connect(Two_v_circ.n, voltageSensor1.n) annotation(
      Line(points = {{278, -10}, {278, -76}, {334, -76}, {334, -10}}, color = {0, 0, 255}));
    connect(voltageSensor1.p, resistor4.n) annotation(
      Line(points = {{334, 10}, {334, 60}, {328, 60}}, color = {0, 0, 255}));
    connect(gain11.u, voltageSensor1.v) annotation(
      Line(points = {{246, 42}, {324, 42}, {324, 0}}, color = {0, 0, 127}));
    connect(currentController1.i_d_ref, gain19.y) annotation(
      Line(points = {{-174, 106}, {-180, 106}}, color = {0, 0, 127}));
    connect(gain12.u, currentSensor4.i) annotation(
      Line(points = {{136, 323}, {368, 323}, {368, 70}, {370, 70}}, color = {0, 0, 127}));
    connect(converterVoltage_c.n, ground1.p) annotation(
      Line(points = {{28, -20}, {40, -20}, {40, 0}}, color = {0, 0, 255}));
    connect(converterVoltage_b.n, ground1.p) annotation(
      Line(points = {{8, 0}, {40, 0}}, color = {0, 0, 255}));
    connect(converterVoltage_a.n, ground1.p) annotation(
      Line(points = {{-12, 20}, {40, 20}, {40, 0}}, color = {0, 0, 255}));
    connect(currentController1.v_d_conv_pu, gain16.u) annotation(
      Line(points = {{-104, 82}, {-100, 82}, {-100, 158}, {-168, 158}, {-168, 220}, {-142, 220}, {-142, 220}}, color = {0, 0, 127}));
    connect(gain6.y, converterVoltage_b.v) annotation(
      Line(points = {{-27.6, 92}, {-4, 92}, {-4, 7}}, color = {0, 0, 127}));
    connect(inductor2.n, converterVoltage_b.p) annotation(
      Line(points = {{-182, 0}, {-14, 0}}, color = {0, 0, 255}));
    connect(gain18.u, energy_Controller1.u_circ_z_pu) annotation(
      Line(points = {{-142, 202}, {-162, 202}, {-162, 166}, {70, 166}, {70, -64}, {224, -64}, {224, 0}, {212, 0}, {212, 0}}, color = {0, 0, 127}));
    connect(gain13.u, currentSensor5.i) annotation(
      Line(points = {{-142, 191}, {-158, 191}, {-158, 174}, {78, 174}, {78, -78}, {268, -78}, {268, 28}}, color = {0, 0, 127}));
    connect(gain13.y, energy_equation1.i_circ_z) annotation(
      Line(points = {{-130.5, 191}, {-120.5, 191}, {-120.5, 192}, {-114, 192}}, color = {0, 0, 127}));
    connect(energy_equation1.v_circ_z, gain18.y) annotation(
      Line(points = {{-114, 202}, {-132, 202}, {-132, 202}, {-132, 202}}, color = {0, 0, 127}));
    connect(gain17.y, energy_Controller1.w_sum_z_pu) annotation(
      Line(points = {{16, 206}, {96, 206}, {96, -22}, {122, -22}, {122, -22}}, color = {0, 0, 127}));
    connect(energy_equation1.w_sum_z, gain17.u) annotation(
      Line(points = {{-10, 206}, {8, 206}, {8, 206}, {8, 206}}, color = {0, 0, 127}));
    connect(CurrentTransform.d, gain15.u) annotation(
      Line(points = {{-218, 90}, {-190, 90}, {-190, 212}, {-142, 212}, {-142, 212}}, color = {0, 0, 127}));
    connect(gain15.y, energy_equation1.i_grid_d) annotation(
      Line(points = {{-134, 212}, {-116, 212}, {-116, 210}, {-114, 210}}, color = {0, 0, 127}));
    connect(gain16.y, energy_equation1.v_conv_d) annotation(
      Line(points = {{-134, 220}, {-118, 220}, {-118, 220}, {-114, 220}}, color = {0, 0, 127}));
    connect(gain14.u, currentSensor5.i) annotation(
      Line(points = {{88, -46}, {78, -46}, {78, -78}, {268, -78}, {268, 28}, {268, 28}}, color = {0, 0, 127}));
    connect(gain9.u, potentialSensor1.phi) annotation(
      Line(points = {{-266.8, 62}, {-277.8, 62}, {-277.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.a, gain9.y) annotation(
      Line(points = {{-240, 62}, {-256, 62}}, color = {0, 0, 127}));
    connect(gain1.u, potentialSensor2.phi) annotation(
      Line(points = {{-266.8, 70}, {-297.8, 70}, {-297.8, 51}}, color = {0, 0, 127}));
    connect(VoltageTransform.b, gain1.y) annotation(
      Line(points = {{-242, 70}, {-258, 70}}, color = {0, 0, 127}));
    connect(gain2.u, potentialSensor3.phi) annotation(
      Line(points = {{-266.8, 78}, {-317.8, 78}, {-317.8, 51}}, color = {0, 0, 127}));
    connect(gain2.y, VoltageTransform.c) annotation(
      Line(points = {{-257.6, 78}, {-241.6, 78}}, color = {0, 0, 127}));
    connect(gain8.u, currentSensor1.i) annotation(
      Line(points = {{-266.8, 86}, {-337.8, 86}, {-337.8, 30}}, color = {0, 0, 127}));
    connect(CurrentTransform.a, gain8.y) annotation(
      Line(points = {{-240, 86}, {-256, 86}}, color = {0, 0, 127}));
    connect(gain3.u, currentSensor2.i) annotation(
      Line(points = {{-266.8, 94}, {-357.8, 94}, {-357.8, 10}}, color = {0, 0, 127}));
    connect(CurrentTransform.b, gain3.y) annotation(
      Line(points = {{-240, 94}, {-256, 94}}, color = {0, 0, 127}));
    connect(gain4.u, currentSensor3.i) annotation(
      Line(points = {{-266.8, 102}, {-377.8, 102}, {-377.8, -10}}, color = {0, 0, 127}));
    connect(CurrentTransform.c, gain4.y) annotation(
      Line(points = {{-240, 102}, {-256, 102}}, color = {0, 0, 127}));
    connect(inductor4.p, currentSensor5.p) annotation(
      Line(points = {{282, 60}, {278, 60}, {278, 36}, {278, 36}}, color = {0, 0, 255}));
    connect(currentSensor5.n, Two_v_circ.p) annotation(
      Line(points = {{278, 16}, {278, 16}, {278, 10}, {278, 10}}, color = {0, 0, 255}));
    connect(energy_Controller1.u_circ_z_pu, gain10.u) annotation(
      Line(points = {{211.4, -0.2}, {226.4, -0.2}, {226.4, 0}, {238, 0}}, color = {0, 0, 127}));
    connect(gain10.y, Two_v_circ.v) annotation(
      Line(points = {{261, 0}, {269, 0}}, color = {0, 0, 127}));
    connect(gain11.y, energy_Controller1.v_DC_pu) annotation(
      Line(points = {{223, 42}, {106, 42}, {106, -4}, {122, -4}}, color = {0, 0, 127}));
    connect(resistor4.p, inductor4.n) annotation(
      Line(points = {{310, 60}, {302, 60}}, color = {0, 0, 255}));
    connect(resistor4.n, currentSensor4.n) annotation(
      Line(points = {{330, 60}, {360, 60}}, color = {0, 0, 255}));
    connect(currentSensor4.p, V_DC_2) annotation(
      Line(points = {{380, 60}, {402, 60}, {402, 60}, {400, 60}}, color = {0, 0, 255}));
    connect(gain7.y, converterVoltage_a.v) annotation(
      Line(points = {{-27.6, 76}, {-24, 76}, {-24, 27}}, color = {0, 0, 127}));
    connect(inductor1.n, converterVoltage_a.p) annotation(
      Line(points = {{-182, 20}, {-34, 20}}, color = {0, 0, 255}));
    connect(gain5.y, converterVoltage_c.v) annotation(
      Line(points = {{-27.6, 108}, {16, 108}, {16, -13}}, color = {0, 0, 127}));
    connect(inductor3.n, converterVoltage_c.p) annotation(
      Line(points = {{-182, -20}, {6, -20}}, color = {0, 0, 255}));
    connect(gain7.u, dQ_w_inv1.a) annotation(
      Line(points = {{-35, 76}, {-46, 76}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.b, gain6.u) annotation(
      Line(points = {{-46, 92}, {-35, 92}}, color = {0, 0, 127}));
    connect(dQ_w_inv1.c, gain5.u) annotation(
      Line(points = {{-46, 108}, {-35, 108}}, color = {0, 0, 127}));
    connect(currentController1.v_q_conv_pu, dQ_w_inv1.q) annotation(
      Line(points = {{-105.21, 100.61}, {-81.21, 100.61}, {-81.21, 102}, {-92, 102}}, color = {0, 0, 127}));
    connect(currentController1.v_d_conv_pu, dQ_w_inv1.d) annotation(
      Line(points = {{-104.9, 81.7}, {-91.9, 81.7}, {-91.9, 82}, {-92, 82}}, color = {0, 0, 127}));
    connect(currentController1.i_q_ref, i_q_pu_ref.y) annotation(
      Line(points = {{-173.1, 115.8}, {-177.1, 115.8}}, color = {0, 0, 127}));
    connect(CurrentTransform.d, currentController1.i_d_mes_pu) annotation(
      Line(points = {{-216, 90}, {-172, 90}, {-172, 84}}, color = {0, 0, 127}));
    connect(CurrentTransform.q, currentController1.i_q_mes_pu) annotation(
      Line(points = {{-216, 100}, {-174, 100}, {-174, 98}, {-172, 98}}, color = {0, 0, 127}));
    connect(VoltageTransform.q, currentController1.v_q_grid_pu) annotation(
      Line(points = {{-216, 76}, {-172, 76}, {-172, 74}}, color = {0, 0, 127}));
    connect(VoltageTransform.d, currentController1.v_d_grid_pu) annotation(
      Line(points = {{-216, 66}, {-174, 66}, {-174, 64}, {-172, 64}}, color = {0, 0, 127}));
    connect(potentialSensor1.p, a) annotation(
      Line(points = {{-280, 30}, {-280, 20}, {-398, 20}}, color = {0, 0, 255}));
    connect(a, currentSensor1.n) annotation(
      Line(points = {{-398, 20}, {-349, 20}}, color = {0, 0, 255}));
    connect(resistor1.n, inductor1.p) annotation(
      Line(points = {{-218, 20}, {-198, 20}, {-198, 20}, {-198, 20}}, color = {0, 0, 255}));
    connect(currentSensor1.p, resistor1.p) annotation(
      Line(points = {{-330, 20}, {-241, 20}}, color = {0, 0, 255}));
    connect(potentialSensor2.p, b) annotation(
      Line(points = {{-298, 30}, {-298, 0}, {-396, 0}}, color = {0, 0, 255}));
    connect(potentialSensor3.p, c) annotation(
      Line(points = {{-318, 30}, {-318, -20}, {-396, -20}}, color = {0, 0, 255}));
    connect(b, currentSensor2.n) annotation(
      Line(points = {{-398, 0}, {-371, 0}}, color = {0, 0, 255}));
    connect(c, currentSensor3.n) annotation(
      Line(points = {{-398, -20}, {-391, -20}}, color = {0, 0, 255}));
    connect(resistor2.n, inductor2.p) annotation(
      Line(points = {{-218, 0}, {-198, 0}, {-198, 0}, {-198, 0}}, color = {0, 0, 255}));
    connect(currentSensor2.p, resistor2.p) annotation(
      Line(points = {{-350, 0}, {-239, 0}}, color = {0, 0, 255}));
    connect(resistor3.n, inductor3.p) annotation(
      Line(points = {{-218, -20}, {-198, -20}, {-198, -20}, {-198, -20}}, color = {0, 0, 255}));
    connect(currentSensor3.p, resistor3.p) annotation(
      Line(points = {{-370, -20}, {-239, -20}}, color = {0, 0, 255}));
    annotation(
      Diagram(coordinateSystem(extent = {{-400, -200}, {400, 400}})),
      Icon(coordinateSystem(extent = {{-400, -200}, {400, 400}}, initialScale = 0.1), graphics = {Rectangle(origin = {0, -1}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-400, 401}, {400, -201}}), Text(origin = {-44, 132}, fillPattern = FillPattern.Solid, extent = {{-236, 114}, {290, -166}}, textString = "MMC_P_C")}),
      __OpenModelica_commandLineOptions = "");
  end MMC_zero_sequence_P_controller;

  model EnergyReferenceController_two
    parameter Real K_p_ec = -0.02;
    parameter Real K_i_ec = -35;
    Real a = 1;
    Modelica.Blocks.Interfaces.RealInput P_in annotation(
      Placement(visible = true, transformation(origin = {-120, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput w_sum_ref annotation(
      Placement(visible = true, transformation(origin = {92, -2}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const(k = 1) annotation(
      Placement(visible = true, transformation(origin = {0, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Add OutputOne annotation(
      Placement(visible = true, transformation(origin = {38, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.Integrator integrator1(k = -50) annotation(
      Placement(visible = true, transformation(origin = {-74, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction1(a = {1, 0}, b = {K_p_ec, K_i_ec}) annotation(
      Placement(visible = true, transformation(origin = {-50, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Logical.Switch switch1 annotation(
      Placement(visible = true, transformation(origin = {-16, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Ramp ramp1(duration = 2, height = 2) annotation(
      Placement(visible = true, transformation(origin = {-92, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold1(threshold = 1.05) annotation(
      Placement(visible = true, transformation(origin = {-64, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    extends SystemParameters;
  equation
    connect(ramp1.y, greaterEqualThreshold1.u) annotation(
      Line(points = {{-81, 0}, {-72.5, 0}, {-72.5, -2}, {-76, -2}}, color = {0, 0, 127}));
    connect(greaterEqualThreshold1.y, switch1.u2) annotation(
      Line(points = {{-53, -2}, {-28, -2}, {-28, 0}}, color = {255, 0, 255}));
    connect(OutputOne.y, w_sum_ref) annotation(
      Line(points = {{50, -8}, {94, -8}, {94, -2}, {92, -2}}, color = {0, 0, 127}));
    connect(OutputOne.u1, switch1.y) annotation(
      Line(points = {{26, -2}, {-6, -2}, {-6, 0}, {-4, 0}}, color = {0, 0, 127}));
    connect(transferFunction1.y, switch1.u3) annotation(
      Line(points = {{-38, -32}, {-36, -32}, {-36, -8}, {-28, -8}, {-28, -8}}, color = {0, 0, 127}));
    connect(integrator1.y, switch1.u1) annotation(
      Line(points = {{-62, 28}, {-44, 28}, {-44, 8}, {-28, 8}, {-28, 8}}, color = {0, 0, 127}));
    connect(P_in, transferFunction1.u) annotation(
      Line(points = {{-120, 30}, {-106, 30}, {-106, -32}, {-62, -32}, {-62, -32}}, color = {0, 0, 127}));
    connect(P_in, integrator1.u) annotation(
      Line(points = {{-120, 30}, {-103, 30}, {-103, 28}, {-86, 28}}, color = {0, 0, 127}));
    connect(const.y, OutputOne.u2) annotation(
      Line(points = {{11, -38}, {26, -38}, {26, -14}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(origin = {0, -1}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 99}, {100, -99}})}));
  end EnergyReferenceController_two;

  model PowerController_two
    Modelica.Blocks.Math.MultiProduct P_pu(nu = 2) annotation(
      Placement(visible = true, transformation(origin = {-82, 20}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
    Modelica.Blocks.Interfaces.RealInput P_ref annotation(
      Placement(visible = true, transformation(origin = {-122, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-111, -81}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput V_DC_mes_pu annotation(
      Placement(visible = true, transformation(origin = {-122, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -4.44089e-16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput I_DC_pu annotation(
      Placement(visible = true, transformation(origin = {-124, 88}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_grid_d annotation(
      Placement(visible = true, transformation(origin = {-113, 53}, extent = {{-11, -11}, {11, 11}}, rotation = 0), iconTransformation(origin = {50, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Interfaces.RealOutput err annotation(
      Placement(visible = true, transformation(origin = {102, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {10, -110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  protected
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-82, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction1(a = {1, 0}, b = {K_vcp, K_vci * 10}) annotation(
      Placement(visible = true, transformation(origin = {-44, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback2 annotation(
      Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    MMC.ff_voltage_controller ff_voltage_controller1 annotation(
      Placement(visible = true, transformation(origin = {-12, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput i_d_ref_pu annotation(
      Placement(visible = true, transformation(origin = {106, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = feed) annotation(
      Placement(visible = true, transformation(origin = {-12, 24}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    extends SystemParameters;
  equation
    connect(P_ref, feedback1.u1) annotation(
      Line(points = {{-122, 0}, {-90, 0}}, color = {0, 0, 127}));
    connect(feedback1.y, err) annotation(
      Line(points = {{-72, 0}, {-62, 0}, {-62, -40}, {102, -40}, {102, -40}}, color = {0, 0, 127}));
    connect(ff_voltage_controller1.V_d, v_grid_d) annotation(
      Line(points = {{-24, 48}, {-106, 48}, {-106, 54}, {-112, 54}}, color = {0, 0, 127}));
    connect(I_DC_pu, P_pu.u[2]) annotation(
      Line(points = {{-124, 88}, {-82, 88}, {-82, 26}, {-82, 26}}, color = {0, 0, 127}));
    connect(P_pu.y, feedback1.u2) annotation(
      Line(points = {{-82, 12}, {-82, 12}, {-82, 8}, {-82, 8}}, color = {0, 0, 127}));
    connect(V_DC_mes_pu, P_pu.u[1]) annotation(
      Line(points = {{-122, 30}, {-84, 30}, {-84, 26}, {-82, 26}}, color = {0, 0, 127}));
    connect(gain1.y, feedback2.u2) annotation(
      Line(points = {{-12, 12}, {-8, 12}, {-8, 8}, {-10, 8}}, color = {0, 0, 127}));
    connect(ff_voltage_controller1.ff, gain1.u) annotation(
      Line(points = {{-12, 42}, {-12, 42}, {-12, 36}, {-12, 36}}, color = {0, 0, 127}));
    connect(transferFunction1.y, feedback2.u1) annotation(
      Line(points = {{-31, 0}, {-18, 0}}, color = {0, 0, 127}));
    connect(feedback1.y, transferFunction1.u) annotation(
      Line(points = {{-72, 0}, {-54, 0}}, color = {0, 0, 127}));
    connect(I_DC_pu, ff_voltage_controller1.i_DC) annotation(
      Line(points = {{-124, 88}, {8, 88}, {8, 52}, {0, 52}}, color = {0, 0, 127}));
    connect(ff_voltage_controller1.V_DC, V_DC_mes_pu) annotation(
      Line(points = {{-11, 63}, {-11, 78}, {-70, 78}, {-70, 30}, {-120, 30}}, color = {0, 0, 127}));
    connect(feedback2.y, i_d_ref_pu) annotation(
      Line(points = {{0, 0}, {100, 0}, {100, 2}, {106, 2}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, 0}, fillPattern = FillPattern.Solid, extent = {{-99, 100}, {99, -100}}, textString = "PowerController")}, coordinateSystem(initialScale = 0.1)));
  end PowerController_two;

  model MMC_Energy_controller_test
    Grid grid1 annotation(
      Placement(visible = true, transformation(origin = {-96, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.MMC_zero_sequence_V mMC_zero_sequence_V1 annotation(
      Placement(visible = true, transformation(origin = {-65.2478, 47.2478}, extent = {{-18.7522, -9.37608}, {18.7522, 18.7522}}, rotation = 0)));
    MMC.MMC_zero_sequence_V mMC_zero_sequence_V2 annotation(
      Placement(visible = true, transformation(origin = {-59.2183, -16.7817}, extent = {{-20.7817, -10.3908}, {20.7817, 20.7817}}, rotation = 0)));
    MMC.Grid grid2 annotation(
      Placement(visible = true, transformation(origin = {-96, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.Grid grid3 annotation(
      Placement(visible = true, transformation(origin = {130, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Cable.HVDC_Cable hVDC_Cable1 annotation(
      Placement(visible = true, transformation(origin = {18, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Cable.HVDC_Cable hVDC_Cable2(l = 20) annotation(
      Placement(visible = true, transformation(origin = {-20, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.MMC_zero_sequence_P_controller mMC_zero_sequence_P_controller1 annotation(
      Placement(visible = true, transformation(origin = {64, 52}, extent = {{16, -8}, {-16, 16}}, rotation = 0)));
    MMC.Grid grid4 annotation(
      Placement(visible = true, transformation(origin = {-96, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.Grid grid5 annotation(
      Placement(visible = true, transformation(origin = {-96, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.MMC_zero_sequence_V mMC_zero_sequence_V3 annotation(
      Placement(visible = true, transformation(origin = {-63.6913, -53.6206}, extent = {{-16.3087, -8.15434}, {16.3087, 16.3087}}, rotation = 0)));
    Cable.HVDC_Cable hVDC_Cable3 annotation(
      Placement(visible = true, transformation(origin = {20, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.MMC_zero_sequence_V mMC_zero_sequence_V4 annotation(
      Placement(visible = true, transformation(origin = {-65.6913, -87.6206}, extent = {{-16.3087, -8.15434}, {16.3087, 16.3087}}, rotation = 0)));
    Cable.HVDC_Cable hVDC_Cable4 annotation(
      Placement(visible = true, transformation(origin = {-28, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    MMC.MMC_zero_sequence_P mMC_zero_sequence_P1 annotation(
      Placement(visible = true, transformation(origin = {60.2201, -52.2201}, extent = {{14.2201, -7.11007}, {-14.2201, 14.2201}}, rotation = 0)));
    MMC.Grid grid6 annotation(
      Placement(visible = true, transformation(origin = {94, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    extends SystemParameters;
  equation
    connect(mMC_zero_sequence_V1.V_DC_2, hVDC_Cable1.V_DC_2_in) annotation(
      Line(points = {{-46, 51}, {8, 51}, {8, 54}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V1.m_V_DC_2, hVDC_Cable1.m_V_DC_2_in) annotation(
      Line(points = {{-46, 39}, {-5, 39}, {-5, 42}, {8, 42}}, color = {0, 0, 255}));
    connect(grid1.A, mMC_zero_sequence_V1.a) annotation(
      Line(points = {{-96, 56}, {-96, 57}, {-84, 57}}, color = {0, 0, 255}));
    connect(grid1.B, mMC_zero_sequence_V1.b) annotation(
      Line(points = {{-96, 50}, {-90, 50}, {-90, 47}, {-84, 47}}, color = {0, 0, 255}));
    connect(grid1.C, mMC_zero_sequence_V1.c) annotation(
      Line(points = {{-96, 44}, {-90, 44}, {-90, 36}, {-84, 36}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_P1.c, grid6.C) annotation(
      Line(points = {{74, -64}, {94, -64}, {94, -60}, {94, -60}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_P1.b, grid6.B) annotation(
      Line(points = {{74, -54}, {94, -54}, {94, -54}, {94, -54}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_P1.a, grid6.A) annotation(
      Line(points = {{74, -48}, {92, -48}, {92, -48}, {94, -48}}, color = {0, 0, 255}));
    connect(hVDC_Cable3.m_V_DC_2_out, mMC_zero_sequence_P1.m_V_DC_2) annotation(
      Line(points = {{30, -62}, {46, -62}, {46, -58}, {46, -58}}, color = {0, 0, 255}));
    connect(hVDC_Cable3.V_DC_2_out, mMC_zero_sequence_P1.V_DC_2) annotation(
      Line(points = {{30, -50}, {46, -50}, {46, -52}, {46, -52}}, color = {0, 0, 255}));
    connect(hVDC_Cable4.m_V_DC_2_out, hVDC_Cable3.m_V_DC_2_in) annotation(
      Line(points = {{-18, -98}, {4, -98}, {4, -62}, {10, -62}, {10, -62}}, color = {0, 0, 255}));
    connect(hVDC_Cable4.V_DC_2_out, hVDC_Cable3.V_DC_2_in) annotation(
      Line(points = {{-18, -86}, {-6, -86}, {-6, -50}, {10, -50}, {10, -50}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V4.m_V_DC_2, hVDC_Cable4.m_V_DC_2_in) annotation(
      Line(points = {{-50, -96}, {-38, -96}, {-38, -98}, {-38, -98}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V4.V_DC_2, hVDC_Cable4.V_DC_2_in) annotation(
      Line(points = {{-50, -86}, {-38, -86}, {-38, -86}, {-38, -86}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V3.m_V_DC_2, hVDC_Cable3.m_V_DC_2_in) annotation(
      Line(points = {{-48, -62}, {10, -62}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V3.V_DC_2, hVDC_Cable3.V_DC_2_in) annotation(
      Line(points = {{-48, -52}, {10, -52}, {10, -50}}, color = {0, 0, 255}));
    connect(grid5.C, mMC_zero_sequence_V4.c) annotation(
      Line(points = {{-96, -96}, {-82, -96}, {-82, -100}, {-82, -100}}, color = {0, 0, 255}));
    connect(grid5.B, mMC_zero_sequence_V4.b) annotation(
      Line(points = {{-96, -90}, {-82, -90}, {-82, -90}, {-82, -90}}, color = {0, 0, 255}));
    connect(grid5.A, mMC_zero_sequence_V4.a) annotation(
      Line(points = {{-96, -84}, {-82, -84}, {-82, -80}, {-82, -80}}, color = {0, 0, 255}));
    connect(grid4.C, mMC_zero_sequence_V3.c) annotation(
      Line(points = {{-96, -62}, {-80, -62}, {-80, -66}, {-80, -66}}, color = {0, 0, 255}));
    connect(grid4.B, mMC_zero_sequence_V3.b) annotation(
      Line(points = {{-96, -56}, {-80, -56}, {-80, -56}, {-80, -56}}, color = {0, 0, 255}));
    connect(grid4.A, mMC_zero_sequence_V3.a) annotation(
      Line(points = {{-96, -50}, {-80, -50}, {-80, -46}, {-80, -46}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_P_controller1.c, grid3.C) annotation(
      Line(points = {{80, 40}, {130, 40}, {130, 42}, {130, 42}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_P_controller1.b, grid3.B) annotation(
      Line(points = {{80, 50}, {130, 50}, {130, 48}, {130, 48}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_P_controller1.a, grid3.A) annotation(
      Line(points = {{80, 60}, {130, 60}, {130, 54}, {130, 54}}, color = {0, 0, 255}));
    connect(hVDC_Cable1.m_V_DC_2_out, mMC_zero_sequence_P_controller1.m_V_DC_2) annotation(
      Line(points = {{28, 42}, {46, 42}, {46, 46}, {48, 46}, {48, 46}}, color = {0, 0, 255}));
    connect(hVDC_Cable1.V_DC_2_out, mMC_zero_sequence_P_controller1.V_DC_2) annotation(
      Line(points = {{28, 54}, {48, 54}, {48, 54}, {48, 54}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V2.a, grid2.A) annotation(
      Line(points = {{-80, -5}, {-96, -5}, {-96, -12}}, color = {0, 0, 255}));
    connect(grid2.B, mMC_zero_sequence_V2.b) annotation(
      Line(points = {{-96, -18}, {-90, -18}, {-90, -16}, {-80, -16}}, color = {0, 0, 255}));
    connect(grid2.C, mMC_zero_sequence_V2.c) annotation(
      Line(points = {{-96, -24}, {-90, -24}, {-90, -29}, {-80, -29}}, color = {0, 0, 255}));
    connect(hVDC_Cable2.m_V_DC_2_out, hVDC_Cable1.m_V_DC_2_in) annotation(
      Line(points = {{-10, -24}, {4, -24}, {4, 42}, {8, 42}, {8, 42}}, color = {0, 0, 255}));
    connect(hVDC_Cable2.V_DC_2_out, hVDC_Cable1.V_DC_2_in) annotation(
      Line(points = {{-10, -12}, {-6, -12}, {-6, 54}, {8, 54}, {8, 54}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V2.V_DC_2, hVDC_Cable2.V_DC_2_in) annotation(
      Line(points = {{-38, -12}, {-30, -12}}, color = {0, 0, 255}));
    connect(mMC_zero_sequence_V2.m_V_DC_2, hVDC_Cable2.m_V_DC_2_in) annotation(
      Line(points = {{-38, -24}, {-30, -24}}, color = {0, 0, 255}));
  end MMC_Energy_controller_test;
  annotation(
    uses(Modelica(version = "3.2.2")));
end MMC;
