package TwoLevelVSC
  model SystemParameters
    constant Real pi = 2 * Modelica.Math.asin(1.0);
    parameter Real S_base = 20000;
    parameter Real V_base = 400;
    parameter Real L_pu = 0.15;
    parameter Real C_pu = 0.88 / 1.015;
    //division by 4 to eliminate the possibillity of V_DC = 0 during startup.
    parameter Real R_pu = 0.01;
    parameter Real V_DC_step = 0.1 * V_DC_base;
    parameter Real V_DC_step_time = 0.4;
    parameter Real P_step_time = 10;
    parameter Real tau_PWM = 1 / 10000;
    parameter Real i_q_reference = 0;
    parameter Real direction_V = 1;
    //protected
    parameter Real direction_P = direction_V * (-1);
    parameter Real I_base = 2 * S_base / (3 * V_base);
    parameter Real omega_base = 50 * 2 * pi;
    parameter Real Z_base = V_base / I_base;
    parameter Real R = R_pu * Z_base;
    parameter Real L = L_pu * Z_base / omega_base;
    parameter Real V_DC_base = V_base * 2;
    parameter Real I_DC_base = S_base / V_DC_base;
    parameter Real Z_DC_base = V_DC_base / I_DC_base;
    parameter Real C = 1 / (C_pu * omega_base * Z_DC_base);
    parameter Real i_load = I_DC_base;
    parameter Real V_DC_start = V_DC_base;
    //Tuning
    parameter Real K_i = R_pu / tau_eq;
    parameter Real K_p = L_pu / (omega_base * tau_eq);
    parameter Real tau_eq = tau_PWM * f;
    parameter Real f = 6;
    parameter Real tau = (3 + 2 * sqrt(2)) * tau_eq;
    parameter Real K_CRCP = 1 / (C_pu * omega_base * sqrt(tau * tau_eq));
    parameter Real K_CRCI = K_CRCP / tau;
  end SystemParameters;

  model TwoLVSC_avg_V
    extends SystemParameters;
    constant Real pi = 2 * Modelica.Math.asin(1.0);
    constant Real phi1 = pi / 2;
    constant Real phi2 = (-2 * pi / 3) + phi1;
    constant Real phi3 = (-4 * pi / 3) + phi1;
    parameter Real i_q_ref = 0;
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1 annotation(
      Placement(visible = true, transformation(origin = {-66, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor2 annotation(
      Placement(visible = true, transformation(origin = {-72, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor3 annotation(
      Placement(visible = true, transformation(origin = {-78, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {-85, -4}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {-91, -10}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {-96, -16}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    TwoLevelVSC.dq_Transform Voltagedq_Transformer annotation(
      Placement(visible = true, transformation(origin = {-48, 14}, extent = {{-4, 4}, {4, -4}}, rotation = 0)));
    TwoLevelVSC.dq_Transform dq_Transform1 annotation(
      Placement(visible = true, transformation(origin = {-52, 26}, extent = {{-4, 4}, {4, -4}}, rotation = 0)));
    TwoLevelVSC.DCref dCref1 annotation(
      Placement(visible = true, transformation(origin = {-27, 79}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    TwoLevelVSC.ff ff1 annotation(
      Placement(visible = true, transformation(origin = {-27, 66}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor4 annotation(
      Placement(visible = true, transformation(origin = {38, 74}, extent = {{-3, 3}, {3, -3}}, rotation = 180)));
    TwoLevelVSC.CurrentController CurrentController1 annotation(
      Placement(visible = true, transformation(origin = {-13, 26}, extent = {{-15, 15}, {15, -15}}, rotation = 0)));
    Modelica.Blocks.Sources.Step iq_reference(height = 0, offset = 0, startTime = 0) annotation(
      Placement(visible = true, transformation(origin = {-35, 39}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-27, 50}, extent = {{5, 5}, {-5, -5}}, rotation = 0)));
    TwoLevelVSC.Inverse_dq_Transform Inverse_dq_Transform1 annotation(
      Placement(visible = true, transformation(origin = {23, 27}, extent = {{-5, 5}, {5, -5}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor2(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor3(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L, i(fixed = false, start = I_base)) annotation(
      Placement(visible = true, transformation(origin = {-12, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor2(L = L, i(fixed = false, start = I_base)) annotation(
      Placement(visible = true, transformation(origin = {-12, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor3(L = L, i(fixed = false, start = I_base)) annotation(
      Placement(visible = true, transformation(origin = {-12, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage conv_a annotation(
      Placement(visible = true, transformation(origin = {32, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage conv_b annotation(
      Placement(visible = true, transformation(origin = {37, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage conv_c annotation(
      Placement(visible = true, transformation(origin = {42, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {52, -10}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
    Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent1 annotation(
      Placement(visible = true, transformation(origin = {64, -10}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = 2 * C, v(fixed = true, start = V_DC_start / 2)) annotation(
      Placement(visible = true, transformation(origin = {79, -7}, extent = {{-7, -7}, {7, 7}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
      Placement(visible = true, transformation(origin = {92, -15}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor4 annotation(
      Placement(visible = true, transformation(origin = {86, 8}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Blocks.Sources.Step step1(height = V_DC_step, offset = V_DC_start * 1.2, startTime = 0.4) annotation(
      Placement(visible = true, transformation(origin = {66, 50}, extent = {{-4, -4}, {4, 4}}, rotation = 180)));
    Modelica.Blocks.Continuous.TransferFunction CRC(a = {1, 0}, b = {K_CRCP, K_CRCI}) annotation(
      Placement(visible = true, transformation(origin = {0, 50}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback2 annotation(
      Placement(visible = true, transformation(origin = {23, 50}, extent = {{7, 7}, {-7, -7}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseA annotation(
      Placement(visible = true, transformation(origin = {-102, -4}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, 71}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseB annotation(
      Placement(visible = true, transformation(origin = {-102, -10}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, -1}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseC annotation(
      Placement(visible = true, transformation(origin = {-102, -16}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, -69}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {101, 9}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {101, 41}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {100, -38}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {101, -41}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Blocks.Math.Gain PU(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 17}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 14}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 11}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 29}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 26}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 23}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {32, 7}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain7(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {37, 7}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain8(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {42, 7}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain9(k = 1 / I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {86, 32}, extent = {{-1, -1}, {1, 1}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain10(k = 2 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {30, 74}, extent = {{-1, -1}, {1, 1}}, rotation = 180)));
    Modelica.Blocks.Math.Gain gain11(k = I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {56, 23}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain12(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {47, 50}, extent = {{-1, -1}, {1, 1}}, rotation = 180)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor2(C = 2 * C, v(fixed = true, start = -V_DC_start / 2)) annotation(
      Placement(visible = true, transformation(origin = {79, -26}, extent = {{-7, -7}, {7, 7}}, rotation = 90)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction1(a = {1.5 / 10000, 1}, b = {1}) annotation(
      Placement(visible = true, transformation(origin = {32, 17}, extent = {{1, -1}, {-1, 1}}, rotation = 90)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction2(a = {1.5 / 10000, 1}, b = {1}) annotation(
      Placement(visible = true, transformation(origin = {37, 22}, extent = {{1, -1}, {-1, 1}}, rotation = 90)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction3(a = {1.5 / 10000, 1}, b = {1}) annotation(
      Placement(visible = true, transformation(origin = {42, 27}, extent = {{1, -1}, {-1, 1}}, rotation = 90)));
  equation
    connect(Inverse_dq_Transform1.c, transferFunction3.u) annotation(
      Line(points = {{29, 31}, {42, 31}, {42, 28}}, color = {0, 0, 127}));
    connect(transferFunction3.y, gain8.u) annotation(
      Line(points = {{42, 26}, {42, 8}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.b, transferFunction2.u) annotation(
      Line(points = {{29, 27}, {37, 27}, {37, 23}, {37, 23}}, color = {0, 0, 127}));
    connect(transferFunction2.y, gain7.u) annotation(
      Line(points = {{37, 21}, {37, 21}, {37, 8}, {37, 8}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.a, transferFunction1.u) annotation(
      Line(points = {{29, 23}, {32, 23}, {32, 18}}, color = {0, 0, 127}));
    connect(transferFunction1.y, gain6.u) annotation(
      Line(points = {{32, 16}, {32, 8}}, color = {0, 0, 127}));
    connect(gain3.y, dq_Transform1.c) annotation(
      Line(points = {{-60, 29}, {-57, 29}}, color = {0, 0, 127}));
    connect(currentSensor3.i, gain3.u) annotation(
      Line(points = {{-96, -13}, {-96, 29}, {-62, 29}}, color = {0, 0, 127}));
    connect(potentialSensor3.phi, PU.u) annotation(
      Line(points = {{-78, 10}, {-78, 17}, {-62, 17}}, color = {0, 0, 127}));
    connect(PU.y, Voltagedq_Transformer.c) annotation(
      Line(points = {{-60, 17}, {-53, 17}}, color = {0, 0, 127}));
    connect(iq_reference.y, CurrentController1.i_q_ref) annotation(
      Line(points = {{-34, 39}, {-31.9, 39}}, color = {0, 0, 127}));
    connect(gain12.u, step1.y) annotation(
      Line(points = {{48, 51}, {55, 51}, {55, 50}, {62, 50}}, color = {0, 0, 127}));
    connect(feedback2.u1, gain12.y) annotation(
      Line(points = {{29, 50}, {37.5, 50}, {37.5, 51}, {46, 51}}, color = {0, 0, 127}));
    connect(potentialSensor4.p, capacitor1.p) annotation(
      Line(points = {{41, 74}, {79, 74}, {79, 0}}, color = {0, 0, 255}));
    connect(capacitor1.p, currentSensor4.p) annotation(
      Line(points = {{79, 0}, {79, 8}, {84, 8}}, color = {0, 0, 255}));
    connect(ground2.p, capacitor1.n) annotation(
      Line(points = {{88, -15}, {79, -15}, {79, -14}}, color = {0, 0, 255}));
    connect(signalCurrent1.n, capacitor1.p) annotation(
      Line(points = {{64, -4}, {64, 8}, {79, 8}, {79, 0}}, color = {0, 0, 255}));
    connect(gain9.u, currentSensor4.i) annotation(
      Line(points = {{86, 31}, {86, 11}}, color = {0, 0, 127}));
    connect(gain9.y, ff1.i_DC) annotation(
      Line(points = {{86, 33}, {86, 66}, {-23, 66}}, color = {0, 0, 127}));
    connect(inductor1.n, conv_a.p) annotation(
      Line(points = {{-9, -4}, {27, -4}}, color = {0, 0, 255}));
    connect(resistor1.n, inductor1.p) annotation(
      Line(points = {{-27, -4}, {-15, -4}}, color = {0, 0, 255}));
    connect(gain10.u, potentialSensor4.phi) annotation(
      Line(points = {{31, 74}, {35, 74}}, color = {0, 0, 127}));
    connect(feedback2.u2, gain10.y) annotation(
      Line(points = {{23, 56}, {23, 74}, {29, 74}}, color = {0, 0, 127}));
    connect(ff1.V_DC, gain10.y) annotation(
      Line(points = {{-27, 70}, {-27, 74}, {29, 74}}, color = {0, 0, 127}));
    connect(dCref1.V_DC, gain10.y) annotation(
      Line(points = {{-26, 75}, {-26, 74}, {29, 74}}, color = {0, 0, 127}));
    connect(signalCurrent1.p, capacitor2.p) annotation(
      Line(points = {{64, -16}, {64, -16}, {64, -33}, {79, -33}, {79, -33}}, color = {0, 0, 255}));
    connect(capacitor2.p, m_V_DC_2) annotation(
      Line(points = {{79, -33}, {79, -33}, {79, -37}, {100, -37}, {100, -38}}, color = {0, 0, 255}));
    connect(capacitor2.n, ground2.p) annotation(
      Line(points = {{79, -19}, {79, -19}, {79, -15}, {88, -15}, {88, -15}}, color = {0, 0, 255}));
    connect(dCref1.y, gain11.u) annotation(
      Line(points = {{-22, 79}, {56, 79}, {56, 24}}, color = {0, 0, 127}));
    connect(dCref1.i_d, dq_Transform1.d) annotation(
      Line(points = {{-31, 79}, {-43.64, 79}, {-43.64, 24.16}, {-49.64, 24.16}}, color = {0, 0, 127}));
    connect(dCref1.V_d, Voltagedq_Transformer.d) annotation(
      Line(points = {{-26, 84}, {-26, 83.64}, {-41.92, 83.64}, {-41.92, 11.64}, {-45.92, 11.64}}, color = {0, 0, 127}));
    connect(gain5.y, dq_Transform1.a) annotation(
      Line(points = {{-60, 23}, {-57, 23}}, color = {0, 0, 127}));
    connect(currentSensor1.i, gain5.u) annotation(
      Line(points = {{-85, -1}, {-85, 23}, {-62, 23}}, color = {0, 0, 127}));
    connect(signalCurrent1.i, gain11.y) annotation(
      Line(points = {{60, -10}, {56, -10}, {56, 22}, {56, 22}, {56, 22}}, color = {0, 0, 127}));
    connect(CRC.u, feedback2.y) annotation(
      Line(points = {{5.2, 50}, {18, 50}}, color = {0, 0, 127}));
    connect(gain8.y, conv_c.v) annotation(
      Line(points = {{42, 6}, {42, 6}, {42, -14}, {42, -14}}, color = {0, 0, 127}));
    connect(gain7.y, conv_b.v) annotation(
      Line(points = {{37, 6}, {37, 6}, {37, -8}, {37, -8}}, color = {0, 0, 127}));
    connect(conv_a.v, gain6.y) annotation(
      Line(points = {{32, -2}, {32, -2}, {32, 6}, {32, 6}}, color = {0, 0, 127}));
    connect(currentSensor2.i, gain4.u) annotation(
      Line(points = {{-91, -7}, {-91, -7}, {-91, 26}, {-62, 26}, {-62, 26}}, color = {0, 0, 127}));
    connect(gain4.y, dq_Transform1.b) annotation(
      Line(points = {{-60, 26}, {-57, 26}, {-57, 26}, {-57, 26}}, color = {0, 0, 127}));
    connect(gain2.y, Voltagedq_Transformer.a) annotation(
      Line(points = {{-60, 11}, {-53, 11}, {-53, 11}, {-53, 11}}, color = {0, 0, 127}));
    connect(gain1.y, Voltagedq_Transformer.b) annotation(
      Line(points = {{-60, 14}, {-53, 14}, {-53, 14}, {-53, 14}}, color = {0, 0, 127}));
    connect(potentialSensor2.phi, gain1.u) annotation(
      Line(points = {{-72, 10}, {-72, 10}, {-72, 14}, {-62, 14}, {-62, 14}}, color = {0, 0, 127}));
    connect(gain2.u, potentialSensor1.phi) annotation(
      Line(points = {{-62, 11}, {-66, 11}, {-66, 10}, {-66, 10}, {-66, 10}}, color = {0, 0, 127}));
    connect(potentialSensor1.p, currentSensor1.n) annotation(
      Line(points = {{-66, 3}, {-66, -4}, {-82, -4}}, color = {0, 0, 255}));
    connect(potentialSensor2.p, currentSensor2.n) annotation(
      Line(points = {{-68, 4}, {-68, -10}, {-88, -10}}, color = {0, 0, 255}));
    connect(potentialSensor3.p, currentSensor3.n) annotation(
      Line(points = {{-74, 4}, {-74, -16}, {-93, -16}}, color = {0, 0, 255}));
    connect(currentSensor1.n, resistor1.p) annotation(
      Line(points = {{-81, -4}, {-33, -4}}, color = {0, 0, 255}));
    connect(PhaseA, currentSensor1.p) annotation(
      Line(points = {{-102, -4}, {-87, -4}}, color = {0, 0, 255}));
    connect(currentSensor2.n, resistor2.p) annotation(
      Line(points = {{-87, -10}, {-33, -10}}, color = {0, 0, 255}));
    connect(PhaseB, currentSensor2.p) annotation(
      Line(points = {{-102, -10}, {-93, -10}}, color = {0, 0, 255}));
    connect(currentSensor3.n, resistor3.p) annotation(
      Line(points = {{-92, -16}, {-33, -16}}, color = {0, 0, 255}));
    connect(PhaseC, currentSensor3.p) annotation(
      Line(points = {{-102, -16}, {-98, -16}}, color = {0, 0, 255}));
    connect(feedback1.y, CurrentController1.i_d_ref) annotation(
      Line(points = {{-31.5, 50}, {-40, 50}, {-40, 36}, {-32, 36}}, color = {0, 0, 127}));
    connect(feedback1.u1, CRC.y) annotation(
      Line(points = {{-23, 50}, {-9, 50}}, color = {0, 0, 127}));
    connect(ff1.ff, feedback1.u2) annotation(
      Line(points = {{-27, 62}, {-27, 58}, {-27.32, 58}, {-27.32, 54}}, color = {0, 0, 127}));
    connect(ff1.V_d, Voltagedq_Transformer.d) annotation(
      Line(points = {{-31, 66}, {-41.72, 66}, {-41.72, 12}, {-45.72, 12}}, color = {0, 0, 127}));
    connect(currentSensor4.n, V_DC_2) annotation(
      Line(points = {{90, 8}, {102, 8}, {102, 10}, {102, 10}}, color = {0, 0, 255}));
    connect(CurrentController1.i_d_mes, dq_Transform1.d) annotation(
      Line(points = {{-29.5, 23.6}, {-47.5, 23.6}, {-47.5, 23.6}, {-47.5, 23.6}}, color = {0, 0, 127}));
    connect(dq_Transform1.q, CurrentController1.i_q_mes) annotation(
      Line(points = {{-49.2, 28.4}, {-31.2, 28.4}, {-31.2, 28.4}, {-31.2, 28.4}}, color = {0, 0, 127}));
    connect(CurrentController1.v_DCd_d, Inverse_dq_Transform1.d) annotation(
      Line(points = {{7.1, 19.1}, {15.1, 19.1}, {15.1, 23.1}, {17.1, 23.1}}, color = {0, 0, 127}));
    connect(CurrentController1.v_DCd_q, Inverse_dq_Transform1.q) annotation(
      Line(points = {{6.5, 33.5}, {16.5, 33.5}, {16.5, 29.5}, {18.5, 29.5}}, color = {0, 0, 127}));
    connect(Voltagedq_Transformer.q, CurrentController1.v_q_conv) annotation(
      Line(points = {{-45.2, 16.4}, {-31.2, 16.4}, {-31.2, 16.4}, {-31.2, 16.4}}, color = {0, 0, 127}));
    connect(Voltagedq_Transformer.d, CurrentController1.v_d_conv) annotation(
      Line(points = {{-45.2, 12.4}, {-31.2, 12.4}, {-31.2, 12.4}, {-31.2, 12.4}}, color = {0, 0, 127}));
    connect(resistor2.n, inductor2.p) annotation(
      Line(points = {{-27, -10}, {-17, -10}}, color = {0, 0, 255}));
    connect(resistor3.n, inductor3.p) annotation(
      Line(points = {{-27, -16}, {-17, -16}}, color = {0, 0, 255}));
    connect(inductor2.n, conv_b.p) annotation(
      Line(points = {{-11, -10}, {32, -10}}, color = {0, 0, 255}));
    connect(inductor3.n, conv_c.p) annotation(
      Line(points = {{-11, -16}, {37, -16}}, color = {0, 0, 255}));
    connect(conv_a.n, ground1.p) annotation(
      Line(points = {{33, -4}, {45, -4}, {45, -10}, {45, -10}}, color = {0, 0, 255}));
    connect(conv_b.n, ground1.p) annotation(
      Line(points = {{38, -10}, {50, -10}}, color = {0, 0, 255}));
    connect(conv_c.n, ground1.p) annotation(
      Line(points = {{43, -16}, {45, -16}, {45, -10}, {45, -10}}, color = {0, 0, 255}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-2, 4}, fillPattern = FillPattern.Solid, extent = {{-74, 74}, {74, -74}}, textString = "2LVSC_avg_V")}, coordinateSystem(grid = {1, 1})),
      Diagram(coordinateSystem(grid = {1, 1})),
      __OpenModelica_commandLineOptions = "");
  end TwoLVSC_avg_V;



  model TwoLVSC_avg_P
    extends SystemParameters;
    constant Real pi = 2 * Modelica.Math.asin(1.0);
    constant Real phi1 = pi / 2;
    constant Real phi2 = (-2 * pi / 3) + phi1;
    constant Real phi3 = (-4 * pi / 3) + phi1;
    parameter Real i_q_ref = 0;
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1 annotation(
      Placement(visible = true, transformation(origin = {-66, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor2 annotation(
      Placement(visible = true, transformation(origin = {-72, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor3 annotation(
      Placement(visible = true, transformation(origin = {-78, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {-85, -4}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {-91, -10}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {-96, -16}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    TwoLevelVSC.dq_Transform Voltagedq_Transformer annotation(
      Placement(visible = true, transformation(origin = {-48, 14}, extent = {{-4, 4}, {4, -4}}, rotation = 0)));
    TwoLevelVSC.dq_Transform dq_Transform1 annotation(
      Placement(visible = true, transformation(origin = {-52, 26}, extent = {{-4, 4}, {4, -4}}, rotation = 0)));
    TwoLevelVSC.DCref dCref1 annotation(
      Placement(visible = true, transformation(origin = {-27, 79}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    TwoLevelVSC.ff ff1 annotation(
      Placement(visible = true, transformation(origin = {-27, 66}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    TwoLevelVSC.CurrentController CurrentController1 annotation(
      Placement(visible = true, transformation(origin = {-13, 26}, extent = {{-15, 15}, {15, -15}}, rotation = 0)));
    Modelica.Blocks.Sources.Step iq_reference(height = i_q_reference, startTime = 0.3) annotation(
      Placement(visible = true, transformation(origin = {-35, 39}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-27, 50}, extent = {{5, 5}, {-5, -5}}, rotation = 0)));
    TwoLevelVSC.Inverse_dq_Transform Inverse_dq_Transform1 annotation(
      Placement(visible = true, transformation(origin = {23, 27}, extent = {{-5, 5}, {5, -5}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor2(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor3(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L) annotation(
      Placement(visible = true, transformation(origin = {-12, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor2(L = L) annotation(
      Placement(visible = true, transformation(origin = {-12, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor3(L = L) annotation(
      Placement(visible = true, transformation(origin = {-12, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage conv_a annotation(
      Placement(visible = true, transformation(origin = {32, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage conv_b annotation(
      Placement(visible = true, transformation(origin = {37, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage conv_c annotation(
      Placement(visible = true, transformation(origin = {42, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {52, -10}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
    Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent1 annotation(
      Placement(visible = true, transformation(origin = {64, -10}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor4 annotation(
      Placement(visible = true, transformation(origin = {86, 8}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Blocks.Sources.Step step1(height = S_base * 0.1 * direction_P, offset = S_base * direction_P, startTime = P_step_time) annotation(
      Placement(visible = true, transformation(origin = {43, 50}, extent = {{-4, -4}, {4, 4}}, rotation = 180)));
    Modelica.Blocks.Continuous.TransferFunction CRC(a = {1, 0}, b = {K_CRCP, K_CRCI}) annotation(
      Placement(visible = true, transformation(origin = {0, 50}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback2 annotation(
      Placement(visible = true, transformation(origin = {23, 50}, extent = {{7, 7}, {-7, -7}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseA annotation(
      Placement(visible = true, transformation(origin = {-102, -4}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, 71}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseB annotation(
      Placement(visible = true, transformation(origin = {-102, -10}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, -1}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseC annotation(
      Placement(visible = true, transformation(origin = {-102, -16}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, -69}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {101, 9}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {101, 41}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {99, -24}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {101, -41}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Blocks.Math.Gain PU(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 17}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 14}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 11}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 29}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 26}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 23}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {32, 7}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain7(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {37, 7}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain8(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {42, 7}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain9(k = 1 / I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {86, 32}, extent = {{-1, -1}, {1, 1}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain10(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {30, 74}, extent = {{-1, -1}, {1, 1}}, rotation = 180)));
    Modelica.Blocks.Math.Gain gain11(k = I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {56, 23}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain12(k = 1 / S_base) annotation(
      Placement(visible = true, transformation(origin = {33, 50}, extent = {{-1, -1}, {1, 1}}, rotation = 180)));
    //The minus has to do with the direction of the power flow. In this model, positive power flow is from the DC side to the AC side.
    Modelica.Blocks.Math.MultiProduct multiProduct1(nu = 2) annotation(
      Placement(visible = true, transformation(origin = {22.5, 62.5}, extent = {{-1.5, -1.5}, {1.5, 1.5}}, rotation = -90)));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor1 annotation(
      Placement(visible = true, transformation(origin = {76, -8}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
  equation
    connect(gain12.u, step1.y) annotation(
      Line(points = {{34, 50}, {39, 50}}, color = {0, 0, 127}));
    connect(feedback2.u1, gain12.y) annotation(
      Line(points = {{29, 50}, {32, 50}}, color = {0, 0, 127}));
    connect(gain9.y, multiProduct1.u[2]) annotation(
      Line(points = {{86, 33}, {86, 66}, {22, 66}, {22, 64}, {23, 64}}, color = {0, 0, 127}));
    connect(gain9.y, ff1.i_DC) annotation(
      Line(points = {{86, 33}, {86, 66}, {-23, 66}}, color = {0, 0, 127}));
    connect(gain9.u, currentSensor4.i) annotation(
      Line(points = {{86, 31}, {86, 11}}, color = {0, 0, 127}));
    connect(signalCurrent1.i, gain11.y) annotation(
      Line(points = {{60, -10}, {56, -10}, {56, 22}}, color = {0, 0, 127}));
    connect(dCref1.y, gain11.u) annotation(
      Line(points = {{-22, 79}, {56, 79}, {56, 24}}, color = {0, 0, 127}));
    connect(gain10.u, voltageSensor1.v) annotation(
      Line(points = {{31, 74}, {72, 74}, {72, -8}, {72, -8}}, color = {0, 0, 127}));
    connect(voltageSensor1.n, m_V_DC_2) annotation(
      Line(points = {{76, -12}, {76, -12}, {76, -23}, {99, -23}, {99, -24}}, color = {0, 0, 255}));
    connect(voltageSensor1.p, currentSensor4.p) annotation(
      Line(points = {{76, -4}, {76, -4}, {76, 8}, {83, 8}, {83, 8}}, color = {0, 0, 255}));
    connect(ff1.V_DC, gain10.y) annotation(
      Line(points = {{-27, 70}, {-27, 74}, {29, 74}}, color = {0, 0, 127}));
    connect(dCref1.V_DC, gain10.y) annotation(
      Line(points = {{-26, 75}, {-26, 74}, {29, 74}}, color = {0, 0, 127}));
    connect(gain10.y, multiProduct1.u[1]) annotation(
      Line(points = {{29, 74}, {23, 74}, {23, 64}}, color = {0, 0, 127}));
    connect(signalCurrent1.n, currentSensor4.p) annotation(
      Line(points = {{64, -4}, {65, -4}, {65, 8}, {83, 8}, {83, 8}}, color = {0, 0, 255}));
    connect(signalCurrent1.p, m_V_DC_2) annotation(
      Line(points = {{64, -16}, {64, -23}, {99, -23}}, color = {0, 0, 255}));
    connect(multiProduct1.y, feedback2.u2) annotation(
      Line(points = {{23, 61}, {23, 61}, {23, 56}, {23, 56}}, color = {0, 0, 127}));
    connect(dCref1.i_d, dq_Transform1.d) annotation(
      Line(points = {{-31, 79}, {-43.64, 79}, {-43.64, 24.16}, {-49.64, 24.16}}, color = {0, 0, 127}));
    connect(dCref1.V_d, Voltagedq_Transformer.d) annotation(
      Line(points = {{-26, 84}, {-26, 83.64}, {-41.92, 83.64}, {-41.92, 11.64}, {-45.92, 11.64}}, color = {0, 0, 127}));
    connect(gain5.y, dq_Transform1.a) annotation(
      Line(points = {{-60, 23}, {-57, 23}}, color = {0, 0, 127}));
    connect(currentSensor1.i, gain5.u) annotation(
      Line(points = {{-85, -1}, {-85, 23}, {-62, 23}}, color = {0, 0, 127}));
    connect(CRC.u, feedback2.y) annotation(
      Line(points = {{5.2, 50}, {18, 50}}, color = {0, 0, 127}));
    connect(gain8.y, conv_c.v) annotation(
      Line(points = {{42, 6}, {42, 6}, {42, -14}, {42, -14}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.c, gain8.u) annotation(
      Line(points = {{29, 31}, {42, 31}, {42, 8}, {42, 8}}, color = {0, 0, 127}));
    connect(gain7.y, conv_b.v) annotation(
      Line(points = {{37, 6}, {37, 6}, {37, -8}, {37, -8}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.b, gain7.u) annotation(
      Line(points = {{29, 27}, {37, 27}, {37, 8}, {37, 8}}, color = {0, 0, 127}));
    connect(conv_a.v, gain6.y) annotation(
      Line(points = {{32, -2}, {32, -2}, {32, 6}, {32, 6}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.a, gain6.u) annotation(
      Line(points = {{29, 23}, {32, 23}, {32, 8}, {32, 8}, {32, 8}}, color = {0, 0, 127}));
    connect(currentSensor3.i, gain3.u) annotation(
      Line(points = {{-96, -13}, {-96, -13}, {-96, 29}, {-62, 29}, {-62, 29}}, color = {0, 0, 127}));
    connect(currentSensor2.i, gain4.u) annotation(
      Line(points = {{-91, -7}, {-91, -7}, {-91, 26}, {-62, 26}, {-62, 26}}, color = {0, 0, 127}));
    connect(gain4.y, dq_Transform1.b) annotation(
      Line(points = {{-60, 26}, {-57, 26}, {-57, 26}, {-57, 26}}, color = {0, 0, 127}));
    connect(gain3.y, dq_Transform1.c) annotation(
      Line(points = {{-60, 29}, {-57, 29}, {-57, 29}, {-57, 29}}, color = {0, 0, 127}));
    connect(gain2.y, Voltagedq_Transformer.a) annotation(
      Line(points = {{-60, 11}, {-53, 11}, {-53, 11}, {-53, 11}}, color = {0, 0, 127}));
    connect(gain1.y, Voltagedq_Transformer.b) annotation(
      Line(points = {{-60, 14}, {-53, 14}, {-53, 14}, {-53, 14}}, color = {0, 0, 127}));
    connect(PU.y, Voltagedq_Transformer.c) annotation(
      Line(points = {{-60, 17}, {-53, 17}, {-53, 17}, {-53, 17}}, color = {0, 0, 127}));
    connect(potentialSensor3.phi, PU.u) annotation(
      Line(points = {{-78, 10}, {-78, 10}, {-78, 17}, {-62, 17}, {-62, 17}}, color = {0, 0, 127}));
    connect(potentialSensor2.phi, gain1.u) annotation(
      Line(points = {{-72, 10}, {-72, 10}, {-72, 14}, {-62, 14}, {-62, 14}}, color = {0, 0, 127}));
    connect(gain2.u, potentialSensor1.phi) annotation(
      Line(points = {{-62, 11}, {-66, 11}, {-66, 10}, {-66, 10}, {-66, 10}}, color = {0, 0, 127}));
    connect(potentialSensor1.p, currentSensor1.n) annotation(
      Line(points = {{-66, 3}, {-66, -4}, {-82, -4}}, color = {0, 0, 255}));
    connect(potentialSensor2.p, currentSensor2.n) annotation(
      Line(points = {{-68, 4}, {-68, -10}, {-88, -10}}, color = {0, 0, 255}));
    connect(potentialSensor3.p, currentSensor3.n) annotation(
      Line(points = {{-74, 4}, {-74, -16}, {-93, -16}}, color = {0, 0, 255}));
    connect(currentSensor1.n, resistor1.p) annotation(
      Line(points = {{-81, -4}, {-33, -4}}, color = {0, 0, 255}));
    connect(PhaseA, currentSensor1.p) annotation(
      Line(points = {{-102, -4}, {-87, -4}}, color = {0, 0, 255}));
    connect(currentSensor2.n, resistor2.p) annotation(
      Line(points = {{-87, -10}, {-33, -10}}, color = {0, 0, 255}));
    connect(PhaseB, currentSensor2.p) annotation(
      Line(points = {{-102, -10}, {-93, -10}}, color = {0, 0, 255}));
    connect(currentSensor3.n, resistor3.p) annotation(
      Line(points = {{-92, -16}, {-33, -16}}, color = {0, 0, 255}));
    connect(PhaseC, currentSensor3.p) annotation(
      Line(points = {{-102, -16}, {-98, -16}}, color = {0, 0, 255}));
    connect(iq_reference.y, CurrentController1.i_q_ref) annotation(
      Line(points = {{-34, 39}, {-31.9, 39}}, color = {0, 0, 127}));
    connect(feedback1.y, CurrentController1.i_d_ref) annotation(
      Line(points = {{-31.5, 50}, {-40, 50}, {-40, 36}, {-32, 36}}, color = {0, 0, 127}));
    connect(feedback1.u1, CRC.y) annotation(
      Line(points = {{-23, 50}, {-9, 50}}, color = {0, 0, 127}));
    connect(ff1.ff, feedback1.u2) annotation(
      Line(points = {{-27, 62}, {-27, 58}, {-27.32, 58}, {-27.32, 54}}, color = {0, 0, 127}));
    connect(ff1.V_d, Voltagedq_Transformer.d) annotation(
      Line(points = {{-31, 66}, {-41.72, 66}, {-41.72, 12}, {-45.72, 12}}, color = {0, 0, 127}));
    connect(currentSensor4.n, V_DC_2) annotation(
      Line(points = {{90, 8}, {102, 8}, {102, 10}, {102, 10}}, color = {0, 0, 255}));
    connect(CurrentController1.i_d_mes, dq_Transform1.d) annotation(
      Line(points = {{-29.5, 23.6}, {-47.5, 23.6}, {-47.5, 23.6}, {-47.5, 23.6}}, color = {0, 0, 127}));
    connect(dq_Transform1.q, CurrentController1.i_q_mes) annotation(
      Line(points = {{-49.2, 28.4}, {-31.2, 28.4}, {-31.2, 28.4}, {-31.2, 28.4}}, color = {0, 0, 127}));
    connect(CurrentController1.v_DCd_d, Inverse_dq_Transform1.d) annotation(
      Line(points = {{7.1, 19.1}, {15.1, 19.1}, {15.1, 23.1}, {17.1, 23.1}}, color = {0, 0, 127}));
    connect(CurrentController1.v_DCd_q, Inverse_dq_Transform1.q) annotation(
      Line(points = {{6.5, 33.5}, {16.5, 33.5}, {16.5, 29.5}, {18.5, 29.5}}, color = {0, 0, 127}));
    connect(Voltagedq_Transformer.q, CurrentController1.v_q_conv) annotation(
      Line(points = {{-45.2, 16.4}, {-31.2, 16.4}, {-31.2, 16.4}, {-31.2, 16.4}}, color = {0, 0, 127}));
    connect(Voltagedq_Transformer.d, CurrentController1.v_d_conv) annotation(
      Line(points = {{-45.2, 12.4}, {-31.2, 12.4}, {-31.2, 12.4}, {-31.2, 12.4}}, color = {0, 0, 127}));
    connect(resistor1.n, inductor1.p) annotation(
      Line(points = {{-27, -4}, {-17, -4}}, color = {0, 0, 255}));
    connect(resistor2.n, inductor2.p) annotation(
      Line(points = {{-27, -10}, {-17, -10}}, color = {0, 0, 255}));
    connect(resistor3.n, inductor3.p) annotation(
      Line(points = {{-27, -16}, {-17, -16}}, color = {0, 0, 255}));
    connect(inductor1.n, conv_a.p) annotation(
      Line(points = {{-11, -4}, {27, -4}}, color = {0, 0, 255}));
    connect(inductor2.n, conv_b.p) annotation(
      Line(points = {{-11, -10}, {32, -10}}, color = {0, 0, 255}));
    connect(inductor3.n, conv_c.p) annotation(
      Line(points = {{-11, -16}, {37, -16}}, color = {0, 0, 255}));
    connect(conv_a.n, ground1.p) annotation(
      Line(points = {{33, -4}, {45, -4}, {45, -10}, {45, -10}}, color = {0, 0, 255}));
    connect(conv_b.n, ground1.p) annotation(
      Line(points = {{38, -10}, {50, -10}}, color = {0, 0, 255}));
    connect(conv_c.n, ground1.p) annotation(
      Line(points = {{43, -16}, {45, -16}, {45, -10}, {45, -10}}, color = {0, 0, 255}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-2, 4}, fillPattern = FillPattern.Solid, extent = {{-74, 74}, {74, -74}}, textString = "2LVSC_avg_P")}, coordinateSystem(grid = {1, 1}, initialScale = 0.1)),
      Diagram(coordinateSystem(grid = {1, 1})),
      __OpenModelica_commandLineOptions = "");
  end TwoLVSC_avg_P;

  model TwoLVSC_switch_P
    extends SystemParameters;
    constant Real pi = 2 * Modelica.Math.asin(1.0);
    constant Real phi1 = pi / 2;
    constant Real phi2 = (-2 * pi / 3) + phi1;
    constant Real phi3 = (-4 * pi / 3) + phi1;
    parameter Real i_q_ref = 0;
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1 annotation(
      Placement(visible = true, transformation(origin = {-66, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor2 annotation(
      Placement(visible = true, transformation(origin = {-72, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor3 annotation(
      Placement(visible = true, transformation(origin = {-78, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {-85, -4}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {-91, -10}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {-96, -16}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    TwoLevelVSC.dq_Transform Voltagedq_Transformer annotation(
      Placement(visible = true, transformation(origin = {-48, 14}, extent = {{-4, 4}, {4, -4}}, rotation = 0)));
    TwoLevelVSC.dq_Transform dq_Transform1 annotation(
      Placement(visible = true, transformation(origin = {-52, 26}, extent = {{-4, 4}, {4, -4}}, rotation = 0)));
    TwoLevelVSC.ff ff1 annotation(
      Placement(visible = true, transformation(origin = {-27, 66}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    TwoLevelVSC.CurrentController CurrentController1 annotation(
      Placement(visible = true, transformation(origin = {-13, 26}, extent = {{-15, 15}, {15, -15}}, rotation = 0)));
    Modelica.Blocks.Sources.Step iq_reference(height = i_q_reference, startTime = 0.3) annotation(
      Placement(visible = true, transformation(origin = {-35, 39}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-27, 50}, extent = {{5, 5}, {-5, -5}}, rotation = 0)));
    TwoLevelVSC.Inverse_dq_Transform Inverse_dq_Transform1 annotation(
      Placement(visible = true, transformation(origin = {23, 27}, extent = {{-5, 5}, {5, -5}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor2(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor3(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L, i(fixed = false, start = I_base)) annotation(
      Placement(visible = true, transformation(origin = {-12, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor2(L = L, i(fixed = false, start = I_base)) annotation(
      Placement(visible = true, transformation(origin = {-12, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor3(L = L, i(fixed = false, start = I_base)) annotation(
      Placement(visible = true, transformation(origin = {-12, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor4 annotation(
      Placement(visible = true, transformation(origin = {86, 0}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Blocks.Sources.Step step1(height = S_base * 0.1 * direction_P, offset = S_base * direction_P, startTime = P_step_time) annotation(
      Placement(visible = true, transformation(origin = {47, 50}, extent = {{-4, -4}, {4, 4}}, rotation = 180)));
    Modelica.Blocks.Continuous.TransferFunction CRC(a = {1, 0}, b = {K_CRCP, K_CRCI}) annotation(
      Placement(visible = true, transformation(origin = {-1, 50}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback2 annotation(
      Placement(visible = true, transformation(origin = {23, 50}, extent = {{7, 7}, {-7, -7}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseA annotation(
      Placement(visible = true, transformation(origin = {-102, -4}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, 71}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseB annotation(
      Placement(visible = true, transformation(origin = {-102, -10}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, -1}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseC annotation(
      Placement(visible = true, transformation(origin = {-102, -16}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, -69}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {101, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {101, 41}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {101, -35}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {101, -41}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Blocks.Math.Gain PU(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 17}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 14}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 11}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 29}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 26}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 23}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = 1) annotation(
      Placement(visible = true, transformation(origin = {32, 20}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain7(k = 1) annotation(
      Placement(visible = true, transformation(origin = {37, 20}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain8(k = 1) annotation(
      Placement(visible = true, transformation(origin = {42, 20}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain9(k = 1 / I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {86, 32}, extent = {{-1, -1}, {1, 1}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain10(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {32, 74}, extent = {{-1, -1}, {1, 1}}, rotation = 180)));
    Modelica.Blocks.Math.Gain gain12(k = 1 / S_base) annotation(
      Placement(visible = true, transformation(origin = {34, 50}, extent = {{-1, -1}, {1, 1}}, rotation = 180)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made1 annotation(
      Placement(visible = true, transformation(origin = {34, -2}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made2 annotation(
      Placement(visible = true, transformation(origin = {39, -2}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made3 annotation(
      Placement(visible = true, transformation(origin = {44, -2}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made4 annotation(
      Placement(visible = true, transformation(origin = {39, -30}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made5 annotation(
      Placement(visible = true, transformation(origin = {34, -30}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made6 annotation(
      Placement(visible = true, transformation(origin = {44, -30}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.PWM_self_made_2 pWM_self_made_21 annotation(
      Placement(visible = true, transformation(origin = {37.5, 8.5}, extent = {{-6.5, -6.5}, {6.5, 6.5}}, rotation = -90)));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor1 annotation(
      Placement(visible = true, transformation(origin = {68.5, -20.5}, extent = {{-3.5, -3.5}, {3.5, 3.5}}, rotation = -90)));
    Modelica.Blocks.Math.MultiProduct multiProduct1(nu = 2) annotation(
      Placement(visible = true, transformation(origin = {23, 61}, extent = {{-3, -3}, {3, 3}}, rotation = -90)));
  equation
    connect(feedback2.u1, gain12.y) annotation(
      Line(points = {{29, 50}, {33, 50}}, color = {0, 0, 127}));
    connect(gain12.u, step1.y) annotation(
      Line(points = {{35, 50}, {43, 50}}, color = {0, 0, 127}));
    connect(gain9.y, ff1.i_DC) annotation(
      Line(points = {{86, 33}, {86, 66}, {-23, 66}}, color = {0, 0, 127}));
    connect(gain9.y, multiProduct1.u[2]) annotation(
      Line(points = {{86, 33}, {86, 66}, {24, 66}, {24, 64}, {23, 64}}, color = {0, 0, 127}));
    connect(gain9.u, currentSensor4.i) annotation(
      Line(points = {{86, 31}, {86, 3}}, color = {0, 0, 127}));
    connect(voltageSensor1.n, m_V_DC_2) annotation(
      Line(points = {{68.5, -24}, {68.5, -35}, {101, -35}}, color = {0, 0, 255}));
    connect(voltageSensor1.p, currentSensor4.p) annotation(
      Line(points = {{68.5, -17}, {65, -17}, {65, 0}, {83, 0}}, color = {0, 0, 255}));
    connect(voltageSensor1.v, gain10.u) annotation(
      Line(points = {{65, -20.5}, {56, -20.5}, {56, 74}, {33, 74}}, color = {0, 0, 127}));
    connect(iGBT_self_made5.pin, m_V_DC_2) annotation(
      Line(points = {{34, -31}, {34, -31}, {34, -35}, {101, -35}, {101, -35}}, color = {0, 0, 255}));
    connect(iGBT_self_made4.pin, m_V_DC_2) annotation(
      Line(points = {{39, -31}, {39, -31}, {39, -35}, {101, -35}, {101, -35}}, color = {0, 0, 255}));
    connect(iGBT_self_made6.pin, m_V_DC_2) annotation(
      Line(points = {{44, -31}, {44, -31}, {44, -35}, {101, -35}, {101, -35}}, color = {0, 0, 255}));
    connect(iGBT_self_made1.pin_n, currentSensor4.p) annotation(
      Line(points = {{34, -1}, {34, -1}, {34, 0}, {83, 0}, {83, 0}}, color = {0, 0, 255}));
    connect(iGBT_self_made2.pin_n, currentSensor4.p) annotation(
      Line(points = {{39, -1}, {39, -1}, {39, 0}, {83, 0}, {83, 0}}, color = {0, 0, 255}));
    connect(iGBT_self_made3.pin_n, currentSensor4.p) annotation(
      Line(points = {{44, -1}, {44, -1}, {44, 0}, {83, 0}, {83, 0}}, color = {0, 0, 255}));
    connect(currentSensor4.n, V_DC_2) annotation(
      Line(points = {{89, 0}, {101, 0}}, color = {0, 0, 255}));
    connect(gain10.y, multiProduct1.u[1]) annotation(
      Line(points = {{31, 74}, {22, 74}, {22, 64}, {23, 64}}, color = {0, 0, 127}));
    connect(multiProduct1.y, feedback2.u2) annotation(
      Line(points = {{23, 57}, {23, 57}, {23, 56}, {23, 56}}, color = {0, 0, 127}));
    connect(gain10.y, ff1.V_DC) annotation(
      Line(points = {{31, 74}, {-27, 74}, {-27, 70}}, color = {0, 0, 127}));
    connect(iq_reference.y, CurrentController1.i_q_ref) annotation(
      Line(points = {{-34, 39}, {-31.9, 39}}, color = {0, 0, 127}));
    connect(CRC.u, feedback2.y) annotation(
      Line(points = {{6, 50}, {18, 50}}, color = {0, 0, 127}));
    connect(feedback1.u1, CRC.y) annotation(
      Line(points = {{-23, 50}, {-8, 50}}, color = {0, 0, 127}));
    connect(pWM_self_made_21.a_not_fire, iGBT_self_made5.fire) annotation(
      Line(points = {{32, 1}, {32, 1}, {32, -30}, {33, -30}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.a_fire, iGBT_self_made1.fire) annotation(
      Line(points = {{33, 1}, {32, 1}, {32, -2}, {33, -2}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.b_not_fire, iGBT_self_made4.fire) annotation(
      Line(points = {{37, 1}, {38, 1}, {38, -30}, {38, -30}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.b_fire, iGBT_self_made2.fire) annotation(
      Line(points = {{38, 1}, {38, 1}, {38, -2}, {38, -2}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.c_not_fire, iGBT_self_made6.fire) annotation(
      Line(points = {{42, 1}, {42, 1}, {42, -30}, {43, -30}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.c_fire, iGBT_self_made3.fire) annotation(
      Line(points = {{43, 1}, {42, 1}, {42, -2}, {43, -2}}, color = {255, 0, 255}));
    connect(gain8.y, pWM_self_made_21.c) annotation(
      Line(points = {{42, 19}, {42, 19}, {42, 16}, {43, 16}}, color = {0, 0, 127}));
    connect(gain7.y, pWM_self_made_21.b) annotation(
      Line(points = {{37, 19}, {37, 17.5}, {38, 17.5}, {38, 17}}, color = {0, 0, 127}));
    connect(gain6.y, pWM_self_made_21.a) annotation(
      Line(points = {{32, 19}, {32, 16}}, color = {0, 0, 127}));
    connect(iGBT_self_made6.pin_n, iGBT_self_made3.pin) annotation(
      Line(points = {{44, -29}, {44, -29}, {44, -3}, {44, -3}}, color = {0, 0, 255}));
    connect(iGBT_self_made4.pin_n, iGBT_self_made2.pin) annotation(
      Line(points = {{39, -29}, {39, -29}, {39, -3}, {39, -3}}, color = {0, 0, 255}));
    connect(iGBT_self_made5.pin_n, iGBT_self_made1.pin) annotation(
      Line(points = {{34, -29}, {34, -29}, {34, -3}, {34, -3}}, color = {0, 0, 255}));
    connect(inductor3.n, iGBT_self_made3.pin) annotation(
      Line(points = {{-9, -16}, {44, -16}, {44, -3}, {44, -3}}, color = {0, 0, 255}));
    connect(inductor2.n, iGBT_self_made2.pin) annotation(
      Line(points = {{-9, -10}, {39, -10}, {39, -3}, {39, -3}}, color = {0, 0, 255}));
    connect(inductor1.n, iGBT_self_made1.pin) annotation(
      Line(points = {{-9, -4}, {34, -4}, {34, -3}, {34, -3}}, color = {0, 0, 255}));
    connect(Inverse_dq_Transform1.b, gain7.u) annotation(
      Line(points = {{29, 27}, {37, 27}, {37, 21}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.c, gain8.u) annotation(
      Line(points = {{29, 31}, {42, 31}, {42, 20}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.a, gain6.u) annotation(
      Line(points = {{29, 23}, {32, 23}, {32, 22}}, color = {0, 0, 127}));
    connect(gain5.y, dq_Transform1.a) annotation(
      Line(points = {{-60, 23}, {-57, 23}}, color = {0, 0, 127}));
    connect(currentSensor1.i, gain5.u) annotation(
      Line(points = {{-85, -1}, {-85, 23}, {-62, 23}}, color = {0, 0, 127}));
    connect(currentSensor3.i, gain3.u) annotation(
      Line(points = {{-96, -13}, {-96, -13}, {-96, 29}, {-62, 29}, {-62, 29}}, color = {0, 0, 127}));
    connect(currentSensor2.i, gain4.u) annotation(
      Line(points = {{-91, -7}, {-91, -7}, {-91, 26}, {-62, 26}, {-62, 26}}, color = {0, 0, 127}));
    connect(gain4.y, dq_Transform1.b) annotation(
      Line(points = {{-60, 26}, {-57, 26}, {-57, 26}, {-57, 26}}, color = {0, 0, 127}));
    connect(gain3.y, dq_Transform1.c) annotation(
      Line(points = {{-60, 29}, {-57, 29}, {-57, 29}, {-57, 29}}, color = {0, 0, 127}));
    connect(gain2.y, Voltagedq_Transformer.a) annotation(
      Line(points = {{-60, 11}, {-53, 11}, {-53, 11}, {-53, 11}}, color = {0, 0, 127}));
    connect(gain1.y, Voltagedq_Transformer.b) annotation(
      Line(points = {{-60, 14}, {-53, 14}, {-53, 14}, {-53, 14}}, color = {0, 0, 127}));
    connect(PU.y, Voltagedq_Transformer.c) annotation(
      Line(points = {{-60, 17}, {-53, 17}, {-53, 17}, {-53, 17}}, color = {0, 0, 127}));
    connect(potentialSensor3.phi, PU.u) annotation(
      Line(points = {{-78, 10}, {-78, 10}, {-78, 17}, {-62, 17}, {-62, 17}}, color = {0, 0, 127}));
    connect(potentialSensor2.phi, gain1.u) annotation(
      Line(points = {{-72, 10}, {-72, 10}, {-72, 14}, {-62, 14}, {-62, 14}}, color = {0, 0, 127}));
    connect(gain2.u, potentialSensor1.phi) annotation(
      Line(points = {{-62, 11}, {-66, 11}, {-66, 10}, {-66, 10}, {-66, 10}}, color = {0, 0, 127}));
    connect(potentialSensor1.p, currentSensor1.n) annotation(
      Line(points = {{-66, 3}, {-66, -4}, {-82, -4}}, color = {0, 0, 255}));
    connect(potentialSensor2.p, currentSensor2.n) annotation(
      Line(points = {{-68, 4}, {-68, -10}, {-88, -10}}, color = {0, 0, 255}));
    connect(potentialSensor3.p, currentSensor3.n) annotation(
      Line(points = {{-74, 4}, {-74, -16}, {-93, -16}}, color = {0, 0, 255}));
    connect(currentSensor1.n, resistor1.p) annotation(
      Line(points = {{-81, -4}, {-33, -4}}, color = {0, 0, 255}));
    connect(PhaseA, currentSensor1.p) annotation(
      Line(points = {{-102, -4}, {-87, -4}}, color = {0, 0, 255}));
    connect(currentSensor2.n, resistor2.p) annotation(
      Line(points = {{-87, -10}, {-33, -10}}, color = {0, 0, 255}));
    connect(PhaseB, currentSensor2.p) annotation(
      Line(points = {{-102, -10}, {-93, -10}}, color = {0, 0, 255}));
    connect(currentSensor3.n, resistor3.p) annotation(
      Line(points = {{-92, -16}, {-33, -16}}, color = {0, 0, 255}));
    connect(PhaseC, currentSensor3.p) annotation(
      Line(points = {{-102, -16}, {-98, -16}}, color = {0, 0, 255}));
    connect(feedback1.y, CurrentController1.i_d_ref) annotation(
      Line(points = {{-31.5, 50}, {-40, 50}, {-40, 36}, {-32, 36}}, color = {0, 0, 127}));
    connect(ff1.ff, feedback1.u2) annotation(
      Line(points = {{-27, 62}, {-27, 58}, {-27.32, 58}, {-27.32, 54}}, color = {0, 0, 127}));
    connect(ff1.V_d, Voltagedq_Transformer.d) annotation(
      Line(points = {{-31, 66}, {-41.72, 66}, {-41.72, 12}, {-45.72, 12}}, color = {0, 0, 127}));
    connect(CurrentController1.i_d_mes, dq_Transform1.d) annotation(
      Line(points = {{-29.5, 23.6}, {-47.5, 23.6}, {-47.5, 23.6}, {-47.5, 23.6}}, color = {0, 0, 127}));
    connect(dq_Transform1.q, CurrentController1.i_q_mes) annotation(
      Line(points = {{-49.2, 28.4}, {-31.2, 28.4}, {-31.2, 28.4}, {-31.2, 28.4}}, color = {0, 0, 127}));
    connect(CurrentController1.v_DCd_d, Inverse_dq_Transform1.d) annotation(
      Line(points = {{7.1, 19.1}, {15.1, 19.1}, {15.1, 23.1}, {17.1, 23.1}}, color = {0, 0, 127}));
    connect(CurrentController1.v_DCd_q, Inverse_dq_Transform1.q) annotation(
      Line(points = {{6.5, 33.5}, {16.5, 33.5}, {16.5, 29.5}, {18.5, 29.5}}, color = {0, 0, 127}));
    connect(Voltagedq_Transformer.q, CurrentController1.v_q_conv) annotation(
      Line(points = {{-45.2, 16.4}, {-31.2, 16.4}, {-31.2, 16.4}, {-31.2, 16.4}}, color = {0, 0, 127}));
    connect(Voltagedq_Transformer.d, CurrentController1.v_d_conv) annotation(
      Line(points = {{-45.2, 12.4}, {-31.2, 12.4}, {-31.2, 12.4}, {-31.2, 12.4}}, color = {0, 0, 127}));
    connect(resistor1.n, inductor1.p) annotation(
      Line(points = {{-27, -4}, {-17, -4}}, color = {0, 0, 255}));
    connect(resistor2.n, inductor2.p) annotation(
      Line(points = {{-27, -10}, {-17, -10}}, color = {0, 0, 255}));
    connect(resistor3.n, inductor3.p) annotation(
      Line(points = {{-27, -16}, {-17, -16}}, color = {0, 0, 255}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-2, 4}, fillPattern = FillPattern.Solid, extent = {{-74, 74}, {74, -74}}, textString = "2LVSC_switch_P")}, coordinateSystem(grid = {1, 1}, initialScale = 0.1)),
      Diagram(coordinateSystem(grid = {1, 1})),
      __OpenModelica_commandLineOptions = "");
  end TwoLVSC_switch_P;

  model TwoLVSC_switch_V
    extends SystemParameters;
    constant Real pi = 2 * Modelica.Math.asin(1.0);
    constant Real phi1 = pi / 2;
    constant Real phi2 = (-2 * pi / 3) + phi1;
    constant Real phi3 = (-4 * pi / 3) + phi1;
    parameter Real i_q_ref = 0;
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1 annotation(
      Placement(visible = true, transformation(origin = {-66, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor2 annotation(
      Placement(visible = true, transformation(origin = {-72, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor3 annotation(
      Placement(visible = true, transformation(origin = {-78, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {-85, -4}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {-91, -10}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {-96, -16}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    TwoLevelVSC.dq_Transform Voltagedq_Transformer annotation(
      Placement(visible = true, transformation(origin = {-48, 14}, extent = {{-4, 4}, {4, -4}}, rotation = 0)));
    TwoLevelVSC.dq_Transform dq_Transform1 annotation(
      Placement(visible = true, transformation(origin = {-52, 26}, extent = {{-4, 4}, {4, -4}}, rotation = 0)));
    TwoLevelVSC.ff ff1 annotation(
      Placement(visible = true, transformation(origin = {-27, 66}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    TwoLevelVSC.CurrentController CurrentController1 annotation(
      Placement(visible = true, transformation(origin = {-13, 26}, extent = {{-15, 15}, {15, -15}}, rotation = 0)));
    Modelica.Blocks.Sources.Step iq_reference(height = 0, offset = 0, startTime = 0) annotation(
      Placement(visible = true, transformation(origin = {-35, 39}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-27, 50}, extent = {{5, 5}, {-5, -5}}, rotation = 0)));
    TwoLevelVSC.Inverse_dq_Transform Inverse_dq_Transform1 annotation(
      Placement(visible = true, transformation(origin = {23, 27}, extent = {{-5, 5}, {5, -5}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor2(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor3(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L, i(fixed = false)) annotation(
      Placement(visible = true, transformation(origin = {-12, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor2(L = L, i(fixed = false, start = 0)) annotation(
      Placement(visible = true, transformation(origin = {-12, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor3(L = L, i(fixed = false, start = 0)) annotation(
      Placement(visible = true, transformation(origin = {-12, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = 2 * C, v(fixed = true, start = V_DC_start / 2)) annotation(
      Placement(visible = true, transformation(origin = {79, -11}, extent = {{-7, -7}, {7, 7}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
      Placement(visible = true, transformation(origin = {88, -21}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor4 annotation(
      Placement(visible = true, transformation(origin = {90, 0}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Blocks.Sources.Step step1(height = V_DC_step, offset = V_DC_start * 1.2, startTime = V_DC_step_time) annotation(
      Placement(visible = true, transformation(origin = {47, 50}, extent = {{-4, -4}, {4, 4}}, rotation = 180)));
    Modelica.Blocks.Continuous.TransferFunction CRC(a = {1, 0}, b = {K_CRCP, K_CRCI}) annotation(
      Placement(visible = true, transformation(origin = {-1, 50}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback2 annotation(
      Placement(visible = true, transformation(origin = {23, 50}, extent = {{7, 7}, {-7, -7}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseA annotation(
      Placement(visible = true, transformation(origin = {-102, -4}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, 71}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseB annotation(
      Placement(visible = true, transformation(origin = {-102, -10}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, -1}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseC annotation(
      Placement(visible = true, transformation(origin = {-102, -16}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, -69}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {105, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {101, 41}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {98, -37}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {101, -41}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Blocks.Math.Gain PU(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 17}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 14}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 11}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 29}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 26}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 23}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = 1) annotation(
      Placement(visible = true, transformation(origin = {32, 20}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain7(k = 1) annotation(
      Placement(visible = true, transformation(origin = {37, 20}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain8(k = 1) annotation(
      Placement(visible = true, transformation(origin = {42, 20}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain9(k = 1 / I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {90, 32}, extent = {{-1, -1}, {1, 1}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain10(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {32, 74}, extent = {{-1, -1}, {1, 1}}, rotation = 180)));
    Modelica.Blocks.Math.Gain gain12(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {34, 50}, extent = {{-1, -1}, {1, 1}}, rotation = 180)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor2(C = 2 * C, v(fixed = true, start = -V_DC_start / 2)) annotation(
      Placement(visible = true, transformation(origin = {79, -30}, extent = {{-7, -7}, {7, 7}}, rotation = 90)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made1 annotation(
      Placement(visible = true, transformation(origin = {34, -2}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made2 annotation(
      Placement(visible = true, transformation(origin = {39, -2}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made3 annotation(
      Placement(visible = true, transformation(origin = {44, -2}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made4 annotation(
      Placement(visible = true, transformation(origin = {39, -30}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made5 annotation(
      Placement(visible = true, transformation(origin = {34, -30}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made6 annotation(
      Placement(visible = true, transformation(origin = {44, -30}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.PWM_self_made_2 pWM_self_made_21 annotation(
      Placement(visible = true, transformation(origin = {37.5, 8.5}, extent = {{-6.5, -6.5}, {6.5, 6.5}}, rotation = -90)));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor1 annotation(
      Placement(visible = true, transformation(origin = {66.5, -17.5}, extent = {{-3.5, -3.5}, {3.5, 3.5}}, rotation = -90)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction1(a = {1e-6, 1}, b = {1}) annotation(
      Placement(visible = true, transformation(origin = {-80, 23}, extent = {{1, -1}, {-1, 1}}, rotation = 180)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction2(a = {1e-6, 1}, b = {1}) annotation(
      Placement(visible = true, transformation(origin = {-75, 26}, extent = {{1, -1}, {-1, 1}}, rotation = 180)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction3(a = {1e-6, 1}, b = {1}) annotation(
      Placement(visible = true, transformation(origin = {-70, 29}, extent = {{1, -1}, {-1, 1}}, rotation = 180)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction4(a = {1e-6, 1}, b = {1}, x(fixed = true, start = I_base)) annotation(
      Placement(visible = true, transformation(origin = {-75, 17}, extent = {{1, -1}, {-1, 1}}, rotation = 180)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction5(a = {1e-6, 1}, b = {1}) annotation(
      Placement(visible = true, transformation(origin = {-69, 14}, extent = {{1, -1}, {-1, 1}}, rotation = 180)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction6(a = {1e-6, 1}, b = {1}) annotation(
      Placement(visible = true, transformation(origin = {-64, 11}, extent = {{1, -1}, {-1, 1}}, rotation = 180)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction7(a = {1e-6, 1}, b = {1}) annotation(
      Placement(visible = true, transformation(origin = {90, 14}, extent = {{1, -1}, {-1, 1}}, rotation = -90)));
    Modelica.Blocks.Continuous.TransferFunction transferFunction8(a = {1e-6, 1}, b = {1}, x(fixed = true, start = V_DC_base)) annotation(
      Placement(visible = true, transformation(origin = {60, 20}, extent = {{1, -1}, {-1, 1}}, rotation = -90)));
  equation
  connect(gain10.u, transferFunction8.y) annotation(
      Line(points = {{33, 74}, {60, 74}, {60, 21}}, color = {0, 0, 127}));
  connect(transferFunction8.u, voltageSensor1.v) annotation(
      Line(points = {{60, 19}, {60, -18}, {63, -18}, {63, -17}}, color = {0, 0, 127}));
    connect(iq_reference.y, CurrentController1.i_q_ref) annotation(
      Line(points = {{-34, 39}, {-31.9, 39}}, color = {0, 0, 127}));
    connect(transferFunction7.u, currentSensor4.i) annotation(
      Line(points = {{90, 13}, {90, 13}, {90, 3}, {90, 3}}, color = {0, 0, 127}));
    connect(currentSensor4.n, V_DC_2) annotation(
      Line(points = {{93, 0}, {105, 0}}, color = {0, 0, 255}));
    connect(gain9.u, transferFunction7.y) annotation(
      Line(points = {{90, 31}, {90, 31}, {90, 15}, {90, 15}}, color = {0, 0, 127}));
    connect(gain2.u, transferFunction6.y) annotation(
      Line(points = {{-62, 11}, {-63, 11}, {-63, 11}, {-63, 11}}, color = {0, 0, 127}));
    connect(transferFunction5.y, gain1.u) annotation(
      Line(points = {{-68, 14}, {-62, 14}, {-62, 14}, {-62, 14}}, color = {0, 0, 127}));
    connect(transferFunction4.y, PU.u) annotation(
      Line(points = {{-74, 17}, {-62, 17}, {-62, 17}, {-62, 17}}, color = {0, 0, 127}));
    connect(transferFunction6.u, potentialSensor1.phi) annotation(
      Line(points = {{-65, 11}, {-66, 11}, {-66, 10}, {-66, 10}}, color = {0, 0, 127}));
    connect(transferFunction5.u, potentialSensor2.phi) annotation(
      Line(points = {{-70, 14}, {-72, 14}, {-72, 10}, {-72, 10}}, color = {0, 0, 127}));
    connect(transferFunction4.u, potentialSensor3.phi) annotation(
      Line(points = {{-76, 18}, {-78, 18}, {-78, 10}, {-78, 10}}, color = {0, 0, 127}));
    connect(transferFunction1.y, gain5.u) annotation(
      Line(points = {{-79, 23}, {-62, 23}, {-62, 23}, {-62, 23}}, color = {0, 0, 127}));
    connect(transferFunction2.y, gain4.u) annotation(
      Line(points = {{-74, 26}, {-62, 26}, {-62, 26}, {-62, 26}}, color = {0, 0, 127}));
    connect(transferFunction3.y, gain3.u) annotation(
      Line(points = {{-69, 29}, {-62, 29}, {-62, 29}, {-62, 29}}, color = {0, 0, 127}));
    connect(transferFunction3.u, currentSensor3.i) annotation(
      Line(points = {{-71, 29}, {-96, 29}, {-96, -13}, {-96, -13}}, color = {0, 0, 127}));
    connect(transferFunction2.u, currentSensor2.i) annotation(
      Line(points = {{-76, 26}, {-91, 26}, {-91, -7}, {-91, -7}}, color = {0, 0, 127}));
    connect(transferFunction1.u, currentSensor1.i) annotation(
      Line(points = {{-81, 23}, {-85, 23}, {-85, -1}, {-85, -1}}, color = {0, 0, 127}));
    connect(potentialSensor2.p, currentSensor2.n) annotation(
      Line(points = {{-71, 4}, {-71, -10}, {-88, -10}}, color = {0, 0, 255}));
    connect(potentialSensor3.p, currentSensor3.n) annotation(
      Line(points = {{-79, 4}, {-79, -16}, {-93, -16}}, color = {0, 0, 255}));
    connect(gain9.y, ff1.i_DC) annotation(
      Line(points = {{90, 33}, {90, 66}, {-23, 66}}, color = {0, 0, 127}));
    connect(gain10.y, feedback2.u2) annotation(
      Line(points = {{31, 74}, {23, 74}, {23, 56}}, color = {0, 0, 127}));
    connect(gain10.y, ff1.V_DC) annotation(
      Line(points = {{31, 74}, {-27, 74}, {-27, 70}}, color = {0, 0, 127}));
    connect(voltageSensor1.p, capacitor1.p) annotation(
      Line(points = {{66.5, -14}, {66, -14}, {66, 0}, {79, 0}, {79, -4}}, color = {0, 0, 255}));
    connect(voltageSensor1.n, capacitor2.p) annotation(
      Line(points = {{66.5, -21}, {66.5, -37}, {79, -37}}, color = {0, 0, 255}));
    connect(capacitor2.p, m_V_DC_2) annotation(
      Line(points = {{79, -37}, {98, -37}}, color = {0, 0, 255}));
    connect(capacitor1.p, currentSensor4.p) annotation(
      Line(points = {{78, -4}, {78, 0}, {87, 0}}, color = {0, 0, 255}));
    connect(feedback2.u1, gain12.y) annotation(
      Line(points = {{29, 50}, {33, 50}}, color = {0, 0, 127}));
    connect(gain12.u, step1.y) annotation(
      Line(points = {{35, 50}, {43, 50}}, color = {0, 0, 127}));
    connect(CRC.u, feedback2.y) annotation(
      Line(points = {{6, 50}, {18, 50}}, color = {0, 0, 127}));
    connect(feedback1.u1, CRC.y) annotation(
      Line(points = {{-23, 50}, {-8, 50}}, color = {0, 0, 127}));
    connect(pWM_self_made_21.a_not_fire, iGBT_self_made5.fire) annotation(
      Line(points = {{32, 1}, {32, 1}, {32, -30}, {33, -30}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.a_fire, iGBT_self_made1.fire) annotation(
      Line(points = {{33, 1}, {32, 1}, {32, -2}, {33, -2}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.b_not_fire, iGBT_self_made4.fire) annotation(
      Line(points = {{37, 1}, {38, 1}, {38, -30}, {38, -30}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.b_fire, iGBT_self_made2.fire) annotation(
      Line(points = {{38, 1}, {38, 1}, {38, -2}, {38, -2}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.c_not_fire, iGBT_self_made6.fire) annotation(
      Line(points = {{42, 1}, {42, 1}, {42, -30}, {43, -30}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.c_fire, iGBT_self_made3.fire) annotation(
      Line(points = {{43, 1}, {42, 1}, {42, -2}, {43, -2}}, color = {255, 0, 255}));
    connect(gain8.y, pWM_self_made_21.c) annotation(
      Line(points = {{42, 19}, {42, 19}, {42, 16}, {43, 16}}, color = {0, 0, 127}));
    connect(gain7.y, pWM_self_made_21.b) annotation(
      Line(points = {{37, 19}, {37, 17.5}, {38, 17.5}, {38, 17}}, color = {0, 0, 127}));
    connect(gain6.y, pWM_self_made_21.a) annotation(
      Line(points = {{32, 19}, {32, 16}}, color = {0, 0, 127}));
    connect(iGBT_self_made1.pin_n, capacitor1.p) annotation(
      Line(points = {{34, -1}, {34, -1}, {34, 0}, {79, 0}, {79, -4}, {79, -4}}, color = {0, 0, 255}));
    connect(iGBT_self_made2.pin_n, capacitor1.p) annotation(
      Line(points = {{39, -1}, {39, -1}, {39, 0}, {79, 0}, {79, -4}, {79, -4}}, color = {0, 0, 255}));
    connect(iGBT_self_made3.pin_n, capacitor1.p) annotation(
      Line(points = {{44, -1}, {44, -1}, {44, 0}, {79, 0}, {79, -4}, {79, -4}}, color = {0, 0, 255}));
    connect(iGBT_self_made5.pin, capacitor2.p) annotation(
      Line(points = {{34, -31}, {34, -31}, {34, -37}, {79, -37}, {79, -37}}, color = {0, 0, 255}));
    connect(iGBT_self_made4.pin, capacitor2.p) annotation(
      Line(points = {{39, -31}, {39, -31}, {39, -37}, {79, -37}, {79, -37}}, color = {0, 0, 255}));
    connect(iGBT_self_made6.pin, capacitor2.p) annotation(
      Line(points = {{44, -31}, {44, -31}, {44, -37}, {79, -37}, {79, -37}}, color = {0, 0, 255}));
    connect(iGBT_self_made6.pin_n, iGBT_self_made3.pin) annotation(
      Line(points = {{44, -29}, {44, -29}, {44, -3}, {44, -3}}, color = {0, 0, 255}));
    connect(iGBT_self_made4.pin_n, iGBT_self_made2.pin) annotation(
      Line(points = {{39, -29}, {39, -29}, {39, -3}, {39, -3}}, color = {0, 0, 255}));
    connect(iGBT_self_made5.pin_n, iGBT_self_made1.pin) annotation(
      Line(points = {{34, -29}, {34, -29}, {34, -3}, {34, -3}}, color = {0, 0, 255}));
    connect(inductor3.n, iGBT_self_made3.pin) annotation(
      Line(points = {{-9, -16}, {44, -16}, {44, -3}, {44, -3}}, color = {0, 0, 255}));
    connect(inductor2.n, iGBT_self_made2.pin) annotation(
      Line(points = {{-9, -10}, {39, -10}, {39, -3}, {39, -3}}, color = {0, 0, 255}));
    connect(inductor1.n, iGBT_self_made1.pin) annotation(
      Line(points = {{-9, -4}, {34, -4}, {34, -3}, {34, -3}}, color = {0, 0, 255}));
    connect(ground2.p, capacitor1.n) annotation(
      Line(points = {{84, -21}, {79, -21}, {79, -19}, {79, -19}, {79, -18}}, color = {0, 0, 255}));
    connect(capacitor1.n, capacitor2.n) annotation(
      Line(points = {{79, -18}, {79, -18}, {79, -23}, {79, -23}}, color = {0, 0, 255}));
    connect(Inverse_dq_Transform1.b, gain7.u) annotation(
      Line(points = {{29, 27}, {37, 27}, {37, 21}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.c, gain8.u) annotation(
      Line(points = {{29, 31}, {42, 31}, {42, 20}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.a, gain6.u) annotation(
      Line(points = {{29, 23}, {32, 23}, {32, 22}}, color = {0, 0, 127}));
    connect(gain5.y, dq_Transform1.a) annotation(
      Line(points = {{-60, 23}, {-57, 23}}, color = {0, 0, 127}));
    connect(gain4.y, dq_Transform1.b) annotation(
      Line(points = {{-60, 26}, {-57, 26}, {-57, 26}, {-57, 26}}, color = {0, 0, 127}));
    connect(gain3.y, dq_Transform1.c) annotation(
      Line(points = {{-60, 29}, {-57, 29}, {-57, 29}, {-57, 29}}, color = {0, 0, 127}));
    connect(gain2.y, Voltagedq_Transformer.a) annotation(
      Line(points = {{-60, 11}, {-53, 11}, {-53, 11}, {-53, 11}}, color = {0, 0, 127}));
    connect(gain1.y, Voltagedq_Transformer.b) annotation(
      Line(points = {{-60, 14}, {-53, 14}, {-53, 14}, {-53, 14}}, color = {0, 0, 127}));
    connect(PU.y, Voltagedq_Transformer.c) annotation(
      Line(points = {{-60, 17}, {-53, 17}, {-53, 17}, {-53, 17}}, color = {0, 0, 127}));
    connect(potentialSensor1.p, currentSensor1.n) annotation(
      Line(points = {{-66, 3}, {-66, -4}, {-82, -4}}, color = {0, 0, 255}));
    connect(currentSensor1.n, resistor1.p) annotation(
      Line(points = {{-81, -4}, {-33, -4}}, color = {0, 0, 255}));
    connect(PhaseA, currentSensor1.p) annotation(
      Line(points = {{-102, -4}, {-87, -4}}, color = {0, 0, 255}));
    connect(currentSensor2.n, resistor2.p) annotation(
      Line(points = {{-87, -10}, {-33, -10}}, color = {0, 0, 255}));
    connect(PhaseB, currentSensor2.p) annotation(
      Line(points = {{-102, -10}, {-93, -10}}, color = {0, 0, 255}));
    connect(currentSensor3.n, resistor3.p) annotation(
      Line(points = {{-92, -16}, {-33, -16}}, color = {0, 0, 255}));
    connect(PhaseC, currentSensor3.p) annotation(
      Line(points = {{-102, -16}, {-98, -16}}, color = {0, 0, 255}));
    connect(feedback1.y, CurrentController1.i_d_ref) annotation(
      Line(points = {{-31.5, 50}, {-40, 50}, {-40, 36}, {-32, 36}}, color = {0, 0, 127}));
    connect(ff1.ff, feedback1.u2) annotation(
      Line(points = {{-27, 62}, {-27, 58}, {-27.32, 58}, {-27.32, 54}}, color = {0, 0, 127}));
    connect(ff1.V_d, Voltagedq_Transformer.d) annotation(
      Line(points = {{-31, 66}, {-41.72, 66}, {-41.72, 12}, {-45.72, 12}}, color = {0, 0, 127}));
    connect(CurrentController1.i_d_mes, dq_Transform1.d) annotation(
      Line(points = {{-29.5, 23.6}, {-47.5, 23.6}, {-47.5, 23.6}, {-47.5, 23.6}}, color = {0, 0, 127}));
    connect(dq_Transform1.q, CurrentController1.i_q_mes) annotation(
      Line(points = {{-49.2, 28.4}, {-31.2, 28.4}, {-31.2, 28.4}, {-31.2, 28.4}}, color = {0, 0, 127}));
    connect(CurrentController1.v_DCd_d, Inverse_dq_Transform1.d) annotation(
      Line(points = {{7.1, 19.1}, {15.1, 19.1}, {15.1, 23.1}, {17.1, 23.1}}, color = {0, 0, 127}));
    connect(CurrentController1.v_DCd_q, Inverse_dq_Transform1.q) annotation(
      Line(points = {{6.5, 33.5}, {16.5, 33.5}, {16.5, 29.5}, {18.5, 29.5}}, color = {0, 0, 127}));
    connect(Voltagedq_Transformer.q, CurrentController1.v_q_conv) annotation(
      Line(points = {{-45.2, 16.4}, {-31.2, 16.4}, {-31.2, 16.4}, {-31.2, 16.4}}, color = {0, 0, 127}));
    connect(Voltagedq_Transformer.d, CurrentController1.v_d_conv) annotation(
      Line(points = {{-45.2, 12.4}, {-31.2, 12.4}, {-31.2, 12.4}, {-31.2, 12.4}}, color = {0, 0, 127}));
    connect(resistor1.n, inductor1.p) annotation(
      Line(points = {{-27, -4}, {-17, -4}}, color = {0, 0, 255}));
    connect(resistor2.n, inductor2.p) annotation(
      Line(points = {{-27, -10}, {-17, -10}}, color = {0, 0, 255}));
    connect(resistor3.n, inductor3.p) annotation(
      Line(points = {{-27, -16}, {-17, -16}}, color = {0, 0, 255}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-2, 4}, fillPattern = FillPattern.Solid, extent = {{-74, 74}, {74, -74}}, textString = "2LVSC_switch_V")}, coordinateSystem(grid = {1, 1}, initialScale = 0.1)),
      Diagram(coordinateSystem(grid = {1, 1})),
      __OpenModelica_commandLineOptions = "",
      experiment(StartTime = 0, StopTime = 0.5, Tolerance = 1e-6, Interval = 0.002),
      __OpenModelica_simulationFlags(lv = "LOG_STATS", r = "SwitchingVariables_res.mat", s = "dassl"));
  end TwoLVSC_switch_V;

  model VSC_avg_test
    Real V_DC_pu_avg;
    Real P_pu_avg;
    Grid grid1 annotation(
      Placement(visible = true, transformation(origin = {-52, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    DCside dCside1 annotation(
      Placement(visible = true, transformation(origin = {12, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    TwoLevelVSC.TwoLVSC_avg_V twoLVSC_avg_V1 annotation(
      Placement(visible = true, transformation(origin = {-16, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    extends SystemParameters;
  equation
    connect(grid1.PinA, twoLVSC_avg_V1.PhaseA) annotation(
      Line(points = {{-52, 16}, {-26, 16}, {-26, 17}}, color = {0, 0, 255}));
    connect(grid1.PinB, twoLVSC_avg_V1.PhaseB) annotation(
      Line(points = {{-52, 10}, {-26, 10}}, color = {0, 0, 255}));
    connect(grid1.PinC, twoLVSC_avg_V1.PhaseC) annotation(
      Line(points = {{-52, 4}, {-43, 4}, {-43, 3}, {-26, 3}}, color = {0, 0, 255}));
    connect(twoLVSC_avg_V1.V_DC_2, dCside1.V_DC_2) annotation(
      Line(points = {{-6, 14}, {12, 14}}, color = {0, 0, 255}));
    connect(twoLVSC_avg_V1.m_V_DC_2, dCside1.m_V_DC_2) annotation(
      Line(points = {{-6, 6}, {12, 6}, {12, 2}}, color = {0, 0, 255}));
    V_DC_pu_avg = (twoLVSC_avg_V1.V_DC_2.v - twoLVSC_avg_V1.m_V_DC_2.v) / V_DC_base;
    P_pu_avg = V_DC_pu_avg * V_DC_base * (-twoLVSC_avg_V1.V_DC_2.i) / S_base;
//positive current is defined as flowing into the pin
  protected
  end VSC_avg_test;

  model VSC_switch_test
    TwoLevelVSC.Grid grid1 annotation(
      Placement(visible = true, transformation(origin = {-34, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    DCside dCside1 annotation(
      Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    TwoLVSC_switch_V twoLVSC_switch_V1 annotation(
      Placement(visible = true, transformation(origin = {0, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    extends SystemParameters;
  equation
    connect(twoLVSC_switch_V1.m_V_DC_2, dCside1.m_V_DC_2) annotation(
      Line(points = {{10, -2}, {36, -2}, {36, -6}, {36, -6}}, color = {0, 0, 255}));
    connect(twoLVSC_switch_V1.V_DC_2, dCside1.V_DC_2) annotation(
      Line(points = {{10, 6}, {36, 6}, {36, 6}, {36, 6}}, color = {0, 0, 255}));
    connect(grid1.PinC, twoLVSC_switch_V1.PhaseC) annotation(
      Line(points = {{-34, -6}, {-10, -6}, {-10, -4}, {-10, -4}}, color = {0, 0, 255}));
    connect(grid1.PinB, twoLVSC_switch_V1.PhaseB) annotation(
      Line(points = {{-34, 0}, {-10, 0}, {-10, 2}, {-10, 2}}, color = {0, 0, 255}));
    connect(grid1.PinA, twoLVSC_switch_V1.PhaseA) annotation(
      Line(points = {{-34, 6}, {-10, 6}, {-10, 10}, {-10, 10}}, color = {0, 0, 255}));
    annotation(
      experiment(StartTime = 0, StopTime = 0.5, Tolerance = 1e-06, Interval = 2e-07),
      __OpenModelica_simulationFlags(lv = "LOG_STATS", r = "SwitchingresultsNoIq_res.mat", s = "dassl"));
  end VSC_switch_test;

  model Cable_test
    Real V_DC_pu_avg;
    Real P_pu_avg;
    Cable.PiModel piModel1 annotation(
      Placement(visible = true, transformation(origin = {2, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Cable.PiModel piModel2(pos = -1) annotation(
      Placement(visible = true, transformation(origin = {2, -12}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    extends SystemParameters;
    Grid grid1 annotation(
      Placement(visible = true, transformation(origin = {-88, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Grid grid2 annotation(
      Placement(visible = true, transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    TwoLVSC_avg_V twoLVSC_avg_V1 annotation(
      Placement(visible = true, transformation(origin = {-60, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    TwoLevelVSC.TwoLVSC_avg_P twoLVSC_avg_P1 annotation(
      Placement(visible = true, transformation(origin = {50, -2}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
    connect(twoLVSC_avg_V1.m_V_DC_2, piModel2.pin_n) annotation(
      Line(points = {{-50, -6}, {-8, -6}, {-8, -10}}, color = {0, 0, 255}));
    connect(piModel2.pin, twoLVSC_avg_P1.m_V_DC_2) annotation(
      Line(points = {{12, -10}, {40, -10}, {40, -6}}, color = {0, 0, 255}));
    connect(twoLVSC_avg_V1.V_DC_2, piModel1.pin) annotation(
      Line(points = {{-50, 2}, {-29, 2}, {-29, 4}, {-8, 4}}, color = {0, 0, 255}));
    connect(piModel1.pin_n, twoLVSC_avg_P1.V_DC_2) annotation(
      Line(points = {{12, 4}, {26, 4}, {26, 2}, {40, 2}}, color = {0, 0, 255}));
    connect(twoLVSC_avg_P1.PhaseC, grid2.C) annotation(
      Line(points = {{60, -8}, {80, -8}, {80, -6}, {80, -6}}, color = {0, 0, 255}));
    connect(twoLVSC_avg_P1.PhaseB, grid2.B) annotation(
      Line(points = {{60, -2}, {80, -2}, {80, 0}, {80, 0}}, color = {0, 0, 255}));
    connect(twoLVSC_avg_P1.PhaseA, grid2.A) annotation(
      Line(points = {{60, 6}, {80, 6}, {80, 6}, {80, 6}}, color = {0, 0, 255}));
    connect(grid1.C, twoLVSC_avg_V1.PhaseC) annotation(
      Line(points = {{-88, -6}, {-70, -6}, {-70, -8}, {-70, -8}}, color = {0, 0, 255}));
    connect(grid1.B, twoLVSC_avg_V1.PhaseB) annotation(
      Line(points = {{-88, 0}, {-72, 0}, {-72, -2}, {-70, -2}}, color = {0, 0, 255}));
    connect(grid1.A, twoLVSC_avg_V1.PhaseA) annotation(
      Line(points = {{-88, 6}, {-70, 6}, {-70, 6}, {-70, 6}}, color = {0, 0, 255}));
    V_DC_pu_avg = (twoLVSC_avg_V1.V_DC_2.v - twoLVSC_avg_V1.m_V_DC_2.v) / V_DC_base;
    P_pu_avg = V_DC_pu_avg * V_DC_base * (-twoLVSC_avg_V1.V_DC_2.i) / S_base;
  end Cable_test;

  model Grid
    parameter Real v_amp = 400;
    constant Real w = 2 * pi * 50;
    parameter Real phi1 = pi / 2;
    parameter Real phi2 = (-2 * pi / 3) + pi / 2;
    parameter Real phi3 = (-4 * pi / 3) + pi / 2;
    Real P;
    Real I_a_pu;
    Real I_b_pu;
    Real I_c_pu;
  protected
    extends SystemParameters;
    Modelica.Electrical.Analog.Sources.SineVoltage PhaseA(V = v_amp, freqHz = 50, phase(displayUnit = "rad") = phi1) annotation(
      Placement(visible = true, transformation(origin = {40, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SineVoltage PhaseB(V = v_amp, freqHz = 50, phase(displayUnit = "rad") = phi2) annotation(
      Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SineVoltage PhaseC(V = v_amp, freqHz = 50, phase(displayUnit = "rad") = phi3) annotation(
      Placement(visible = true, transformation(origin = {40, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Interfaces.Pin PinA annotation(
      Placement(visible = true, transformation(origin = {0, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-2, 64}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PinB annotation(
      Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-2, 2}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PinC annotation(
      Placement(visible = true, transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-2, -58}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
  equation
    connect(PhaseA.n, ground1.p) annotation(
      Line(points = {{50, 62}, {70, 62}, {70, 0}}, color = {0, 0, 255}));
    connect(PinA, PhaseA.p) annotation(
      Line(points = {{0, 62}, {30, 62}}, color = {0, 0, 255}));
    I_a_pu = PhaseA.i / I_base;
    I_b_pu = PhaseB.i / I_base;
    I_c_pu = PhaseC.i / I_base;
    P = PhaseA.v * PhaseA.i + PhaseB.v * PhaseB.i + PhaseC.v * PhaseC.i;
    connect(PinC, PhaseC.p) annotation(
      Line(points = {{0, -60}, {30, -60}, {30, -60}, {30, -60}}, color = {0, 0, 255}));
    connect(PinB, PhaseB.p) annotation(
      Line(points = {{0, 0}, {30, 0}, {30, 0}, {30, 0}}, color = {0, 0, 255}));
    connect(PhaseC.n, ground1.p) annotation(
      Line(points = {{50, -60}, {70, -60}, {70, 0}, {70, 0}}, color = {0, 0, 255}));
    connect(PhaseB.n, ground1.p) annotation(
      Line(points = {{50, 0}, {70, 0}, {70, 0}, {70, 0}}, color = {0, 0, 255}));
    annotation(
      Diagram(coordinateSystem(initialScale = 0.1)),
      Icon(graphics = {Line(origin = {0, 1}, points = {{-4, 99}, {-4, -99}, {4, -99}, {4, 99}, {-4, 99}, {-4, 99}}, color = {255, 0, 0}, thickness = 3.5)}));
  end Grid;

  model DCside
    Real P;
    Real V_DC;
    Real P_pu;
    Real V_DC_pu;
    parameter Real load_current = i_load * direction_V;
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
    Modelica.Blocks.Sources.Step step1(height = step * direction_V, offset = i_load * direction_V, startTime = start_time) annotation(
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

  class CurrentController
    extends SystemParameters;
    Modelica.Blocks.Interfaces.RealInput v_q_conv annotation(
      Placement(visible = true, transformation(origin = {-80, -86}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput i_d_mes annotation(
      Placement(visible = true, transformation(origin = {-80, 28}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput i_q_mes annotation(
      Placement(visible = true, transformation(origin = {-80, -28}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput v_DCd_d annotation(
      Placement(visible = true, transformation(origin = {102, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {134, 46}, extent = {{-34, -34}, {34, 34}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput v_DCd_q annotation(
      Placement(visible = true, transformation(origin = {102, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {130, -50}, extent = {{-30, -30}, {30, 30}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput i_q_ref annotation(
      Placement(visible = true, transformation(origin = {-80, -58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput i_d_ref annotation(
      Placement(visible = true, transformation(origin = {-80, 55}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-40, 55}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback2 annotation(
      Placement(visible = true, transformation(origin = {-40, -58}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    Modelica.Blocks.Continuous.TransferFunction VRC(a = {1, 0}, b = {K_p, K_i}) annotation(
      Placement(visible = true, transformation(origin = {4, 55}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.TransferFunction VRC_2(a = {1, 0}, b = {K_p, K_i}) annotation(
      Placement(visible = true, transformation(origin = {0, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = -L_pu) annotation(
      Placement(visible = true, transformation(origin = {-30, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = L_pu) annotation(
      Placement(visible = true, transformation(origin = {-30, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    CurrentAdder currentAdder1 annotation(
      Placement(visible = true, transformation(origin = {48, 55}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    CurrentAdder currentAdder2 annotation(
      Placement(visible = true, transformation(origin = {48, -59}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
    Modelica.Blocks.Interfaces.RealInput v_d_conv annotation(
      Placement(visible = true, transformation(origin = {-80, 84}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(v_d_conv, currentAdder1.u) annotation(
      Line(points = {{-80, 84}, {48, 84}, {48, 69}}, color = {0, 0, 127}));
    connect(currentAdder2.y, v_DCd_q) annotation(
      Line(points = {{60, -58}, {96, -58}, {96, -58}, {102, -58}}, color = {0, 0, 127}));
    connect(currentAdder1.y, v_DCd_d) annotation(
      Line(points = {{60, 54}, {92, 54}, {92, 56}, {102, 56}}, color = {0, 0, 127}));
    connect(gain1.y, currentAdder2.u2) annotation(
      Line(points = {{-19, 14}, {48, 14}, {48, -45}}, color = {0, 0, 127}));
    connect(VRC_2.y, currentAdder2.u1) annotation(
      Line(points = {{11, -60}, {-5.5, -60}, {-5.5, -57}, {36, -57}}, color = {0, 0, 127}));
    connect(v_q_conv, currentAdder2.u) annotation(
      Line(points = {{-80, -86}, {48, -86}, {48, -69}}, color = {0, 0, 127}));
    connect(i_d_mes, gain1.u) annotation(
      Line(points = {{-80, 26}, {-72, 26}, {-72, 16}, {-42, 16}}, color = {0, 0, 127}));
    connect(VRC.y, currentAdder1.u1) annotation(
      Line(points = {{17, 55}, {36, 55}}, color = {0, 0, 127}));
    connect(VRC.u, feedback1.y) annotation(
      Line(points = {{-6, 55}, {-31, 55}}, color = {0, 0, 127}));
    connect(gain2.y, currentAdder1.u2) annotation(
      Line(points = {{-18, -16}, {0, -16}, {0, 20}, {48, 20}, {48, 45}}, color = {0, 0, 127}));
    connect(i_d_mes, feedback1.u2) annotation(
      Line(points = {{-80, 28}, {-40, 28}, {-40, 47}}, color = {0, 0, 127}));
    connect(i_d_ref, feedback1.u1) annotation(
      Line(points = {{-80, 55}, {-48, 55}}, color = {0, 0, 127}));
    connect(i_q_mes, gain2.u) annotation(
      Line(points = {{-80, -28}, {-70, -28}, {-70, -16}, {-42, -16}}, color = {0, 0, 127}));
    connect(i_q_mes, feedback2.u2) annotation(
      Line(points = {{-80, -30}, {-40, -30}, {-40, -52}}, color = {0, 0, 127}));
    connect(VRC_2.u, feedback2.y) annotation(
      Line(points = {{-12, -58}, {-31, -58}}, color = {0, 0, 127}));
    connect(i_q_ref, feedback2.u1) annotation(
      Line(points = {{-80, -58}, {-48, -58}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {23, -52}, fillPattern = FillPattern.Solid, extent = {{-111, 158}, {57, -62}}, textString = "Current controller")}, coordinateSystem(initialScale = 0.1)),
      uses(Modelica(version = "3.2.2")));
  end CurrentController;

  class dq_Transform
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
    d = 2 / 3 * (a * cos(w * time) + b * cos(w * time - 2 * pi / 3) + c * cos(w * time + 2 * pi / 3));
    q = 2 / 3 * ((-a * sin(w * time)) - b * sin(w * time - 2 * pi / 3) - c * sin(w * time + 2 * pi / 3));
    annotation(
      Icon(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(origin = {0, -1}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 101}, {100, -99}}), Text(origin = {-60, 82}, fillPattern = FillPattern.Solid, extent = {{-34, 34}, {74, -90}}, textString = "123"), Line(points = {{-100, -100}, {100, 100}, {100, 100}}), Text(origin = {16, -37}, fillPattern = FillPattern.Solid, extent = {{-24, -35}, {54, 43}}, textString = "dq")}),
      uses(Modelica(version = "3.2.2")),
      Diagram);
  end dq_Transform;

  class Inverse_dq_Transform
    parameter Real pi = 2 * Modelica.Math.asin(1.0);
    parameter Real w = 2 * pi * 50;
    Modelica.Blocks.Interfaces.RealInput d annotation(
      Placement(visible = true, transformation(origin = {-160, 72}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput q annotation(
      Placement(visible = true, transformation(origin = {-150, 82}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput a annotation(
      Placement(visible = true, transformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput b annotation(
      Placement(visible = true, transformation(origin = {120, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput c annotation(
      Placement(visible = true, transformation(origin = {130, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    a = d * cos(w * time) - q * sin(w * time);
    b = d * cos(w * time - 2 * pi / 3) - q * sin(w * time - 2 * pi / 3);
    c = d * cos(w * time + 2 * pi / 3) - q * sin(w * time + 2 * pi / 3);
    annotation(
      Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(points = {{-100, -100}, {100, 100}, {100, 100}}), Text(origin = {-38, 68}, fillPattern = FillPattern.Solid, extent = {{28, 74}, {-46, -90}}, textString = "dq"), Text(origin = {74, -87}, fillPattern = FillPattern.Solid, extent = {{2, -1}, {-92, 75}}, textString = "123")}, coordinateSystem(initialScale = 0.1)),
      uses(Modelica(version = "3.2.2")));
  end Inverse_dq_Transform;

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
    y = u - u1 + u2;
    annotation(
      Icon(coordinateSystem(initialScale = 0.1), graphics = {Ellipse(origin = {1, -1}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-101, 99}, {101, -99}}, endAngle = 360), Line(origin = {0, -5}, points = {{0, 65}, {0, -65}, {0, -65}}, thickness = 0.5), Line(origin = {0.64, -1.64}, points = {{-73, 0}, {73, 0}, {73, 0}}, thickness = 0.5), Line(origin = {-120, 30}, points = {{-10, 0}, {10, 0}}, thickness = 0.5)}),
      uses(Modelica(version = "3.2.2")));
  end CurrentAdder;

  class DCref
    Modelica.Blocks.Interfaces.RealInput i_d annotation(
      Placement(visible = true, transformation(origin = {-176, 56}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-116, 4}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput V_DC annotation(
      Placement(visible = true, transformation(origin = {-204, -26}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -104}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
    Modelica.Blocks.Interfaces.RealInput V_d annotation(
      Placement(visible = true, transformation(origin = {-112, -14}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {2, 116}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(visible = true, transformation(origin = {108, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {108, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    y = i_d * V_d / V_DC;
    annotation(
      Icon(graphics = {Ellipse(origin = {-2, 7}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-88, 87}, {88, -87}}, endAngle = 360), Line(origin = {-1.5, 0}, points = {{-80.5, 0}, {81.5, 0}, {79.5, 0}}), Text(origin = {-8, 31}, fillPattern = FillPattern.Solid, extent = {{-56, 29}, {56, -29}}, textString = "v_d_conv*i_d"), Text(origin = {-4, -19}, fillPattern = FillPattern.Solid, extent = {{-56, 29}, {56, -29}}, textString = "V_DC")}, coordinateSystem(initialScale = 0.1)),
      uses(Modelica(version = "3.2.2")));
  end DCref;

  model ff
    Modelica.Blocks.Interfaces.RealInput i_DC annotation(
      Placement(visible = true, transformation(origin = {-108, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {98, -2}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
    Modelica.Blocks.Interfaces.RealInput V_DC annotation(
      Placement(visible = true, transformation(origin = {-112, 52}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-10, 108}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Interfaces.RealInput V_d annotation(
      Placement(visible = true, transformation(origin = {-114, -58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-118, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput ff annotation(
      Placement(visible = true, transformation(origin = {-2, -114}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-8, -100}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  equation
    ff = i_DC * V_DC / V_d;
    annotation(
      Icon(graphics = {Ellipse(origin = {-10, -1}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-86, 87}, {86, -87}}, endAngle = 360), Line(origin = {-6, 0}, points = {{-78, 0}, {78, 0}, {78, 0}}), Text(origin = {0, 6}, fillPattern = FillPattern.Solid, extent = {{-60, 30}, {32, -2}}, textString = "i_DC*V_DC"), Text(origin = {6, -26}, fillPattern = FillPattern.Solid, extent = {{-60, 30}, {24, -2}}, textString = "v_d_conv")}, coordinateSystem(initialScale = 0.1)),
      uses(Modelica(version = "3.2.2")));
  end ff;

  model Comparator
    Modelica.Blocks.Interfaces.RealInput sawtooth annotation(
      Placement(visible = true, transformation(origin = {-108, 68}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput conv_voltage annotation(
      Placement(visible = true, transformation(origin = {-110, -66}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanOutput switch_state annotation(
      Placement(visible = true, transformation(origin = {116, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {116, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    if conv_voltage > sawtooth then
      switch_state = true;
    else
      switch_state = false;
    end if;
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, 6}, fillPattern = FillPattern.Solid, extent = {{-89, 86}, {89, -86}}, textString = "u>u1")}));
  end Comparator;

  model IGBT_self_made
    Modelica.Blocks.Interfaces.BooleanInput fire annotation(
      Placement(visible = true, transformation(origin = {-88, -8}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-88, -8}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin pin annotation(
      Placement(visible = true, transformation(origin = {0, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(
      Placement(visible = true, transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Ideal.IdealDiode idealDiode1 annotation(
      Placement(visible = true, transformation(origin = {32, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Ideal.IdealCommutingSwitch idealCommutingSwitch1 annotation(
      Placement(visible = true, transformation(origin = {-46, -9}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  equation
    connect(fire, idealCommutingSwitch1.control) annotation(
      Line(points = {{-90, -8}, {-58, -8}}, color = {255, 0, 255}));
    connect(idealCommutingSwitch1.p, pin) annotation(
      Line(points = {{-46, -20}, {-46, -80}, {0, -80}, {0, -94}}, color = {0, 0, 255}));
    connect(idealCommutingSwitch1.n2, pin_n) annotation(
      Line(points = {{-46, 2}, {-46, 84}, {0, 84}, {0, 92}}, color = {0, 0, 255}));
    connect(idealDiode1.p, pin) annotation(
      Line(points = {{32, -16}, {32, -80}, {0, -80}, {0, -94}}, color = {0, 0, 255}));
    connect(idealDiode1.n, pin_n) annotation(
      Line(points = {{32, 4}, {32, 84}, {0, 84}, {0, 92}}, color = {0, 0, 255}));
    annotation(
      Icon(coordinateSystem(initialScale = 0.1), graphics = {Line(origin = {29.9472, 35}, points = {{-29.9472, 55}, {-29.9472, 25}, {10.0528, 25}, {10.0528, -15}, {-9.94721, -15}, {30.0528, -15}, {10.0528, -15}, {10.0528, -15}, {-9.94721, -55}, {30.0528, -55}, {10.0528, -15}}), Line(origin = {20, -54}, points = {{20, 34}, {20, -6}, {-20, -6}, {-20, -34}}), Line(origin = {-29.9472, -35}, points = {{29.9472, -55}, {29.9472, -25}, {-10.0528, -25}, {-10.0528, 15}, {-30.0528, 15}, {9.94721, 15}, {-10.0528, 15}, {9.94721, 55}, {-30.0528, 55}, {-10.0528, 15}}), Line(origin = {-20, 55}, points = {{-20, -35}, {-20, 5}, {20, 5}, {20, 35}}), Line(origin = {-72.7929, -11.7929}, points = {{-21.2071, -14.2071}, {-7.20711, -14.2071}, {20.7929, 13.7929}})}));
  end IGBT_self_made;

  model PWM_self_made_2
    extends SystemParameters;
    Modelica.Blocks.Interfaces.RealInput a annotation(
      Placement(visible = true, transformation(origin = {-94, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput b annotation(
      Placement(visible = true, transformation(origin = {-96, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-96, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput c annotation(
      Placement(visible = true, transformation(origin = {-92, -66}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanOutput a_fire annotation(
      Placement(visible = true, transformation(origin = {86, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanOutput a_not_fire annotation(
      Placement(visible = true, transformation(origin = {86, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanOutput b_fire annotation(
      Placement(visible = true, transformation(origin = {86, 3}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanOutput b_not_fire annotation(
      Placement(visible = true, transformation(origin = {86, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanOutput c_fire annotation(
      Placement(visible = true, transformation(origin = {86, -57}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanOutput c_not_fire annotation(
      Placement(visible = true, transformation(origin = {86, -78}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.SawTooth sawTooth1(amplitude = 2, offset = -1, period = tau_PWM) annotation(
      Placement(visible = true, transformation(origin = {-82, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    TwoLevelVSC.Comparator comparator1 annotation(
      Placement(visible = true, transformation(origin = {-8, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Logical.Not not1 annotation(
      Placement(visible = true, transformation(origin = {58, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    TwoLevelVSC.Comparator comparator2 annotation(
      Placement(visible = true, transformation(origin = {-8, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Logical.Not not11 annotation(
      Placement(visible = true, transformation(origin = {58, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    TwoLevelVSC.Comparator comparator3 annotation(
      Placement(visible = true, transformation(origin = {-6, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Logical.Not not12 annotation(
      Placement(visible = true, transformation(origin = {60, -78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(comparator3.switch_state, c_fire) annotation(
      Line(points = {{5.6, -56.8}, {47.6, -56.8}, {47.6, -57}, {88, -57}}, color = {255, 0, 255}));
    connect(not12.y, c_not_fire) annotation(
      Line(points = {{71, -78}, {88, -78}}, color = {255, 0, 255}));
    connect(not1.y, a_not_fire) annotation(
      Line(points = {{69, 34}, {86, 34}, {86, 32}}, color = {255, 0, 255}));
    connect(not11.y, b_not_fire) annotation(
      Line(points = {{69, -18}, {88, -18}}, color = {255, 0, 255}));
    connect(comparator2.switch_state, b_fire) annotation(
      Line(points = {{3.6, 3.2}, {46.6, 3.2}, {46.6, 2}, {86, 2}}, color = {255, 0, 255}));
    connect(comparator1.switch_state, a_fire) annotation(
      Line(points = {{3.6, 55.2}, {88, 55.2}, {88, 54}}, color = {255, 0, 255}));
    connect(sawTooth1.y, comparator2.sawtooth) annotation(
      Line(points = {{-70, 84}, {-54, 84}, {-54, 10}, {-20, 10}, {-20, 12}}, color = {0, 0, 127}));
    connect(sawTooth1.y, comparator3.sawtooth) annotation(
      Line(points = {{-70, 84}, {-54, 84}, {-54, -50}, {-18, -50}, {-18, -48}}, color = {0, 0, 127}));
    connect(comparator3.switch_state, not12.u) annotation(
      Line(points = {{6, -56}, {40, -56}, {40, -78}, {48, -78}, {48, -78}}, color = {255, 0, 255}));
    connect(comparator1.switch_state, not1.u) annotation(
      Line(points = {{3.6, 55.2}, {45.6, 55.2}, {45.6, 33.2}, {45.6, 33.2}}, color = {255, 0, 255}));
    connect(comparator2.switch_state, not11.u) annotation(
      Line(points = {{3.6, 3.2}, {43.6, 3.2}, {43.6, -18.8}, {45.6, -18.8}}, color = {255, 0, 255}));
    connect(c, comparator3.conv_voltage) annotation(
      Line(points = {{-92, -64}, {-18, -64}, {-18, -60}}, color = {0, 0, 127}));
    connect(comparator1.sawtooth, sawTooth1.y) annotation(
      Line(points = {{-20, 65}, {-54, 65}, {-54, 86}, {-69, 86}}, color = {0, 0, 127}));
    connect(a, comparator1.conv_voltage) annotation(
      Line(points = {{-94, 50}, {-22, 50}}, color = {0, 0, 127}));
    connect(b, comparator2.conv_voltage) annotation(
      Line(points = {{-96, 0}, {-20, 0}, {-20, -2}}, color = {0, 0, 127}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, 8}, fillPattern = FillPattern.Solid, extent = {{-93, 78}, {93, -78}}, textString = "PWM_self_made")}, coordinateSystem(initialScale = 0.1)));
  end PWM_self_made_2;

  model TwoLVSC_avg_V_CC_test
    extends SystemParameters;
    constant Real pi = 2 * Modelica.Math.asin(1.0);
    constant Real phi1 = pi / 2;
    constant Real phi2 = (-2 * pi / 3) + phi1;
    constant Real phi3 = (-4 * pi / 3) + phi1;
    parameter Real i_q_ref = 0;
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1 annotation(
      Placement(visible = true, transformation(origin = {-66, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor2 annotation(
      Placement(visible = true, transformation(origin = {-72, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor3 annotation(
      Placement(visible = true, transformation(origin = {-78, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {-85, -4}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {-91, -10}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {-96, -16}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    TwoLevelVSC.dq_Transform Voltagedq_Transformer annotation(
      Placement(visible = true, transformation(origin = {-48, 14}, extent = {{-4, 4}, {4, -4}}, rotation = 0)));
    TwoLevelVSC.dq_Transform dq_Transform1 annotation(
      Placement(visible = true, transformation(origin = {-52, 26}, extent = {{-4, 4}, {4, -4}}, rotation = 0)));
    TwoLevelVSC.CurrentController CurrentController1 annotation(
      Placement(visible = true, transformation(origin = {-13, 26}, extent = {{-15, 15}, {15, -15}}, rotation = 0)));
    Modelica.Blocks.Sources.Step iq_reference(height = 1, startTime = 0.5) annotation(
      Placement(visible = true, transformation(origin = {-35, 39}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.Inverse_dq_Transform Inverse_dq_Transform1 annotation(
      Placement(visible = true, transformation(origin = {23, 27}, extent = {{-5, 5}, {5, -5}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor2(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor3(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L, i(fixed = false, start = I_base)) annotation(
      Placement(visible = true, transformation(origin = {-12, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor2(L = L, i(fixed = false, start = I_base)) annotation(
      Placement(visible = true, transformation(origin = {-12, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor3(L = L, i(fixed = false, start = I_base)) annotation(
      Placement(visible = true, transformation(origin = {-12, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage conv_a annotation(
      Placement(visible = true, transformation(origin = {32, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage conv_b annotation(
      Placement(visible = true, transformation(origin = {37, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage conv_c annotation(
      Placement(visible = true, transformation(origin = {42, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {52, -10}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
    Modelica.Blocks.Math.Gain PU(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 17}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 14}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 11}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 29}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 26}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 23}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {32, 7}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain7(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {37, 7}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain8(k = V_base) annotation(
      Placement(visible = true, transformation(origin = {42, 7}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Sources.Step step1(height = 1, startTime = 0.3) annotation(
      Placement(visible = true, transformation(origin = {-35, 35}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.Grid grid1 annotation(
      Placement(visible = true, transformation(origin = {-107, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(grid1.C, currentSensor3.p) annotation(
      Line(points = {{-107, -16}, {-99, -16}, {-99, -16}, {-99, -16}}, color = {0, 0, 255}));
    connect(grid1.B, currentSensor2.p) annotation(
      Line(points = {{-107, -10}, {-94, -10}, {-94, -10}, {-94, -10}}, color = {0, 0, 255}));
    connect(grid1.A, currentSensor1.p) annotation(
      Line(points = {{-107, -4}, {-88, -4}, {-88, -4}, {-88, -4}}, color = {0, 0, 255}));
    connect(step1.y, CurrentController1.i_d_ref) annotation(
      Line(points = {{-34, 35}, {-30, 35}, {-30, 35}, {-29, 35}}, color = {0, 0, 127}));
    connect(inductor1.n, conv_a.p) annotation(
      Line(points = {{-9, -4}, {27, -4}}, color = {0, 0, 255}));
    connect(resistor1.n, inductor1.p) annotation(
      Line(points = {{-27, -4}, {-15, -4}}, color = {0, 0, 255}));
    connect(gain5.y, dq_Transform1.a) annotation(
      Line(points = {{-60, 23}, {-57, 23}}, color = {0, 0, 127}));
    connect(currentSensor1.i, gain5.u) annotation(
      Line(points = {{-85, -1}, {-85, 23}, {-62, 23}}, color = {0, 0, 127}));
    connect(gain8.y, conv_c.v) annotation(
      Line(points = {{42, 6}, {42, 6}, {42, -14}, {42, -14}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.c, gain8.u) annotation(
      Line(points = {{29, 31}, {42, 31}, {42, 8}, {42, 8}}, color = {0, 0, 127}));
    connect(gain7.y, conv_b.v) annotation(
      Line(points = {{37, 6}, {37, 6}, {37, -8}, {37, -8}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.b, gain7.u) annotation(
      Line(points = {{29, 27}, {37, 27}, {37, 8}, {37, 8}}, color = {0, 0, 127}));
    connect(conv_a.v, gain6.y) annotation(
      Line(points = {{32, -2}, {32, -2}, {32, 6}, {32, 6}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.a, gain6.u) annotation(
      Line(points = {{29, 23}, {32, 23}, {32, 8}, {32, 8}, {32, 8}}, color = {0, 0, 127}));
    connect(currentSensor3.i, gain3.u) annotation(
      Line(points = {{-96, -13}, {-96, -13}, {-96, 29}, {-62, 29}, {-62, 29}}, color = {0, 0, 127}));
    connect(currentSensor2.i, gain4.u) annotation(
      Line(points = {{-91, -7}, {-91, -7}, {-91, 26}, {-62, 26}, {-62, 26}}, color = {0, 0, 127}));
    connect(gain4.y, dq_Transform1.b) annotation(
      Line(points = {{-60, 26}, {-57, 26}, {-57, 26}, {-57, 26}}, color = {0, 0, 127}));
    connect(gain3.y, dq_Transform1.c) annotation(
      Line(points = {{-60, 29}, {-57, 29}, {-57, 29}, {-57, 29}}, color = {0, 0, 127}));
    connect(gain2.y, Voltagedq_Transformer.a) annotation(
      Line(points = {{-60, 11}, {-53, 11}, {-53, 11}, {-53, 11}}, color = {0, 0, 127}));
    connect(gain1.y, Voltagedq_Transformer.b) annotation(
      Line(points = {{-60, 14}, {-53, 14}, {-53, 14}, {-53, 14}}, color = {0, 0, 127}));
    connect(PU.y, Voltagedq_Transformer.c) annotation(
      Line(points = {{-60, 17}, {-53, 17}, {-53, 17}, {-53, 17}}, color = {0, 0, 127}));
    connect(potentialSensor3.phi, PU.u) annotation(
      Line(points = {{-78, 10}, {-78, 10}, {-78, 17}, {-62, 17}, {-62, 17}}, color = {0, 0, 127}));
    connect(potentialSensor2.phi, gain1.u) annotation(
      Line(points = {{-72, 10}, {-72, 10}, {-72, 14}, {-62, 14}, {-62, 14}}, color = {0, 0, 127}));
    connect(gain2.u, potentialSensor1.phi) annotation(
      Line(points = {{-62, 11}, {-66, 11}, {-66, 10}, {-66, 10}, {-66, 10}}, color = {0, 0, 127}));
    connect(potentialSensor1.p, currentSensor1.n) annotation(
      Line(points = {{-66, 3}, {-66, -4}, {-82, -4}}, color = {0, 0, 255}));
    connect(potentialSensor2.p, currentSensor2.n) annotation(
      Line(points = {{-68, 4}, {-68, -10}, {-88, -10}}, color = {0, 0, 255}));
    connect(potentialSensor3.p, currentSensor3.n) annotation(
      Line(points = {{-74, 4}, {-74, -16}, {-93, -16}}, color = {0, 0, 255}));
    connect(currentSensor1.n, resistor1.p) annotation(
      Line(points = {{-81, -4}, {-33, -4}}, color = {0, 0, 255}));
    connect(currentSensor2.n, resistor2.p) annotation(
      Line(points = {{-87, -10}, {-33, -10}}, color = {0, 0, 255}));
    connect(currentSensor3.n, resistor3.p) annotation(
      Line(points = {{-92, -16}, {-33, -16}}, color = {0, 0, 255}));
    connect(iq_reference.y, CurrentController1.i_q_ref) annotation(
      Line(points = {{-34, 39}, {-31.9, 39}}, color = {0, 0, 127}));
    connect(CurrentController1.i_d_mes, dq_Transform1.d) annotation(
      Line(points = {{-29.5, 23.6}, {-47.5, 23.6}, {-47.5, 23.6}, {-47.5, 23.6}}, color = {0, 0, 127}));
    connect(dq_Transform1.q, CurrentController1.i_q_mes) annotation(
      Line(points = {{-49.2, 28.4}, {-31.2, 28.4}, {-31.2, 28.4}, {-31.2, 28.4}}, color = {0, 0, 127}));
    connect(CurrentController1.v_DCd_d, Inverse_dq_Transform1.d) annotation(
      Line(points = {{7.1, 19.1}, {15.1, 19.1}, {15.1, 23.1}, {17.1, 23.1}}, color = {0, 0, 127}));
    connect(CurrentController1.v_DCd_q, Inverse_dq_Transform1.q) annotation(
      Line(points = {{6.5, 33.5}, {16.5, 33.5}, {16.5, 29.5}, {18.5, 29.5}}, color = {0, 0, 127}));
    connect(Voltagedq_Transformer.q, CurrentController1.v_q_conv) annotation(
      Line(points = {{-45.2, 16.4}, {-31.2, 16.4}, {-31.2, 16.4}, {-31.2, 16.4}}, color = {0, 0, 127}));
    connect(Voltagedq_Transformer.d, CurrentController1.v_d_conv) annotation(
      Line(points = {{-45.2, 12.4}, {-31.2, 12.4}, {-31.2, 12.4}, {-31.2, 12.4}}, color = {0, 0, 127}));
    connect(resistor2.n, inductor2.p) annotation(
      Line(points = {{-27, -10}, {-17, -10}}, color = {0, 0, 255}));
    connect(resistor3.n, inductor3.p) annotation(
      Line(points = {{-27, -16}, {-17, -16}}, color = {0, 0, 255}));
    connect(inductor2.n, conv_b.p) annotation(
      Line(points = {{-11, -10}, {32, -10}}, color = {0, 0, 255}));
    connect(inductor3.n, conv_c.p) annotation(
      Line(points = {{-11, -16}, {37, -16}}, color = {0, 0, 255}));
    connect(conv_a.n, ground1.p) annotation(
      Line(points = {{33, -4}, {45, -4}, {45, -10}, {45, -10}}, color = {0, 0, 255}));
    connect(conv_b.n, ground1.p) annotation(
      Line(points = {{38, -10}, {50, -10}}, color = {0, 0, 255}));
    connect(conv_c.n, ground1.p) annotation(
      Line(points = {{43, -16}, {45, -16}, {45, -10}, {45, -10}}, color = {0, 0, 255}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-2, 4}, fillPattern = FillPattern.Solid, extent = {{-74, 74}, {74, -74}}, textString = "2LVSC_avg_V")}, coordinateSystem(grid = {1, 1})),
      Diagram(coordinateSystem(grid = {1, 1})),
      __OpenModelica_commandLineOptions = "");
  end TwoLVSC_avg_V_CC_test;

  model time_gain
    parameter Real Step_times[1, :] = {0.1:0.1:1};
    parameter Real gains[:, 1] = [10:(-1):(-10)];
    Modelica.Blocks.Interfaces.RealInput u annotation(
      Placement(visible = true, transformation(origin = {-118, 2}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(visible = true, transformation(origin = {114, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  algorithm
    y := gains[1, sum(Step_times < time)] * u;
    annotation(
      Icon(graphics = {Line(origin = {-0.276393, 0}, points = {{-99.7236, 100}, {-99.7236, -100}, {100.276, 0}, {-99.7236, 100}})}));
  end time_gain;

  model TwoLVSC_switch_V_no_delay
    extends SystemParameters;
    constant Real pi = 2 * Modelica.Math.asin(1.0);
    constant Real phi1 = pi / 2;
    constant Real phi2 = (-2 * pi / 3) + phi1;
    constant Real phi3 = (-4 * pi / 3) + phi1;
    parameter Real i_q_ref = 0;
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor1 annotation(
      Placement(visible = true, transformation(origin = {-66, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor2 annotation(
      Placement(visible = true, transformation(origin = {-72, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.PotentialSensor potentialSensor3 annotation(
      Placement(visible = true, transformation(origin = {-78, 7}, extent = {{-3, -3}, {3, 3}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {-85, -4}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {-91, -10}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {-96, -16}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    TwoLevelVSC.dq_Transform Voltagedq_Transformer annotation(
      Placement(visible = true, transformation(origin = {-48, 14}, extent = {{-4, 4}, {4, -4}}, rotation = 0)));
    TwoLevelVSC.dq_Transform dq_Transform1 annotation(
      Placement(visible = true, transformation(origin = {-52, 26}, extent = {{-4, 4}, {4, -4}}, rotation = 0)));
    TwoLevelVSC.ff ff1 annotation(
      Placement(visible = true, transformation(origin = {-27, 66}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    TwoLevelVSC.CurrentController CurrentController1 annotation(
      Placement(visible = true, transformation(origin = {-13, 26}, extent = {{-15, 15}, {15, -15}}, rotation = 0)));
    Modelica.Blocks.Sources.Step iq_reference(height = 0, offset = 0, startTime = 0) annotation(
      Placement(visible = true, transformation(origin = {-35, 39}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-27, 50}, extent = {{5, 5}, {-5, -5}}, rotation = 0)));
    TwoLevelVSC.Inverse_dq_Transform Inverse_dq_Transform1 annotation(
      Placement(visible = true, transformation(origin = {23, 27}, extent = {{-5, 5}, {5, -5}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor1(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor2(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor resistor3(R = R) annotation(
      Placement(visible = true, transformation(origin = {-28, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor1(L = L, i(fixed = false)) annotation(
      Placement(visible = true, transformation(origin = {-12, -4}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor2(L = L, i(fixed = false, start = 0)) annotation(
      Placement(visible = true, transformation(origin = {-12, -10}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor inductor3(L = L, i(fixed = false, start = 0)) annotation(
      Placement(visible = true, transformation(origin = {-12, -16}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = 2 * C, v(fixed = true, start = V_DC_start / 2)) annotation(
      Placement(visible = true, transformation(origin = {79, -11}, extent = {{-7, -7}, {7, 7}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
      Placement(visible = true, transformation(origin = {88, -21}, extent = {{-4, -4}, {4, 4}}, rotation = 90)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor4 annotation(
      Placement(visible = true, transformation(origin = {90, 0}, extent = {{3, -3}, {-3, 3}}, rotation = 180)));
    Modelica.Blocks.Sources.Step step1(height = V_DC_step, offset = V_DC_start * 1.2, startTime = V_DC_step_time) annotation(
      Placement(visible = true, transformation(origin = {47, 50}, extent = {{-4, -4}, {4, 4}}, rotation = 180)));
    Modelica.Blocks.Continuous.TransferFunction CRC(a = {1, 0}, b = {K_CRCP, K_CRCI}) annotation(
      Placement(visible = true, transformation(origin = {-1, 50}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback feedback2 annotation(
      Placement(visible = true, transformation(origin = {23, 50}, extent = {{7, 7}, {-7, -7}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseA annotation(
      Placement(visible = true, transformation(origin = {-102, -4}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, 71}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseB annotation(
      Placement(visible = true, transformation(origin = {-102, -10}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, -1}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin PhaseC annotation(
      Placement(visible = true, transformation(origin = {-102, -16}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-101, -69}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {105, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {101, 41}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2 annotation(
      Placement(visible = true, transformation(origin = {98, -37}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {101, -41}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
    Modelica.Blocks.Math.Gain PU(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 17}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 14}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / V_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 11}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 29}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 26}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain5(k = 1 / I_base) annotation(
      Placement(visible = true, transformation(origin = {-61, 23}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain6(k = 1) annotation(
      Placement(visible = true, transformation(origin = {32, 20}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain7(k = 1) annotation(
      Placement(visible = true, transformation(origin = {37, 20}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain8(k = 1) annotation(
      Placement(visible = true, transformation(origin = {42, 20}, extent = {{-1, -1}, {1, 1}}, rotation = -90)));
    Modelica.Blocks.Math.Gain gain9(k = 1 / I_DC_base) annotation(
      Placement(visible = true, transformation(origin = {90, 32}, extent = {{-1, -1}, {1, 1}}, rotation = 90)));
    Modelica.Blocks.Math.Gain gain10(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {32, 74}, extent = {{-1, -1}, {1, 1}}, rotation = 180)));
    Modelica.Blocks.Math.Gain gain12(k = 1 / V_DC_base) annotation(
      Placement(visible = true, transformation(origin = {34, 50}, extent = {{-1, -1}, {1, 1}}, rotation = 180)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor2(C = 2 * C, v(fixed = true, start = -V_DC_start / 2)) annotation(
      Placement(visible = true, transformation(origin = {79, -30}, extent = {{-7, -7}, {7, 7}}, rotation = 90)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made1 annotation(
      Placement(visible = true, transformation(origin = {34, -2}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made2 annotation(
      Placement(visible = true, transformation(origin = {39, -2}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made3 annotation(
      Placement(visible = true, transformation(origin = {44, -2}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made4 annotation(
      Placement(visible = true, transformation(origin = {39, -30}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made5 annotation(
      Placement(visible = true, transformation(origin = {34, -30}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.IGBT_self_made iGBT_self_made6 annotation(
      Placement(visible = true, transformation(origin = {44, -30}, extent = {{-1, -1}, {1, 1}}, rotation = 0)));
    TwoLevelVSC.PWM_self_made_2 pWM_self_made_21 annotation(
      Placement(visible = true, transformation(origin = {37.5, 8.5}, extent = {{-6.5, -6.5}, {6.5, 6.5}}, rotation = -90)));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor1 annotation(
      Placement(visible = true, transformation(origin = {66.5, -17.5}, extent = {{-3.5, -3.5}, {3.5, 3.5}}, rotation = -90)));
  equation
    connect(gain10.u, voltageSensor1.v) annotation(
      Line(points = {{33, 74}, {63, 74}, {63, -17}, {63, -17}}, color = {0, 0, 127}));
    connect(gain9.u, currentSensor4.i) annotation(
      Line(points = {{90, 31}, {90, 31}, {90, 3}, {90, 3}}, color = {0, 0, 127}));
    connect(gain3.u, currentSensor3.i) annotation(
      Line(points = {{-62, 29}, {-96, 29}, {-96, -13}, {-96, -13}}, color = {0, 0, 127}));
    connect(gain4.u, currentSensor2.i) annotation(
      Line(points = {{-62, 26}, {-91, 26}, {-91, -7}, {-91, -7}}, color = {0, 0, 127}));
    connect(gain5.u, currentSensor1.i) annotation(
      Line(points = {{-62, 23}, {-85, 23}, {-85, -1}, {-85, -1}}, color = {0, 0, 127}));
    connect(PU.u, potentialSensor3.phi) annotation(
      Line(points = {{-62, 17}, {-78, 17}, {-78, 10}, {-78, 10}}, color = {0, 0, 127}));
    connect(gain1.u, potentialSensor2.phi) annotation(
      Line(points = {{-62, 14}, {-72, 14}, {-72, 10}, {-72, 10}}, color = {0, 0, 127}));
    connect(potentialSensor1.phi, gain2.u) annotation(
      Line(points = {{-66, 10}, {-66, 10}, {-66, 11}, {-62, 11}, {-62, 11}}, color = {0, 0, 127}));
    connect(currentSensor4.n, V_DC_2) annotation(
      Line(points = {{93, 0}, {105, 0}}, color = {0, 0, 255}));
    connect(potentialSensor2.p, currentSensor2.n) annotation(
      Line(points = {{-71, 4}, {-71, -10}, {-88, -10}}, color = {0, 0, 255}));
    connect(potentialSensor3.p, currentSensor3.n) annotation(
      Line(points = {{-79, 4}, {-79, -16}, {-93, -16}}, color = {0, 0, 255}));
    connect(gain9.y, ff1.i_DC) annotation(
      Line(points = {{90, 33}, {90, 66}, {-23, 66}}, color = {0, 0, 127}));
    connect(gain10.y, feedback2.u2) annotation(
      Line(points = {{31, 74}, {23, 74}, {23, 56}}, color = {0, 0, 127}));
    connect(gain10.y, ff1.V_DC) annotation(
      Line(points = {{31, 74}, {-27, 74}, {-27, 70}}, color = {0, 0, 127}));
    connect(iq_reference.y, CurrentController1.i_q_ref) annotation(
      Line(points = {{-34, 39}, {-31.9, 39}}, color = {0, 0, 127}));
    connect(voltageSensor1.p, capacitor1.p) annotation(
      Line(points = {{66.5, -14}, {66, -14}, {66, 0}, {79, 0}, {79, -4}}, color = {0, 0, 255}));
    connect(voltageSensor1.n, capacitor2.p) annotation(
      Line(points = {{66.5, -21}, {66.5, -37}, {79, -37}}, color = {0, 0, 255}));
    connect(capacitor2.p, m_V_DC_2) annotation(
      Line(points = {{79, -37}, {98, -37}}, color = {0, 0, 255}));
    connect(capacitor1.p, currentSensor4.p) annotation(
      Line(points = {{78, -4}, {78, 0}, {87, 0}}, color = {0, 0, 255}));
    connect(feedback2.u1, gain12.y) annotation(
      Line(points = {{29, 50}, {33, 50}}, color = {0, 0, 127}));
    connect(gain12.u, step1.y) annotation(
      Line(points = {{35, 50}, {43, 50}}, color = {0, 0, 127}));
    connect(CRC.u, feedback2.y) annotation(
      Line(points = {{6, 50}, {18, 50}}, color = {0, 0, 127}));
    connect(feedback1.u1, CRC.y) annotation(
      Line(points = {{-23, 50}, {-8, 50}}, color = {0, 0, 127}));
    connect(pWM_self_made_21.a_not_fire, iGBT_self_made5.fire) annotation(
      Line(points = {{32, 1}, {32, 1}, {32, -30}, {33, -30}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.a_fire, iGBT_self_made1.fire) annotation(
      Line(points = {{33, 1}, {32, 1}, {32, -2}, {33, -2}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.b_not_fire, iGBT_self_made4.fire) annotation(
      Line(points = {{37, 1}, {38, 1}, {38, -30}, {38, -30}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.b_fire, iGBT_self_made2.fire) annotation(
      Line(points = {{38, 1}, {38, 1}, {38, -2}, {38, -2}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.c_not_fire, iGBT_self_made6.fire) annotation(
      Line(points = {{42, 1}, {42, 1}, {42, -30}, {43, -30}}, color = {255, 0, 255}));
    connect(pWM_self_made_21.c_fire, iGBT_self_made3.fire) annotation(
      Line(points = {{43, 1}, {42, 1}, {42, -2}, {43, -2}}, color = {255, 0, 255}));
    connect(gain8.y, pWM_self_made_21.c) annotation(
      Line(points = {{42, 19}, {42, 19}, {42, 16}, {43, 16}}, color = {0, 0, 127}));
    connect(gain7.y, pWM_self_made_21.b) annotation(
      Line(points = {{37, 19}, {37, 17.5}, {38, 17.5}, {38, 17}}, color = {0, 0, 127}));
    connect(gain6.y, pWM_self_made_21.a) annotation(
      Line(points = {{32, 19}, {32, 16}}, color = {0, 0, 127}));
    connect(iGBT_self_made1.pin_n, capacitor1.p) annotation(
      Line(points = {{34, -1}, {34, -1}, {34, 0}, {79, 0}, {79, -4}, {79, -4}}, color = {0, 0, 255}));
    connect(iGBT_self_made2.pin_n, capacitor1.p) annotation(
      Line(points = {{39, -1}, {39, -1}, {39, 0}, {79, 0}, {79, -4}, {79, -4}}, color = {0, 0, 255}));
    connect(iGBT_self_made3.pin_n, capacitor1.p) annotation(
      Line(points = {{44, -1}, {44, -1}, {44, 0}, {79, 0}, {79, -4}, {79, -4}}, color = {0, 0, 255}));
    connect(iGBT_self_made5.pin, capacitor2.p) annotation(
      Line(points = {{34, -31}, {34, -31}, {34, -37}, {79, -37}, {79, -37}}, color = {0, 0, 255}));
    connect(iGBT_self_made4.pin, capacitor2.p) annotation(
      Line(points = {{39, -31}, {39, -31}, {39, -37}, {79, -37}, {79, -37}}, color = {0, 0, 255}));
    connect(iGBT_self_made6.pin, capacitor2.p) annotation(
      Line(points = {{44, -31}, {44, -31}, {44, -37}, {79, -37}, {79, -37}}, color = {0, 0, 255}));
    connect(iGBT_self_made6.pin_n, iGBT_self_made3.pin) annotation(
      Line(points = {{44, -29}, {44, -29}, {44, -3}, {44, -3}}, color = {0, 0, 255}));
    connect(iGBT_self_made4.pin_n, iGBT_self_made2.pin) annotation(
      Line(points = {{39, -29}, {39, -29}, {39, -3}, {39, -3}}, color = {0, 0, 255}));
    connect(iGBT_self_made5.pin_n, iGBT_self_made1.pin) annotation(
      Line(points = {{34, -29}, {34, -29}, {34, -3}, {34, -3}}, color = {0, 0, 255}));
    connect(inductor3.n, iGBT_self_made3.pin) annotation(
      Line(points = {{-9, -16}, {44, -16}, {44, -3}, {44, -3}}, color = {0, 0, 255}));
    connect(inductor2.n, iGBT_self_made2.pin) annotation(
      Line(points = {{-9, -10}, {39, -10}, {39, -3}, {39, -3}}, color = {0, 0, 255}));
    connect(inductor1.n, iGBT_self_made1.pin) annotation(
      Line(points = {{-9, -4}, {34, -4}, {34, -3}, {34, -3}}, color = {0, 0, 255}));
    connect(ground2.p, capacitor1.n) annotation(
      Line(points = {{84, -21}, {79, -21}, {79, -19}, {79, -19}, {79, -18}}, color = {0, 0, 255}));
    connect(capacitor1.n, capacitor2.n) annotation(
      Line(points = {{79, -18}, {79, -18}, {79, -23}, {79, -23}}, color = {0, 0, 255}));
    connect(Inverse_dq_Transform1.b, gain7.u) annotation(
      Line(points = {{29, 27}, {37, 27}, {37, 21}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.c, gain8.u) annotation(
      Line(points = {{29, 31}, {42, 31}, {42, 20}}, color = {0, 0, 127}));
    connect(Inverse_dq_Transform1.a, gain6.u) annotation(
      Line(points = {{29, 23}, {32, 23}, {32, 22}}, color = {0, 0, 127}));
    connect(gain5.y, dq_Transform1.a) annotation(
      Line(points = {{-60, 23}, {-57, 23}}, color = {0, 0, 127}));
    connect(gain4.y, dq_Transform1.b) annotation(
      Line(points = {{-60, 26}, {-57, 26}, {-57, 26}, {-57, 26}}, color = {0, 0, 127}));
    connect(gain3.y, dq_Transform1.c) annotation(
      Line(points = {{-60, 29}, {-57, 29}, {-57, 29}, {-57, 29}}, color = {0, 0, 127}));
    connect(gain2.y, Voltagedq_Transformer.a) annotation(
      Line(points = {{-60, 11}, {-53, 11}, {-53, 11}, {-53, 11}}, color = {0, 0, 127}));
    connect(gain1.y, Voltagedq_Transformer.b) annotation(
      Line(points = {{-60, 14}, {-53, 14}, {-53, 14}, {-53, 14}}, color = {0, 0, 127}));
    connect(PU.y, Voltagedq_Transformer.c) annotation(
      Line(points = {{-60, 17}, {-53, 17}, {-53, 17}, {-53, 17}}, color = {0, 0, 127}));
    connect(potentialSensor1.p, currentSensor1.n) annotation(
      Line(points = {{-66, 3}, {-66, -4}, {-82, -4}}, color = {0, 0, 255}));
    connect(currentSensor1.n, resistor1.p) annotation(
      Line(points = {{-81, -4}, {-33, -4}}, color = {0, 0, 255}));
    connect(PhaseA, currentSensor1.p) annotation(
      Line(points = {{-102, -4}, {-87, -4}}, color = {0, 0, 255}));
    connect(currentSensor2.n, resistor2.p) annotation(
      Line(points = {{-87, -10}, {-33, -10}}, color = {0, 0, 255}));
    connect(PhaseB, currentSensor2.p) annotation(
      Line(points = {{-102, -10}, {-93, -10}}, color = {0, 0, 255}));
    connect(currentSensor3.n, resistor3.p) annotation(
      Line(points = {{-92, -16}, {-33, -16}}, color = {0, 0, 255}));
    connect(PhaseC, currentSensor3.p) annotation(
      Line(points = {{-102, -16}, {-98, -16}}, color = {0, 0, 255}));
    connect(feedback1.y, CurrentController1.i_d_ref) annotation(
      Line(points = {{-31.5, 50}, {-40, 50}, {-40, 36}, {-32, 36}}, color = {0, 0, 127}));
    connect(ff1.ff, feedback1.u2) annotation(
      Line(points = {{-27, 62}, {-27, 58}, {-27.32, 58}, {-27.32, 54}}, color = {0, 0, 127}));
    connect(ff1.V_d, Voltagedq_Transformer.d) annotation(
      Line(points = {{-31, 66}, {-41.72, 66}, {-41.72, 12}, {-45.72, 12}}, color = {0, 0, 127}));
    connect(CurrentController1.i_d_mes, dq_Transform1.d) annotation(
      Line(points = {{-29.5, 23.6}, {-47.5, 23.6}, {-47.5, 23.6}, {-47.5, 23.6}}, color = {0, 0, 127}));
    connect(dq_Transform1.q, CurrentController1.i_q_mes) annotation(
      Line(points = {{-49.2, 28.4}, {-31.2, 28.4}, {-31.2, 28.4}, {-31.2, 28.4}}, color = {0, 0, 127}));
    connect(CurrentController1.v_DCd_d, Inverse_dq_Transform1.d) annotation(
      Line(points = {{7.1, 19.1}, {15.1, 19.1}, {15.1, 23.1}, {17.1, 23.1}}, color = {0, 0, 127}));
    connect(CurrentController1.v_DCd_q, Inverse_dq_Transform1.q) annotation(
      Line(points = {{6.5, 33.5}, {16.5, 33.5}, {16.5, 29.5}, {18.5, 29.5}}, color = {0, 0, 127}));
    connect(Voltagedq_Transformer.q, CurrentController1.v_q_conv) annotation(
      Line(points = {{-45.2, 16.4}, {-31.2, 16.4}, {-31.2, 16.4}, {-31.2, 16.4}}, color = {0, 0, 127}));
    connect(Voltagedq_Transformer.d, CurrentController1.v_d_conv) annotation(
      Line(points = {{-45.2, 12.4}, {-31.2, 12.4}, {-31.2, 12.4}, {-31.2, 12.4}}, color = {0, 0, 127}));
    connect(resistor1.n, inductor1.p) annotation(
      Line(points = {{-27, -4}, {-17, -4}}, color = {0, 0, 255}));
    connect(resistor2.n, inductor2.p) annotation(
      Line(points = {{-27, -10}, {-17, -10}}, color = {0, 0, 255}));
    connect(resistor3.n, inductor3.p) annotation(
      Line(points = {{-27, -16}, {-17, -16}}, color = {0, 0, 255}));
    annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-2, 4}, fillPattern = FillPattern.Solid, extent = {{-74, 74}, {74, -74}}, textString = "2LVSC_switch_V")}, coordinateSystem(grid = {1, 1}, initialScale = 0.1)),
      Diagram(coordinateSystem(grid = {1, 1})),
      __OpenModelica_commandLineOptions = "",
      experiment(StartTime = 0, StopTime = 0.5, Tolerance = 1e-6, Interval = 0.002),
      __OpenModelica_simulationFlags(lv = "LOG_STATS", r = "SwitchingVariables_res.mat", s = "dassl"));
  end TwoLVSC_switch_V_no_delay;

  model ThyristorSwitch
    Modelica.Blocks.Interfaces.BooleanInput fire annotation(
      Placement(visible = true, transformation(origin = {-88, -8}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-88, -8}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin pin annotation(
      Placement(visible = true, transformation(origin = {0, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(
      Placement(visible = true, transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Ideal.IdealDiode idealDiode1 annotation(
      Placement(visible = true, transformation(origin = {32, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Semiconductors.Thyristor thyristor1 annotation(
      Placement(visible = true, transformation(origin = {-42, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  equation
    connect(thyristor1.Anode, pin) annotation(
      Line(points = {{-42, -18}, {-42, -18}, {-42, -80}, {0, -80}, {0, -92}}, color = {0, 0, 255}));
    connect(pin_n, thyristor1.Cathode) annotation(
      Line(points = {{0, 90}, {-42, 90}, {-42, 2}, {-42, 2}}, color = {0, 0, 255}));
    connect(fire, thyristor1.Gate) annotation(
      Line(points = {{-88, -8}, {-52, -8}, {-52, 0}, {-52, 0}}, color = {255, 0, 255}));
    connect(idealDiode1.p, pin) annotation(
      Line(points = {{32, -16}, {32, -80}, {0, -80}, {0, -94}}, color = {0, 0, 255}));
    connect(idealDiode1.n, pin_n) annotation(
      Line(points = {{32, 4}, {32, 84}, {0, 84}, {0, 92}}, color = {0, 0, 255}));
    annotation(
      Icon(coordinateSystem(initialScale = 0.1), graphics = {Line(origin = {29.9472, 35}, points = {{-29.9472, 55}, {-29.9472, 25}, {10.0528, 25}, {10.0528, -15}, {-9.94721, -15}, {30.0528, -15}, {10.0528, -15}, {10.0528, -15}, {-9.94721, -55}, {30.0528, -55}, {10.0528, -15}}), Line(origin = {20, -54}, points = {{20, 34}, {20, -6}, {-20, -6}, {-20, -34}}), Line(origin = {-29.9472, -35}, points = {{29.9472, -55}, {29.9472, -25}, {-10.0528, -25}, {-10.0528, 15}, {-30.0528, 15}, {9.94721, 15}, {-10.0528, 15}, {9.94721, 55}, {-30.0528, 55}, {-10.0528, 15}}), Line(origin = {-20, 55}, points = {{-20, -35}, {-20, 5}, {20, 5}, {20, 35}}), Line(origin = {-72.7929, -11.7929}, points = {{-21.2071, -14.2071}, {-7.20711, -14.2071}, {20.7929, 13.7929}})}));
  end ThyristorSwitch;
  annotation(
    uses(Modelica(version = "3.2.2")));
end TwoLevelVSC;
