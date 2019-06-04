package Cable
  model CableParameters
    parameter Real l = 100/5;
    parameter Real S_n = 1000e6;
    parameter Real V_n = 313.5e3 "line-to-line RMS voltage";
    parameter Real V_DC_base = 2 * sqrt(2 / 3) * V_n;
    parameter Real S_base = S_n;
    parameter Real C = l * 0.1983e-6;
    parameter Real G = l * 7.633e-11;
    parameter Real R1 = l * 1.1724e-1;
    parameter Real R2 = l * 8 - 2072e-2;
    parameter Real R3 = l * 1.1946e-2;
    parameter Real L1 = l * 2.2861e-4;
    parameter Real L2 = l * 1.5522e-3;
    parameter Real L3 = l * 3.29433e-3;
  end CableParameters;








  model PiSection
  extends CableParameters;
    Modelica.Electrical.Analog.Interfaces.Pin pin annotation(
      Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(
      Placement(visible = true, transformation(origin = {78, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor R_1(R = R1) annotation(
      Placement(visible = true, transformation(origin = {-15, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor R_2(R = R2) annotation(
      Placement(visible = true, transformation(origin = {-15, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Resistor R_3(R = R3) annotation(
      Placement(visible = true, transformation(origin = {-15, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor L_1(L = L1) annotation(
      Placement(visible = true, transformation(origin = {15, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor L_2(L = L2) annotation(
      Placement(visible = true, transformation(origin = {15, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Inductor L_3(L = L3) annotation(
      Placement(visible = true, transformation(origin = {15, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Conductor conductor1(G = G) annotation(
      Placement(visible = true, transformation(origin = {-40, -70}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Conductor conductor2(G = G) annotation(
      Placement(visible = true, transformation(origin = {40, -70}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = C) annotation(
      Placement(visible = true, transformation(origin = {-60, -70}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor2(C = C) annotation(
      Placement(visible = true, transformation(origin = {60, -70}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {0, -96}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(capacitor2.n, ground1.p) annotation(
      Line(points = {{60, -80}, {60, -80}, {60, -86}, {0, -86}, {0, -86}}, color = {0, 0, 255}));
    connect(conductor2.n, ground1.p) annotation(
      Line(points = {{40, -80}, {40, -80}, {40, -86}, {0, -86}, {0, -86}}, color = {0, 0, 255}));
    connect(capacitor1.n, ground1.p) annotation(
      Line(points = {{-60, -80}, {-60, -80}, {-60, -86}, {0, -86}, {0, -86}}, color = {0, 0, 255}));
    connect(conductor1.n, ground1.p) annotation(
      Line(points = {{-40, -80}, {-40, -80}, {-40, -86}, {0, -86}, {0, -86}}, color = {0, 0, 255}));
    connect(pin_n, conductor2.p) annotation(
      Line(points = {{78, 0}, {40, 0}, {40, -60}, {40, -60}}, color = {0, 0, 255}));
    connect(pin_n, capacitor2.p) annotation(
      Line(points = {{78, 0}, {60, 0}, {60, -60}, {60, -60}}, color = {0, 0, 255}));
    connect(pin, conductor1.p) annotation(
      Line(points = {{-80, 0}, {-40, 0}, {-40, -60}, {-40, -60}}, color = {0, 0, 255}));
    connect(pin, capacitor1.p) annotation(
      Line(points = {{-80, 0}, {-60, 0}, {-60, -60}, {-60, -60}}, color = {0, 0, 255}));
    connect(L_2.n, pin_n) annotation(
      Line(points = {{26, 0}, {78, 0}, {78, 0}, {78, 0}}, color = {0, 0, 255}));
    connect(R_2.n, L_2.p) annotation(
      Line(points = {{-4, 0}, {4, 0}, {4, 0}, {4, 0}}, color = {0, 0, 255}));
    connect(R_3.n, L_3.p) annotation(
      Line(points = {{-4, -30}, {4, -30}, {4, -32}, {4, -32}}, color = {0, 0, 255}));
    connect(R_3.p, R_2.p) annotation(
      Line(points = {{-26, -30}, {-24, -30}, {-24, 0}, {-26, 0}}, color = {0, 0, 255}));
    connect(L_3.n, L_2.n) annotation(
      Line(points = {{26, -32}, {26, -32}, {26, 0}, {26, 0}}, color = {0, 0, 255}));
    connect(L_1.n, L_2.n) annotation(
      Line(points = {{26, 30}, {26, 30}, {26, 0}, {26, 0}}, color = {0, 0, 255}));
    connect(R_1.n, L_1.p) annotation(
      Line(points = {{-4, 30}, {4, 30}, {4, 30}, {4, 30}}, color = {0, 0, 255}));
    connect(R_1.p, R_2.p) annotation(
      Line(points = {{-26, 30}, {-24, 30}, {-24, 0}, {-26, 0}}, color = {0, 0, 255}));
    connect(pin, R_2.p) annotation(
      Line(points = {{-80, -2}, {-24, -2}, {-24, 0}, {-26, 0}}, color = {0, 0, 255}));
    annotation(
      Icon(graphics = {Line(origin = {-0.5, 0}, points = {{-99.5, 0}, {100.5, 0}, {-97.5, 0}}), Line(origin = {-40, -40}, points = {{0, 40}, {0, -40}, {0, 40}}), Line(origin = {40, -40}, points = {{0, 40}, {0, -40}, {0, 40}})}));
  end PiSection;




  model HVDC_Cable
  Real P_pu_in;
  Real V_pu_in;
  Real P_pu_out;
  Real V_pu_out;
  protected
  Real P_pu_upper;
  Real P_pu_lower;
  Real V__pu_upper;
  Real V__pu_lower;
  extends CableParameters;
    Modelica.Electrical.Analog.Interfaces.Pin V_DC_2_in annotation(
      Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin m_V_DC_2_in annotation(
      Placement(visible = true, transformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.NegativePin V_DC_2_out annotation(
      Placement(visible = true, transformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.NegativePin m_V_DC_2_out annotation(
      Placement(visible = true, transformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Cable.PiSection piSection1 annotation(
      Placement(visible = true, transformation(origin = {-70, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Cable.PiSection piSection2 annotation(
      Placement(visible = true, transformation(origin = {-30, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Cable.PiSection piSection3 annotation(
      Placement(visible = true, transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Cable.PiSection piSection4 annotation(
      Placement(visible = true, transformation(origin = {34, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Cable.PiSection piSection5 annotation(
      Placement(visible = true, transformation(origin = {70, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Cable.PiSection piSection6 annotation(
      Placement(visible = true, transformation(origin = {-70, -60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Cable.PiSection piSection7 annotation(
      Placement(visible = true, transformation(origin = {-30, -60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Cable.PiSection piSection8 annotation(
      Placement(visible = true, transformation(origin = {0, -60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Cable.PiSection piSection9 annotation(
      Placement(visible = true, transformation(origin = {30, -60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Cable.PiSection piSection10 annotation(
      Placement(visible = true, transformation(origin = {70, -60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  equation
  P_pu_upper = V_DC_2_in.v*V_DC_2_in.i/S_base;
  P_pu_lower = m_V_DC_2_in.v*m_V_DC_2_in.i/S_base;
  V__pu_upper = V_DC_2_in.v/V_DC_base;
  V__pu_lower = m_V_DC_2_in.v/V_DC_base;
  P_pu_in = P_pu_upper  + P_pu_lower;
  V_pu_in = V__pu_upper - V__pu_lower;
  P_pu_out = -(V_DC_2_out.v*V_DC_2_out.i + m_V_DC_2_out.v*m_V_DC_2_out.i)/S_base;
  V_pu_out = (V_DC_2_out.v - m_V_DC_2_out.v)/V_DC_base;
    connect(piSection10.pin_n, m_V_DC_2_out) annotation(
      Line(points = {{80, -60}, {98, -60}, {98, -60}, {100, -60}}, color = {0, 0, 255}));
    connect(piSection9.pin_n, piSection10.pin) annotation(
      Line(points = {{40, -60}, {60, -60}, {60, -60}, {60, -60}}, color = {0, 0, 255}));
    connect(piSection8.pin_n, piSection9.pin) annotation(
      Line(points = {{10, -60}, {22, -60}, {22, -60}, {20, -60}, {20, -60}}, color = {0, 0, 255}));
    connect(piSection7.pin_n, piSection8.pin) annotation(
      Line(points = {{-20, -60}, {-10, -60}, {-10, -60}, {-10, -60}}, color = {0, 0, 255}));
    connect(piSection6.pin_n, piSection7.pin) annotation(
      Line(points = {{-60, -60}, {-40, -60}, {-40, -60}, {-40, -60}}, color = {0, 0, 255}));
    connect(m_V_DC_2_in, piSection6.pin) annotation(
      Line(points = {{-100, -60}, {-80, -60}, {-80, -60}, {-80, -60}}, color = {0, 0, 255}));
    connect(piSection4.pin_n, piSection5.pin) annotation(
      Line(points = {{44, 60}, {60, 60}, {60, 60}, {60, 60}}, color = {0, 0, 255}));
    connect(piSection3.pin_n, piSection4.pin) annotation(
      Line(points = {{10, 60}, {24, 60}}, color = {0, 0, 255}));
    connect(piSection2.pin_n, piSection3.pin) annotation(
      Line(points = {{-20, 60}, {-10, 60}, {-10, 60}, {-10, 60}}, color = {0, 0, 255}));
    connect(piSection1.pin_n, piSection2.pin) annotation(
      Line(points = {{-60, 60}, {-40, 60}, {-40, 60}, {-40, 60}}, color = {0, 0, 255}));
    connect(piSection5.pin_n, V_DC_2_out) annotation(
      Line(points = {{80, 60}, {98, 60}, {98, 60}, {100, 60}}, color = {0, 0, 255}));
    connect(V_DC_2_in, piSection1.pin) annotation(
      Line(points = {{-100, 60}, {-80, 60}, {-80, 60}, {-80, 60}}, color = {0, 0, 255}));
  
  annotation(
      Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {2, 12}, fillPattern = FillPattern.Solid, extent = {{-84, 86}, {84, -86}}, textString = "HVDC_cable")}));
  end HVDC_Cable;












  annotation(
    uses(Modelica(version = "3.2.2")));
end Cable;
