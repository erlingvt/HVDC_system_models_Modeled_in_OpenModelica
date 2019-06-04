package Interface
  model Pin2Grid
    import Modelica.Constants.pi;
    parameter Real fb = fb;
    parameter Real Vb = Vb;
    parameter Real Ib = Ib;
    Modelica.Electrical.Analog.Interfaces.Pin a annotation(
      Placement(visible = true, transformation(origin = {104, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin b annotation(
      Placement(visible = true, transformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin c annotation(
      Placement(visible = true, transformation(origin = {104, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage1 annotation(
      Placement(visible = true, transformation(origin = {52, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage2 annotation(
      Placement(visible = true, transformation(origin = {40, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage3 annotation(
      Placement(visible = true, transformation(origin = {32, -40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
      Placement(visible = true, transformation(origin = {2, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput vdL annotation(
      Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput vqL annotation(
      Placement(visible = true, transformation(origin = {-120, 44}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput idL annotation(
      Placement(visible = true, transformation(origin = {-110, -54}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {-110, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput iqL annotation(
      Placement(visible = true, transformation(origin = {-110, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {-110, -90}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 1 / Ib) annotation(
      Placement(visible = true, transformation(origin = {-82, -54}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = 1 / Ib) annotation(
      Placement(visible = true, transformation(origin = {-82, -82}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 annotation(
      Placement(visible = true, transformation(origin = {60, -40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 annotation(
      Placement(visible = true, transformation(origin = {74, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor3 annotation(
      Placement(visible = true, transformation(origin = {86, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = Vb) annotation(
      Placement(visible = true, transformation(origin = {-80, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain4(k = Vb) annotation(
      Placement(visible = true, transformation(origin = {-80, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Interface.DQ_InvTransform dQ_InvTransform1 annotation(
      Placement(visible = true, transformation(origin = {-29, 61}, extent = {{-23, -23}, {23, 23}}, rotation = 0)));
  Interface.DQTransform dQTransform1 annotation(
      Placement(visible = true, transformation(origin = {-27, -69}, extent = {{19, -19}, {-19, 19}}, rotation = 0)));
  equation
    connect(dQTransform1.c, currentSensor2.i) annotation(
      Line(points = {{-4, -54}, {74, -54}, {74, -10}, {74, -10}}, color = {0, 0, 127}));
    connect(dQTransform1.b, currentSensor1.i) annotation(
      Line(points = {{-4, -68}, {60, -68}, {60, -50}, {60, -50}}, color = {0, 0, 127}));
    connect(dQTransform1.a, currentSensor3.i) annotation(
      Line(points = {{-4, -84}, {86, -84}, {86, 30}, {86, 30}}, color = {0, 0, 127}));
    connect(dQTransform1.q, gain2.u) annotation(
      Line(points = {{-50, -84}, {-68, -84}, {-68, -82}, {-70, -82}}, color = {0, 0, 127}));
    connect(dQTransform1.d, gain1.u) annotation(
      Line(points = {{-50, -54}, {-68, -54}, {-68, -54}, {-70, -54}}, color = {0, 0, 127}));
    connect(idL, gain1.y) annotation(
      Line(points = {{-110, -54}, {-101, -54}, {-101, -54}, {-92, -54}, {-92, -54}, {-93, -54}, {-93, -54}, {-94, -54}}, color = {0, 0, 127}));
    connect(dQ_InvTransform1.c, signalVoltage3.v) annotation(
      Line(points = {{-2, 42}, {32, 42}, {32, -32}, {32, -32}}, color = {0, 0, 127}));
    connect(dQ_InvTransform1.b, signalVoltage2.v) annotation(
      Line(points = {{-2, 62}, {40, 62}, {40, 8}, {40, 8}}, color = {0, 0, 127}));
    connect(dQ_InvTransform1.a, signalVoltage1.v) annotation(
      Line(points = {{-2, 80}, {52, 80}, {52, 48}, {52, 48}}, color = {0, 0, 127}));
    connect(gain3.y, dQ_InvTransform1.q) annotation(
      Line(points = {{-68, 44}, {-60, 44}, {-60, 42}, {-56, 42}}, color = {0, 0, 127}));
    connect(vqL, gain3.u) annotation(
      Line(points = {{-120, 44}, {-107, 44}, {-107, 44}, {-94, 44}, {-94, 44}, {-92, 44}}, color = {0, 0, 127}));
    connect(gain4.y, dQ_InvTransform1.d) annotation(
      Line(points = {{-68, 80}, {-61, 80}, {-61, 79}, {-57, 79}}, color = {0, 0, 127}));
    connect(currentSensor1.p, c) annotation(
      Line(points = {{70, -40}, {104, -40}, {104, -40}, {104, -40}}, color = {0, 0, 255}));
    connect(currentSensor2.p, b) annotation(
      Line(points = {{84, 0}, {102, 0}, {102, 0}, {104, 0}}, color = {0, 0, 255}));
    connect(currentSensor3.p, a) annotation(
      Line(points = {{96, 40}, {104, 40}, {104, 40}, {104, 40}}, color = {0, 0, 255}));
    connect(signalVoltage3.p, currentSensor1.n) annotation(
      Line(points = {{42, -40}, {50, -40}, {50, -40}, {50, -40}}, color = {0, 0, 255}));
    connect(signalVoltage2.p, currentSensor2.n) annotation(
      Line(points = {{50, 0}, {64, 0}, {64, 0}, {64, 0}}, color = {0, 0, 255}));
    connect(signalVoltage1.p, currentSensor3.n) annotation(
      Line(points = {{62, 40}, {76, 40}, {76, 40}, {76, 40}}, color = {0, 0, 255}));
    connect(vdL, gain4.u) annotation(
      Line(points = {{-120, 80}, {-92, 80}, {-92, 80}, {-92, 80}}, color = {0, 0, 127}));
    connect(iqL, gain2.y) annotation(
      Line(points = {{-110, -80}, {-94, -80}, {-94, -82}, {-92, -82}}, color = {0, 0, 127}));
    connect(ground1.p, signalVoltage1.n) annotation(
      Line(points = {{2, 1.49012e-07}, {12, 1.49012e-07}, {12, 40}, {42, 40}, {42, 40}, {42, 40}, {42, 40}}, color = {0, 0, 255}));
    connect(ground1.p, signalVoltage2.n) annotation(
      Line(points = {{2, 1.49012e-07}, {30, 1.49012e-07}, {30, 1.49012e-07}, {30, 1.49012e-07}}, color = {0, 0, 255}));
    connect(signalVoltage3.n, ground1.p) annotation(
      Line(points = {{22, -40}, {12, -40}, {12, 0}, {2, 0}, {2, 0}}, color = {0, 0, 255}));
    annotation(
      uses(Modelica(version = "3.2.2")),
      Icon(graphics = {Rectangle(fillColor = {214, 214, 214}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, -2}, extent = {{-40, 28}, {40, -28}}, textString = "Grid"), Text(origin = {-79, 92}, extent = {{-11, 6}, {11, -6}}, textString = "vdL"), Text(origin = {-79, 54}, extent = {{-11, 6}, {11, -6}}, textString = "vqL"), Text(origin = {-79, -48}, extent = {{-11, 6}, {11, -6}}, textString = "idL"), Text(origin = {-77, -90}, extent = {{-11, 6}, {11, -6}}, textString = "iqL")}, coordinateSystem(initialScale = 0.1)));
  end Pin2Grid;

  model RefrenceFrameTransformation
    Real delta;
    Real Isre;
    Real Isim;
    Modelica.Blocks.Interfaces.RealInput IdL annotation(
      Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput IqL annotation(
      Placement(visible = true, transformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput vdL annotation(
      Placement(visible = true, transformation(origin = {-110, -40}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {-110, -90}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput vqL annotation(
      Placement(visible = true, transformation(origin = {-110, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {-110, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    OpalRT.NonElectrical.Connector.PwPin p annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
////ANGLE CALCULATION/////////////////////////////////////////////////////////////////////
    delta = atan2(p.vi, p.vr);
////POWER PIN CURRENTS////////////////////////////////////////////////////////////////////
    p.ir = Isre;
    p.ii = Isim;
////TRANSFORMATIONS//////////////////////////////////////////////////////////////////////
// Transforms currents between local and global:
    IdL = Isre * cos(delta) + Isim * sin(delta);
    IqL = Isre * sin(delta) - Isim * cos(delta);
// Transforms grid voltage between local and global:
    vdL = p.vr * cos(delta) + p.vi * sin(delta);
    vqL = p.vr * sin(delta) - p.vi * cos(delta);
    annotation(
      uses(Modelica(version = "3.2.2")),
      Icon(graphics = {Rectangle(fillColor = {214, 214, 214}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-17, 15}, extent = {{-31, 15}, {59, -41}}, textString = "Transformations", fontSize = 15), Text(origin = {2, -19}, extent = {{-32, 15}, {32, -15}}, textString = "Reference frame", fontSize = 15), Text(origin = {-85, 49}, extent = {{-23, 9}, {23, -9}}, textString = "idL", fontSize = 10), Text(origin = {-85, 91}, extent = {{-23, 9}, {23, -9}}, textString = "iqL", fontSize = 10), Text(origin = {-83, -49}, extent = {{-23, 9}, {23, -9}}, textString = "vqL", fontSize = 10), Text(origin = {-81, -89}, extent = {{-23, 9}, {23, -9}}, textString = "vdL", fontSize = 10)}, coordinateSystem(initialScale = 0.1)),
      Diagram(coordinateSystem(initialScale = 0.1)));
  
  end RefrenceFrameTransformation;

  model PwPin_To_3PH_Grid
    parameter Real Vb = Vb;
    parameter Real Ib = Ib;
    parameter Real fb = fb;
    Modelica.Electrical.Analog.Interfaces.Pin a annotation(
      Placement(visible = true, transformation(origin = {104, 25}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin b annotation(
      Placement(visible = true, transformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Analog.Interfaces.Pin c annotation(
      Placement(visible = true, transformation(origin = {104, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    OpalRT.NonElectrical.Connector.PwPin pwPin1 annotation(
      Placement(visible = true, transformation(origin = {-110, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Interface.Pin2Grid pin2Grid1 annotation(
      Placement(visible = true, transformation(origin = {45, -3.10862e-15}, extent = {{-35, -35}, {35, 35}}, rotation = 0)));
  Interface.RefrenceFrameTransformation refrenceFrameTransformation1 annotation(
      Placement(visible = true, transformation(origin = {-47, -6.21725e-15}, extent = {{35, 35}, {-35, -35}}, rotation = 0)));
  equation
    connect(refrenceFrameTransformation1.IqL, pin2Grid1.iqL) annotation(
      Line(points = {{-8, -32}, {6, -32}, {6, -32}, {6, -32}}, color = {0, 0, 127}));
    connect(refrenceFrameTransformation1.IdL, pin2Grid1.idL) annotation(
      Line(points = {{-8, -18}, {6, -18}, {6, -18}, {6, -18}}, color = {0, 0, 127}));
    connect(refrenceFrameTransformation1.vqL, pin2Grid1.vqL) annotation(
      Line(points = {{-8, 18}, {8, 18}, {8, 18}, {6, 18}}, color = {0, 0, 127}));
    connect(refrenceFrameTransformation1.vdL, pin2Grid1.vdL) annotation(
      Line(points = {{-8, 32}, {6, 32}, {6, 32}, {6, 32}}, color = {0, 0, 127}));
    connect(pin2Grid1.c, c) annotation(
      Line(points = {{84, -25}, {100.8, -25}, {100.8, -24}, {104, -24}}, color = {0, 0, 255}));
    connect(pin2Grid1.b, b) annotation(
      Line(points = {{84, 0}, {104.8, 0}, {104.8, -2}}, color = {0, 0, 255}));
    connect(pin2Grid1.a, a) annotation(
      Line(points = {{84, 25}, {93.8, 25}, {93.8, 23}, {104, 23}}, color = {0, 0, 255}));
    connect(refrenceFrameTransformation1.p, pwPin1) annotation(
      Line(points = {{-86, -1}, {-90.5, -1}, {-90.5, 0}, {-110, 0}}));
    annotation(
      uses(Modelica(version = "3.2.2")),
      Icon(graphics = {Rectangle(fillColor = {214, 214, 214}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-3, 23}, extent = {{-21, 19}, {21, -19}}, textString = "Power Pin to", fontSize = 25), Text(origin = {-1, -6}, extent = {{-49, 24}, {49, -24}}, textString = "3 phase grid", fontSize = 25)}));
  
  end PwPin_To_3PH_Grid;

  model DQTransform
    parameter Real pi = 2 * Modelica.Math.asin(1.0);
    parameter Real w = 2 * pi * 50;
    Modelica.Blocks.Interfaces.RealInput a annotation(
      Placement(visible = true, transformation(origin = {-116, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput b annotation(
      Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput c annotation(
      Placement(visible = true, transformation(origin = {-126, -74}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput d annotation(
      Placement(visible = true, transformation(origin = {120, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput q annotation(
      Placement(visible = true, transformation(origin = {120, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    d = 2 / 3 * (a * cos(w * time) + b * cos(w * time - 2 * pi / 3) + c * cos(w * time - 4 * pi / 3));
    q = 2 / 3 * ((-a * sin(w * time)) - b * sin(w * time - 2 * pi / 3) - c * sin(w * time - 4 * pi / 3));
    annotation(
      Icon(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(origin = {0, -1}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 101}, {100, -99}}), Text(origin = {-60, 82}, fillPattern = FillPattern.Solid, extent = {{-34, 34}, {74, -90}}, textString = "123"), Line(points = {{-100, -100}, {100, 100}, {100, 100}}), Text(origin = {16, -37}, fillPattern = FillPattern.Solid, extent = {{-24, -35}, {54, 43}}, textString = "dq")}),
      uses(Modelica(version = "3.2.2")),
      Diagram);
  
  end DQTransform;

  model DQ_InvTransform
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
      Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(points = {{-100, -100}, {100, 100}, {100, 100}}), Text(origin = {-38, 68}, fillPattern = FillPattern.Solid, extent = {{28, 74}, {-46, -90}}, textString = "dq"), Text(origin = {74, -87}, fillPattern = FillPattern.Solid, extent = {{2, -1}, {-92, 75}}, textString = "123")}, coordinateSystem(initialScale = 0.1)),
      uses(Modelica(version = "3.2.2")));
  
  end DQ_InvTransform;





end Interface;
