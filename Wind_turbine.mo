model Wind_turbine
import Modelica.Constants.pi;
  //Parameters
  parameter Real Prated = 2e6;
parameter Real J = 3e6;
parameter Real B = 2e3;
parameter Real bladeLength = 40;
parameter Real rho = 1.2;
parameter Real bladeAngleTimeConstant = 1;
parameter Real w_0 = 2;
parameter Real lambda_opt = 8.1;
parameter Real Cp_max = 0.48;
  //Speed controller parameters:
  parameter Real speedControllerKp = 1e6/0.1;
parameter Real speedControllerKi = 1e6/0.1/10;
parameter Real speedControllerKt = 1;
parameter Real speedControllerUpperLimit = 5e6;
parameter Real speedControllerLowerLimit = 0;
  //Power controller parameters:
  parameter Real powerControllerKp = 0.5*5/1e6;
parameter Real powerControllerKi = 12*5/1e6/100;
parameter Real powerControllerKt = 1;
parameter Real powerControllerUpperLimit = 30;
parameter Real powerControllerLowerLimit = 0;

//protected
  parameter Real A = pi*bladeLength^2;
  //Region 2 and 3 boundary:
  parameter Real u_reg23bdry = (Prated / Cp_max / (0.5*rho*A))^(1/3); 
parameter Real w_reg23bdry = lambda_opt * u_reg23bdry / bladeLength;




  model Turbine
  parameter Real rho = rho;
  parameter Real A = A;
  parameter Real J = J;
  parameter Real B = B;
  parameter Real bladeAngleTimeConstant = bladeAngleTimeConstant;
  parameter Real bladeLength = bladeLength;
  parameter Real w_0 = w_0;
  
    Modelica.Blocks.Interfaces.RealInput WindSpeed annotation(
      Placement(visible = true, transformation(origin = {-220, 82}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-210, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product1 annotation(
      Placement(visible = true, transformation(origin = {-170, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = 1 / 2 * rho * A)  annotation(
      Placement(visible = true, transformation(origin = {-110, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product3 annotation(
      Placement(visible = true, transformation(origin = {-70, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Division division1 annotation(
      Placement(visible = true, transformation(origin = {-42, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Wind_turbine.cp_calculation cp_calculation1 annotation(
      Placement(visible = true, transformation(origin = {-110, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Wind_turbine.lambda_calculation lambda_calculation1(bladeLength = bladeLength)  annotation(
      Placement(visible = true, transformation(origin = {-148, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add3 add31(k1 = +1, k2 = -1, k3 = -1)  annotation(
      Placement(visible = true, transformation(origin = {10, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Tgen annotation(
      Placement(visible = true, transformation(origin = {-220, -38}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-210, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = 1 / J)  annotation(
      Placement(visible = true, transformation(origin = {50, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator1(y(fixed = true), y_start = w_0)  annotation(
      Placement(visible = true, transformation(origin = {84, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput omega annotation(
      Placement(visible = true, transformation(origin = {210, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {210, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain3(k = B)  annotation(
      Placement(visible = true, transformation(origin = {50, 22}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Pgen annotation(
      Placement(visible = true, transformation(origin = {210, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {210, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product4 annotation(
      Placement(visible = true, transformation(origin = {170, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput BladeAngle annotation(
      Placement(visible = true, transformation(origin = {-220, 2}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-210, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.TransferFunction transferFunction1(a = {bladeAngleTimeConstant, 1}, b = {1})  annotation(
      Placement(visible = true, transformation(origin = {-150, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product2 annotation(
      Placement(visible = true, transformation(origin = {-138, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
  connect(product2.y, gain1.u) annotation(
      Line(points = {{-127, 82}, {-123, 82}}, color = {0, 0, 127}));
  connect(product1.y, product2.u1) annotation(
      Line(points = {{-159, 82}, {-156, 82}, {-156, 82}, {-153, 82}, {-153, 88}, {-149, 88}, {-149, 87}, {-151, 87}, {-151, 88}}, color = {0, 0, 127}));
  connect(product2.u2, WindSpeed) annotation(
      Line(points = {{-150, 76}, {-156, 76}, {-156, 70}, {-190, 70}, {-190, 82}, {-220, 82}, {-220, 82}}, color = {0, 0, 127}));
  connect(BladeAngle, transferFunction1.u) annotation(
      Line(points = {{-220, 2}, {-191, 2}, {-191, 2}, {-162, 2}, {-162, 2}, {-162, 2}, {-162, 2}, {-162, 2}}, color = {0, 0, 127}));
  connect(transferFunction1.y, cp_calculation1.BladeAngle) annotation(
      Line(points = {{-139, 2}, {-133, 2}, {-133, 2}, {-127, 2}, {-127, 28}, {-123, 28}, {-123, 28}, {-124, 28}, {-124, 28}, {-123, 28}}, color = {0, 0, 127}));
  connect(product4.u1, integrator1.y) annotation(
      Line(points = {{158, -52}, {120, -52}, {120, 62}, {96, 62}, {96, 62}}, color = {0, 0, 127}));
  connect(product4.y, Pgen) annotation(
      Line(points = {{181, -58}, {205, -58}, {205, -58}, {209, -58}}, color = {0, 0, 127}));
  connect(product4.u2, Tgen) annotation(
      Line(points = {{158, -64}, {-186, -64}, {-186, -38}, {-220, -38}}, color = {0, 0, 127}));
  connect(gain3.u, integrator1.y) annotation(
      Line(points = {{62, 22}, {120, 22}, {120, 62}, {96, 62}}, color = {0, 0, 127}));
  connect(gain3.y, add31.u3) annotation(
      Line(points = {{39, 22}, {-8, 22}, {-8, 54}, {-2, 54}}, color = {0, 0, 127}));
  connect(gain2.y, integrator1.u) annotation(
      Line(points = {{61, 62}, {71, 62}, {71, 62}, {71, 62}}, color = {0, 0, 127}));
  connect(integrator1.y, omega) annotation(
      Line(points = {{95, 62}, {148, 62}, {148, 62}, {203, 62}, {203, 62}, {205, 62}, {205, 62}, {209, 62}}, color = {0, 0, 127}));
  connect(integrator1.y, division1.u2) annotation(
      Line(points = {{95, 62}, {121, 62}, {121, -18}, {-59, -18}, {-59, 64}, {-53, 64}, {-53, 63}, {-55, 63}, {-55, 64}}, color = {0, 0, 127}));
  connect(integrator1.y, lambda_calculation1.omega) annotation(
      Line(points = {{95, 62}, {107, 62}, {107, 62}, {121, 62}, {121, -18}, {-179, -18}, {-179, 32}, {-159, 32}, {-159, 31}, {-161, 31}, {-161, 32}}, color = {0, 0, 127}));
  connect(add31.y, gain2.u) annotation(
      Line(points = {{21, 62}, {37, 62}, {37, 62}, {36, 62}, {36, 62}, {37, 62}}, color = {0, 0, 127}));
  connect(division1.y, add31.u1) annotation(
      Line(points = {{-31, 70}, {-3, 70}, {-3, 70}, {-4, 70}, {-4, 70}, {-3, 70}}, color = {0, 0, 127}));
  connect(Tgen, add31.u2) annotation(
      Line(points = {{-220, -38}, {-20, -38}, {-20, 62}, {-2, 62}}, color = {0, 0, 127}));
  connect(lambda_calculation1.lambda, cp_calculation1.lambda) annotation(
      Line(points = {{-137, 36}, {-123, 36}, {-123, 36}, {-124, 36}, {-124, 36}, {-123, 36}}, color = {0, 0, 127}));
  connect(lambda_calculation1.WindSpeed, WindSpeed) annotation(
      Line(points = {{-160, 42}, {-190, 42}, {-190, 82}, {-220, 82}, {-220, 82}}, color = {0, 0, 127}));
  connect(cp_calculation1.Cp, product3.u2) annotation(
      Line(points = {{-99, 32}, {-85, 32}, {-85, 70}, {-81, 70}, {-81, 70}, {-83, 70}, {-83, 70}}, color = {0, 0, 127}));
  connect(division1.u1, product3.y) annotation(
      Line(points = {{-54, 76}, {-58, 76}}, color = {0, 0, 127}));
  connect(gain1.y, product3.u1) annotation(
      Line(points = {{-99, 82}, {-83, 82}, {-83, 82}, {-84, 82}, {-84, 82}, {-83, 82}}, color = {0, 0, 127}));
  connect(WindSpeed, product1.u2) annotation(
      Line(points = {{-220, 82}, {-186, 82}, {-186, 76}, {-182, 76}, {-182, 76}}, color = {0, 0, 127}));
  connect(product1.u1, WindSpeed) annotation(
      Line(points = {{-182, 88}, {-184, 88}, {-184, 88}, {-186, 88}, {-186, 82}, {-220, 82}, {-220, 77}, {-220, 77}, {-220, 82}}, color = {0, 0, 127}));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -200}, {200, 200}})),
      Icon(coordinateSystem(extent = {{-200, -200}, {200, 200}}), graphics = {Rectangle(origin = {0, -1}, fillColor = {214, 214, 214}, fillPattern = FillPattern.Solid, extent = {{-200, 201}, {200, -199}}), Text(origin = {-47, 32}, extent = {{-59, 60}, {143, -126}}, textString = "Turbine dynamics", fontSize = 25), Text(origin = {-246, 127}, extent = {{-16, 11}, {34, -27}}, textString = "WindSpeed", fontSize = 12), Text(origin = {-236, 22}, extent = {{-38, 20}, {38, -20}}, textString = "BladeAngle", fontSize = 12), Text(origin = {-230, -80}, extent = {{-38, 20}, {38, -20}}, textString = "Tgen", fontSize = 12), Text(origin = {222, 60}, extent = {{-16, 12}, {16, -12}}, textString = "omega", fontSize = 12), Text(origin = {218, -40}, extent = {{-16, 12}, {16, -12}}, textString = "Pgen", fontSize = 12)}),
      __OpenModelica_commandLineOptions = "");
  end Turbine;





  model cp_calculation
    Real lambdaai;
    Modelica.Blocks.Interfaces.RealInput lambda annotation(
      Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput BladeAngle annotation(
      Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Cp annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
  lambdaai = 1/(1/(lambda+0.08*BladeAngle) - 0.035/(BladeAngle^3+1));
  Cp = 0.5176*(116/lambdaai - 0.4*BladeAngle - 5) * exp(-21/lambdaai) + 0.0068*lambda;
  annotation(
      Icon(graphics = {Text(origin = {-20, 36}, extent = {{-14, 10}, {46, -46}}, textString = "Cp", fontSize = 35), Text(origin = {1, -19}, extent = {{-69, 43}, {69, -43}}, textString = "calculation", fontSize = 35), Rectangle(origin = {0, 1}, extent = {{-100, 99}, {100, -101}})}));end cp_calculation;








  model lambda_calculation
  parameter Real bladeLength = bladeLength; 
    Modelica.Blocks.Interfaces.RealInput WindSpeed annotation(
      Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput omega annotation(
      Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -52}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput lambda annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Division division1 annotation(
      Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = bladeLength)  annotation(
      Placement(visible = true, transformation(origin = {-70, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
  connect(gain1.y, division1.u1) annotation(
      Line(points = {{-58, -40}, {-40, -40}, {-40, 6}, {-14, 6}}, color = {0, 0, 127}));
  connect(WindSpeed, division1.u2) annotation(
      Line(points = {{-120, 60}, {-80, 60}, {-80, -6}, {-14, -6}}, color = {0, 0, 127}));
  connect(division1.y, lambda) annotation(
      Line(points = {{9, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(omega, gain1.u) annotation(
      Line(points = {{-120, -40}, {-82, -40}, {-82, -40}, {-82, -40}}, color = {0, 0, 127}));
  annotation(
      Icon(graphics = {Text(origin = {-1, 20}, extent = {{-29, 40}, {29, -40}}, textString = "lambda", fontSize = 35), Text(origin = {-23, -7}, extent = {{-21, 17}, {79, -55}}, textString = "calculation", fontSize = 35), Rectangle(extent = {{-100, 100}, {100, -100}})}));end lambda_calculation;


  model Speed_controller
  parameter Real w_reg23bdry = w_reg23bdry; 
  parameter Real u_reg23bdry = u_reg23bdry;
  parameter Real lambda_opt = lambda_opt;
  parameter Real bladeLength = bladeLength;
  parameter Real speedControllerKp = speedControllerKp;
  parameter Real speedControllerKi = speedControllerKi;
  parameter Real speedControllerKt = speedControllerKt;
  parameter Real speedControllerLowerLimit = speedControllerLowerLimit;
  parameter Real speedControllerUpperLimit = speedControllerUpperLimit;
    Modelica.Blocks.Interfaces.RealInput WindSpeed annotation(
      Placement(visible = true, transformation(origin = {-220, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-210, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput omega annotation(
      Placement(visible = true, transformation(origin = {-220, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-210, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Tgen annotation(
      Placement(visible = true, transformation(origin = {210, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {210, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1(threshold = u_reg23bdry)  annotation(
      Placement(visible = true, transformation(origin = {-170, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.Switch switch1 annotation(
      Placement(visible = true, transformation(origin = {-130, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = w_reg23bdry)  annotation(
      Placement(visible = true, transformation(origin = {-170, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = lambda_opt / bladeLength)  annotation(
      Placement(visible = true, transformation(origin = {-170, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add1(k2 = -1)  annotation(
      Placement(visible = true, transformation(origin = {-90, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = -1)  annotation(
      Placement(visible = true, transformation(origin = {-60, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain3(k = speedControllerKp)  annotation(
      Placement(visible = true, transformation(origin = {-14, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain4(k = speedControllerKi) annotation(
      Placement(visible = true, transformation(origin = {-14, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain5(k = speedControllerKt) annotation(
      Placement(visible = true, transformation(origin = {26, -40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator1(y_start = 0)  annotation(
      Placement(visible = true, transformation(origin = {54, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add2(k2 = +1) annotation(
      Placement(visible = true, transformation(origin = {22, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add3(k2 = +1) annotation(
      Placement(visible = true, transformation(origin = {90, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add4(k1 = -1, k2 = +1) annotation(
      Placement(visible = true, transformation(origin = {90, -40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter limiter1(limitsAtInit = true, uMax = speedControllerUpperLimit, uMin = speedControllerLowerLimit)  annotation(
      Placement(visible = true, transformation(origin = {150, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
    connect(const.y, switch1.u1) annotation(
      Line(points = {{-158, 72}, {-150, 72}, {-150, 48}, {-142, 48}}, color = {0, 0, 127}));
    connect(greaterThreshold1.y, switch1.u2) annotation(
      Line(points = {{-159, 40}, {-143, 40}, {-143, 40}, {-143, 40}}, color = {255, 0, 255}));
    connect(gain1.y, switch1.u3) annotation(
      Line(points = {{-159, -2}, {-157.5, -2}, {-157.5, -2}, {-156, -2}, {-156, -2}, {-151, -2}, {-151, 32}, {-148, 32}, {-148, 32}, {-145.5, 32}, {-145.5, 32}, {-143, 32}}, color = {0, 0, 127}));
    connect(switch1.y, add1.u1) annotation(
      Line(points = {{-119, 40}, {-101, 40}, {-101, 40}, {-103, 40}}, color = {0, 0, 127}));
    connect(limiter1.y, Tgen) annotation(
      Line(points = {{162, 0}, {204, 0}, {204, 0}, {210, 0}}, color = {0, 0, 127}));
    connect(add4.u2, limiter1.y) annotation(
      Line(points = {{102, -46}, {176, -46}, {176, 0}, {162, 0}, {162, 0}}, color = {0, 0, 127}));
    connect(add3.y, add4.u1) annotation(
      Line(points = {{102, 28}, {120, 28}, {120, -34}, {102, -34}, {102, -34}}, color = {0, 0, 127}));
    connect(add3.y, limiter1.u) annotation(
      Line(points = {{102, 28}, {120, 28}, {120, 0}, {138, 0}, {138, 0}}, color = {0, 0, 127}));
    connect(gain5.u, add4.y) annotation(
      Line(points = {{38, -40}, {79, -40}}, color = {0, 0, 127}));
    connect(integrator1.y, add3.u2) annotation(
      Line(points = {{66, -6}, {72, -6}, {72, 22}, {78, 22}}, color = {0, 0, 127}));
    connect(gain3.y, add3.u1) annotation(
      Line(points = {{-2, 34}, {78, 34}}, color = {0, 0, 127}));
    connect(add2.y, integrator1.u) annotation(
      Line(points = {{34, -6}, {42, -6}, {42, -6}, {42, -6}}, color = {0, 0, 127}));
    connect(gain5.y, add2.u2) annotation(
      Line(points = {{15, -40}, {6, -40}, {6, -12}, {10, -12}}, color = {0, 0, 127}));
    connect(gain4.y, add2.u1) annotation(
      Line(points = {{-2, 0}, {8, 0}, {8, 0}, {10, 0}}, color = {0, 0, 127}));
    connect(gain4.u, gain2.y) annotation(
      Line(points = {{-26, 0}, {-40, 0}, {-40, 34}, {-48, 34}, {-48, 34}}, color = {0, 0, 127}));
    connect(gain2.y, gain3.u) annotation(
      Line(points = {{-48, 34}, {-28, 34}, {-28, 34}, {-26, 34}}, color = {0, 0, 127}));
    connect(add1.y, gain2.u) annotation(
      Line(points = {{-79, 34}, {-73, 34}}, color = {0, 0, 127}));
    connect(omega, add1.u2) annotation(
      Line(points = {{-220, -40}, {-164, -40}, {-164, -40}, {-108, -40}, {-108, 28}, {-102, 28}, {-102, 28}, {-102, 28}, {-102, 28}}, color = {0, 0, 127}));
    connect(gain1.u, WindSpeed) annotation(
      Line(points = {{-182, -2}, {-184.5, -2}, {-184.5, -2}, {-187, -2}, {-187, -2}, {-192, -2}, {-192, 40}, {-220, 40}, {-220, 40}, {-220, 40}, {-220, 40}}, color = {0, 0, 127}));
    connect(WindSpeed, greaterThreshold1.u) annotation(
      Line(points = {{-220, 40}, {-211, 40}, {-211, 40}, {-202, 40}, {-202, 40}, {-184, 40}, {-184, 40}, {-183, 40}, {-183, 40}, {-182.5, 40}, {-182.5, 40}, {-182, 40}}, color = {0, 0, 127}));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -200}, {200, 200}})),
      Icon(coordinateSystem(extent = {{-200, -200}, {200, 200}}), graphics = {Rectangle(fillColor = {214, 214, 214}, fillPattern = FillPattern.Solid, extent = {{-200, 100}, {200, -100}}), Text(origin = {-21, 33}, extent = {{-49, 31}, {103, -93}}, textString = "Speed controller", fontSize = 25), Text(origin = {-236, 60}, extent = {{-22, 10}, {22, -10}}, textString = "WindSpeed", fontSize = 12), Text(origin = {-227, -20}, extent = {{-19, 10}, {19, -10}}, textString = "omega", fontSize = 12), Text(origin = {222, 17}, extent = {{-12, -13}, {12, 13}}, textString = "Tgen", fontSize = 12)}),
      __OpenModelica_commandLineOptions = "");
  end Speed_controller;






  Wind_turbine.Turbine turbine_dynamics(A = A, B = B, J = J, bladeAngleTimeConstant = bladeAngleTimeConstant, bladeLength = bladeLength, rho = rho, w_0 = w_0)  annotation(
    Placement(visible = true, transformation(origin = {0, 52}, extent = {{-40, -40}, {40, 40}}, rotation = 0)));
  Wind_turbine.Speed_controller speed_controller1(bladeLength = bladeLength, lambda_opt = lambda_opt, speedControllerKi = speedControllerKi, speedControllerKp = speedControllerKp, speedControllerKt = speedControllerKt, speedControllerLowerLimit = speedControllerLowerLimit, speedControllerUpperLimit = speedControllerUpperLimit, u_reg23bdry = u_reg23bdry,  w_reg23bdry = w_reg23bdry)  annotation(
    Placement(visible = true, transformation(origin = {0, -12}, extent = {{-40, -40}, {40, 40}}, rotation = 0)));

  model Power_controller
  parameter Real u_reg23bdry = u_reg23bdry;
  parameter Real powerControllerKp = powerControllerKp;
  parameter Real powerControllerKi = powerControllerKi;
  parameter Real powerControllerKt = powerControllerKt;
  parameter Real powerControllerUpperLimit = powerControllerUpperLimit;
  parameter Real powerControllerLowerLimit = powerControllerLowerLimit;
  parameter Real Prated = Prated;
  
    Modelica.Blocks.Interfaces.RealInput WindSpeed annotation(
      Placement(visible = true, transformation(origin = {-220, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-210, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput BladeAngle annotation(
      Placement(visible = true, transformation(origin = {210, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {210, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Pgen annotation(
      Placement(visible = true, transformation(origin = {-220, 74}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-210, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1(threshold = u_reg23bdry)  annotation(
      Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Logical.Switch switch1 annotation(
      Placement(visible = true, transformation(origin = {130, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0)  annotation(
      Placement(visible = true, transformation(origin = {90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = Prated) annotation(
      Placement(visible = true, transformation(origin = {-166, 102}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add1(k1 = -1)  annotation(
      Placement(visible = true, transformation(origin = {-130, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = powerControllerKp)  annotation(
      Placement(visible = true, transformation(origin = {-52, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = powerControllerKi) annotation(
      Placement(visible = true, transformation(origin = {-90, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add2(k1 = +1, k2 = +1) annotation(
      Placement(visible = true, transformation(origin = {14, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain3(k = powerControllerKt)  annotation(
      Placement(visible = true, transformation(origin = {-22, 20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter limiter1(limitsAtInit = true, uMax = powerControllerUpperLimit, uMin = powerControllerLowerLimit)  annotation(
      Placement(visible = true, transformation(origin = {50, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add3(k1 = +1, k2 = +1) annotation(
      Placement(visible = true, transformation(origin = {-54, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add4(k1 = -1, k2 = +1) annotation(
      Placement(visible = true, transformation(origin = {14, 20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator1 annotation(
      Placement(visible = true, transformation(origin = {-20, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
    connect(add1.y, gain1.u) annotation(
      Line(points = {{-119, 80}, {-64, 80}}, color = {0, 0, 127}));
    connect(gain1.y, add2.u1) annotation(
      Line(points = {{-41, 80}, {1, 80}}, color = {0, 0, 127}));
    connect(integrator1.y, add2.u2) annotation(
      Line(points = {{-9, 48}, {-5, 48}, {-5, 68}, {1, 68}}, color = {0, 0, 127}));
    connect(add3.y, integrator1.u) annotation(
      Line(points = {{-43, 48}, {-32, 48}}, color = {0, 0, 127}));
    connect(add4.u2, limiter1.y) annotation(
      Line(points = {{26, 14}, {72, 14}, {72, 74}, {62, 74}}, color = {0, 0, 127}));
    connect(add4.u1, add2.y) annotation(
      Line(points = {{26, 26}, {32, 26}, {32, 74}, {25, 74}}, color = {0, 0, 127}));
    connect(gain3.u, add4.y) annotation(
      Line(points = {{-10, 20}, {3, 20}}, color = {0, 0, 127}));
    connect(add3.u2, gain3.y) annotation(
      Line(points = {{-66, 42}, {-70, 42}, {-70, 20}, {-33, 20}}, color = {0, 0, 127}));
    connect(gain2.y, add3.u1) annotation(
      Line(points = {{-79, 54}, {-74, 54}, {-74, 54}, {-69, 54}, {-69, 54}, {-67, 54}}, color = {0, 0, 127}));
    connect(limiter1.y, switch1.u1) annotation(
      Line(points = {{61, 74}, {112, 74}, {112, 8}, {118, 8}}, color = {0, 0, 127}));
    connect(add2.y, limiter1.u) annotation(
      Line(points = {{25, 74}, {38, 74}}, color = {0, 0, 127}));
    connect(gain2.u, add1.y) annotation(
      Line(points = {{-102, 54}, {-110, 54}, {-110, 80}, {-118, 80}, {-118, 81}, {-118, 81}, {-118, 80}}, color = {0, 0, 127}));
    connect(Pgen, add1.u2) annotation(
      Line(points = {{-220, 74}, {-144, 74}, {-144, 74}, {-142, 74}}, color = {0, 0, 127}));
    connect(const1.y, add1.u1) annotation(
      Line(points = {{-155, 102}, {-150, 102}, {-150, 86}, {-146.5, 86}, {-146.5, 86}, {-143, 86}}, color = {0, 0, 127}));
    connect(WindSpeed, greaterThreshold1.u) annotation(
      Line(points = {{-220, 0}, {78, 0}, {78, 0}, {78, 0}}, color = {0, 0, 127}));
    connect(greaterThreshold1.y, switch1.u2) annotation(
      Line(points = {{102, 0}, {118, 0}, {118, 0}, {118, 0}}, color = {255, 0, 255}));
    connect(const.y, switch1.u3) annotation(
      Line(points = {{102, -30}, {112, -30}, {112, -8}, {118, -8}, {118, -8}}, color = {0, 0, 127}));
    connect(switch1.y, BladeAngle) annotation(
      Line(points = {{142, 0}, {202, 0}, {202, 0}, {210, 0}}, color = {0, 0, 127}));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -200}, {200, 200}})),
      Icon(coordinateSystem(extent = {{-200, -200}, {200, 200}}), graphics = {Text(origin = {-234, 62}, extent = {{-24, 16}, {24, -16}}, textString = "WindSpeed", fontSize = 12), Text(origin = {-227, -20}, extent = {{-23, 8}, {23, -8}}, textString = "Pgen", fontSize = 12), Rectangle(fillColor = {214, 214, 214}, fillPattern = FillPattern.Solid, extent = {{-200, 100}, {200, -100}}), Text(origin = {240, 16}, extent = {{-16, 12}, {16, -12}}, textString = "BladeAngle", fontSize = 12), Text(origin = {2, 2},extent = {{-60, 26}, {60, -26}}, textString = "Power controller", fontSize = 25)}),
      __OpenModelica_commandLineOptions = "");
  end Power_controller;




  Wind_turbine.Power_controller power_controller1(Prated = Prated, powerControllerKi = powerControllerKi, powerControllerKp = powerControllerKp, powerControllerKt = powerControllerKt, powerControllerLowerLimit = powerControllerLowerLimit, powerControllerUpperLimit = powerControllerUpperLimit, u_reg23bdry = u_reg23bdry)  annotation(
    Placement(visible = true, transformation(origin = {0, -56}, extent = {{-40, -40}, {40, 40}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Pgen annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput WindSpeed annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  model Wind_speed_time_series
    Modelica.Blocks.Sources.TimeTable timeTable1(table = [0, 7.1017; 300.0000, 5.5907; 600.0000, 3.5508; 900.0000, 1.2088; 1.2000e+03, 2.3420; 1.5000e+03, 1.4354; 1.8000e+03, 2.5687; 2.1000e+03, 0.9066; 2.4000e+03, 1.2843; 2700, 1.1332; 3.0000e+03, 0.5288; 3.3000e+03, 1.1332; 3.6000e+03, 0.4533; 3.9000e+03, 1.3599; 4.2000e+03, 1.2088; 4.5000e+03, 0.4533; 4.8000e+03, 1.9643; 5.1000e+03, 1.6621; 5400, 2.1154; 5.7000e+03, 3.9286; 6.0000e+03, 3.8530; 6.3000e+03, 4.0041; 6.6000e+03, 4.9107; 6.9000e+03, 6.6484; 7.2000e+03, 7.9327; 7.5000e+03, 8.9904; 7.8000e+03, 8.3105; 8100, 7.8572; 8.4000e+03, 8.6882; 8.7000e+03, 8.8393; 9.0000e+03, 8.6882; 9.3000e+03, 7.7816; 9.6000e+03, 7.6305; 9.9000e+03, 7.7061; 1.0200e+04, 6.9506; 1.0500e+04, 6.4973; 10800, 6.1951; 1.1100e+04, 5.2129; 1.1400e+04, 5.2129; 1.1700e+04, 5.5907; 1.2000e+04, 5.9684; 1.2300e+04, 5.8929; 1.2600e+04, 6.0440; 1.2900e+04, 6.2706; 1.3200e+04, 6.1195; 13500, 7.1772; 1.3800e+04, 9.1415; 1.4100e+04, 9.6704; 1.4400e+04, 9.5193; 1.4700e+04, 9.2171; 1.5000e+04, 9.5193; 1.5300e+04, 9.8215; 1.5600e+04, 9.9726; 1.5900e+04, 9.7459; 16200, 9.6704; 1.6500e+04, 9.2926; 1.6800e+04, 9.5948; 1.7100e+04, 10.1237; 1.7400e+04, 10.5014; 1.7700e+04, 10.4259; 1.8000e+04, 10.6525; 1.8300e+04, 10.3503; 1.8600e+04, 10.9547; 18900, 11.1814; 1.9200e+04, 10.4259; 1.9500e+04, 10.4259; 1.9800e+04, 10.8036; 2.0100e+04, 10.4259; 2.0400e+04, 10.3503; 2.0700e+04, 9.0660; 2.1000e+04, 9.5948; 2.1300e+04, 9.2171; 21600, 10.1992; 2.1900e+04, 11.0303; 2.2200e+04, 10.8036; 2.2500e+04, 11.7102; 2.2800e+04, 12.9190; 2.3100e+04, 12.5413; 2.3400e+04, 12.9190; 2.3700e+04, 13.5234; 2.4000e+04, 13.8256; 24300, 13.2968; 2.4600e+04, 12.7679; 2.4900e+04, 13.2212; 2.5200e+04, 12.6168; 2.5500e+04, 12.6924; 2.5800e+04, 12.3146; 2.6100e+04, 12.2391; 2.6400e+04, 11.7102; 2.6700e+04, 11.8613; 27000, 13.1457; 2.7300e+04, 12.8435; 2.7600e+04, 11.7858; 2.7900e+04, 11.6347; 2.8200e+04, 11.6347; 2.8500e+04, 11.5591; 2.8800e+04, 11.4836; 2.9100e+04, 10.8036; 2.9400e+04, 10.9547; 29700, 11.1814; 3.0000e+04, 10.9547; 3.0300e+04, 10.8792; 3.0600e+04, 10.5014; 3.0900e+04, 10.7281; 3.1200e+04, 10.2748; 3.1500e+04, 10.5770; 3.1800e+04, 12.4657; 3.2100e+04, 12.7679; 32400, 12.0880; 3.2700e+04, 11.5591; 3.3000e+04, 11.7858; 3.3300e+04, 11.7102; 3.3600e+04, 11.4836; 3.3900e+04, 11.5591; 3.4200e+04, 10.6525; 3.4500e+04, 10.8792; 3.4800e+04, 10.4259; 35100, 10.5014; 3.5400e+04, 11.0303; 3.5700e+04, 10.6525; 3.6000e+04, 10.6525; 3.6300e+04, 10.1992; 3.6600e+04, 9.5948; 3.6900e+04, 9.1415; 3.7200e+04, 9.4437; 3.7500e+04, 10.0481; 37800, 10.4259; 3.8100e+04, 11.4080; 3.8400e+04, 11.3325; 3.8700e+04, 11.4080; 3.9000e+04, 11.4080; 3.9300e+04, 11.4080; 3.9600e+04, 11.4080; 3.9900e+04, 11.8613; 4.0200e+04, 11.4080; 40500, 10.6525; 4.0800e+04, 9.8215; 4.1100e+04, 8.9149; 4.1400e+04, 10.2748; 4.1700e+04, 10.4259; 4.2000e+04, 10.5770; 4.2300e+04, 10.8792; 4.2600e+04, 11.5591; 4.2900e+04, 12.1635; 43200, 11.1814; 4.3500e+04, 10.5014; 4.3800e+04, 11.0303; 4.4100e+04, 11.5591; 4.4400e+04, 10.9547; 4.4700e+04, 10.7281; 4.5000e+04, 9.8215; 4.5300e+04, 10.5014; 4.5600e+04, 11.4080; 45900, 12.0880; 4.6200e+04, 12.1635; 4.6500e+04, 11.4836; 4.6800e+04, 10.7281; 4.7100e+04, 10.2748; 4.7400e+04, 10.0481; 4.7700e+04, 8.9149; 4.8000e+04, 7.7061; 4.8300e+04, 8.2349; 48600, 7.9327; 4.8900e+04, 8.9904; 4.9200e+04, 9.4437; 4.9500e+04, 8.3860; 4.9800e+04, 8.1594; 5.0100e+04, 9.0660; 5.0400e+04, 9.0660; 5.0700e+04, 8.6882; 5.1000e+04, 7.8572; 51300, 8.6882; 5.1600e+04, 8.3105; 5.1900e+04, 8.3860; 5.2200e+04, 8.3860; 5.2500e+04, 8.2349; 5.2800e+04, 8.5371; 5.3100e+04, 8.3105; 5.3400e+04, 7.4794; 5.3700e+04, 6.1195; 54000, 6.4217; 5.4300e+04, 6.8750; 5.4600e+04, 5.9684; 5.4900e+04, 6.0440; 5.5200e+04, 4.6841; 5.5500e+04, 5.9684; 5.5800e+04, 7.1017; 5.6100e+04, 7.1017; 5.6400e+04, 7.5550; 56700, 7.6305; 5.7000e+04, 7.3283; 5.7300e+04, 7.7061; 5.7600e+04, 7.4039; 5.7900e+04, 7.5550; 5.8200e+04, 9.3682; 5.8500e+04, 8.8393; 5.8800e+04, 9.6704; 5.9100e+04, 11.7858; 59400, 11.2569; 5.9700e+04, 12.5413; 6.0000e+04, 12.2391; 6.0300e+04, 13.5990; 6.0600e+04, 14.3545; 6.0900e+04, 13.8256; 6.1200e+04, 13.2212; 6.1500e+04, 12.2391; 6.1800e+04, 11.2569; 62100, 14.7322; 6.2400e+04, 15.5633; 6.2700e+04, 15.6388; 6.3000e+04, 14.8833; 6.3300e+04, 13.7501; 6.3600e+04, 12.8435; 6.3900e+04, 11.8613; 6.4200e+04, 11.7102; 6.4500e+04, 11.1058; 64800, 12.6168; 6.5100e+04, 11.0303; 6.5400e+04, 11.3325; 6.5700e+04, 11.3325; 6.6000e+04, 12.2391; 6.6300e+04, 11.9369; 6.6600e+04, 11.0303; 6.6900e+04, 12.3146; 6.7200e+04, 11.1814; 67500, 12.0880; 6.7800e+04, 11.7858; 6.8100e+04, 11.1814; 6.8400e+04, 11.7858; 6.8700e+04, 11.1814; 6.9000e+04, 10.5770; 6.9300e+04, 11.5591; 6.9600e+04, 11.9369; 6.9900e+04, 12.3902; 70200, 11.4836; 7.0500e+04, 11.1814; 7.0800e+04, 10.2748; 7.1100e+04, 10.5014; 7.1400e+04, 11.1058; 7.1700e+04, 11.3325; 7.2000e+04, 12.7679; 7.2300e+04, 12.1635; 7.2600e+04, 11.9369; 72900, 12.2391; 7.3200e+04, 12.0880; 7.3500e+04, 12.0880; 7.3800e+04, 14.3545; 7.4100e+04, 14.0523; 7.4400e+04, 11.3325; 7.4700e+04, 10.7281; 7.5000e+04, 14.8078; 7.5300e+04, 14.9589]) annotation(
      Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput WindSpeed annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
    connect(timeTable1.y, WindSpeed) annotation(
      Line(points = {{9, 0}, {110, 0}}, color = {0, 0, 127}));
    annotation(
      uses(Modelica(version = "3.2.2")),
      Icon(graphics = {Text(origin = {136, 17}, extent = {{-10, 9}, {10, -9}}, textString = "WindSpeed", fontSize = 12), Rectangle(fillColor = {214, 214, 214}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, 14}, extent = {{-32, 14}, {32, -14}}, textString = "Wind speed", fontSize = 20), Text(origin = {3, -12}, extent = {{-15, 10}, {15, -10}}, textString = "time series", fontSize = 20)}));
  end Wind_speed_time_series;
equation
  connect(WindSpeed, speed_controller1.WindSpeed) annotation(
    Line(points = {{-120, 0}, {-68, 0}, {-68, -4}, {-42, -4}}, color = {0, 0, 127}));
  connect(speed_controller1.Tgen, turbine_dynamics.Tgen) annotation(
    Line(points = {{42, -12}, {60, -12}, {60, -82}, {-62, -82}, {-62, 32}, {-42, 32}}, color = {0, 0, 127}));
  connect(turbine_dynamics.omega, speed_controller1.omega) annotation(
    Line(points = {{42, 60}, {64, 60}, {64, -86}, {-66, -86}, {-66, -20}, {-42, -20}}, color = {0, 0, 127}));
  connect(power_controller1.BladeAngle, turbine_dynamics.BladeAngle) annotation(
    Line(points = {{42, -56}, {52, -56}, {52, -80}, {-64, -80}, {-64, 52}, {-42, 52}}, color = {0, 0, 127}));
  connect(WindSpeed, turbine_dynamics.WindSpeed) annotation(
    Line(points = {{-120, 0}, {-68, 0}, {-68, 72}, {-42, 72}}, color = {0, 0, 127}));
  connect(turbine_dynamics.Pgen, power_controller1.Pgen) annotation(
    Line(points = {{42, 40}, {62, 40}, {62, -84}, {-64, -84}, {-64, -64}, {-42, -64}}, color = {0, 0, 127}));
  connect(turbine_dynamics.Pgen, Pgen) annotation(
    Line(points = {{42, 40}, {62, 40}, {62, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(WindSpeed, power_controller1.WindSpeed) annotation(
    Line(points = {{-120, 0}, {-68, 0}, {-68, -48}, {-42, -48}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2"), iPSL(version = "1.1.0")),
    Icon(coordinateSystem(initialScale = 0.1), graphics = {Text(origin = {-124, 20}, extent = {{-16, 10}, {16, -10}}, textString = "WindSpeed", fontSize = 8), Text(origin = {123, 19}, extent = {{-11, 11}, {11, -11}}, textString = "Pgen", fontSize = 8), Rectangle(fillColor = {214, 214, 214}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {8, 8}, extent = {{-46, 18}, {24, -8}}, textString = "Wind", fontSize = 25), Text(origin = {-4, -11}, extent = {{-12, 11}, {18, -15}}, textString = "turbine", fontSize = 25)}),
  Diagram(graphics = {Text(origin = {-73, -94}, extent = {{-23, 12}, {25, -14}}, textString = "Note: Add region IV if needed")}, coordinateSystem(initialScale = 0.1)));
end Wind_turbine;
