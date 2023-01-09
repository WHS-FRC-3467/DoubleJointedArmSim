// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** This is a sample program to demonstrate the use of arm simulation with existing code. */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;
  private static final int kJoystickPort = 0;

  // The P gain for the PID controller that drives this arm.
  private static final double kArmKp = 40.0;
  private static final double kArmKi = 50.0;


  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);

  // Standard classes for controlling our arm
  private final PIDController m_controller = new PIDController(kArmKp, kArmKi, 0);
  private final Encoder m_topEncoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  private final Encoder m_bottomEncoder = new Encoder(kEncoderAChannel+2, kEncoderBChannel+2);

  private final PWMSparkMax m_topMotor = new PWMSparkMax(kMotorPort);
  private final PWMSparkMax m_bottomMotor = new PWMSparkMax(kMotorPort+1);
  private final Joystick m_joystick = new Joystick(kJoystickPort);

  // Simulation classes help us simulate what's going on, including gravity.
  private static final double m_armReduction = 600;
  private static final double m_arm_topMass = 10.0; // Kilograms
  private static final double m_arm_topLength = Units.inchesToMeters(38.5);
  private static final double m_arm_bottomMass = 4.0; // Kilograms
  private static final double m_arm_bottomLength = Units.inchesToMeters(27);

  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_arm_topSim =
      new SingleJointedArmSim(
          m_armGearbox,
          m_armReduction,
          SingleJointedArmSim.estimateMOI(m_arm_topLength, m_arm_topMass),
          m_arm_topLength,
          Units.degreesToRadians(-75),
          Units.degreesToRadians(260),
          m_arm_topMass,
          true,
          VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
          );
  private final SingleJointedArmSim m_arm_bottomSim =
          new SingleJointedArmSim(
              m_armGearbox,
              m_armReduction,
              SingleJointedArmSim.estimateMOI(m_arm_bottomLength, m_arm_bottomMass),
              m_arm_bottomLength,
              Units.degreesToRadians(30),
              Units.degreesToRadians(150),
              m_arm_bottomMass,
              true,
              VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
              );
  private final EncoderSim m_topEncoderSim = new EncoderSim(m_topEncoder);
  private final EncoderSim m_bottomEncoderSim = new EncoderSim(m_bottomEncoder);


  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  // private final MechanismRoot2d fieldElements = m_mech2d.getRoot()

  private final Mechanism2d m_mech2d = new Mechanism2d(90, 90);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 45, 21.75);
  private final MechanismLigament2d m_arm_bottom =
      m_armPivot.append(
            new MechanismLigament2d(
              "Arm Bottom",
              27, 
              -90, 
              10, 
              new Color8Bit(Color.kGold)));
  private final MechanismLigament2d m_arm_tower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 18, -90, 10, new Color8Bit(Color.kSilver)));

  private final MechanismLigament2d m_aframe_1 =
      m_armPivot.append(new MechanismLigament2d("aframe1", 24, -50, 10, new Color8Bit(Color.kSilver)));
  private final MechanismLigament2d m_bumper =
      m_arm_tower.append(new MechanismLigament2d("Bumper", 4.75, 0, 300, new Color8Bit(Color.kRed)));
  private final MechanismLigament2d m_arm_top =
      m_arm_bottom.append(
          new MechanismLigament2d(
              "Arm Top",
              28.5 + 3.0,
              Units.radiansToDegrees(m_arm_topSim.getAngleRads()),
              10,
              new Color8Bit(Color.kPurple)));
    private final MechanismLigament2d m_intake =
    m_arm_top.append(
        new MechanismLigament2d(
            "Intake",
            7,
            Units.radiansToDegrees(m_arm_topSim.getAngleRads()),
            40,
            new Color8Bit(Color.kWhite)));
        

  @Override
  public void robotInit() {
    m_topEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
    m_bottomEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
    SmartDashboard.putNumber("Setpoint top (degrees)", 180);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", 0);
    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_arm_topSim.setInput(m_topMotor.get() * RobotController.getBatteryVoltage());
    m_arm_bottomSim.setInput(m_bottomMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_arm_topSim.update(0.020);
    m_arm_bottomSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_topEncoderSim.setDistance(m_arm_topSim.getAngleRads());
    m_bottomEncoderSim.setDistance(m_arm_bottomSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_arm_topSim.getCurrentDrawAmps() + m_arm_bottomSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm_top.setAngle(Units.radiansToDegrees(m_arm_topSim.getAngleRads()));
    m_arm_bottom.setAngle(Units.radiansToDegrees(m_arm_bottomSim.getAngleRads()));
  }

  @Override
  public void teleopPeriodic() {

      // Here, we run PID control like normal, with a constant setpoint of 75 degrees.
      var pidOutput = m_controller.calculate(m_topEncoder.getDistance(), Units.degreesToRadians(SmartDashboard.getNumber("Setpoint top (degrees)", 180) - Units.radiansToDegrees(m_arm_bottomSim.getAngleRads())));
      m_topMotor.setVoltage(pidOutput);

      var pidOutputBottom = m_controller.calculate(m_bottomEncoder.getDistance(), Units.degreesToRadians(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0)));
      m_bottomMotor.setVoltage(pidOutputBottom);
  
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_topMotor.set(0.0);
  }
}
