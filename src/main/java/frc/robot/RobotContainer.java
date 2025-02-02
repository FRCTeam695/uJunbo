// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
//import frc.robot.subsystems.MotorSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
//import frc.robot.subsystems.PreSeasonSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.TalonElevator;
//import edu.wpi.first.wpilibj.PWM;
//import edu.wpi.first.wpilibj.Servo;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //private final PreSeasonSubsystem mySubsystem = new PreSeasonSubsystem();
  //private final MotorSubsystem driveTrain = new MotorSubsystem(mySubsystem, 56); // mySubsystem = PreSeasonSubsystem
  //private final TalonElevator elevator = new TalonElevator();
  private final AlgaeArm arm = new AlgaeArm();

  // Joysticks (Not controller) - Deprecated
  //public static CommandJoystick myLeftJoystick;
  //public static CommandJoystick myRightJoystick;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static final CommandXboxController m_driverController =
      new CommandXboxController(0);

  public CommandXboxController getController() {
    return m_driverController;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Joystick buttons (Not controller)
    //myLeftJoystick = new CommandJoystick(0);
    //myRightJoystick = new CommandJoystick(1);
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /*m_driverController.a().onTrue(elevator.setHeightLevel(TalonElevator.Heights.L1));
    m_driverController.b().onTrue(elevator.setHeightLevel(TalonElevator.Heights.L2));
    m_driverController.x().onTrue(elevator.setHeightLevel(TalonElevator.Heights.L3));
    m_driverController.y().onTrue(elevator.setHeightLevel(TalonElevator.Heights.L4));
    //m_driverController.y().onTrue(elevator.talonSet(34));
    m_driverController.rightBumper().onTrue(elevator.setHeightLevel(TalonElevator.Heights.L0));
    // Must reference the actual class name when enum (a class) is made within a class

    DoubleSupplier rightY = ()-> m_driverController.getRightY()*-1; //fwd = +
    //m_driverController.leftBumper().whileTrue(elevator.setHeightJoystick(rightY));
    m_driverController.leftBumper().whileTrue(elevator.setHeightJoystickOpen(rightY));*/


    // Arm pitch
    m_driverController.a().whileTrue(arm.runPitch(() -> 20));
    m_driverController.b().whileTrue(arm.runPitch(() -> 30));

    m_driverController.rightBumper().whileTrue(arm.runIntakeIn(() -> -0.5));
    m_driverController.leftBumper().whileTrue(arm.runIntakeOut());

    arm.setDefaultCommand(arm.stop());
  }
} // 40.75 inches = 30 rotations

// Terminal
// git add .
// git commit -m 'message'
// git push origin (HEAD:main2)