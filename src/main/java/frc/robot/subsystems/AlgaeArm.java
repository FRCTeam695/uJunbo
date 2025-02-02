package frc.robot.subsystems;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase{
    public SparkMax intakeMotor;
    public SparkMax pitchMotor;
    public RelativeEncoder m_pitchEncoder;
    public RelativeEncoder m_intakeEncoder;

    public SparkClosedLoopController m_pidControl;
    public SparkMaxConfig configPitch;
    public SparkMaxConfig configIntake;

    public AlgaeArm () {
        intakeMotor = new SparkMax(54, SparkLowLevel.MotorType.kBrushless);
        pitchMotor = new SparkMax(53, SparkLowLevel.MotorType.kBrushless);

        // Pitch motor
        m_pidControl = pitchMotor.getClosedLoopController();
        configPitch = new SparkMaxConfig();
        
        configPitch
            .smartCurrentLimit(10)
            .idleMode(IdleMode.kBrake);
        configPitch.closedLoop
            .pid(0.02, 0.0, 0.0);

        m_pitchEncoder = pitchMotor.getEncoder();
        m_pitchEncoder.setPosition(0);

        pitchMotor.configure(configPitch, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Intake motor
        configIntake = new SparkMaxConfig();

        configIntake
            .smartCurrentLimit(20)
            .idleMode(IdleMode.kBrake);

        m_intakeEncoder = intakeMotor.getEncoder();
        m_intakeEncoder.setPosition(0);
        
        intakeMotor.configure(configIntake, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command runPitch(DoubleSupplier rot) {
        return run(() -> 
        m_pidControl.setReference(
            rot.getAsDouble(), 
            ControlType.kPosition))
        .until(() -> Math.abs(intakeMotor.getEncoder().getPosition() - rot.getAsDouble()) < 0.1);
    }

    public Command runIntakeIn(DoubleSupplier speed) {
        return runOnce(() ->
        configIntake.smartCurrentLimit(5))
        .andThen(run(() -> intakeMotor.set(speed.getAsDouble())));
    }

    public Command runIntakeOut() {
        return runOnce(() ->
        configIntake.smartCurrentLimit(40))
        .andThen(run(() -> intakeMotor.set(-1)));
    }

    public Command stop() {
        return run(()->{
            intakeMotor.set(0);
            m_pidControl.setReference(0, ControlType.kPosition);
        })
        .until(() -> Math.abs(intakeMotor.getEncoder().getPosition() - 0) < 0.1)
        .andThen(()->pitchMotor.getEncoder().setPosition(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Motor Rotations", pitchMotor.getEncoder().getPosition());
    }
    
}
