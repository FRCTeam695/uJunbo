package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AlgaeArm extends SubsystemBase{
    private SparkMax intakeMotor;
    private SparkMax pitchMotor;
    private RelativeEncoder m_pitchEncoder;
    private RelativeEncoder m_intakeEncoder;

    private SparkClosedLoopController m_pidControl;
    private SparkMaxConfig configPitch;
    private SparkMaxConfig configIntake;

    public Trigger hasAlgae;
    private boolean algaeContained;
    private boolean armHasHitApex = false;

    public AlgaeArm () {
        intakeMotor = new SparkMax(54, SparkLowLevel.MotorType.kBrushless);
        pitchMotor = new SparkMax(53, SparkLowLevel.MotorType.kBrushless);

        // Pitch motor
        m_pidControl = pitchMotor.getClosedLoopController();
        configPitch = new SparkMaxConfig();
        
        configPitch
            .smartCurrentLimit(20)
            .idleMode(IdleMode.kBrake);
        configPitch.closedLoop
            .pid(0.03, 0.0, 0.0);

        m_pitchEncoder = pitchMotor.getEncoder();
        m_pitchEncoder.setPosition(0);

        pitchMotor.configure(configPitch, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Intake motor
        configIntake = new SparkMaxConfig();

        configIntake
            .smartCurrentLimit(10)
            .idleMode(IdleMode.kBrake);

        m_intakeEncoder = intakeMotor.getEncoder();
        m_intakeEncoder.setPosition(0);
        
        intakeMotor.configure(configIntake, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        algaeContained = false;
        // Trigger
        hasAlgae = new Trigger(()-> algaeContained);
    }

    public Command requireSubsystem(){
        return new WaitCommand(0);
    }

    public Command runAlgalizer(DoubleSupplier targetRot, DoubleSupplier setSpeed) {
        return 
            runOnce(()->{armHasHitApex = false;}).andThen(run(() -> {
            double targetPitch = targetRot.getAsDouble();
            m_pidControl.setReference(targetPitch, ControlType.kPosition);

            configIntake.smartCurrentLimit(3);
            intakeMotor.set(setSpeed.getAsDouble());
            SmartDashboard.putNumber("roller current", intakeMotor.getOutputCurrent());
            double armPosition = pitchMotor.getEncoder().getPosition();
            if(!armHasHitApex){
                armHasHitApex = armPosition > 17;
            }
            if(algaeContained){
                intakeMotor.set(0);
            }
            else{
                algaeContained = Math.abs(targetPitch - armPosition) < 2 && armHasHitApex;  
                intakeMotor.set(setSpeed.getAsDouble());
            }
        }));
    }

    public Command holdPitch() {
        return run(() -> {
            intakeMotor.set(0);
        });
    }

    /*public Command runPitch(DoubleSupplier targetRot) {
        return run(() -> 
        m_pidControl.setReference(
            targetRot.getAsDouble(), 
            ControlType.kPosition));
        //.until(() -> Math.abs(pitchMotor.getEncoder().getPosition() - targetRot.getAsDouble()) < 0.1);
        // Stops moving arm when position reaches within 0.1 rotations of target rotation
    }*/

    /*public Command runIntakeIn(DoubleSupplier setSpeed) {
        return runOnce(() ->
        configIntake.smartCurrentLimit(1))
        .andThen(run(() -> intakeMotor.set(setSpeed.getAsDouble())));
    }*/

    public Command runIntakeOut() {
        return run(() -> {
            configIntake.smartCurrentLimit(40);
            intakeMotor.set(-1);
            algaeContained = false;
        });
    }

    public Command stop() {
        return run(()->{
            intakeMotor.set(0);
            m_pidControl.setReference(0, ControlType.kPosition);
        });
        //.until(() -> Math.abs(pitchMotor.getEncoder().getPosition() - 0) < 0.1);
        //.andThen(()->pitchMotor.getEncoder().setPosition(0));
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("Motor Rotations", pitchMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Has Algae", hasAlgae.getAsBoolean());
    }
    
}
