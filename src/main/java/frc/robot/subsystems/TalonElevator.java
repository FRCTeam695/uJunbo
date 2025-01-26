package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.Volts; // Static so that everything is there (No need to write Units.)

import java.util.function.DoubleSupplier;

public class TalonElevator extends SubsystemBase{ // EXTENDS SUBSYSTEMBASE!!!!!!!!!
    private TalonFX m_talon;
    private MotionMagicVoltage m_request;
    private VoltageOut m_voltReq;
    static final double rotationsPerInch = 1/1.35833; // 1.35833 inch/rot

    // 50Hz NetworkTable variables
    // Creates a new field that contains all output variables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable elevatorTable = inst.getTable("Elevator");

    // Position
    private final DoublePublisher motorRotPub = elevatorTable.getDoubleTopic("Motor rotations").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher rotationsTargetPub = elevatorTable.getDoubleTopic("Position Target").publish(PubSubOption.periodic(0.02));
    // Velocity
    private final DoublePublisher velocityPub = elevatorTable.getDoubleTopic("Velocity").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher velocityTargetPub = elevatorTable.getDoubleTopic("Velocity Target").publish(PubSubOption.periodic(0.02));
    // kS & kG (Feed forward)
    private final DoublePublisher closedLoopPub = elevatorTable.getDoubleTopic("Closed Loop Output").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher FFPub = elevatorTable.getDoubleTopic("Feed Forward").publish(PubSubOption.periodic(0.02));

    // Constructor
    public TalonElevator() {
        // Declarations
        m_talon = new TalonFX(50); // Falcon 500
        var talonFXConfigs = new TalonFXConfiguration(); // All paramater configs
        m_request = new MotionMagicVoltage(0); // Trapezoid config

        m_talon.setControl(m_request.withUpdateFreqHz(50));

        m_voltReq = new VoltageOut(0);

        // Limits and modes
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Set neutral mode
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 15; // Amps

        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40; // Rotations
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        
        // Tuning
        var slot0Configs = talonFXConfigs.Slot0;
        // kG and kS is mainly for overshoot + undershoot tuning
        // 0.5V is needed for gravity/friction
        slot0Configs.kG = 0.312; // Gravity 0.312 volt
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        slot0Configs.kS = 0.10; // Friction 0.07 volt
        slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        // kV and kA pairs with MM
        slot0Configs.kV = 0.12; // 0.12 volt/rps 
        slot0Configs.kA = 0.006; // 0.006 volt/rps/s 
        // kP and kD accounts for errors created by in-match hits
        slot0Configs.kP = 3.7; // 3.7 volt/(r*s) 
        slot0Configs.kD = 0.1; // 0.1 volt/rps 

        // Motion Magic (Trapezoid speed control)
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100; //rot/sec
        motionMagicConfigs.MotionMagicAcceleration = 150; //rot/sec^2
        motionMagicConfigs.MotionMagicJerk = 2000; //rot/sec^3

        m_talon.getConfigurator().apply(talonFXConfigs);
        m_talon.setPosition(0);  
    }

    // Setting elevator talon to spin to a certain height
    // a, b, x, and right bumper control different set heights
    public Command setHeightLevel(Heights setpoint) {
        return runOnce(() -> 
        {
            m_talon.setControl(m_request.withPosition(setpoint.heightInches*rotationsPerInch));
        });
    }

    // Joystick commanded open-loop
    public Command setHeightJoystickOpen(DoubleSupplier setpoint) {
        DutyCycleOut req = new DutyCycleOut(0);
        return run(() ->
        {
            m_talon.setControl(req.withOutput(setpoint.getAsDouble()));
        });
    }

    // Enum of certain heights
    public enum Heights { // An enum is a class of defined objects
        L0 ("L0", 0),
        L1 ("L1", 13.5833),
        L2 ("L2", 27.1666),
        L3 ("L3", 40.75),
        L4 ("L4", 54.3333);

        String level;
        double heightInches;

        // Constructor
        Heights(String level, double heightInches) {
            this.level = level;
            this.heightInches = heightInches;
        }
    }
    
    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Voltage Output", m_talon.getOutput().getValueAsDouble());
        // Field variable outputs
        // Position
        motorRotPub.set(m_talon.getPosition(true).getValueAsDouble());
        rotationsTargetPub.set(m_talon.getClosedLoopReference(true).getValueAsDouble());
        // Velocity
        velocityPub.set(m_talon.getVelocity(true).getValueAsDouble());
        velocityTargetPub.set(m_talon.getClosedLoopReferenceSlope(true).getValueAsDouble());
        // kS & kG (Feed forward)
        closedLoopPub.set(m_talon.getClosedLoopProportionalOutput(true).getValueAsDouble());
        FFPub.set(m_talon.getClosedLoopFeedForward(true).getValueAsDouble());
    }
}

// SmartDashboard output values (noob method)
/*SmartDashboard.putNumber("Motor Rotations", myTalon.getPosition().getValueAsDouble());
SmartDashboard.putNumber("Closed Loop Output", myTalon.getClosedLoopOutput().getValueAsDouble());
SmartDashboard.putNumber("FF Output", myTalon.getClosedLoopFeedForward().getValueAsDouble());
SmartDashboard.putNumber("Velocity", myTalon.getVelocity().getValueAsDouble());*/


/*public Command setHeightJoystick(DoubleSupplier setpoint) {
    return run(() ->
    {
        m_talon.setControl(m_request.withPosition(setpoint.getAsDouble()*40));
    });
}*/