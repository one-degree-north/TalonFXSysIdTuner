package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Mechanism extends SubsystemBase {
    /* TODO: Set CAN ID and CAN Bus */
    private final TalonFX m_leftFrontDrive = new TalonFX(8);
    // Left back drive is master drive motor
    private final TalonFX m_leftBackDrive = new TalonFX(6);
    private final TalonFX m_rightFrontDrive = new TalonFX(2);
    private final TalonFX m_rightBackDrive = new TalonFX(4);

    private final TalonFX m_leftFrontAngle = new TalonFX(7);
    private final TalonFX m_leftBackAngle = new TalonFX(5);
    private final TalonFX m_rightFrontAngle = new TalonFX(1);
    private final TalonFX m_rightBackAngle = new TalonFX(3);

    private final CANcoder m_leftFrontCANCoder = new CANcoder(12);
    private final CANcoder m_leftBackCANCoder = new CANcoder(11);
    private final CANcoder m_rightFrontCANCoder = new CANcoder(9);
    private final CANcoder m_rightBackCANCoder = new CANcoder(10);

    private final double leftFrontEncoderOffset = Units.degreesToRotations(55.8);
    private final double leftBackEncoderOffset = Units.degreesToRotations(42.27);
    private final double rightFrontEncoderOffset = Units.degreesToRotations(37.79);
    private final double rightBackEncoderOffset = Units.degreesToRotations(-92);

    /* TODO: Uncomment this line to add a follower motor */
    
    private final DutyCycleOut m_joystickControl = new DutyCycleOut(0);
    private final VoltageOut m_sysidControl = new VoltageOut(0);
    private final PositionVoltage m_angleConrol = new PositionVoltage(0);

    private SysIdRoutine m_SysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Default ramp rate is acceptable
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                null,          // Default timeout is acceptable
                                       // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts)-> m_leftBackDrive.setControl(m_sysidControl.withOutput(volts.in(Volts))),
                null,
                this));

    public Mechanism() {
        setName("Mechanism");

        configureMotors();

        resetToAbsolute();

        /* Speed up signals for better charaterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
            m_leftBackDrive.getPosition(),
            m_leftBackDrive.getVelocity(),
            m_leftBackDrive.getMotorVoltage(),
            m_leftFrontDrive.getPosition(),
            m_leftFrontDrive.getVelocity(),
            m_leftFrontDrive.getMotorVoltage(),
            m_rightBackDrive.getPosition(),
            m_rightBackDrive.getVelocity(),
            m_rightBackDrive.getMotorVoltage(),
            m_rightFrontDrive.getPosition(),
            m_rightFrontDrive.getVelocity(),
            m_rightFrontDrive.getMotorVoltage());

        /* Optimize out the other signals, since they're not particularly helpful for us */

        SignalLogger.start();
    }

    public void resetToAbsolute() {
        m_leftFrontAngle.setPosition(m_leftFrontCANCoder.getAbsolutePosition().getValue()-leftFrontEncoderOffset);
        m_leftBackAngle.setPosition(m_leftBackCANCoder.getAbsolutePosition().getValue()-leftBackEncoderOffset);
        m_rightFrontAngle.setPosition(m_rightFrontCANCoder.getAbsolutePosition().getValue()-rightFrontEncoderOffset);        
        m_rightBackAngle.setPosition(m_rightBackCANCoder.getAbsolutePosition().getValue()-rightBackEncoderOffset);
    }

    public void configureMotors() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 35;
        cfg.CurrentLimits.SupplyCurrentThreshold = 60;
        cfg.CurrentLimits.SupplyTimeThreshold = 0.1;

        cfg.Feedback.SensorToMechanismRatio = 6.12;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.Slot0.kP = 0.15;

        m_leftBackDrive.getConfigurator().apply(cfg);
        m_leftFrontDrive.setControl(new Follower(m_leftBackDrive.getDeviceID(), false));
        m_rightBackDrive.setControl(new Follower(m_leftBackDrive.getDeviceID(), false));
        m_rightFrontDrive.setControl(new Follower(m_leftBackDrive.getDeviceID(), false));

        TalonFXConfiguration angleCfg = new TalonFXConfiguration();
        angleCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        angleCfg.CurrentLimits.SupplyCurrentLimit = 35;
        angleCfg.CurrentLimits.SupplyCurrentThreshold = 60;
        angleCfg.CurrentLimits.SupplyTimeThreshold = 0.1;

        angleCfg.Feedback.SensorToMechanismRatio = 150.0/7.0;
        angleCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        angleCfg.Slot0.kP = 110.0;
        angleCfg.ClosedLoopGeneral.ContinuousWrap = true;

        m_leftBackAngle.getConfigurator().apply(angleCfg);
        m_leftFrontAngle.getConfigurator().apply(angleCfg);
        m_rightBackAngle.getConfigurator().apply(angleCfg);
        m_rightFrontAngle.getConfigurator().apply(angleCfg);

        CANcoderConfiguration cancoderCfg = new CANcoderConfiguration();
        cancoderCfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        
        m_leftBackCANCoder.getConfigurator().apply(cancoderCfg);
        m_leftFrontCANCoder.getConfigurator().apply(cancoderCfg);
        m_rightBackCANCoder.getConfigurator().apply(cancoderCfg);
        m_rightFrontCANCoder.getConfigurator().apply(cancoderCfg);

    }

    public Command joystickDriveCommand(DoubleSupplier output) {
        return run(()->m_leftBackDrive.setControl(m_joystickControl.withOutput(output.getAsDouble())));
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        m_leftBackAngle.setControl(m_angleConrol.withPosition(0));
        m_leftFrontAngle.setControl(m_angleConrol.withPosition(0));
        m_rightBackAngle.setControl(m_angleConrol.withPosition(0));
        m_rightFrontAngle.setControl(m_angleConrol.withPosition(0));

        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Left Front CANCoder", Units.rotationsToDegrees(m_leftFrontCANCoder.getAbsolutePosition().getValue()));
        SmartDashboard.putNumber("Right Front CANCoder", Units.rotationsToDegrees(m_rightFrontCANCoder.getAbsolutePosition().getValue()));
        SmartDashboard.putNumber("Left Back CANCoder", Units.rotationsToDegrees(m_leftBackCANCoder.getAbsolutePosition().getValue()));
        SmartDashboard.putNumber("Right Back CANCoder", Units.rotationsToDegrees(m_rightBackCANCoder.getAbsolutePosition().getValue()));

        SmartDashboard.putNumber("Left Front TalonFX", Units.rotationsToDegrees(m_leftFrontAngle.getPosition().getValue()));
        SmartDashboard.putNumber("Right Front TalonFX", Units.rotationsToDegrees(m_rightFrontAngle.getPosition().getValue()));
        SmartDashboard.putNumber("Left Back TalonFX", Units.rotationsToDegrees(m_leftBackAngle.getPosition().getValue()));
        SmartDashboard.putNumber("Right Back TalonFX", Units.rotationsToDegrees(m_rightBackAngle.getPosition().getValue()));
        
    }
}
