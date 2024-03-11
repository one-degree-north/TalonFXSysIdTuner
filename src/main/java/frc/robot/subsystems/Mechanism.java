package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Mechanism extends SubsystemBase {
    /* TODO: Set CAN ID and CAN Bus */
    private final TalonFX m_motorToTest = new TalonFX(13, "*");

    /* TODO: Uncomment this line to add a follower motor */
    private final TalonFX m_followerMotorToTest = new TalonFX(14, "*");

    private final TalonFX m_motorToBrake = new TalonFX(15, "*");
    
    private final DutyCycleOut m_joystickControl = new DutyCycleOut(0);
    private final VoltageOut m_sysidControl = new VoltageOut(0);

    private final MotionMagicVoltage positionLock = new MotionMagicVoltage(0);

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);

    private double elevatorPositionSetpoint = 0;

    private SysIdRoutine m_SysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Default ramp rate is acceptable
                Volts.of(2), // Reduce dynamic voltage to 4 to prevent motor brownout
                null,          // Default timeout is acceptable
                                       // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts)-> m_motorToTest.setControl(m_sysidControl.withOutput(volts.in(Volts))),
                null,
                this));

    public Mechanism() {
        setName("Mechanism");

        configureMotors();

        /* Speed up signals for better charaterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
            m_motorToTest.getPosition(),
            m_motorToTest.getVelocity(),
            m_motorToTest.getMotorVoltage());

        /* Optimize out the other signals, since they're not particularly helpful for us */
        m_motorToTest.optimizeBusUtilization();

        Timer.delay(1);
        m_motorToBrake.setPosition(0);
        m_motorToTest.setPosition(-encoder.getAbsolutePosition() - (-0.2778));

        SignalLogger.start();

        setElevatorPosition(1.7);
    }

    public void configureMotors() {
        /* TODO: Edit this method to add follower motor and set
           SensorToMechanismRatio, MotorInvert
         */
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 35;
        cfg.CurrentLimits.SupplyCurrentThreshold = 60;
        cfg.CurrentLimits.SupplyTimeThreshold = 0.1;

        cfg.Feedback.SensorToMechanismRatio = 120.0/1.0;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_motorToTest.getConfigurator().apply(cfg);

        /* TODO: Uncomment to add follower motor */
        m_followerMotorToTest.setControl(new Follower(m_motorToTest.getDeviceID(), true));

        TalonFXConfiguration breakConfig = new TalonFXConfiguration();
        breakConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        breakConfig.CurrentLimits.SupplyCurrentLimit = 35;
        breakConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        breakConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
        breakConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        breakConfig.Feedback.SensorToMechanismRatio = 5.0/1.0;
        breakConfig.Slot0.withKP(Constants.elevatorkP).withKS(Constants.elevatorkS)
        .withKV(Constants.elevatorkV);

        breakConfig.MotionMagic.MotionMagicCruiseVelocity = 2;
        breakConfig.MotionMagic.MotionMagicAcceleration = 5;
        breakConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motorToBrake.getConfigurator().apply(breakConfig);
    }

    public void setElevatorPosition(double setpoint) {
        elevatorPositionSetpoint = setpoint;
    }

    public Command joystickDriveCommand(DoubleSupplier output) {
        return run(()->m_motorToTest.setControl(m_joystickControl.withOutput(output.getAsDouble())));
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
    }
    
    @Override
    public void periodic() {
        m_motorToBrake.setControl(positionLock.withPosition(elevatorPositionSetpoint)
        .withFeedForward(Constants.elevatorkG * Math.sin(Units.rotationsToRadians(m_motorToTest.getPosition().getValue()))
        )
        );


    }
}
