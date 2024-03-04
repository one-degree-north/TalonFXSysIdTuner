package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Mechanism extends SubsystemBase {
    /* TODO: Set CAN ID and CAN Bus */
    private final CANSparkMax m_motorToTest = new CANSparkMax(16, MotorType.kBrushless);

    /* TODO: Uncomment this line to add a follower motor */
    private final CANSparkMax m_followerMotorToTest = new CANSparkMax(17, MotorType.kBrushless);
    
    

    private SysIdRoutine m_SysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Default ramp rate is acceptable
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                null          // Default timeout is acceptable
                                       // Log state with Phoenix SignalLogger class
                ),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts)-> m_motorToTest.setVoltage(volts.in(Volts)),
                null,
                this));

    public Mechanism() {
        setName("Mechanism");

        configureMotors();
    }

    public void configureMotors() {
        m_motorToTest.restoreFactoryDefaults();
        m_motorToTest.setSmartCurrentLimit(60);

        double gearRatio = 80.0/1.0;
        // GEAR RATIO (convert from motor rotations to output rotations)
        m_motorToTest.getEncoder().setPositionConversionFactor(1/gearRatio);

        // GEAR RATIO DIVIDED BY 60 (convert from motor RPM to output RPS)
        m_motorToTest.getEncoder().setVelocityConversionFactor((1/gearRatio)*(1/60.0));

        m_followerMotorToTest.follow(m_followerMotorToTest, true);
    }

    public Command joystickDriveCommand(DoubleSupplier output) {
        return run(()->m_motorToTest.set(output.getAsDouble()));
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Pivot Encoder", m_motorToTest.getEncoder().getPosition());
    }
}
