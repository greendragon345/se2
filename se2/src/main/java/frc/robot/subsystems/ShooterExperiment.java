package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterExperiment extends SubsystemBase {

    private final TalonFX motor;
    private final SimpleMotorFeedforward ff;
    private double targetVelocity;

    public ShooterExperiment() {
        motor = new TalonFX(Constants.motorPortNum);
        motor.configFactoryDefault();
        ff = new SimpleMotorFeedforward(Constants.shooterKs, Constants.shooterKv);
        motor.config_kD(0, Constants.kdMotor);
        motor.config_kP(0, Constants.kpMotor);
        motor.config_kI(0, Constants.kiMotor);

        targetVelocity = 0;
    }

    public void setP(double p) {
        motor.set(ControlMode.PercentOutput, p);
    }

    public void setV(double v) {
        motor.set(ControlMode.Velocity, v*Constants.pulsesPerMeterS/10, 
                DemandType.ArbitraryFeedForward, ff.calculate(v));
    }

    public double getVelocity() {
        return motor.getSelectedSensorVelocity()/Constants.pulsesPerMeterS*10;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Velocity", this::getVelocity, null);
        builder.addDoubleProperty("Error", motor::getClosedLoopError, null);
        builder.addDoubleProperty("Output", motor::getMotorOutputPercent, null);
        builder.addDoubleProperty("Setpoint", motor::getClosedLoopTarget, null);

        builder.addDoubleProperty("Target Velocity", null, (num) -> {targetVelocity = num;});
        SmartDashboard.putData(new RunCommand(() -> setV(targetVelocity), this));

        super.initSendable(builder);
    }
}
