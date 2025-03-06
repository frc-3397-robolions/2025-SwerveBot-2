package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveRobot extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private final PIDController m_keepAnglePID = new PIDController(Constants.KeepAngle.kp, Constants.KeepAngle.ki, Constants.KeepAngle.kd);

    public DriveRobot(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSupX, DoubleSupplier rotationSupY/*, BooleanSupplier robotCentricSup*/) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        double currentAngleRad = s_Swerve.getGyroYaw().getRadians() % (2 * Math.PI);
        SmartDashboard.putNumber("Current Angle", currentAngleRad);

        double desiredAngle = getDesiredAngle(rotationSupX, rotationSupY);

        m_keepAnglePID.enableContinuousInput(0, (2 * Math.PI));

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = () -> m_keepAnglePID.calculate(currentAngleRad, desiredAngle);
        //this.robotCentricSup = robotCentricSup;
    }

    public Double getDesiredAngle(DoubleSupplier rotationSupX, DoubleSupplier rotationSupY) {
        double desiredAngleRad = 0;

        // SmartDashboard.putNumber("Controller X", rotationSupX);
        // SmartDashboard.putNumber("Controller Y", rotationSupY);
        if (rotationSupX.getAsDouble() > 0) {
            desiredAngleRad = (Math.PI / 2) - Math.atan((-1 * rotationSupY.getAsDouble()) /
                rotationSupX.getAsDouble());
        } else if (rotationSupX.getAsDouble() < 0) {
            desiredAngleRad = (3 * (Math.PI / 2)) - Math.atan((-1 * rotationSupY.getAsDouble()) /
                rotationSupX.getAsDouble());
        }

        return desiredAngleRad;
    }


    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}