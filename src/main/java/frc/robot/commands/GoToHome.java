package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Claw;
//import frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class GoToHome extends Command {    
    private Claw claw;

    public GoToHome(Claw claw_in) {
        this.claw = claw_in;
        addRequirements(claw_in);

        //this.robotCentricSup = robotCentricSup;
    }

    public Double getDesiredAngle(Double rotationSupX, Double rotationSupY) {
        double desiredAngleRad = 0;

        // SmartDashboard.putNumber("Controller X", rotationSupX);
        // SmartDashboard.putNumber("Controller Y", rotationSupY);
        if (rotationSupX > 0) {
            desiredAngleRad = (Math.PI / 2) - Math.atan((-1 * rotationSupY) /
                rotationSupX);
        } else if (rotationSupX < 0) {
            desiredAngleRad = (3 * (Math.PI / 2)) - Math.atan((-1 * rotationSupY) /
                rotationSupX);
        }

        return desiredAngleRad;
    }


    @Override
    public void execute() {
        
    }
}