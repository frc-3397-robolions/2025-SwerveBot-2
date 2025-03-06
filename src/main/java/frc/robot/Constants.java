package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;


public final class Constants {

    public static final class ButtonMap {
        public static final int L1 = 5;
        public static final int L2 = 3;
        public static final int L3 = 4;
        public static final int L4 = 6;
        public static final int L23 = 1;
        public static final int L34 = 2;
        public static final int Floor = 1; // axis
        public static final int Barge = 3; // axis
        public static final int Processor = 2; // axis
        public static final int cIn = 9;
        public static final int cOut = 10;
        public static final int aIn = 3; // trigger -- XBox
        public static final int aOut = 6; // XBox
        public static final int ClimbIn = 2; // trigger  -- XBox
        public static final int ClimbOut = 5; // Xbox
        public static final int Callibration = 10; // Xbox
    }

    public static final double stickDeadband = 0.1;

    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
      public static final double kDeadband = 0.08;
      public static final double kCubic = 0.95;
      public static final double kLinear = 0.05;
      public static final int kOperatorControllerPort = 0;
    }

    public static final class KeepAngle {
        public static final double kp = 4.3;
        public static final double ki = 0.0;
        public static final double kd = 0.03;
    }
    
    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class CurrentLimit {
        public static final int kDrive = 60;
        public static final int kAzimuth = 20;
        public static final int kIntakeAngle = 40;
        public static final int kIntakeWheels = 20;
        public static final int kShooter = 40;
      }

      public static final class GlobalConstants {
        public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
        public static final int PCHID = 20;
        public static final int PDHID = 1;
        public static final double kLoopTime = 0.020;
      }

      public static final class States {
        
        public static Map<PositionState, Double> DesiredHeightMap = Map.of(
            PositionState.Floor, 0.0,
            PositionState.Home, 0.0,
            PositionState.L2, 0.0,
            PositionState.L23, 0.0,
            PositionState.L3, 0.0,
            PositionState.L34, 0.0,
            PositionState.L4, 0.0,
            PositionState.Barge, 0.0,
            PositionState.Processor, 0.0
            );

        // degrees
        public static Map<PositionState, Double> DesiredAngleMap = Map.of(
                //PositionState.Floor, 100.0,
                PositionState.Home, 0.0,
                PositionState.L2, 45.0,
                PositionState.L23, 180.0,
                PositionState.L3, 45.0,
                PositionState.L34, 180.0,
                PositionState.L4, 90.0
                //PositionState.Barge, 135.0,
                //PositionState.Processor, 170.0
                );

        public enum PositionState {
            Floor,
            Home,
            L2,
            L23,
            L3,
            L34,
            L4,
            Barge,
            Processor
        }
      }

      public static final class ClawConstants {
        public static final int CANIDAngle = 15;
        public static final int CANIDDrive = 11;
        public static final double kP = 0.5; // TODO
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;      
        public static final double kGearRatio = (58.0/10.0) * (58.0/18.0) * (30.0/15.0);
        public static final double kAnglePositionFactor = ((2 * Math.PI) / (kGearRatio));
        public static final double kPositionTolerance = 0.04;
        public static final double kIntakePower = 0.75;
        public static final double kOuttakePower = -1;
        public static final double kMaxVel = 7;
        public static final double kMaxAccel = 5;
      }

      public static final class ElevatorConstants {
        public static final int CANIDMotor1 = 13; //TODO: fix
        public static final int CANIDMotor2 = 14;
        public static final double kP = 0.75; // TODO
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;      
        public static final double kHeightPositionFactor = (1.76 * Math.PI);  // pitch diameter of sprocket
        public static final double kPositionTolerance = 0.04; 
        public static final double kMaxVel = 7;
        public static final double kMaxAccel = 5;
      }

      public static final class IntakeConstants {
        public static final int kCANID = 11;
        public static final int distanceStop = 1;
        public static final double kPower = 0.75;
        public static final double kVelocityFactor = 60;
        public static final double kNEOMaxSpeed = 5820 / 60;
        public static final double kv = 1.0 / (kNEOMaxSpeed * kVelocityFactor);
      }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}