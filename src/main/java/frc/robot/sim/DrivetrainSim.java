// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
/**
* Implementation of a simulation of robot physics, sensors, motor controllers Includes a Simulated
* PhotonVision system and one vision target.
*
* <p>This class and its methods are only relevant during simulation. While on the real robot, the
* real motors/sensors/physics are used instead.
*/
public class DrivetrainSim {
    // Simulated Motor Controllers
    PWMSim leftLeader = new PWMSim(0);
    PWMSim rightLeader = new PWMSim(1);

    // Simulation Physics
    // Configure these to match your drivetrain's physical dimensions
    // and characterization results.
    LinearSystem<N2, N2, N2> drivetrainSystem =
            LinearSystemId.identifyDrivetrainSystem(1, 0.2, 1.5, 0.3);
    DifferentialDrivetrainSim drivetrainSimulator =
            new DifferentialDrivetrainSim(
                    drivetrainSystem,
                    DCMotor.getCIM(4),
                    1,
                    Units.feetToMeters(2.0),
                    Units.inchesToMeters(6.0 / 2.0),
                    null);

    // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    double camDiagFOV = 170.0; // degrees - assume wide-angle camera
    double camPitch = Units.radiansToDegrees(Robot.CAMERA_PITCH_RADIANS); // degrees
    double camHeightOffGround = Robot.CAMERA_HEIGHT_METERS; // meters
    double maxLEDRange = 50; // meters
    int camResolutionWidth = 640; // pixels
    int camResolutionHeight = 480; // pixels
    double minTargetArea = 4; // square pixels

    SimVisionSystem simVisionAvoid =
            new SimVisionSystem(
                    "avoid",
                    camDiagFOV,
                    camPitch,
                    new Transform2d(),
                    camHeightOffGround,
                    maxLEDRange,
                    camResolutionWidth,
                    camResolutionHeight,
                    minTargetArea);
        SimVisionSystem simVisionTarget =
                    new SimVisionSystem(
                            "target",
                            camDiagFOV,
                            camPitch,
                            new Transform2d(),
                            camHeightOffGround,
                            maxLEDRange,
                            camResolutionWidth,
                            camResolutionHeight,
                            minTargetArea);

    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 208
    double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
    // See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // pages 4 and 5
    double tgtXPos = Units.feetToMeters(82.5+(7.7/2));
    double tgtYPos =
            Units.feetToMeters(7.5/2);
    Pose2d farTargetPose = new Pose2d(new Translation2d(tgtXPos, tgtYPos), new Rotation2d(0.0));
    Pose2d nearTargetPose = new Pose2d(new Translation2d(Units.feetToMeters(15),Units.feetToMeters(15)),new Rotation2d(0,0));
    Pose2d nearTargetPose2 = new Pose2d(new Translation2d(Units.feetToMeters(15),Units.feetToMeters(30)),new Rotation2d(0,0));

    Field2d field = new Field2d();

    //Field2d target = new Field2d();

    public DrivetrainSim() {
        simVisionAvoid.addSimVisionTarget(
                new SimVisionTarget(nearTargetPose, Robot.TARGET_HEIGHT_METERS, targetWidth, targetHeight));
        simVisionAvoid.addSimVisionTarget(
                        new SimVisionTarget(nearTargetPose2, Robot.TARGET_HEIGHT_METERS, targetWidth, targetHeight));
        simVisionTarget.addSimVisionTarget(        
                new SimVisionTarget(farTargetPose, Robot.TARGET_HEIGHT_METERS, targetWidth, targetHeight));
        
        field.getObject("FarGoal").setPose(farTargetPose);
        //SmartDashboard.putData("Target");
        field.getObject("NearGoal").setPose(nearTargetPose);
        field.getObject("NearGoa2l").setPose(nearTargetPose2);
        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("BatterySize", Units.feetToMeters(1));
        SmartDashboard.putNumber("Goal Size", Units.feetToMeters(7.5));
    }

    /**
    * Perform all periodic drivetrain simulation related tasks to advance our simulation of robot
    * physics forward by a single 20ms step.
    */
    public void update() {
        double leftMotorCmd = 0;
        double rightMotorCmd = 0;

        leftMotorCmd = leftLeader.getSpeed();
        rightMotorCmd = rightLeader.getSpeed();
        

        drivetrainSimulator.setInputs(
                leftMotorCmd * RobotController.getInputVoltage(),
                -rightMotorCmd * RobotController.getInputVoltage());
        drivetrainSimulator.update(0.02);

        // Update PhotonVision based on our new robot position.
        simVisionAvoid.processFrame(drivetrainSimulator.getPose());
        simVisionTarget.processFrame(drivetrainSimulator.getPose());

        field.setRobotPose(drivetrainSimulator.getPose());
    }

    /**
    * Resets the simulation back to a pre-defined pose Useful to simulate the action of placing the
    * robot onto a specific spot in the field (IE, at the start of each match).
    *
    * @param pose
    */
    public void resetPose(Pose2d pose) {
        drivetrainSimulator.setPose(pose);
    }
}
