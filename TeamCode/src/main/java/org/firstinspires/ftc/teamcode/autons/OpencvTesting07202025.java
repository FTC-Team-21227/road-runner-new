package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.ExcludePipeline;

@Autonomous(name = "7/20/25 OpenCV Testing")
public class OpencvTesting07202025 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Opencv pipeline
        PipeCamera cam = new PipeCamera(hardwareMap,telemetry,true,"red", false);

        // Positions
        Pose2d initialPose = new Pose2d(10.5, -63.3, Math.toRadians(90));
        double firstSpecDistance = -51;

        // Initialize hardware
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1_V3Robot arm1 = new ARM1_V3Robot(hardwareMap);
        ARM2_V3Robot arm2 = new ARM2_V3Robot(hardwareMap);
        CLAW_NEW claw = new CLAW_NEW(hardwareMap);
        INTAKE_ANGLE_NEW intake_angle = new INTAKE_ANGLE_NEW(hardwareMap);
        CLAW_ANGLE_NEW claw_angle = new CLAW_ANGLE_NEW(hardwareMap);


        // Write trajectories
        Vector2d firstPlace = new Vector2d(10.5,firstSpecDistance);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) // first specimen
                .waitSeconds(0.4)
                .strafeTo(firstPlace);

        // Rotate claw servos to starting position
        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(),
                        intake_angle.RotatePosition1(),
                        claw_angle.forward()
                )
        );

        // Build trajectories
        Action firstTrajectory = tab1.build();

        // Wait for start
        waitForStart();

        // Run trajectories
        Actions.runBlocking(
                new SequentialAction(
                        // First specimen
                    new ParallelAction(
                        claw.closeClaw(),
                        intake_angle.RotatePosition3(),
                        arm1.liftRung_First(0,1.15),
                        arm2.liftRung2_First(0,1.15),
                        firstTrajectory,
                        claw.openClawMore(1.15)
                    ),
            cam.comp(0) // Compute sample position
            )
        );

        Pose2d color_pose = PoseStorage.grabColorPose;

        if (ExcludePipeline.printStuff) RobotLog.dd("COLOR_POSE", color_pose.position.x + ", " + color_pose.position.y + ", " + color_pose.heading.toDouble());

        // Create and build color_pose Trajectory
        Action openCVTrajectory = drive.actionBuilder(new Pose2d(firstPlace, Math.toRadians(90)))
                .strafeTo(color_pose.position)
                .build();

        // Go to color_pose
        Actions.runBlocking(
                new SequentialAction(
            new ParallelAction(
                        cam.close(0),
                        openCVTrajectory,

                        // Arm and claw controls
                        intake_angle.RotatePosition0_left(0.5),
                        arm1.liftVertFloor(0.2,1.1),
                        arm2.liftVertFloor(0.2,1.1)
            ),

            new ParallelAction(
                intake_angle.RotatePosition2(),
                claw.closeClaw(0.4)
            )
        )

        );
    }
}
