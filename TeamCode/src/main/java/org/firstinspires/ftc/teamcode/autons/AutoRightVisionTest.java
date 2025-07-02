package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "TheCVTest_Right")
public class AutoRightVisionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10.5, -63.3, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        PipeCamera_Deubg cam = new PipeCamera_Deubg(hardwareMap,telemetry,true,"yellow", true);

        ARM1_V3Robot arm1 = new ARM1_V3Robot(hardwareMap);
        ARM2_V3Robot arm2 = new ARM2_V3Robot(hardwareMap);
        CLAW_NEW claw = new CLAW_NEW(hardwareMap);
        INTAKE_ANGLE_NEW intake_angle = new INTAKE_ANGLE_NEW(hardwareMap);
        CLAW_ANGLE_NEW claw_angle = new CLAW_ANGLE_NEW(hardwareMap);
        double firstSpecDistance = -51;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                .setTangent(Math.toRadians(90))
                .waitSeconds(0.3)
                .splineToConstantHeading(new Vector2d(10.5,firstSpecDistance),Math.toRadians(90));

        MecanumDrive.PARAMS.axialGain = 2.0;
        MecanumDrive.PARAMS.lateralGain = 1.0;

        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(),
                        intake_angle.RotatePosition1(),
                        claw_angle.forward()
                )
        );

        Action firstTrajectory = tab1.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        //first specimen
                        new ParallelAction(
                                claw.closeClaw(),
                                intake_angle.RotatePosition3(),
                                arm1.liftRung_First(0,1.15),
                                arm2.liftRung2_First(0,1.15),
                                firstTrajectory,
                                cam.comp(1.15)
                        )
                )
        );
    }
}
