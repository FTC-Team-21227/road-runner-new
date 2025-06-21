package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "YELLOWLEFT_SUPERCVTest")
public class AutoLeftPureVisionTest extends LinearOpMode {
    public static String color = "red";
    public static boolean inclYellow = true;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10.5, -51, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        PipeCamera cam = new PipeCamera(hardwareMap,telemetry, false,color, inclYellow);

        ARM1_V3Robot arm1 = new ARM1_V3Robot(hardwareMap);
        ARM2_V3Robot arm2 = new ARM2_V3Robot(hardwareMap);
        CLAW_NEW claw = new CLAW_NEW(hardwareMap);
        INTAKE_ANGLE_NEW intake_angle = new INTAKE_ANGLE_NEW(hardwareMap);
        CLAW_ANGLE_NEW claw_angle = new CLAW_ANGLE_NEW(hardwareMap);
        SWEEPER sweeper = new SWEEPER(hardwareMap);
        double firstSpecDistance = -51;
        double otherSpecDistance = -36;
        double wallGrab = -46;
        double wallGrab1 = 21.5;
        double wallGrabAngle = -45;
        double frictionConstant = 0;
//        pushing timing to the limits

        double X = 0;

        double Y = 0;

        PoseStorage.grabColorPose = new Pose2d(10.5+X,-46+Y,Math.toRadians(90));
        PoseStorage.target1 = 42.1839;
        PoseStorage.target2 = 156.39;

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        //first specimen
                        new ParallelAction(
                                cam.comp(0),
                                arm1.liftHighBasket(100)
                        )
                )
        );
        Pose2d pose = drive.localizer.getPose();
        PoseStorage.currentPose =
        new Pose2d(
                pose.position.x + 15*Math.sqrt(2),
                pose.position.y + 15*Math.sqrt(2),
                Math.toRadians(45)
        );
        PoseStorage.arm1 = arm1;
        PoseStorage.arm2 = arm2;
    }
}
