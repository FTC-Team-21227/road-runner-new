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
@Autonomous(name = "SUPERCVTest_Left")
public class AutoLeftPureVisionTest extends LinearOpMode {
    public static String color = "red";
    public static boolean inclYellow = true;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10.5, -51, Math.toRadians(90));
        PipeCamera_Deubg cam = new PipeCamera_Deubg(hardwareMap,telemetry, false,color, inclYellow);

        ARM1_V3Robot arm1 = new ARM1_V3Robot(hardwareMap);

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
    }
}
