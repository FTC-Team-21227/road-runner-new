package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.dashboard.config.Config;
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
@Config
@Autonomous(name = "SUPERCVTest_Right")
public class AutoRightPureVisionTest extends LinearOpMode {
    public static String color = "red";
    public static boolean inclYellow = false;
    @Override
    public void runOpMode() throws InterruptedException {
        PipeCamera cam = new PipeCamera(hardwareMap,telemetry, true,color, inclYellow);
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
