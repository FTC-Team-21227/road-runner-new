package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "SUPERCVTest_Right")
public class AutoRightPureVisionTest extends LinearOpMode {
    public static String color = "red";
    public static boolean inclYellow = false;
    @Override
    public void runOpMode() throws InterruptedException {
        PipeCamera_Debug cam = new PipeCamera_Debug(hardwareMap,telemetry, true, color, inclYellow);
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
