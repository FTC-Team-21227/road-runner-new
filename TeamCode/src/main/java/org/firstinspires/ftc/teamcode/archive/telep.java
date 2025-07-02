package org.firstinspires.ftc.teamcode.archive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

//@TeleOp(name = "telep")
public class telep extends LinearOpMode {
    private MecanumDrive_Lock drive;
    boolean startedHolding = false;
    Pose2d target;
    private List<Action> runningActions = new ArrayList<>();
    TelemetryPacket p = new TelemetryPacket();

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        //getting all the motors, servos, and sensors from the hardware map
        Pose2d initialPose;
        initialPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive_Lock(hardwareMap, initialPose);

        // Put initialization blocks here.
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                drive.updatePoseEstimate();
                // Put loop blocks here.
                if (gamepad2.x) {
                    holdRobotPosition();
                } else {
                    startedHolding = false;
                }
                List<Action> newActions = new ArrayList<>();
                for (Action action : runningActions) {
                    if (action.run(p)) {
                        newActions.add(action);
                    }
                }
                runningActions = newActions;
            }
        }
    }

    private void holdRobotPosition() {
        if (!startedHolding) {
            target = drive.localizer.getPose();
            Action a = drive.actionBuilder(drive.localizer.getPose(), () -> gamepad2.x)
                    .strafeToLinearHeading(target.position, target.heading.toDouble())
                    .build();
            runningActions.add(a);
            startedHolding = true;
        }
    }
}