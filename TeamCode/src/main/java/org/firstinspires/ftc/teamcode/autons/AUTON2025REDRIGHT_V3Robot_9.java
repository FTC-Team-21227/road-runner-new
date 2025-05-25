package org.firstinspires.ftc.teamcode.autons;

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

@Autonomous(name = "AUTONRIGHT_V3_5+1_NewARMs")
//Trying for 5+1 auto just by pushing the samples faster
public class AUTON2025REDRIGHT_V3Robot_9 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10.5, -63.3, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ARM1_V3Robot arm1 = new ARM1_V3Robot(hardwareMap);
        ARM2_V3Robot arm2 = new ARM2_V3Robot(hardwareMap);
        CLAW_NEW claw = new CLAW_NEW(hardwareMap);
        INTAKE_ANGLE_NEW intake_angle = new INTAKE_ANGLE_NEW(hardwareMap);
        CLAW_ANGLE_NEW claw_angle = new CLAW_ANGLE_NEW(hardwareMap);
        SWEEPER sweeper = new SWEEPER(hardwareMap);
        double firstSpecDistance = -48;
        double otherSpecDistance = -37;
        double wallGrab = -46;
        double wallGrab1 = 21;
        double wallGrabAngle = -45;
        double frictionConstant = 0;
//        pushing timing to the limits

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                .setTangent(Math.toRadians(150))
                .splineToConstantHeading(new Vector2d(-5.5,firstSpecDistance),Math.toRadians(90));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-5.5, firstSpecDistance, Math.toRadians(90))) //push colored samples
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(0,firstSpecDistance-3),Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(49,-55),Math.toRadians(0));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(49, -55, Math.toRadians(90))) //push colored samples
                .strafeTo(new Vector2d(59,-55));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(59, -55, Math.toRadians(90))) //push colored samples
                .strafeTo(new Vector2d(64,-53.5))
                .waitSeconds(2.1)
                .turnTo(Math.toRadians(65));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(64, -53.5, drive.localizer.getPose().heading.toDouble())) //go to second specimen
                .strafeToLinearHeading(new Vector2d(67,-60),Math.toRadians(90));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(67, -60, Math.toRadians(90))) //pick up and place second specimen
                .setTangent(Math.toRadians(135))
                .waitSeconds(0.6)
                .splineToConstantHeading(new Vector2d(-2.5,firstSpecDistance),Math.toRadians(90));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(-2.5, firstSpecDistance, Math.toRadians(90))) //go to third specimen
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-45));
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(wallGrab1, wallGrab,Math.toRadians(wallGrabAngle))) //pick up and place second specimen
                .setTangent(Math.toRadians(135))
                .waitSeconds(0.6)
                .splineToLinearHeading(new Pose2d(0,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(0, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-45));
        TrajectoryActionBuilder tab10 = drive.actionBuilder(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle))) //pick up and place second specimen
                .setTangent(Math.toRadians(135))
                .waitSeconds(0.6)
                .splineToLinearHeading(new Pose2d(3.5,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
        TrajectoryActionBuilder tab11 = drive.actionBuilder(new Pose2d(3.5, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-45));
        TrajectoryActionBuilder tab12 = drive.actionBuilder(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle))) //pick up and place fourth specimen
                .setTangent(Math.toRadians(135))
                .waitSeconds(0.6)
                .splineToLinearHeading(new Pose2d(7,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
        TrajectoryActionBuilder tab13 = drive.actionBuilder(new Pose2d(7, otherSpecDistance, Math.toRadians(-90))) //park
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-45));
        TrajectoryActionBuilder tab14 = drive.actionBuilder(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle))) //park
                .strafeToLinearHeading(new Vector2d(-45,-74),Math.toRadians(45),new TranslationalVelConstraint(120), new ProfileAccelConstraint(-40,100));

        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(),
                        intake_angle.RotatePosition1(),
                        claw_angle.forward()
                )
        );

        Action firstTrajectory = tab1.build();
        Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();
        Action fourthTrajectory = tab4.build();
        Action fifthTrajectory = tab5.build();
        Action sixthTrajectory = tab6.build();
        Action seventhTrajectory = tab7.build();
        Action eighthTrajectory = tab8.build();
        Action ninthTrajectory = tab9.build();
        Action tenthTrajectory = tab10.build();
        Action eleventhTrajectory = tab11.build();
        Action twelfthTrajectory = tab12.build();
        Action thirteenthTrajectory = tab13.build();
        Action fourteenthTrajectory = tab14.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        //first specimen
                        new ParallelAction(
                                claw.closeClaw(),
                                intake_angle.RotatePosition3(),
                                arm1.liftRung_First(0,1.3),
                                arm2.liftRung2_First(0,1.3),
                                firstTrajectory
                        ),
                        new ParallelAction(
                                claw.openClawMore(),
                                intake_angle.RotatePosition2(0.2),
                                secondTrajectory,
                                intake_angle.RotatePosition0_left(1.5),
                                arm1.liftFloor(1.5,1),
                                arm2.liftFloor(1.5,1),
                                claw.closeClaw(2.3)
                        ),
                        new ParallelAction(
                                arm1.liftRung2(0.2,1.2),
                                arm2.liftDown(0.2,1.2),
                                claw.openClaw(1),
                                thirdTrajectory
                        ),
                        new ParallelAction(
                                arm1.liftFloor(0,1.2),
                                arm2.liftFloor(0,1.2),
                                claw.closeClaw(1)
                        ),
                        new ParallelAction(
                                arm1.liftRung2(0.4,1.2),
                                arm2.liftDown(0.4,1.2),
                                claw.openClaw(1.8),
                                arm1.liftFloor(2.6,1.2),
                                arm2.liftFloor(2.6,1.2),
                                claw_angle.backward(0.4),
                                fourthTrajectory,
                                claw.closeClaw(3.6)
                        ),
                        new ParallelAction(
                                arm1.liftWall_First(0.2,1.4),
                                arm2.liftWall2_First(0.2,1.4),
                                claw_angle.forward(0.4),
                                intake_angle.RotatePositionNegative2(),
                                claw.openClaw(1.4)
                        ),
                        //face the wall and go to second specimen
                        new ParallelAction(
                                fifthTrajectory
                        ),
                        claw.closeClaw(),
                        //pick up and place second specimen
                        new ParallelAction(
                                arm1.liftRung_FirstSecond(0.2),
                                arm2.liftRung2_First(0.2),
                                sixthTrajectory,
                                intake_angle.RotatePosition0(0.4),
                                claw_angle.backward(0.4)
                        ),
                        //go to third specimen
                        new ParallelAction(
                                claw.openClawMore(),
                                seventhTrajectory,
                                arm1.liftFloor(0.2,1.5), //0.5
                                arm2.liftFloor(0.2,1.5),
                                claw_angle.backward(0),
                                intake_angle.RotatePosition0(0.5)
                        ),
                        //pick up and place third specimen
                        claw.closeClaw(),
                        new ParallelAction(
                                arm1.liftRung2(0.2,1.7),
                                arm2.liftRung2(0.2,1.7),
                                eighthTrajectory,
                                claw_angle.forward(0.5)
                        ),
                        //go to fourth specimen
                        new ParallelAction(
                                claw.openClawMore(),
                                ninthTrajectory,
                                arm1.liftFloor(0.2,1.5),
                                arm2.liftFloor(0.2,1.5),
                                claw_angle.backward(0)
                        ),
                        //pick up and place fourth specimen
                        claw.closeClaw(),
                        new ParallelAction(
                                arm1.liftRung2(0.2,1.7), //0.3
                                arm2.liftRung2(0.2,1.7),
                                tenthTrajectory,
                                claw_angle.forward(0.5)
                        ),
                        //go to fifth specimen
                        new ParallelAction(
                                claw.openClawMore(),
                                eleventhTrajectory,
                                arm1.liftFloor(0.2,1.5),
                                arm2.liftFloor(0.2,1.5),
                                claw_angle.backward(0)
                        ),
                        //pick up and place fifth specimen
                        claw.closeClaw(),
                        new ParallelAction(
                                arm1.liftRung2(0.2,1.7),
                                arm2.liftRung2(0.2,1.7),
                                twelfthTrajectory,
                                claw_angle.forward(0.5)
                        ),
                        new ParallelAction(
                                claw.openClawMore(),
                                thirteenthTrajectory,
                                arm1.liftFloor(0.2,1.5),
                                arm2.liftFloor(0.2,1.5),
                                claw_angle.backward(0)
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                                arm1.liftHighBasket(0.3),
                                arm2.liftHighBasket(0.3),
                                intake_angle.RotatePosition0_basket(1),
                                fourteenthTrajectory
                        ),
                        claw.openClaw()
                )
        );
        PoseStorage.currentPose = new Pose2d(
                drive.localizer.getPose().position.y + 72.3,                  // New x becomes original y (rotation)
                -drive.localizer.getPose().position.x + 75.3,                 // New y becomes negative original x (rotation)
                drive.localizer.getPose().heading.plus(-Math.PI/2).toDouble()     // Subtract 90 degrees (π/2 radians) from heading
        );    }
}
