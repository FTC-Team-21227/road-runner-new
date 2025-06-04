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

@Autonomous(name = "RIGHT_5+1_Hyperspeed_Consistent")
public class AUTON2025REDRIGHT_V4Robot_2 extends LinearOpMode {
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
        double firstSpecDistance = -51;
        double otherSpecDistance = -38;
        double wallGrab = -45.5;
        double wallGrab1 = 21;
        double wallGrabAngle = -45;
        double frictionConstant = 0;
//        pushing timing to the limits

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                .setTangent(Math.toRadians(90))
                .waitSeconds(0.3)
                .splineToConstantHeading(new Vector2d(10.5,firstSpecDistance),Math.toRadians(90));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10.5, firstSpecDistance, Math.toRadians(90))) //push colored samples
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(14,-59),Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(22,-61),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30+frictionConstant, -59),Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(40,-42),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40,-22),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(45+frictionConstant, -8),Math.toRadians(0))
//                .strafeTo(new Vector2d(50, -48))
                .splineToConstantHeading(new Vector2d(52+frictionConstant,-22),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(52,-46),Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(52, -22),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(57.5+frictionConstant, -16),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60.5+frictionConstant, -22),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(60.5, -46),Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60.5, -22),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(66+frictionConstant, -16),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(69+frictionConstant, -22),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(69, -60.5),Math.toRadians(-90), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-20, 80));
        MecanumDrive.PARAMS.axialGain = 2.0;
        MecanumDrive.PARAMS.lateralGain = 1.0;
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(69, -60.5, Math.toRadians(90))) //pick up and place second specimen
                .setTangent(Math.toRadians(175))
                .waitSeconds(0.4)
                .splineToConstantHeading(new Vector2d(-6.5,firstSpecDistance+2),Math.toRadians(90));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(-6.5, firstSpecDistance+2, Math.toRadians(90))) //go to third specimen
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(9, firstSpecDistance-3),Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-45))
                .strafeToLinearHeading(new Vector2d(wallGrab1,wallGrab),Math.toRadians(wallGrabAngle));
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(wallGrab1, wallGrab,Math.toRadians(wallGrabAngle))) //pick up and place second specimen
                .setTangent(Math.toRadians(135))
//                .waitSeconds(0.4)
                .splineToLinearHeading(new Pose2d(3.5,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(3.5, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-45));
        TrajectoryActionBuilder tab10 = drive.actionBuilder(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle))) //pick up and place second specimen
                .setTangent(Math.toRadians(135))
//                .waitSeconds(0.4)
                .splineToLinearHeading(new Pose2d(6,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
        TrajectoryActionBuilder tab11 = drive.actionBuilder(new Pose2d(7, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-45));
        TrajectoryActionBuilder tab12 = drive.actionBuilder(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle))) //pick up and place fourth specimen
                .setTangent(Math.toRadians(135))
//                .waitSeconds(0.4)
                .splineToLinearHeading(new Pose2d(8.5,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
        TrajectoryActionBuilder tab13 = drive.actionBuilder(new Pose2d(8.5, otherSpecDistance, Math.toRadians(-90))) //park
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-45));
        TrajectoryActionBuilder tab14 = drive.actionBuilder(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle))) //park
                .strafeToLinearHeading(new Vector2d(-49,-55),Math.toRadians(45),new TranslationalVelConstraint(120), new ProfileAccelConstraint(-40,100));

        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(),
                        intake_angle.RotatePosition1(),
                        claw_angle.forward()
                )
        );

        Action firstTrajectory = tab1.build();
        Action secondTrajectory = tab2.build();
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
                                arm1.liftRung_First(0,1.15),
                                arm2.liftRung2_First(0,1.15),
                                firstTrajectory
                        ),
                        new ParallelAction(
                                claw.openClawMore(),
                                secondTrajectory,
                                arm1.liftWall_First(1,1.4),
                                arm2.liftWall2_First(1,1.4),
                                claw_angle.forward(0.4),
                                intake_angle.RotatePositionNegative2(1)
                        ),
                        claw.closeClaw(),
                        //pick up and place second specimen
                        new ParallelAction(
                                arm1.liftRung_FirstSecond(0.2),
                                arm2.liftRung2_First(0.2),
                                sixthTrajectory,
                                claw_angle.backward(0.2),
                                intake_angle.RotatePosition4(0.4)
                        ),
                        //go to third specimen
                        new ParallelAction(
                                seventhTrajectory,
                                claw.openClawMore(),
                                intake_angle.RotatePositionNegative1(0),
                                arm1.liftFloor(0.3,1.1, false), //0.5
                                arm2.liftFloor(0.3,1.1, false),
                                claw_angle.backward(0),
                                intake_angle.RotatePosition0(0.5),
                                claw.closeClaw(1.2)
                        ),
                        //pick up and place third specimen
                        new ParallelAction(
                                arm1.liftRung2(0,1.2),
                                arm2.liftRung2(0,1.2),
                                eighthTrajectory,
                                claw_angle.forward(0)
                        ),
                        //go to fourth specimen
                        new ParallelAction(
                                ninthTrajectory,
                                claw.openClawMore(),
                                arm1.liftFloor(0.2,1.1, false),
                                arm2.liftFloor(0.2,1.1,false),
                                claw_angle.backward(0),
                                claw.closeClaw(1.05)
                        ),
                        //pick up and place fourth specimen
                        new ParallelAction(
                                arm1.liftRung2(0,1.2), //0.3
                                arm2.liftRung2(0,1.2),
                                tenthTrajectory,
                                claw_angle.forward(0)
                        ),
                        //go to fifth specimen
                        new ParallelAction(
                                eleventhTrajectory,
                                claw.openClawMore(),
                                arm1.liftFloor(0.2,1.1, false),
                                arm2.liftFloor(0.2,1.1, false),
                                claw_angle.backward(0),
                                claw.closeClaw(1.05)
                        ),
                        //pick up and place fifth specimen
                        new ParallelAction(
                                arm1.liftRung2(0,1.2),
                                arm2.liftRung2(0,1.2),
                                twelfthTrajectory,
                                claw_angle.forward(0)
                        ),
                        new ParallelAction(
                                thirteenthTrajectory,
                                claw.openClawMore(),
                                arm1.liftFloor(0.2,1.1, false),
                                arm2.liftFloor(0.2,1.1, false),
                                claw_angle.backward(0),
                                claw.closeClaw(1.05)
                        ),
                        new ParallelAction(
                                arm1.liftHighBasket(),
                                arm2.liftHighBasket(),
                                intake_angle.RotatePosition0_basket(1),
                                fourteenthTrajectory
                        ),
                        claw.openClaw(),
                        arm1.liftHighBasket(100)
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
