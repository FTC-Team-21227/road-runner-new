//package org.firstinspires.ftc.teamcode.autons;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.PinpointDrive;
//import org.firstinspires.ftc.teamcode.PinpointDrive_Left;
//
////@Autonomous(name = "AUTONLEFT_V3Robot_4sample")
////5 sample auto
//public class AUTON2025REDLEFT_V3Robot extends LinearOpMode{
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d initialPose = new Pose2d(0, 92, Math.toRadians(0));
//        PinpointDrive_Left drive = new PinpointDrive_Left(hardwareMap, initialPose);
//        ARM1_V2Robot arm1 = new ARM1_V2Robot(hardwareMap);
//        ARM2_V2Robot arm2 = new ARM2_V2Robot(hardwareMap);
//        CLAW_NEW claw = new CLAW_NEW(hardwareMap);
//        INTAKE_ANGLE_NEW intake_angle = new INTAKE_ANGLE_NEW(hardwareMap);
//        CLAW_ANGLE_NEW claw_angle = new CLAW_ANGLE_NEW(hardwareMap);
//        SWEEPER sweeper = new SWEEPER(hardwareMap);
//
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .strafeTo(new Vector2d(10, 92))
////                .waitSeconds(1)
////                .setTangent(0)
//                .strafeToLinearHeading(new Vector2d(9, 112), Math.toRadians(-45));// loaded sample go to basket
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(9, 112, Math.toRadians(-45)))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(9.2, 107), Math.toRadians(0)); //get 1st sample from the left side
////                .strafeTo(new Vector2d(10.5, 109.5)); //get 1st sample
//        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(9.2, 107, Math.toRadians(0)))
//                .waitSeconds(1.5)
//                .strafeToLinearHeading(new Vector2d(9, 112), Math.toRadians(-45)) //go away from wall bec arms lifting
//                .waitSeconds(0.7);
//        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(9, 112, Math.toRadians(-45)))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(9.8, 118.25), Math.toRadians(0)); //get 2nd sample from the left side
////                .strafeTo(new Vector2d(10.5, 119.5)); //get 1st sample
//        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(9.8, 118.25, Math.toRadians(0)))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(8, 112.5), Math.toRadians(-45)) //go away from wall bec arms lifting
//                .waitSeconds(0.7);
//        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(8, 112.5, Math.toRadians(-45)))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(10,118),Math.toRadians(10))
//                .waitSeconds(1.5)
//                .turnTo(Math.toRadians(28)); //get 3rd sample from the left side
////                .strafeTo(new Vector2d(10.5, 122.5)); //get 1st sample
//        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(10,118, drive.pose.heading.toDouble()))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(8, 112.5), Math.toRadians(-45)) //go away from wall bec arms lifting
//                .waitSeconds(0.7);
//        TrajectoryActionBuilder tab11 = drive.actionBuilder(new Pose2d(8.5, 112, Math.toRadians(-45)))
//                .waitSeconds(0.5)
//                .strafeToSplineHeading(new Vector2d(52, 94), Math.toRadians(-90))//avoid bumping into submersible
//                .strafeTo(new Vector2d(52.5, 89),new TranslationalVelConstraint(15)); //touch bar
//
//
//
//        Actions.runBlocking(
//            new ParallelAction(
//                claw.closeClaw(),
//                intake_angle.RotatePosition1(),
//                claw_angle.forward()
//            )
//        );
//
//
//        Action firstTrajectory = tab1.build();
////        Action secondTrajectory = tab2.build();
//        Action thirdTrajectory = tab3.build();
//        Action fourthTrajectory = tab4.build();
//        Action fifthTrajectory = tab5.build();
//        Action sixthTrajectory = tab6.build();
//        Action seventhTrajectory = tab7.build();
//        Action eighthTrajectory = tab8.build();
//        Action eleventhTrajectory = tab11.build();
//
//        waitForStart();
//
//
//
//        Actions.runBlocking(
//            new SequentialAction(
//                new ParallelAction(
//                    intake_angle.RotatePosition0_basket(),
//                    claw.closeClaw(),
//                    claw_angle.backward(0.5),
//                    arm1.liftHighBasket(),
//                    arm2.liftHighBasket(),
//                    firstTrajectory
//                ),
//                claw.openClaw(),
//                new ParallelAction(
////                                secondTrajectory,
//                    thirdTrajectory,
//                    intake_angle.RotatePosition0_left(1),
//                    claw_angle.forward(1),
//                    arm1.waitLiftFloor(1.5),
//                    arm2.waitLiftFloor(1.5)
//                ),
//                claw.closeClaw(),
//                new ParallelAction(
//                    fourthTrajectory,
//                    intake_angle.RotatePosition0_basket(1.5),
//                    claw_angle.backward(1),
//                    arm1.waitLiftHighBasket(0.5),
//                    arm2.waitLiftHighBasket(0.5)
//                ),
//                claw.openClaw(),
//                new ParallelAction(
//                    fifthTrajectory,
//                    intake_angle.RotatePosition0_left(1),
//                    claw_angle.forward(1),
//                    arm1.waitLiftFloor(1.5),
//                    arm2.waitLiftFloor(1.5)
//                ),
//                claw.closeClaw(),
//                new ParallelAction(
//                    sixthTrajectory,
//                    intake_angle.RotatePosition0_basket(1.5),
//                    claw_angle.backward(1),
//                    arm1.waitLiftHighBasket(1),
//                    arm2.waitLiftHighBasket(1)
//                ),
//                claw.openClaw(),
//                new ParallelAction(
//                    seventhTrajectory,
//                    intake_angle.RotatePosition0_left(1),
//                    claw_angle.forward(1),
//                    arm1.waitLiftWall(1.5),
//                    arm2.waitLiftWall(1.5),
//                    arm1.waitLiftFloor(4.5,1, 0.25),
//                    arm2.waitLiftFloor(4.5,1)
//                ),
//                claw.closeClaw(),
//                new ParallelAction(
//                    eighthTrajectory,
//                    intake_angle.RotatePosition0_basket(1.5),
//                    claw_angle.backward(1.5),
//                    arm1.waitLiftHighBasket(1),
//                    arm2.waitLiftHighBasket(1)
//                ),
//                claw.openClaw(),
//                new ParallelAction(
//                    intake_angle.RotatePosition0_left(1),
//                    claw_angle.forward(1),
//                    eleventhTrajectory,
//                    arm1.waitLiftRung(1.5),
//                    arm2.waitLiftRung(1.5)
//                )
//            )
//        );
//        PoseStorage.currentPose = drive.pose;
//    }
//}
