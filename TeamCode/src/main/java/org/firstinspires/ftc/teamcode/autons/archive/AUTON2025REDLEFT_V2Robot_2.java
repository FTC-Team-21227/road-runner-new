//package org.firstinspires.ftc.teamcode.autons;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive_Left;
//
////@Autonomous(name = "AUTONLEFT_V2Robot_OLD")
//public class AUTON2025REDLEFT_V2Robot_2 extends LinearOpMode{
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d initialPose = new Pose2d(0, 92, Math.toRadians(0));
//        MecanumDrive_Left drive = new MecanumDrive_Left(hardwareMap, initialPose);
//        ARM1_NEW arm1 = new ARM1_NEW(hardwareMap);
//        ARM2_NEW arm2 = new ARM2_NEW(hardwareMap);
//        CLAW claw = new CLAW(hardwareMap);
//        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);
//        CLAW_ANGLE claw_angle = new CLAW_ANGLE(hardwareMap);
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .waitSeconds(1)
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(4.5, 114, Math.toRadians(135)), Math.toRadians(30));// loaded sample go to basket
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(4.5, 114, Math.toRadians(135)))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(10, 109.5), Math.toRadians(360)); //get 1st sample from the left side
////                .strafeTo(new Vector2d(10.5, 109.5)); //get 1st sample
//        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(10, 109.5, Math.toRadians(0)))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(4.5, 114), Math.toRadians(-225)); //go away from wall bec arms lifting
//        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(4.5, 114, Math.toRadians(135)))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(9.5, 119.5), Math.toRadians(360)); //get 2nd sample from the left side
////                .strafeTo(new Vector2d(10.5, 119.5)); //get 1st sample
//        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(9.5, 119.5, Math.toRadians(0)))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(4.5, 114), Math.toRadians(-225)); //go away from wall bec arms lifting
//        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(4.5,114, Math.toRadians(135)))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(10.5, 122.5), Math.toRadians(15)); //get 3rd sample from the left side
////                .strafeTo(new Vector2d(10.5, 122.5)); //get 1st sample
//        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(10.5,122.5, Math.toRadians(15)))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(4.5, 114), Math.toRadians(-225)); //go away from wall bec arms lifting
//        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(4.5,114, Math.toRadians(135)))
//                .waitSeconds(0.5)
//                .strafeToSplineHeading(new Vector2d(54, 96), Math.toRadians(270))//avoid bumping into submersible
//                .strafeTo(new Vector2d(54, 93)); //touch bar
//        Actions.runBlocking(
//                new SequentialAction(
//                        claw.closeClaw(),
//                        intake_angle.RotatePosition1(),
//                        claw_angle.forward()
//                )
//        );
//
//        waitForStart();
//
//        Action firstTrajectory = tab1.build();
////        Action secondTrajectory = tab2.build();
//        Action thirdTrajectory = tab3.build();
//        Action fourthTrajectory = tab4.build();
//        Action fifthTrajectory = tab5.build();
//        Action sixthTrajectory = tab6.build();
//        Action seventhTrajectory = tab7.build();
//        Action eighthTrajectory = tab8.build();
//        Action ninthTrajectory = tab9.build();
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                intake_angle.RotatePosition0(),
//                                claw.closeClaw(),
//                                arm1.liftHighBasket(),
//                                arm2.liftHighBasket(),
//                                firstTrajectory
//                        ),
//                        claw.openClaw(),
//                        thirdTrajectory,
//                        new ParallelAction(
////                                secondTrajectory,
//                                arm1.waitLiftFloor(0),
//                                arm2.waitLiftFloor(0)
//                        ),
//                        claw.closeClaw(),
//                        new ParallelAction(
//                            arm1.waitLiftHighBasket(0.5),
//                            arm2.waitLiftHighBasket(0.5)
//                        ),
//                        fourthTrajectory,
//                        claw.openClaw(),
//                        fifthTrajectory,
//                        new ParallelAction(
//                                arm1.waitLiftFloor(0),
//                                arm2.waitLiftFloor(0)
//                        ),
//                        claw.closeClaw(),
//                        new ParallelAction(
//                                arm1.waitLiftHighBasket(0.5),
//                                arm2.waitLiftHighBasket(0.5)
//                        ),
//                        sixthTrajectory,
//                        claw.openClaw(),
//                        seventhTrajectory,
//                        new ParallelAction(
//                                arm1.waitLiftFloor(0),
//                                arm2.waitLiftFloor(0)
//                        ),
//                        claw.closeClaw(),
//                        new ParallelAction(
//                                arm1.waitLiftHighBasket(0.5),
//                                arm2.waitLiftHighBasket(0.5)
//                        ),
//                        eighthTrajectory,
//                        claw.openClaw(),
//                        ninthTrajectory
//                )
//        );
//        PoseStorage.currentPose = drive.pose;
//    }
//}
