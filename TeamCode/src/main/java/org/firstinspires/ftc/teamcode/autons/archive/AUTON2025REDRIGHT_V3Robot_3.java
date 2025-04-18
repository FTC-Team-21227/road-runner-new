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
//import org.firstinspires.ftc.teamcode.PinpointDrive;
//
////@Autonomous(name = "AUTONRIGHT_V3_5spec_FastTiming")
//// 5+1 using fast timing
//public class AUTON2025REDRIGHT_V3Robot_3 extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d initialPose = new Pose2d(10.5, -63.3, Math.toRadians(-90));
//        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
//
//        ARM1_V2Robot arm1 = new ARM1_V2Robot(hardwareMap);
//        ARM2_V2Robot arm2 = new ARM2_V2Robot(hardwareMap);
//        CLAW_NEW claw = new CLAW_NEW(hardwareMap);
//        INTAKE_ANGLE_NEW intake_angle = new INTAKE_ANGLE_NEW(hardwareMap);
//        CLAW_ANGLE_NEW claw_angle = new CLAW_ANGLE_NEW(hardwareMap);
//        SWEEPER sweeper = new SWEEPER(hardwareMap);
//        double firstSpecDistance = -36.5;
//        double otherSpecDistance = -34;
////        pushing timing to the limits
//
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(9,firstSpecDistance),Math.toRadians(90));
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(9, firstSpecDistance, Math.toRadians(-90))) //push colored samples
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(9,-42),Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(39, -47),Math.toRadians(0))
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(39,-25),Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(50, -19),Math.toRadians(0))
//                .strafeTo(new Vector2d(50, -48))
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(50, -25),Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(60, -19),Math.toRadians(0))
//                .strafeTo(new Vector2d(60, -47.25));
//                //improved, we want 1 s shape
////                .splineToConstantHeading(new Vector2d(9,-45),Math.toRadians(-90))
////                .splineToConstantHeading(new Vector2d(46.5, -27),Math.toRadians(90))
////                .splineToConstantHeading(new Vector2d(53, -15),Math.toRadians(0)) //make u's of this
////                .setTangent(Math.toRadians(-90))
////                .splineToConstantHeading(new Vector2d(53, -45),Math.toRadians(-90)) //y value may need to be changed, push 1st
//////                .setTangent(Math.toRadians(90))
////                .splineToConstantHeading(new Vector2d(53, -25),Math.toRadians(60))
////                .splineToConstantHeading(new Vector2d(66, -12),Math.toRadians(0))
////                .setTangent(Math.toRadians(-90))
////                .splineToConstantHeading(new Vector2d(65, -47),Math.toRadians(-90)) //push 2nd
////                .setTangent(Math.toRadians(90))
////                .splineToConstantHeading(new Vector2d(66, -25),Math.toRadians(60))
////                .splineToConstantHeading(new Vector2d(69, -12),Math.toRadians(0));
//        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(60, -47.25, Math.toRadians(-90))) //go to second specimen
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(60, -25),Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(67, -19),Math.toRadians(0))
//                .strafeTo(new Vector2d(67,-44));
//        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(67, -44, Math.toRadians(-90))) //pick up and place second specimen
//                .setTangent(Math.toRadians(170))
//                .waitSeconds(0.4)
//                .splineToConstantHeading(new Vector2d(6.5,otherSpecDistance),Math.toRadians(90));
//        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(6.5, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
//                .setTangent(Math.toRadians(-90))
////                .splineToConstantHeading(new Vector2d(40,-40),Math.toRadians(0))
////                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(10));
//                .splineToConstantHeading(new Vector2d(6.5,otherSpecDistance-3),Math.toRadians(-45))
//                .splineToConstantHeading(new Vector2d(40,-48.25),Math.toRadians(-12));
////                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(30));
//        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(40, -48.25, Math.toRadians(-90))) //pick up and place second specimen
//                .setTangent(Math.toRadians(170))
//                .waitSeconds(0.4)
//                .splineToConstantHeading(new Vector2d(4,otherSpecDistance),Math.toRadians(90));
//        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(4, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
//                .setTangent(Math.toRadians(-90))
////                .splineToConstantHeading(new Vector2d(40,-40),Math.toRadians(0))
////                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(10));
////                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(30));
//                .splineToConstantHeading(new Vector2d(4,otherSpecDistance-3),Math.toRadians(-45))
//                .splineToConstantHeading(new Vector2d(40,-48.25),Math.toRadians(-12));
//        TrajectoryActionBuilder tab10 = drive.actionBuilder(new Pose2d(40, -48.25, Math.toRadians(-90))) //pick up and place second specimen
//                .setTangent(Math.toRadians(170))
//                .waitSeconds(0.4)
//                .splineToConstantHeading(new Vector2d(1.5,otherSpecDistance),Math.toRadians(90));
//        TrajectoryActionBuilder tab11 = drive.actionBuilder(new Pose2d(1.5, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
//                .setTangent(Math.toRadians(-90))
////                .splineToConstantHeading(new Vector2d(40,-40),Math.toRadians(0))
////                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(30));
//                .splineToConstantHeading(new Vector2d(1.5,otherSpecDistance-3),Math.toRadians(-45))
//                .splineToConstantHeading(new Vector2d(40,-48.25),Math.toRadians(-12));
//        TrajectoryActionBuilder tab12 = drive.actionBuilder(new Pose2d(40, -48.25, Math.toRadians(-90))) //pick up and place fourth specimen
//                .setTangent(Math.toRadians(170))
//                .waitSeconds(0.4)
//                .splineToConstantHeading(new Vector2d(11,otherSpecDistance),Math.toRadians(90));
//        TrajectoryActionBuilder tab13 = drive.actionBuilder(new Pose2d(11, otherSpecDistance, Math.toRadians(-90))) //park
//                .setTangent(Math.toRadians(-90))
////                .strafeTo(new Vector2d(30,-58));
//                .splineToConstantHeading(new Vector2d(11,otherSpecDistance-3),Math.toRadians(-45))
//                .splineToSplineHeading(new Pose2d(36,-50,Math.toRadians(-60)),Math.toRadians(-45));
//
//        Actions.runBlocking(
//            new SequentialAction(
//                claw.closeClaw(),
//                intake_angle.RotatePosition1(),
//                claw_angle.forward()
//            )
//        );
//
//        Action firstTrajectory = tab1.build();
//        //Action secondTrajectory = tab2.build();
//        Action thirdTrajectory = tab3.build();
//        //Action fourthTrajectory = tab4.build();
//        Action fifthTrajectory = tab5.build();
//        Action sixthTrajectory = tab6.build();
//        Action seventhTrajectory = tab7.build();
//        Action eighthTrajectory = tab8.build();
//        Action ninthTrajectory = tab9.build();
//        Action tenthTrajectory = tab10.build();
//        Action eleventhTrajectory = tab11.build();
//        Action twelfthTrajectory = tab12.build();
//        Action thirteenthTrajectory = tab13.build();
//
//        waitForStart();
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        //first specimen
//                        new ParallelAction(
//                                claw.closeClaw(),
//                                intake_angle.RotatePosition0(0),
//                                arm1.waitLiftRung2(0.15,1.3),
//                                arm2.waitLiftRung2(0.15,1.3),
//                                firstTrajectory
//                        ),
//                        claw.openClaw(),
//                        //push colored samples
//                        new ParallelAction(
//                                thirdTrajectory,
//                                arm1.waitLiftDown(),
//                                arm2.waitLiftDown()
//                        ),
//                        //face the wall and go to second specimen
//                        new ParallelAction(
//                            fifthTrajectory,
//                            arm1.waitLiftWall2(1.5,1.5),
//                            arm2.waitLiftWall2(1.5,1.5),
//                            claw_angle.backward(1),
//                            intake_angle.RotatePositionNegative1(1.5),
//                            sweeper.RotatePosition0(2.3)
//                        ),
//                        claw.closeClaw(),
//                        //pick up and place second specimen
//                        new ParallelAction(
//                            sweeper.RotatePosition1(0.5),
//                            arm1.waitLiftRung2(0.3),
//                            arm2.waitLiftRung2(0.3),
//                            sixthTrajectory,
//                            claw_angle.forward(1),
//                            intake_angle.RotatePosition0(0.5)
//                        ),
//                        //go to third specimen
//                        claw.openClaw(),
//                        new ParallelAction(
//                            seventhTrajectory,
//                            arm1.waitLiftWall2(0.5,1.5),
//                            arm2.waitLiftWall2(0.5,1.5),
//                            claw_angle.backward(0.5),
//                            intake_angle.RotatePositionNegative1(1)
//                        ),
//                        //pick up and place third specimen
//                        claw.closeClaw(),
//                        new ParallelAction(
//                            arm1.waitLiftRung2(0.3),
//                            arm2.waitLiftRung2(0.3),
//                            eighthTrajectory,
//                            claw_angle.forward(1),
//                            intake_angle.RotatePosition0(0.5)
//                        ),
//                        //go to fourth specimen
//                        claw.openClaw(),
//                        new ParallelAction(
//                            ninthTrajectory,
//                            arm1.waitLiftWall2(0.5,1.5),
//                            arm2.waitLiftWall2(0.5,1.5),
//                            claw_angle.backward(0.5),
//                            intake_angle.RotatePositionNegative1(1)
//                        ),
//                        //pick up and place fourth specimen
//                        claw.closeClaw(),
//                        new ParallelAction(
//                            arm1.waitLiftRung2(0.3),
//                            arm2.waitLiftRung2(0.3),
//                            tenthTrajectory,
//                            claw_angle.forward(1),
//                            intake_angle.RotatePosition0(0.5)
//                        ),
//                        //go to fifth specimen
//                        claw.openClaw(),
//                        new ParallelAction(
//                            eleventhTrajectory,
//                            arm1.waitLiftWall2(0.5,1.5),
//                            arm2.waitLiftWall2(0.5,1.5),
//                            claw_angle.backward(0.5),
//                            intake_angle.RotatePositionNegative1(1)
//                        ),
//                        //pick up and place fifth specimen
//                        claw.closeClaw(),
//                        new ParallelAction(
//                            arm1.waitLiftRung2(0.3),
//                            arm2.waitLiftRung2(0.3),
//                            twelfthTrajectory,
//                            claw_angle.forward(1),
//                            intake_angle.RotatePosition0(0.5)
//                        ),
//                        claw.openClaw(),
//                        new ParallelAction(
//                            arm1.waitLiftFloor(0.5),
//                            arm2.waitLiftFloor(0.5),
//                            thirteenthTrajectory
//                        )
//                )
//        );
//    }
//}
