//package org.firstinspires.ftc.teamcode.autons.archive;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.autons.ARM1_V3Robot;
//import org.firstinspires.ftc.teamcode.autons.ARM2_V3Robot;
//import org.firstinspires.ftc.teamcode.autons.CLAW_ANGLE_NEW;
//import org.firstinspires.ftc.teamcode.autons.CLAW_NEW;
//import org.firstinspires.ftc.teamcode.autons.INTAKE_ANGLE_NEW;
//import org.firstinspires.ftc.teamcode.autons.PoseStorage;
//import org.firstinspires.ftc.teamcode.autons.SWEEPER;
//
////@Autonomous(name = "AUTONRIGHT_V3_5+1_OldPush")
////Trying for 5+1 auto just by pushing the samples faster
//public class AUTON2025REDRIGHT_V3Robot_10 extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d initialPose = new Pose2d(10.5, -63.3, Math.toRadians(90));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//
//        ARM1_V3Robot arm1 = new ARM1_V3Robot(hardwareMap);
//        ARM2_V3Robot arm2 = new ARM2_V3Robot(hardwareMap);
//        CLAW_NEW claw = new CLAW_NEW(hardwareMap);
//        INTAKE_ANGLE_NEW intake_angle = new INTAKE_ANGLE_NEW(hardwareMap);
//        CLAW_ANGLE_NEW claw_angle = new CLAW_ANGLE_NEW(hardwareMap);
//        SWEEPER sweeper = new SWEEPER(hardwareMap);
//        double firstSpecDistance = -48;
//        double otherSpecDistance = -37;
//        double wallGrab = -46;
//        double wallGrab1 = 21;
//        double wallGrabAngle = -45;
//        double frictionConstant = 0;
////        pushing timing to the limits
//
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
//                .setTangent(Math.toRadians(150))
//                .splineToConstantHeading(new Vector2d(-5.5,firstSpecDistance),Math.toRadians(90));
//        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-5.5, firstSpecDistance, Math.toRadians(90))) //push colored samples
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-3.5,firstSpecDistance-3),Math.toRadians(-45))
//                .splineToConstantHeading(new Vector2d(32.5+frictionConstant, firstSpecDistance-3),Math.toRadians(45))
//                .splineToConstantHeading(new Vector2d(35.5,-12),Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(44+frictionConstant, -8),Math.toRadians(0))
////                .strafeTo(new Vector2d(50, -48))
//                .splineToConstantHeading(new Vector2d(49+frictionConstant,-22),Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(49,-46),Math.toRadians(-90))
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(49, -22),Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(57+frictionConstant, -16),Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(60+frictionConstant, -22),Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(60, -46),Math.toRadians(-90))
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(60, -22),Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(67+frictionConstant, -16),Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(67+frictionConstant, -22),Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(67, -55.5),Math.toRadians(-90));
//        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(60, -55.5, Math.toRadians(90))) //pick up and place second specimen
//                .setTangent(Math.toRadians(135))
//                .waitSeconds(0.4)
//                .splineToConstantHeading(new Vector2d(-5.5,firstSpecDistance),Math.toRadians(90));
//        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(-5.5, firstSpecDistance, Math.toRadians(90))) //go to third specimen
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-45));
//        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(wallGrab1, wallGrab,Math.toRadians(wallGrabAngle))) //pick up and place second specimen
//                .setTangent(Math.toRadians(135))
//                .waitSeconds(0.4)
//                .splineToLinearHeading(new Pose2d(3.5,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
//        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(3.5, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-45));
//        TrajectoryActionBuilder tab10 = drive.actionBuilder(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle))) //pick up and place second specimen
//                .setTangent(Math.toRadians(135))
//                .waitSeconds(0.4)
//                .splineToLinearHeading(new Pose2d(7,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
//        TrajectoryActionBuilder tab11 = drive.actionBuilder(new Pose2d(7, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-45));
//        TrajectoryActionBuilder tab12 = drive.actionBuilder(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle))) //pick up and place fourth specimen
//                .setTangent(Math.toRadians(135))
//                .waitSeconds(0.4)
//                .splineToLinearHeading(new Pose2d(10,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
//        TrajectoryActionBuilder tab13 = drive.actionBuilder(new Pose2d(10, otherSpecDistance, Math.toRadians(-90))) //park
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-45));
//        TrajectoryActionBuilder tab14 = drive.actionBuilder(new Pose2d(wallGrab1,wallGrab,Math.toRadians(wallGrabAngle))) //park
//                .waitSeconds(0.3)
//                .strafeToLinearHeading(new Vector2d(-49,-55),Math.toRadians(45),new TranslationalVelConstraint(120), new ProfileAccelConstraint(-40,100));
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        claw.closeClaw(),
//                        intake_angle.RotatePosition1(),
//                        claw_angle.forward()
//                )
//        );
//
//        Action firstTrajectory = tab1.build();
//        Action secondTrajectory = tab2.build();
//        Action sixthTrajectory = tab6.build();
//        Action seventhTrajectory = tab7.build();
//        Action eighthTrajectory = tab8.build();
//        Action ninthTrajectory = tab9.build();
//        Action tenthTrajectory = tab10.build();
//        Action eleventhTrajectory = tab11.build();
//        Action twelfthTrajectory = tab12.build();
//        Action thirteenthTrajectory = tab13.build();
//        Action fourteenthTrajectory = tab14.build();
//
//        waitForStart();
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        //first specimen
//                        new ParallelAction(
//                                claw.closeClaw(),
//                                intake_angle.RotatePosition3(),
//                                arm1.liftRung_First(0,1.3),
//                                arm2.liftRung2_First(0,1.3),
//                                firstTrajectory
//                        ),
//                        new ParallelAction(
//                                claw.openClawMore(),
//                                secondTrajectory,
//                                arm1.liftWall_First(1,1.4),
//                                arm2.liftWall2_First(1,1.4),
//                                claw_angle.forward(0.4),
//                                intake_angle.RotatePositionNegative2(1)
//                        ),
//                        claw.closeClaw(),
//                        //pick up and place second specimen
//                        new ParallelAction(
//                                arm1.liftRung_FirstSecond(0.2),
//                                arm2.liftRung2_First(0.2),
//                                sixthTrajectory,
//                                claw_angle.backward(0.2),
//                                intake_angle.RotatePosition4(0.4)
//                        ),
//                        //go to third specimen
//                        new ParallelAction(
//                                claw.openClawMore(),
//                                seventhTrajectory,
//                                arm1.liftFloor(0.2,1.3), //0.5
//                                arm2.liftFloor(0.2,1.3),
//                                claw_angle.backward(0),
//                                intake_angle.RotatePosition0(0.5),
//                                claw.closeClaw(1.5)
//                        ),
//                        //pick up and place third specimen
////                        claw.closeClaw(),
//                        new ParallelAction(
//                                arm1.liftRung2(0.2,1.3),
//                                arm2.liftRung2(0.2,1.3),
//                                eighthTrajectory,
//                                claw_angle.forward(0.2)
//                        ),
//                        //go to fourth specimen
//                        new ParallelAction(
//                                claw.openClawMore(),
//                                ninthTrajectory,
//                                arm1.liftFloor(0.2,1.3),
//                                arm2.liftFloor(0.2,1.3),
//                                claw_angle.backward(0)
//                        ),
//                        //pick up and place fourth specimen
//                        claw.closeClaw(),
//                        new ParallelAction(
//                                arm1.liftRung2(0.2,1.3), //0.3
//                                arm2.liftRung2(0.2,1.3),
//                                tenthTrajectory,
//                                claw_angle.forward(0.2)
//                        ),
//                        //go to fifth specimen
//                        new ParallelAction(
//                                claw.openClawMore(),
//                                eleventhTrajectory,
//                                arm1.liftFloor(0.2,1.3),
//                                arm2.liftFloor(0.2,1.3),
//                                claw_angle.backward(0),
//                                claw.closeClaw(1.5)
//                        ),
//                        //pick up and place fifth specimen
////                        claw.closeClaw(),
//                        new ParallelAction(
//                                arm1.liftRung2(0.2,1.3),
//                                arm2.liftRung2(0.2,1.3),
//                                twelfthTrajectory,
//                                claw_angle.forward(0.2)
//                        ),
//                        new ParallelAction(
//                                claw.openClawMore(),
//                                thirteenthTrajectory,
//                                arm1.liftFloor(0.2,1.3),
//                                arm2.liftFloor(0.2,1.3),
//                                claw_angle.backward(0)
//                        ),
//                        claw.closeClaw(),
//                        new ParallelAction(
//                                arm1.liftHighBasket(0.3),
//                                arm2.liftHighBasket(0.3),
//                                intake_angle.RotatePosition0_basket(1),
//                                fourteenthTrajectory
//                        ),
//                        claw.openClaw(),
//                        arm1.liftHighBasket(100)
//                )
//        );
//        PoseStorage.currentPose = new Pose2d(
//                drive.localizer.getPose().position.y + 72.3,                  // New x becomes original y (rotation)
//                -drive.localizer.getPose().position.x + 75.3,                 // New y becomes negative original x (rotation)
//                drive.localizer.getPose().heading.plus(-Math.PI/2).toDouble()     // Subtract 90 degrees (π/2 radians) from heading
//        );
//        PoseStorage.arm1 = arm1;
//        PoseStorage.arm2 = arm2;
//    }
//}
