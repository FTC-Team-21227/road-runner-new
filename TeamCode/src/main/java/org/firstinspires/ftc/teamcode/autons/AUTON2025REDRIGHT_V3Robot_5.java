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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Disabled
@Autonomous(name = "RIGHT_5+1_Old")
//Trying for 5+1 auto just by pushing the samples faster
public class AUTON2025REDRIGHT_V3Robot_5 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10.5, -63.3, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ARM1_V3Robot arm1 = new ARM1_V3Robot(hardwareMap);
        ARM2_V3Robot arm2 = new ARM2_V3Robot(hardwareMap);
        CLAW_NEW claw = new CLAW_NEW(hardwareMap);
        INTAKE_ANGLE_NEW intake_angle = new INTAKE_ANGLE_NEW(hardwareMap);
        CLAW_ANGLE_NEW claw_angle = new CLAW_ANGLE_NEW(hardwareMap);
        SWEEPER sweeper = new SWEEPER(hardwareMap);
        double firstSpecDistance = -37.5;
        double otherSpecDistance = -35.5;
        double wallGrab = -46.5; //48.75;
        double wallGrabAngle = -85;
        double frictionConstant = 0;
//        pushing timing to the limits
        MecanumDrive.PARAMS.axialGain = 2.0;
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                .setTangent(Math.toRadians(120))
                .splineToConstantHeading(new Vector2d(-1.5,firstSpecDistance),Math.toRadians(90));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-1.5, firstSpecDistance, Math.toRadians(-90))) //push colored samples
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-1.5,firstSpecDistance-3),Math.toRadians(-45))
//                .splineToConstantHeading(new Vector2d(32.5+frictionConstant, firstSpecDistance-3),Math.toRadians(45))
//                .splineToConstantHeading(new Vector2d(35.5,-22),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(41+frictionConstant, -52,Math.toRadians(-30)),Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(41+frictionConstant, -22),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(48.5+frictionConstant, -16),Math.toRadians(0))
//                .strafeTo(new Vector2d(50, -48))
                .splineToConstantHeading(new Vector2d(51.5+frictionConstant,-22),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(51.5,-46),Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(51.5, -22),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(58.5+frictionConstant, -16),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(61.5+frictionConstant, -22),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(61.5, -46),Math.toRadians(-90));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(61.5, -46, Math.toRadians(-90))) //go to second specimen
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(61.5, -22),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(66+frictionConstant, -16),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(69+frictionConstant, -22),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(69, -35.5),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(69, -46),Math.toRadians(-90), new TranslationalVelConstraint(15));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(69, -44.5, Math.toRadians(-90))) //pick up and place second specimen
                .setTangent(Math.toRadians(170))
                .waitSeconds(0.4)
//                .splineToConstantHeading(new Vector2d(7.5,otherSpecDistance),Math.toRadians(90));
                .splineToLinearHeading(new Pose2d(5.5,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(5.5, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(40,-40),Math.toRadians(0))
//                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(10));
                .splineToConstantHeading(new Vector2d(5.5,otherSpecDistance-3),Math.toRadians(-45))
//                .splineToConstantHeading(new Vector2d(40,-48.25),Math.toRadians(-12));
                .splineToSplineHeading(new Pose2d(40,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-12));
//                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(30));
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(40, wallGrab,Math.toRadians(wallGrabAngle))) //pick up and place second specimen
                .setTangent(Math.toRadians(170))
                .waitSeconds(0.4)
//                .splineToConstantHeading(new Vector2d(8.5,otherSpecDistance),Math.toRadians(90));
                .splineToLinearHeading(new Pose2d(7.5,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(7.5, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(40,-40),Math.toRadians(0))
//                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(10));
//                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(30));
                .splineToConstantHeading(new Vector2d(7.5,otherSpecDistance-3),Math.toRadians(-45))
//                .splineToConstantHeading(new Vector2d(40,-48),Math.toRadians(-12));
                .splineToSplineHeading(new Pose2d(40,wallGrab+0.3,Math.toRadians(wallGrabAngle)),Math.toRadians(-12));
        TrajectoryActionBuilder tab10 = drive.actionBuilder(new Pose2d(40, wallGrab+0.3,Math.toRadians(wallGrabAngle))) //pick up and place second specimen
                .setTangent(Math.toRadians(170))
                .waitSeconds(0.4)
                .splineToLinearHeading(new Pose2d(11,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
        TrajectoryActionBuilder tab11 = drive.actionBuilder(new Pose2d(11, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(40,-40),Math.toRadians(0))
//                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(30));
                .splineToConstantHeading(new Vector2d(11,otherSpecDistance-3),Math.toRadians(-45))
//                .splineToConstantHeading(new Vector2d(40,-48),Math.toRadians(-12));
                .splineToSplineHeading(new Pose2d(40,wallGrab+0.6,Math.toRadians(wallGrabAngle)),Math.toRadians(-12));
        TrajectoryActionBuilder tab12 = drive.actionBuilder(new Pose2d(40, wallGrab+0.6,Math.toRadians(wallGrabAngle))) //pick up and place fourth specimen
                .setTangent(Math.toRadians(170))
                .waitSeconds(0.4)
                .splineToLinearHeading(new Pose2d(14.5,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
        TrajectoryActionBuilder tab13 = drive.actionBuilder(new Pose2d(14.5, otherSpecDistance, Math.toRadians(-90))) //park
                .setTangent(Math.toRadians(-90))
//                .strafeTo(new Vector2d(30,-58));
                .splineToConstantHeading(new Vector2d(14.5,otherSpecDistance-3),Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(40,wallGrab+0.9,Math.toRadians(wallGrabAngle)),Math.toRadians(-12));
        TrajectoryActionBuilder tab14 = drive.actionBuilder(new Pose2d(40, wallGrab+0.9,Math.toRadians(wallGrabAngle))) //park
//                .waitSeconds(0)
                .strafeToLinearHeading(new Vector2d(-44,-65),Math.toRadians(40),new TranslationalVelConstraint(120), new ProfileAccelConstraint(-40,100));

        Actions.runBlocking(
            new SequentialAction(
                claw.closeClaw(),
                intake_angle.RotatePosition1(),
                claw_angle.forward()
            )
        );

        Action firstTrajectory = tab1.build();
        //Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();
        //Action fourthTrajectory = tab4.build();
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
                                intake_angle.RotatePosition0(0),
                                arm1.liftRung2(0.15,1.3,false),
                                arm2.liftRung2/*First*/(0.15,1.3,false),
                                firstTrajectory
                        ),

                        //push colored samples
                        new ParallelAction(
                                claw.openClawMore(),
                                thirdTrajectory,
                                arm1.liftDown(1),
                                arm2.liftDown(1)
//                                sweeper.RotatePosition0(3),
//                                sweeper.RotatePosition1(3.5),
//                                sweeper.RotatePosition0(5.7)
                        ),
                        //face the wall and go to second specimen
                        new ParallelAction(
                            fifthTrajectory,
//                            sweeper.RotatePosition1(0),
                            arm1.liftWall2_First(1.0,1.7),
                            arm2.liftWall2(1.0,1.7),
                            claw_angle.backward(0.8),
                            intake_angle.RotatePositionNegative1(1.5),
                            sweeper.RotatePosition0(3)
                        ),
                        claw.closeClaw()
                )
        );
        MecanumDrive.PARAMS.lateralGain = 1.0;
        MecanumDrive.PARAMS.headingGain = 2.0;
        Actions.runBlocking(
                new SequentialAction(
                        //pick up and place second specimen
                        new ParallelAction(
                            sweeper.RotatePosition1(0.5),
                            arm1.liftRung2(0.3),
                            arm2.liftRung2(0.3),
                            sixthTrajectory,
                            claw_angle.forward(0.5),
                            intake_angle.RotatePosition0(0.4)
                        ),
                        //go to third specimen
                        new ParallelAction(
                            claw.openClawMore(),
                            seventhTrajectory,
                            arm1.liftWall2(0.4,1.5), //0.5
                            arm2.liftWall2(0.4,1.5),
                            claw_angle.backward(0),
                            intake_angle.RotatePositionNegative1(0.6)
                        ),
                        //pick up and place third specimen
                        claw.closeClaw(),
                        new ParallelAction(
                            arm1.liftRung2(0.3,1.7),
                            arm2.liftRung2(0.3,1.7),
                            eighthTrajectory,
                            claw_angle.forward(0.5),
                            intake_angle.RotatePosition0(0.4)
                        ),
                        //go to fourth specimen
                        new ParallelAction(
                            claw.openClawMore(),
                            ninthTrajectory,
                            arm1.liftWall2(0.4,1.5),
                            arm2.liftWall2(0.4,1.5),
                            claw_angle.backward(0),
                            intake_angle.RotatePositionNegative1(0.6)
                        ),
                        //pick up and place fourth specimen
                        claw.closeClaw(),
                        new ParallelAction(
                            arm1.liftRung2(0.3,1.7), //0.3
                            arm2.liftRung2(0.3,1.7),
                            tenthTrajectory,
                            claw_angle.forward(0.5),
                            intake_angle.RotatePosition0(0.4)
                        ),
                        //go to fifth specimen
                        new ParallelAction(
                            claw.openClawMore(),
                            eleventhTrajectory,
                            arm1.liftWall2(0.4,1.5),
                            arm2.liftWall2(0.4,1.5),
                            claw_angle.backward(0),
                            intake_angle.RotatePositionNegative1(0.6)
                        ),
                        //pick up and place fifth specimen
                        claw.closeClaw(),
                        new ParallelAction(
                            arm1.liftRung2(0.3,1.7),
                            arm2.liftRung2(0.3,1.7),
                            twelfthTrajectory,
                            claw_angle.forward(0.5),
                            intake_angle.RotatePosition0(0.4)
                        ),
                        new ParallelAction(
                            claw.openClawMore(),
                            thirteenthTrajectory,
                            arm1.liftWall2(0.4,1.5),
                            arm2.liftWall2(0.4,1.5),
                            claw_angle.backward(0),
                            intake_angle.RotatePositionNegative1(0.6)
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                            arm1.liftHighBasket(0.3),
                            arm2.liftHighBasket(0.3),
                            intake_angle.RotatePosition0_basket(1),
                            fourteenthTrajectory
                        ),
                        claw.openClaw(),
                        arm1.liftHighBasket(1000)
                )
        );
        Pose2d pose = drive.localizer.getPose();
        PoseStorage.currentPose =
                new Pose2d(
                        pose.position,
                        pose.heading.toDouble()-Math.toRadians(90)
                );    }
}
