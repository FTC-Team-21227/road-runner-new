//package org.firstinspires.ftc.teamcode.autons.archive;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive_Left;
//import org.firstinspires.ftc.teamcode.autons.ARM1_V3Robot;
//import org.firstinspires.ftc.teamcode.autons.ARM2_V3Robot;
//import org.firstinspires.ftc.teamcode.autons.CLAW_ANGLE_NEW;
//import org.firstinspires.ftc.teamcode.autons.CLAW_NEW;
//import org.firstinspires.ftc.teamcode.autons.INTAKE_ANGLE_NEW;
//import org.firstinspires.ftc.teamcode.autons.PipeCamera;
//import org.firstinspires.ftc.teamcode.autons.PoseStorage;
//import org.firstinspires.ftc.teamcode.autons.SWEEPER;
//
////@Autonomous(name = "REDLEFT_0+6_CV")
////5 sample auto
//public class AUTON2025REDLEFT_V4Robot_2 extends LinearOpMode{
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d initialPose = new Pose2d(0, 92, Math.toRadians(0));
//        MecanumDrive_Left drive = new MecanumDrive_Left(hardwareMap, initialPose);
//        PipeCamera cam = new PipeCamera(hardwareMap,telemetry,false,"red",true);
//        ARM1_V3Robot arm1 = new ARM1_V3Robot(hardwareMap);
//        ARM2_V3Robot arm2 = new ARM2_V3Robot(hardwareMap);
//        CLAW_NEW claw = new CLAW_NEW(hardwareMap);
//        INTAKE_ANGLE_NEW intake_angle = new INTAKE_ANGLE_NEW(hardwareMap);
//        CLAW_ANGLE_NEW claw_angle = new CLAW_ANGLE_NEW(hardwareMap);
//        SWEEPER sweeper = new SWEEPER(hardwareMap);
//
//        double X = 0;
//        boolean cont = true;
//        boolean a = false;
//        boolean b= false;
//        boolean x= false;
//        boolean y= false;
//        boolean up= false;
//        boolean down= false;
//        boolean left= false;
//        boolean right= false;
//        boolean RB= false;
//        boolean LB= false;
//        boolean back= false;
//        boolean decimal = false;
//        boolean right_stick_button = false;
//        boolean start= false;
//        boolean LS = false;
//        while (cont && !isStopRequested()){
//            if (gamepad1.a && !a){
//                if (decimal){
//                    X += 0.1;
//                }
//                else {
//                    X = 10 * X + 1;
//                }
//            }
//            a = gamepad1.a;
//            if (gamepad1.b && !b){
//                if (decimal){
//                    X += 0.2;
//                }
//                else {X = 10*X+2;}
//            }
//            b = gamepad1.b;
//            if (gamepad1.x && !x){
//                if (decimal){
//                    X += 0.3;
//                }
//                else {X = 10*X+3;}
//            }
//            x = gamepad1.x;
//            if (gamepad1.y && !y){
//                if (decimal){
//                    X += 0.4;
//                }
//                else {X = 10*X+4;}
//            }
//            y = gamepad1.y;
//            if (gamepad1.dpad_up && !up){
//                if (decimal){
//                    X += 0.5;
//                }
//                else {X = 10*X+5;}
//            }
//            up = gamepad1.dpad_up;
//            if (gamepad1.dpad_down && !down){
//                if (decimal){
//                    X += 0.6;
//                }
//                else {X = 10*X+6;}
//            }
//            down = gamepad1.dpad_down;
//            if (gamepad1.dpad_left && !left){
//                if (decimal){
//                    X += 0.7;
//                }
//                else {X = 10*X+7;}
//            }
//            left = gamepad1.dpad_left;
//            if (gamepad1.dpad_right && !right){
//                if (decimal){
//                    X += 0.8;
//                }
//                else {X = 10*X+8;}
//            }
//            right = gamepad1.dpad_right;
//            if (gamepad1.right_bumper && !RB){
//                if (decimal){
//                    X += 0.9;
//                }
//                else {X = 10*X+9;}
//            }
//            RB = gamepad1.right_bumper;
//            if (gamepad1.left_bumper && !LB){
//                if (decimal){
//                    X += 0.0;
//                }
//                else {X = 10*X+0;}
//            }
//            LB = gamepad1.left_bumper;
//            if (gamepad1.left_stick_button && !LS){
//                X *= -1;
//            }
//            LS = gamepad1.left_stick_button;
//            if (gamepad1.back && !back){
//                decimal = true;
//            }
//            back = gamepad1.back;
//            if (gamepad1.right_stick_button && !right_stick_button){
//                X = 0;
//                decimal = false;
//            }
//            right_stick_button = gamepad1.right_stick_button;
//            if (gamepad1.start && !start){
//                cont = false;
//            }
//            start = gamepad1.start;
//            telemetry.addData("Pos X ",X);
//            if (decimal){
//                telemetry.addData("In decimal mode ", "only 1 decimal place permitted");
//            }
//            telemetry.addLine("a=1, b=2, x=3, y=4, up=5, down=6, left=7, right=8, RB=9, LB=0, back=decimal, Left Stick Button = negative,  start = continue, Right Stick Button = erase");
//            telemetry.update();
//        }
//        double Y = 0;
//        cont = true;
//        back = false;
//        while (cont && !isStopRequested()){
//            if (gamepad1.a && !a){
//                if (decimal){
//                    Y += 0.1;
//                }
//                else {
//                    Y = 10 * Y + 1;
//                }
//            }
//            a = gamepad1.a;
//            if (gamepad1.b && !b){
//                if (decimal){
//                    Y += 0.2;
//                }
//                else {Y = 10*Y+2;}
//            }
//            b = gamepad1.b;
//            if (gamepad1.x && !x){
//                if (decimal){
//                    Y += 0.3;
//                }
//                else {Y = 10*Y+3;}
//            }
//            x = gamepad1.x;
//            if (gamepad1.y && !y){
//                if (decimal){
//                    Y += 0.4;
//                }
//                else {Y = 10*Y+4;}
//            }
//            y = gamepad1.y;
//            if (gamepad1.dpad_up && !up){
//                if (decimal){
//                    Y += 0.5;
//                }
//                else {Y = 10*Y+5;}
//            }
//            up = gamepad1.dpad_up;
//            if (gamepad1.dpad_down && !down){
//                if (decimal){
//                    Y += 0.6;
//                }
//                else {Y = 10*Y+6;}
//            }
//            down = gamepad1.dpad_down;
//            if (gamepad1.dpad_left && !left){
//                if (decimal){
//                    Y += 0.7;
//                }
//                else {Y = 10*Y+7;}
//            }
//            left = gamepad1.dpad_left;
//            if (gamepad1.dpad_right && !right){
//                if (decimal){
//                    Y += 0.8;
//                }
//                else {Y = 10*Y+8;}
//            }
//            right = gamepad1.dpad_right;
//            if (gamepad1.right_bumper && !RB){
//                if (decimal){
//                    Y += 0.9;
//                }
//                else {Y = 10*Y+9;}
//            }
//            RB = gamepad1.right_bumper;
//            if (gamepad1.left_bumper && !LB){
//                if (decimal){
//                    Y += 0.0;
//                }
//                else {Y = 10*Y+0;}
//            }
//            LB = gamepad1.left_bumper;
//            if (gamepad1.back && !back){
//                decimal = true;
//            }
//            back = gamepad1.back;
//            if (gamepad1.left_stick_button && !LS){
//                Y *= -1;
//            }
//            LS = gamepad1.left_stick_button;
//            if (gamepad1.right_stick_button && !right_stick_button){
//                Y = 0;
//                decimal = false;
//            }
//            right_stick_button = gamepad1.right_stick_button;
//            if (gamepad1.start && !start){
//                cont = false;
//            }
//            start = gamepad1.start;
//            telemetry.addData("Pos Y ",Y);
//            if (decimal){
//                telemetry.addData("In decimal mode ", "only 1 decimal place permitted");
//            }
//            telemetry.addLine("a=1, b=2, x=3, y=4, up=5, down=6, left=7, right=8, RB=9, LB=0, back=decimal, Left Stick Button = negative, start = continue, Right Stick Button = erase");
//            telemetry.update();
//        }
//
//        telemetry.addData("Pos X ",X);
//        telemetry.addData("Pos Y ",Y);
//        telemetry.addLine("If incorrect, stop and reinit");
//        telemetry.update();
//
//
//        double X1 = 0;
//        cont = true;
//        back = false;
//        while (cont && !isStopRequested()){
//            if (gamepad1.a && !a){
//                if (decimal){
//                    X1 += 0.1;
//                }
//                else {
//                    X1 = 10 * X1 + 1;
//                }
//            }
//            a = gamepad1.a;
//            if (gamepad1.b && !b){
//                if (decimal){
//                    X1 += 0.2;
//                }
//                else {X1 = 10*X1+2;}
//            }
//            b = gamepad1.b;
//            if (gamepad1.x && !x){
//                if (decimal){
//                    X1 += 0.3;
//                }
//                else {X1 = 10*X1+3;}
//            }
//            x = gamepad1.x;
//            if (gamepad1.y && !y){
//                if (decimal){
//                    X1 += 0.4;
//                }
//                else {X1 = 10*X1+4;}
//            }
//            y = gamepad1.y;
//            if (gamepad1.dpad_up && !up){
//                if (decimal){
//                    X1 += 0.5;
//                }
//                else {X1 = 10*X1+5;}
//            }
//            up = gamepad1.dpad_up;
//            if (gamepad1.dpad_down && !down){
//                if (decimal){
//                    X1 += 0.6;
//                }
//                else {X1 = 10*X1+6;}
//            }
//            down = gamepad1.dpad_down;
//            if (gamepad1.dpad_left && !left){
//                if (decimal){
//                    X1 += 0.7;
//                }
//                else {X1 = 10*X1+7;}
//            }
//            left = gamepad1.dpad_left;
//            if (gamepad1.dpad_right && !right){
//                if (decimal){
//                    X1 += 0.8;
//                }
//                else {X1 = 10*X1+8;}
//            }
//            right = gamepad1.dpad_right;
//            if (gamepad1.right_bumper && !RB){
//                if (decimal){
//                    X1 += 0.9;
//                }
//                else {X1 = 10*X1+9;}
//            }
//            RB = gamepad1.right_bumper;
//            if (gamepad1.left_bumper && !LB){
//                if (decimal){
//                    X1 += 0.0;
//                }
//                else {X1 = 10*X1+0;}
//            }
//            LB = gamepad1.left_bumper;
//            if (gamepad1.left_stick_button && !LS){
//                X1 *= -1;
//            }
//            LS = gamepad1.left_stick_button;
//            if (gamepad1.back && !back){
//                decimal = true;
//            }
//            back = gamepad1.back;
//            if (gamepad1.right_stick_button && !right_stick_button){
//                X1 = 0;
//                decimal = false;
//            }
//            right_stick_button = gamepad1.right_stick_button;
//            if (gamepad1.start && !start){
//                cont = false;
//            }
//            start = gamepad1.start;
//            telemetry.addData("Pos X1 ",X1);
//            if (decimal){
//                telemetry.addData("In decimal mode ", "only 1 decimal place permitted");
//            }
//            telemetry.addLine("a=1, b=2, x=3, y=4, up=5, down=6, left=7, right=8, RB=9, LB=0, back=decimal, Left Stick Button = negative,  start = continue, Right Stick Button = erase");
//            telemetry.update();
//        }
//        double Y1 = 0;
//        cont = true;
//        back = false;
//        while (cont && !isStopRequested()){
//            if (gamepad1.a && !a){
//                if (decimal){
//                    Y1 += 0.1;
//                }
//                else {
//                    Y1 = 10 * Y1 + 1;
//                }
//            }
//            a = gamepad1.a;
//            if (gamepad1.b && !b){
//                if (decimal){
//                    Y1 += 0.2;
//                }
//                else {Y1 = 10*Y1+2;}
//            }
//            b = gamepad1.b;
//            if (gamepad1.x && !x){
//                if (decimal){
//                    Y1 += 0.3;
//                }
//                else {Y1 = 10*Y1+3;}
//            }
//            x = gamepad1.x;
//            if (gamepad1.y && !y){
//                if (decimal){
//                    Y1 += 0.4;
//                }
//                else {Y1 = 10*Y1+4;}
//            }
//            y = gamepad1.y;
//            if (gamepad1.dpad_up && !up){
//                if (decimal){
//                    Y1 += 0.5;
//                }
//                else {Y1 = 10*Y1+5;}
//            }
//            up = gamepad1.dpad_up;
//            if (gamepad1.dpad_down && !down){
//                if (decimal){
//                    Y1 += 0.6;
//                }
//                else {Y1 = 10*Y1+6;}
//            }
//            down = gamepad1.dpad_down;
//            if (gamepad1.dpad_left && !left){
//                if (decimal){
//                    Y1 += 0.7;
//                }
//                else {Y1 = 10*Y1+7;}
//            }
//            left = gamepad1.dpad_left;
//            if (gamepad1.dpad_right && !right){
//                if (decimal){
//                    Y1 += 0.8;
//                }
//                else {Y1 = 10*Y1+8;}
//            }
//            right = gamepad1.dpad_right;
//            if (gamepad1.right_bumper && !RB){
//                if (decimal){
//                    Y1 += 0.9;
//                }
//                else {Y1 = 10*Y1+9;}
//            }
//            RB = gamepad1.right_bumper;
//            if (gamepad1.left_bumper && !LB){
//                if (decimal){
//                    Y1 += 0.0;
//                }
//                else {Y1 = 10*Y1+0;}
//            }
//            LB = gamepad1.left_bumper;
//            if (gamepad1.back && !back){
//                decimal = true;
//            }
//            back = gamepad1.back;
//            if (gamepad1.left_stick_button && !LS){
//                Y1 *= -1;
//            }
//            LS = gamepad1.left_stick_button;
//            if (gamepad1.right_stick_button && !right_stick_button){
//                Y1 = 0;
//                decimal = false;
//            }
//            right_stick_button = gamepad1.right_stick_button;
//            if (gamepad1.start && !start){
//                cont = false;
//            }
//            start = gamepad1.start;
//            telemetry.addData("Pos Y1 ",Y1);
//            if (decimal){
//                telemetry.addData("In decimal mode ", "only 1 decimal place permitted");
//            }
//            telemetry.addLine("a=1, b=2, x=3, y=4, up=5, down=6, left=7, right=8, RB=9, LB=0, back=decimal, Left Stick Button = negative, start = continue, Right Stick Button = erase");
//            telemetry.update();
//        }
//
//        telemetry.addData("Pos X1 ",X1);
//        telemetry.addData("Pos Y1 ",Y1);
//        telemetry.addLine("If incorrect, stop and reinit");
//        telemetry.update();
//
//        PoseStorage.grabYellowPose = new Pose2d(64.5+Y, 90-X,Math.toRadians(-90));
//
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
////                .strafeTo(new Vector2d(10, 92))
////                .waitSeconds(1)
////                .setTangent(0)
//                .strafeToLinearHeading(new Vector2d(8, 112.5), Math.toRadians(-45));// loaded sample go to basket
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(8, 112.5, Math.toRadians(-45)))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(8.4, 107), Math.toRadians(0)); //get 1st sample from the left side
////                .strafeTo(new Vector2d(10.5, 109.5)); //get 1st sample
//        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(8.4, 107, Math.toRadians(0)))
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(8, 112.5), Math.toRadians(-45)) //go away from wall bec arms lifting
//                .waitSeconds(0.7);
//        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(8, 112.5, Math.toRadians(-45)))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(8.6, 119), Math.toRadians(1.5)); //get 2nd sample from the left side
////                .strafeTo(new Vector2d(10.5, 119.5)); //get 1st sample
//        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(8.6, 119, Math.toRadians(0)))
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(8, 112.5), Math.toRadians(-45)) //go away from wall bec arms lifting
//                .waitSeconds(0.7);
//        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(8, 112.5, Math.toRadians(-45)))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(10.5,118),Math.toRadians(10))
//                .waitSeconds(0.3)
//                .turnTo(Math.toRadians(28)); //get 3rd sample from the left side
////                .strafeTo(new Vector2d(10.5, 122.5)); //get 1st sample
//        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(10.5,118, drive.localizer.getPose().heading.toDouble()))
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(7, 113.5), Math.toRadians(-45)) //go away from wall bec arms lifting
//                .waitSeconds(0.7);
//        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(7, 113.5, Math.toRadians(-45)))
//                .waitSeconds(0.5)
//                .strafeToSplineHeading(new Vector2d(64.5+Y,105),Math.toRadians(-90), new TranslationalVelConstraint(70))
////                .waitSeconds(1.5) //get 4th sample from the sub
//                .strafeTo(new Vector2d(64.5+Y, 90-X/*+2*/), new TranslationalVelConstraint(30)); //get 4th sample
////                .strafeTo(new Vector2d(64.5+Y,90-X),new TranslationalVelConstraint(30));
//        TrajectoryActionBuilder tab10 = drive.actionBuilder(new Pose2d(64.5+Y,90-X, Math.toRadians(-90)))
//                .setTangent(Math.toRadians(180))
//                .waitSeconds(0.3) //change0.5
//                .splineToConstantHeading(new Vector2d(64.5,100),Math.toRadians(180), new TranslationalVelConstraint(70))
//                .splineToSplineHeading(new Pose2d(6, 108, Math.toRadians(-45)), Math.toRadians(165),new TranslationalVelConstraint(70)) //go away from wall bec arms lifting
//                .waitSeconds(0.7);//change0.7
//        TrajectoryActionBuilder tab11 = drive.actionBuilder(new Pose2d(6, 108, Math.toRadians(-45)))
//                .waitSeconds(0.5)
//                .strafeToSplineHeading(new Vector2d(64.5+Y,105),Math.toRadians(-90), new TranslationalVelConstraint(70))
////                .waitSeconds(1.5) //get 4th sample from the sub
//                .strafeTo(new Vector2d(64.5+Y, 90-X/*+2*/), new TranslationalVelConstraint(30)); //get 4th sample
////                .strafeTo(new Vector2d(64.5+Y,90-X),new TranslationalVelConstraint(30));
//        TrajectoryActionBuilder tab12 = drive.actionBuilder(new Pose2d(64.5+Y,90-X, Math.toRadians(-90)))
//                .setTangent(Math.toRadians(180))
//                .waitSeconds(0.3) //change0.5
//                .splineToConstantHeading(new Vector2d(64.5,100),Math.toRadians(180), new TranslationalVelConstraint(70))
//                .splineToSplineHeading(new Pose2d(6, 108, Math.toRadians(-45)), Math.toRadians(165),new TranslationalVelConstraint(70)) //go away from wall bec arms lifting
//                .waitSeconds(0.7);//change0.7
//        TrajectoryActionBuilder tab13 = drive.actionBuilder(new Pose2d(6,108,Math.toRadians(-45)))
//                .setTangent(Math.toRadians(0))
//                .waitSeconds(0.5)
//                .splineToSplineHeading(new Pose2d(52.5, 105, Math.toRadians(-90)),Math.toRadians(0),new TranslationalVelConstraint(70))//avoid bumping into submersible
//                .splineToConstantHeading(new Vector2d(52.5, 87),Math.toRadians(0),new TranslationalVelConstraint(15)); //touch bar
////                .splineToSplineHeading(new Pose2d(52.5, 105, Math.toRadians(-90)),Math.toRadians(-45),new TranslationalVelConstraint(70))//avoid bumping into submersible
////                .splineToConstantHeading(new Vector2d(64.5, 87),Math.toRadians(-45),new TranslationalVelConstraint(15)); //touch bar
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
//        Action ninthTrajectory = tab9.build();
//        Action tenthTrajectory = tab10.build();
//        Action eleventhTrajectory = tab11.build();
//        Action twelfthTrajectory = tab12.build();
//        Action thirteenthTrajectory = tab13.build();
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
//                    claw_angle.backward(0),
//                    arm1.liftHighBasket(0,1.6),
//                    arm2.liftHighBasket(0,1.6),
//                    firstTrajectory
//                ),
//                claw.openClaw(),
//                new ParallelAction(
////                                secondTrajectory,
//                    thirdTrajectory,
//                    intake_angle.RotatePosition0_left(0.5),
//                    claw_angle.forward(0.2),
//                    arm1.liftFloor(0.3,1.6),
//                    arm2.liftFloor(0.3,1.6)
//                ),
//                claw.closeClaw(),
//                new ParallelAction(
//                    fourthTrajectory,
//                    intake_angle.RotatePosition0_basket(0.5),
//                    claw_angle.backward(0.2),
//                    arm1.liftHighBasket(0.3,1.6),
//                    arm2.liftHighBasket(0.3,1.6)
//                ),
//                claw.openClaw(),
//                new ParallelAction(
//                    fifthTrajectory,
//                    intake_angle.RotatePosition0_left(0.5),
//                    claw_angle.forward(0.2),
//                    arm1.liftFloor(0.3,1.6),
//                    arm2.liftFloor(0.3,1.6)
//                ),
//                claw.closeClaw(),
//                new ParallelAction(
//                    sixthTrajectory,
//                    intake_angle.RotatePosition0_basket(0.5),
//                    claw_angle.backward(0.2),
//                    arm1.liftHighBasket(0.3,1.6),
//                    arm2.liftHighBasket(0.3,1.6)
//                ),
//                claw.openClaw(),
//                new ParallelAction(
//                    seventhTrajectory,
//                    intake_angle.RotatePosition0_left(0.5),
//                    claw_angle.forward(0.2),
//                    arm1.liftWall(0.3,1.6),
//                    arm2.liftWall(0.3,1.6),
//                    arm1.liftFloor(2.8,0.4,0.25),
//                    arm2.liftFloor(2.8,0.4)
//                ),
//                claw.closeClaw(),
//                new ParallelAction(
//                    eighthTrajectory,
//                    intake_angle.RotatePosition0_basket(0.5),
//                    claw_angle.backward(0.2),
//                    arm1.liftHighBasket(0.3,1.6),
//                    arm2.liftHighBasket(0.3,1.6)
//                ),
//                claw.openClaw(),
//                new ParallelAction(
//                    ninthTrajectory,
//                    claw_angle.forward(0.5),
//                    claw.openClaw(0.5),
//                    intake_angle.RotatePosition2(1),
//                    arm1.liftVertSub_Auto(0.3),
//                    arm2.liftVertSub(0.3)
////                    arm2.waitLiftVertFloor(4,0.7)
//                ),
//                cam.comp(0)
//            )
//        );
//        Action toYellow1 = drive.actionBuilder(new Pose2d(64.5+Y, 90-X/*+2*/, Math.toRadians(-90))) //push colored samples
//                .strafeTo(PoseStorage.grabYellowPose.position)
//                .build();
//        PoseStorage.grabYellowPose = new Pose2d(64.5+Y1, 90-X1,Math.toRadians(-90));
//        Actions.runBlocking(
//            new SequentialAction(
//                toYellow1,
//                new ParallelAction(
//                    arm1.liftVertFloor(0,0.7, 0.2),
//                    claw.closeClaw(0.6)
//                ),
//                new ParallelAction(
//                    tenthTrajectory,
//                    intake_angle.RotatePosition0_basket(0.3),
//                    arm1.liftVertSub(0.3),
////                    arm2.waitLiftWall(0.2),
//                    claw_angle.backward(1.5),
//                    arm1.liftHighBasket(2,1.8),
//                    arm2.liftHighBasket(2,1.8)
//                ),
//                    claw.openClaw(),
//                    new ParallelAction(
//                            eleventhTrajectory,
//                            claw_angle.forward(0.5),
//                            claw.openClaw(0.5),
//                            intake_angle.RotatePosition2(2),
//                            arm1.liftVertSub_Auto(0.3),
//                            arm2.liftVertSub(0.3)
//                    ),
//                    cam.comp(0)
//            )
//        );
//        Action toYellow2 = drive.actionBuilder(new Pose2d(64.5+Y, 90-X/*+2*/, Math.toRadians(-90))) //push colored samples
//                .strafeTo(PoseStorage.grabYellowPose.position)
//                .build();
//        Actions.runBlocking(
//            new SequentialAction(
//                toYellow2,
//                new ParallelAction(
//                    arm1.liftVertFloor(0,0.7, 0.2),
//                    claw.closeClaw(0.6)
//                ),
//                    new ParallelAction(
//                            cam.close(0),
//                            twelfthTrajectory,
//                            intake_angle.RotatePosition0_basket(0.3),
//                            arm1.liftVertSub(0.3),
////                    arm2.waitLiftWall(0.2),
//                            claw_angle.backward(1.5),
//                            arm1.liftHighBasket(2,1.8),
//                            arm2.liftHighBasket(2,1.8)
//                    ),
//                claw.openClaw(),
//                new ParallelAction(
//                    intake_angle.RotatePosition0_left(0.8),
//                    claw_angle.forward(0.8),
//                    thirteenthTrajectory,
//                    arm1.waitLiftWallWall(0.3)
////                    arm2.waitLift(1.2)
//                )
//            )
//        );
//        PoseStorage.currentPose = drive.localizer.getPose();
//    }
//}
