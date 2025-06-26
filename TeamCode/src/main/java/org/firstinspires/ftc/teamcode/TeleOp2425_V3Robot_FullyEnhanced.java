package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Twist2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autons.ARM1_V3Robot;
import org.firstinspires.ftc.teamcode.autons.ARM2_V3Robot;
import org.firstinspires.ftc.teamcode.autons.CLAW_ANGLE_NEW;
import org.firstinspires.ftc.teamcode.autons.CLAW_NEW;
import org.firstinspires.ftc.teamcode.autons.INTAKE_ANGLE_NEW;
import org.firstinspires.ftc.teamcode.autons.PoseStorage;
import org.firstinspires.ftc.teamcode.vision.ExcludePipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOp2425_V3Robot_FullyEnhanced")
public class TeleOp2425_V3Robot_FullyEnhanced extends LinearOpMode {
    //PID controllers for ARM1 and ARM2
    private PIDController controller1;
    private PIDController controller2;
    //PIDF gains
    double p1 = TunePID_MotionProfile.p1, i1 = TunePID_MotionProfile.i1, d1 = TunePID_MotionProfile.d1;
    double f1 = TunePID_MotionProfile.f1;
    double p2 = TunePID_MotionProfile.p2, i2 = TunePID_MotionProfile.i2, d2 = TunePID_MotionProfile.d2;
    double f2 = TunePID_MotionProfile.f2;
    //ARM1, ARM2 target positions, in degrees
    double target1 = 0;
    double target2 = 0;
    double Target1;
    double Target2;
    //ticks to degrees conversion, very useful
    private final double ticks_in_degree_1 = TunePID_MotionProfile.ticks_in_degree_1;
    private final double ticks_in_degree_2 = TunePID_MotionProfile.ticks_in_degree_2;
    private final double L1 = TunePID_MotionProfile.L1;
    private final double L2 = TunePID_MotionProfile.L2;
    private final double x1 = TunePID_MotionProfile.x1;
    private final double x2 = TunePID_MotionProfile.x2;
    private final double m1 = TunePID_MotionProfile.m1;
    private final double m2 = TunePID_MotionProfile.m2;
    private final double highBasket1 = Subsystem_Constants.highBasket1;
    private final double highRung1 = Subsystem_Constants.highRung1;
    private final double highRung1_2 = Subsystem_Constants.highRung1_2;
    private final double highRung1_First = Subsystem_Constants.highRung1_First_teleop;
    private final double wall1 = Subsystem_Constants.wall1;
    private final double wall1_2 = Subsystem_Constants.wall1_2_teleop;
    private final double wall1_First = Subsystem_Constants.wall1_First;
    private final double lowBasket1 = Subsystem_Constants.lowBasket1;
    private final double floor1 = Subsystem_Constants.floor1_teleop;
    private final double down1 = Subsystem_Constants.down1;
    private final double sub1 = Subsystem_Constants.sub1;
    private final double vertSub1 = Subsystem_Constants.vertSub1;
    private final double vertFloor1 = Subsystem_Constants.vertFloor1;

    private final double highBasket2 = Subsystem_Constants.highBasket2_teleop;
    final double highRung2 = Subsystem_Constants.highRung2;
    final double highRung2_2 = Subsystem_Constants.highRung2_2_teleop;
    private final double wall2 = Subsystem_Constants.wall2;
    private final double wall2_2 = Subsystem_Constants.wall2_2;
    private final double wall2_First = Subsystem_Constants.wall2_First_teleop;
    private final double lowBasket2 = Subsystem_Constants.lowBasket2;
    private final double floor2 = Subsystem_Constants.floor2_teleop;
    private final double down2 = Subsystem_Constants.down2;
    private final double sub2 = Subsystem_Constants.sub2;
    private final double vertSub2 = Subsystem_Constants.vertSub2;
    private final double vertFloor2 = Subsystem_Constants.vertFloor2;
    private final double highRung2_First = Subsystem_Constants.highRung2_First;

    final double clawScale0 = Subsystem_Constants.clawScale0;
    final double clawScale1 = Subsystem_Constants.clawScale1;
    final double closeClaw = Subsystem_Constants.closeClaw;
    final double openClaw = Subsystem_Constants.openClaw;

    final double intake_AngleScale0 = Subsystem_Constants.intake_AngleScale0;
    final double intake_AngleScale1 = Subsystem_Constants.intake_AngleScale1;
    final double intake_AngleFloor = Subsystem_Constants.intake_AngleFloor_teleop;
    final double intake_AngleBasket = Subsystem_Constants.intake_AngleBasket_teleop;
    final double intake_AngleRung = Subsystem_Constants.intake_AngleRung;
    final double intake_AngleRung_Second = Subsystem_Constants.intake_AngleRung_Second;
    final double intake_AngleStart = Subsystem_Constants.intake_AngleStart;
    final double intake_AngleWall = Subsystem_Constants.intake_AngleWall;
    final double intake_AngleWall_First = Subsystem_Constants.intake_AngleWall_First;
    final double intake_AngleVertical = Subsystem_Constants.intake_AngleVertical;

    final double claw_AngleScale0 = Subsystem_Constants.claw_AngleScale0;
    final double claw_AngleScale1 = Subsystem_Constants.claw_AngleScale1;
    final double claw_AngleForward = Subsystem_Constants.claw_AngleForward;
    final double claw_AngleBackward = Subsystem_Constants.claw_AngleBackward;
    final double claw_AngleLeft = Subsystem_Constants.claw_AngleLeft;

    final double sweeperScale0 = Subsystem_Constants.sweeperScale0;
    final double sweeperScale1 = Subsystem_Constants.sweeperScale1;
    final double closeSweeper = Subsystem_Constants.closeSweeper;
    final double openSweeper = Subsystem_Constants.openSweeper;
//    private MecanumDrive_Lock drive;
    private GoBildaPinpointDriver pinpoint;
    private DcMotor W_BL;
    private DcMotor W_BR;
    private DcMotor W_FR;
    private DcMotor W_FL;
    private DcMotor ARM1; //bottom arm
    private DcMotor ARM2; //top arm
    private CRServo Hook;
    private Servo Intake_Angle;
    private Servo Claw;
    private Servo Claw_Angle;
    private Servo Sweeper;
    private TouchSensor ARM1Sensor;
    private TouchSensor ARM2Sensor;
    double Heading_Angle;
    double Motor_power_BR;
    double imu_rotation;
    double Motor_power_BL;
    double Targeting_Angle;
    double Motor_fwd_power;
    double Motor_power_FL;
    double Motor_side_power;
    double Motor_power_FR;
    double Motor_Rotation_power;
    double Motor_Power;
    double Motor_Trans_Power;
    double Motor_Rot_Power;
    double tim;
    boolean ARM1calibrated = true;
    boolean ARM2calibrated = true;
    boolean hanging = false;
    int arm1Pos;
    int arm2Pos;
    String state = "a";
    String eState = "a";
    String mode = "sample";
    double stateTime;
    boolean manual = false;
    boolean eManual = false;
    boolean rightTriggerPressed = false;
    boolean leftTriggerPressed = false;
    boolean leftTrigger2Pressed = false;
    boolean rightBumperPressed = false;
    boolean leftStickPressed = false;
    boolean rightTrigger2Pressed = false;
    boolean hookUp = false;
    boolean hookDown = false;
    double claw = openClaw;
    double claw_angle = claw_AngleForward;
    double intake_angle = intake_AngleFloor;
    double sweeper = openSweeper;
    boolean servoReset = false;
    double servoResetTime;
    double Lift_Power = 1;
    double currTime = 0;
    double initialHeading;
    boolean back = true;
    boolean back2 = true;
    boolean startedHolding = false;
    Pose2d target;
    private List<Action> runningActions = new ArrayList<>();
    double ARM1_OFFSET = TunePID_MotionProfile.ARM1_OFFSET;
    double ARM2_OFFSET = TunePID_MotionProfile.ARM2_OFFSET;
    // ARM1 Motion Profile Variables
//    private double arm1StartPos;
//    private double arm1TargetPos;
//    private double arm1DDec;
//    private boolean arm1Decelerating = false;
//
//    // ARM2 Motion Profile Variables
//    private double arm2StartPos;
//    private double arm2TargetPos;
//    private double arm2DDec;
//    private boolean arm2Decelerating = false;

    // Profile Parameters (tune these!)
//    private final double V_MAX = TunePID_MotionProfile.V_MAX; // Encoder ticks/sec (≈ 100° /sec if 1 tick/degree)
//    private final double A_DEC = TunePID_MotionProfile.A_DEC; // Ticks/sec² (adjust for smooth stopping)
//    private final double LOOP_TIME = TunePID_MotionProfile.LOOP_TIME; // 20ms (typical FTC loop time)
    Pose2d pose;
    double x;
    double y;
//    boolean inSub;
    boolean lockedMode = false;
//    private final double A_LIN_DEC = 2;
//    private final double A_ANG_DEC = 2;
//    ARM1_V3Robot arm1;
//    ARM2_V3Robot arm2;
//    CLAW_NEW cLaw;
//    INTAKE_ANGLE_NEW iNtake_angle;
//    CLAW_ANGLE_NEW cLaw_angle;
    double target1pid;
    double target2pid;
//    private final Pose2d pickupPose = new Pose2d(23.556, 52.028,Math.toRadians(225));
//    private final Pose2d scorePose = new Pose2d(34.300, 63.909, Math.toRadians(180));
    // Add these class variables at the top with your other declarations
    private int rbPressCount = 0;
    private double lastRbPressTime = 0;
    private static final double RB_PRESS_TIMEOUT = 1; // 1 second timeout between presses
    boolean overrideSpeed = false;
    boolean printStuff = ExcludePipeline.printStuff;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException{
        //getting all the motors, servos, and sensors from the hardware map
        Pose2d initialPose;
        try {
            initialPose = PoseStorage.currentPose;
//            telemetry.addData("Yay1!","Yay1!");
        }
        catch (Exception e){
            initialPose = new Pose2d(0,0,Math.toRadians(0));
//            telemetry.addData("No!","No!");
        }
        initialHeading = Math.toDegrees(initialPose.heading.toDouble());
//        drive = new MecanumDrive_Lock(hardwareMap,initialPose);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        W_BL = hardwareMap.get(DcMotor.class, "W_BL");
        W_BR = hardwareMap.get(DcMotor.class, "W_BR");
        W_FR = hardwareMap.get(DcMotor.class, "W_FR");
        W_FL = hardwareMap.get(DcMotor.class, "W_FL");
        ARM1 = hardwareMap.get(DcMotor.class, "ARM1");
        ARM2 = hardwareMap.get(DcMotor.class, "ARM2");
        Hook = hardwareMap.get(CRServo.class, "Hook");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Intake_Angle = hardwareMap.get(Servo.class,"Intake_Angle");
        Claw_Angle = hardwareMap.get(Servo.class,"Claw_Angle");
        Sweeper = hardwareMap.get(Servo.class,"Sweeper");
        ARM1Sensor = hardwareMap.get(TouchSensor.class, "ARM1Sensor");
        ARM2Sensor = hardwareMap.get(TouchSensor.class, "ARM2Sensor");
//        arm1 = PoseStorage.arm1;
//        arm2 = PoseStorage.arm2;
//        cLaw = new CLAW_NEW(hardwareMap);
//        iNtake_angle = new INTAKE_ANGLE_NEW(hardwareMap);
//        cLaw_angle = new CLAW_ANGLE_NEW(hardwareMap);

        // Put initialization blocks here.
        Initialization();
        if (opModeIsActive()) {
            Intake_Angle.setPosition(intake_AngleFloor);
            intake_angle = intake_AngleFloor;
            servoResetTime = getRuntime();
            servoReset = true;

            // Put run blocks here.
            while (opModeIsActive()) {
                currTime = getRuntime();
                pinpoint.update();
//                PoseVelocity2d currentVel = drive.updatePoseEstimate();
//                pose = drive.localizer.getPose();
//                x = pose.position.x;
//                y = pose.position.y;

//                inSub = inSub();
                arm1Pos = ARM1.getCurrentPosition();
                arm2Pos = ARM2.getCurrentPosition();
                // Put loop blocks here.
//                if (gamepad2.b){
//                    holdRobotPosition(currentVel);
//                }
//                else if (mode.equals("specimenCycle") && gamepad1.left_trigger > 0.1){
//                    cycleSpecimen();
//                }
//                else {
//                    startedHolding = false;
                    Calculate_IMU_Rotation_Power(); //calculates each motor power based on IMU reading
                    Calculate_Motor_Power(); //calculates translational and rotational motor power
                    //set power to each wheel motor
                    W_BL.setPower(Motor_power_BL);
                    W_BR.setPower(Motor_power_BR);
                    W_FR.setPower(Motor_power_FR);
                    W_FL.setPower(Motor_power_FL);
//                }
//                if (mode.equals("specimenCycle") && !startedHolding && gamepad1.left_trigger > 0.1){
//                    runningActions.clear();
//                }
//                List<Action> newActions = new ArrayList<>();
//                for (Action action : runningActions) {
//                    if (action.run(p)) {
//                        newActions.add(action);
//                    }
//                }
//                runningActions = newActions;
                //controls the arm motor powers
                if (!(ARM1calibrated && ARM2calibrated)) {
                    ARM_Calibration(); //calibration function
                }
                else {
                    ARM_SetTargets(); //gamepad presets and other things to set the target positions for each arm motor
                    ARM_PID_Control(); //PID control function based on target positions
                }
                //automatic servo control based on presets, test if can be overridden
                if (!manual) {
                    Control_Servo_States();
                }
                //controls the intake servos manually
                Intake_Control();
                //controls the hook servo
                Hook_Control();
                if (!eManual && !overrideSpeed) {
                    Control_Enhanced_States();
                }
                //reset imu if necessary
                if (gamepad2.back && !back2) {
                    pinpoint.resetPosAndIMU();
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    initialHeading = 0;
                    Targeting_Angle = 0;
                    Heading_Angle = 0;
                }
                back2 = gamepad2.back;
                if (gamepad1.back && !back) { //change to sub pos for easy align
                    initialHeading = 0;
                    Targeting_Angle = 0;
                    Heading_Angle = 0;
//                    if (mode.equals("specimenCycle") || mode.equals("specimenCollect")) drive = new MecanumDrive_Lock(hardwareMap,new Pose2d(8,-2,Math.toRadians(90)));
//                    else if (mode.equals("sample")) {
//                        drive = new MecanumDrive_Lock(hardwareMap,new Pose2d(10,-16,Math.toRadians(0)));
////                        initialHeading = -90;
//                        Targeting_Angle = -90;
//                        Heading_Angle = -90;
//                    }
//                    else
                    pinpoint.resetPosAndIMU();
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
                back = gamepad1.back;
                telemetry.addData("mode", mode);
                telemetry.addData("eState", eState);
                telemetry.addData("state", state);
                telemetry.addData("eManual", eManual);
                telemetry.addData("manual", manual);
                telemetry.addData("Rotation Power", Motor_Rotation_power);
                telemetry.addData("Motor_Translation Power", Motor_Trans_Power);
                if (printStuff) {
//                    telemetry.addData("inSub", inSub);
                    telemetry.addData("Heading", Heading_Angle);
                    telemetry.addData("Initial Heading", initialHeading);
                    telemetry.addData("Targeting Angle", Targeting_Angle);
                    telemetry.addData("Motor Power", Motor_Power);
                    telemetry.addData("Side Power", Motor_side_power);
                    telemetry.addData("FWD Power", Motor_fwd_power);
                    telemetry.addData("IMU_Rotation Power", imu_rotation);
//                    telemetry.addData("Rotation Power", Motor_Rotation_power);
//                    telemetry.addData("Motor_Translation Power", Motor_Trans_Power);
                    telemetry.addData("Drive x", x);
                    telemetry.addData("Drive y", y);
                    telemetry.addData("ARM1Pos: ", arm1Pos / ticks_in_degree_1);
                    telemetry.addData("ARM1Target: ", target1);
                    telemetry.addData("ARM1TargetPID: ", target1pid);
                    telemetry.addData("ARM2 Current Angle: ", arm2Pos / ticks_in_degree_2);
                    telemetry.addData("ARM2 Target Angle: ", target2);
                    telemetry.addData("ARM2 TargetPID: ", target2pid);
                    telemetry.addData("Claw", Claw.getPosition());
                    telemetry.addData("Intake angle", Intake_Angle.getPosition());
                    telemetry.addData("Claw angle", Claw_Angle.getPosition());
                    telemetry.addData("ARM1 Power", ARM1.getPower());
                    telemetry.addData("ARM2 Power", ARM2.getPower());
                    telemetry.addData("State time", stateTime);
                    telemetry.addData("Run time", currTime);
                    telemetry.addData("Loop time", getRuntime() - currTime);
                    telemetry.addData("ARM1Calibrated", ARM1calibrated);
                    telemetry.addData("ARM2Calibrated", ARM2calibrated);
                    telemetry.addData("ARM1 Sensor", ARM1Sensor.isPressed());
                    telemetry.addData("ARM2 Sensor", ARM2Sensor.isPressed());
                    telemetry.update();

//                    TelemetryPacket packet = new TelemetryPacket();
//                    packet.fieldOverlay().setStroke("#3F51B5");
//                    Drawing.drawRobot(packet.fieldOverlay(), pose);
//                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                }

            }
        }
    }
//    private void holdRobotPosition(PoseVelocity2d currentVel) {
//        if (!startedHolding) {
//            // Calculate stopping distance using current velocity
//            double stoppingDistance = currentVel.linearVel.sqrNorm() / (2 * A_LIN_DEC);
//            double stoppingAngDistance = Math.pow(currentVel.angVel,2) / (2 * A_ANG_DEC);
//
//            target = pose.plus(
//                    new Twist2d(currentVel.linearVel.times(stoppingDistance), Math.signum(currentVel.angVel) * stoppingAngDistance)
//            );
//
//            Action a = drive.actionBuilder(pose, () -> gamepad2.b)
//                    .strafeToLinearHeading(target.position, target.heading.toDouble())
//                    .build();
//            runningActions.add(a);
//            startedHolding = true;
//        }
//    }
//    private void cycleSpecimen() {
//        //use a different drive for this, if we even want to use roadrunner actions for this
//        if (!startedHolding) {
//            // Calculate stopping distance using current velocity
//
//            double firstSpecDistance = -48;
//            double otherSpecDistance = -37;
//            double wallGrab = -46;
//            double wallGrab1 = 21;
//            double wallGrabAngle = -45;
//            double frictionConstant = 0;
//            Action score = drive.actionBuilder2(pickupPose) //pick up and place fourth specimen
//                    .setTangent(Math.toRadians(45))
//                    .waitSeconds(0.6)
//                    .splineToLinearHeading(scorePose,Math.toRadians(0))
//                    .build();
//            Action grab = drive.actionBuilder2(scorePose) //park
//                    .setTangent(Math.toRadians(180))
//                    .splineToLinearHeading(pickupPose,Math.toRadians(225))
//                    .build();
//            for (int i = 0; i < 10; i ++) {
//                runningActions.add(
//                        new SequentialAction(
//                                cLaw.closeClaw(),
//                                new ParallelAction(
//                                        arm1.liftRung2(0.2, 1.7),
//                                        arm2.liftRung2(0.2, 1.7),
//                                        score,
//                                        cLaw_angle.forward(0.5)
//                                ),
//                                new ParallelAction(
//                                        cLaw.openClawMore(),
//                                        grab,
//                                        arm1.liftFloor(0.2, 1.5),
//                                        arm2.liftFloor(0.2, 1.5),
//                                        cLaw_angle.backward(0)
//                                )
//                        )
//                );
//            }
//            startedHolding = true;
//        }
//    }
    private void Control_Servo_States() {
        if (state.equals("highBasket")) { //Intake Angle 0, swivel Claw Angle to face the basket
            if (printStuff) telemetry.addData("In high basket", "yes");
            if (getRuntime() - stateTime > 0.25) {
                if (intake_angle != intake_AngleBasket) {
                    Intake_Angle.setPosition(intake_AngleBasket);
                    intake_angle = intake_AngleBasket;
                }
                if (claw_angle != claw_AngleBackward) {
                    Claw_Angle.setPosition(claw_AngleBackward);
                    claw_angle = claw_AngleBackward;
                    if (printStuff) telemetry.addData("fd", "whyu not rotating");
                }
            }
        } else if (state.equals("highRung")) { //Intake Angle 0, Claw Angle 0
            if (getRuntime() - stateTime > 0.25) {
                if (intake_angle != intake_AngleRung) {
                    Intake_Angle.setPosition(intake_AngleRung);
                    intake_angle = intake_AngleRung;
                }
                if (claw_angle != claw_AngleForward) {
                    Claw_Angle.setPosition(claw_AngleForward);
                    claw_angle = claw_AngleForward;
                }
                if (printStuff) telemetry.addData("In high rung", "yes");
            }
        } else if (state.equals("highRung_Up")) { //Intake Angle 0, Claw Angle 0
            if (getRuntime() - stateTime > 0.25) {
                if (intake_angle != intake_AngleRung) {
                    Intake_Angle.setPosition(intake_AngleRung_Second);
                    intake_angle = intake_AngleRung_Second;
                }
                if (claw_angle != claw_AngleBackward) {
                    Claw_Angle.setPosition(claw_AngleBackward);
                    claw_angle = claw_AngleBackward;
                }
                if (printStuff) telemetry.addData("In high rung", "yes");
            }
        } else if (state.equals("wall")) {
            if (claw_angle != claw_AngleBackward) {
                Claw_Angle.setPosition(claw_AngleBackward);
                claw_angle = claw_AngleBackward;
            }
            if (getRuntime() - stateTime > 0.5) {
                if (intake_angle != intake_AngleWall) {
                    Intake_Angle.setPosition(intake_AngleWall);
                    intake_angle = intake_AngleWall;
                }
                if (printStuff) telemetry.addData("In wall", "yes");
            }
        } else if (state.equals("enterSub")) {
            if (getRuntime() - stateTime > 0.25) {
                if (intake_angle != intake_AngleFloor) {
                    Intake_Angle.setPosition(intake_AngleFloor);
                    intake_angle = intake_AngleFloor;
                }
                if (claw_angle != claw_AngleForward) {
                    Claw_Angle.setPosition(claw_AngleForward);
                    claw_angle = claw_AngleForward;
                }
                if (printStuff) telemetry.addData("In enter sub", "yes");
            }
        }
        else if (state.equals("enterSubVert")) {
            if (getRuntime() - stateTime > 0.25) {
                if (claw_angle != claw_AngleForward) {
                    Claw_Angle.setPosition(claw_AngleForward);
                    claw_angle = claw_AngleForward;
                }
                if (printStuff) telemetry.addData("In enter sub vert", "yes");
            }
        }
        else if (state.equals("droppedHighBasket")) {
            if (getRuntime() - stateTime > 0.2) {
                if (intake_angle != intake_AngleBasket-0.2) {
                    Intake_Angle.setPosition(intake_AngleBasket-0.2);
                    intake_angle = intake_AngleBasket-0.2;
                }
                if (printStuff) telemetry.addData("In dropped high basket", "yes");
            }
        }
        else if (state.equals("drop")) {
            if (getRuntime() - stateTime > 0.2) {
                if (claw_angle != claw_AngleForward) {
                    Claw_Angle.setPosition(claw_AngleForward);
                    claw_angle = claw_AngleForward;
                }
            }
            if (getRuntime() - stateTime > 1.5) {
                if (intake_angle != intake_AngleWall_First) {
                    Intake_Angle.setPosition(intake_AngleWall_First);
                    intake_angle = intake_AngleWall_First;
                }
                if (printStuff) telemetry.addData("drop", "yes");
            }
        }
        else if (state.equals("reversedFloor")){
            if (getRuntime() - stateTime > 0) {
                if (intake_angle != intake_AngleFloor) {
                    Intake_Angle.setPosition(intake_AngleFloor);
                    intake_angle = intake_AngleFloor;
                }
                if (claw_angle != claw_AngleBackward) {
                    Claw_Angle.setPosition(claw_AngleBackward);
                    claw_angle = claw_AngleBackward;
                }
            }
            if (printStuff) telemetry.addData("In reversed floor", "yes");
        }
    }
    private void Intake_Control(){
        if (gamepad1.right_trigger > 0.1 && !rightTriggerPressed) {
            if (claw == closeClaw){
                claw = openClaw;
            }
            else {
                claw = closeClaw;
            }
            Claw.setPosition(claw);
            if (state.equals("highBasket") || state.equals("lowBasket")){
                state = "droppedHighBasket";
                stateTime = getRuntime();
                eManual = false;
                manual = false;
            }
//            else if (state.equals("drop")){
//                state = "droppedObs";
//                stateTime = getRuntime();
//                eManual = false;
//                manual = false;
//            }
//            else if (state.equals("enterSub") && eState.equals("exitingSub")){
//                state = "droppedObs";
//                stateTime = getRuntime();
//                eManual = false;
//                manual = false;
//            }
        }
        rightTriggerPressed = gamepad1.right_trigger > 0.1;
        if (gamepad1.left_trigger > 0.1 && !leftTriggerPressed && intake_angle < 3) {
            if (intake_angle == intake_AngleFloor || intake_angle == intake_AngleBasket-0.2 || intake_angle == intake_AngleRung){
                intake_angle = intake_AngleVertical;
            }
            else{
                intake_angle = intake_AngleFloor;
            }
            Intake_Angle.setPosition(intake_angle);
            manual = true;
        }
        leftTriggerPressed = gamepad1.left_trigger > 0.1;
        if (gamepad2.left_trigger > 0.1 && !leftTrigger2Pressed && !mode.equals("specimenCycle")) {
            if (claw_angle == claw_AngleForward){
                claw_angle = claw_AngleBackward;
            }
            else {
                claw_angle = claw_AngleForward;
            }
            Claw_Angle.setPosition(claw_angle);
            manual = true;
        }
        leftTrigger2Pressed = gamepad2.left_trigger > 0.1; //suspicious one; change later
        if (gamepad1.left_stick_button && !leftStickPressed){ //reset claw angle and intake angle
            Intake_Angle.setPosition(intake_AngleFloor);
            intake_angle = intake_AngleFloor;
            servoResetTime = getRuntime();
            servoReset = true;
        }
        leftStickPressed = gamepad1.left_stick_button;
        if (servoReset && (getRuntime() > servoResetTime + 0.5)){
            Claw_Angle.setPosition(claw_AngleForward);
            claw_angle = claw_AngleForward;
            servoReset = false;
        }
    }
    private void Hook_Control(){
        if (hookDown) {
            tim = getRuntime();
            Hook.setPower(0.5);
            hookDown = false;
        }
        if (hookUp) {
            tim = getRuntime();
            Hook.setPower(-0.5);
            hookUp = false;
        }
        if (getRuntime() > tim + 0.5) {
            if (hanging) {
                target1 = -5;
            }
            if (getRuntime() > tim + 2) {
                Hook.setPower(0);
                if (hanging) {
                    target2 = highRung2 - 15;
                }
            }
        }
        if (gamepad2.right_trigger > 0.1 && !rightTrigger2Pressed) {
            sweeper = openSweeper - sweeper;
            Sweeper.setPosition(sweeper);
        }
        rightTrigger2Pressed = gamepad2.right_trigger > 0.1;
    }
    private void Control_Enhanced_States(){ //control driving
        if (mode.equals("sample")) {
            if (eState.equals("enteringSub")) { //Intake Angle 0, swivel Claw Angle to face the basket
                double clampedPos = Math.max(sub1, arm1Pos / ticks_in_degree_1);
                if (arm1Pos / ticks_in_degree_1 < sub1 + 40) {
                    Motor_Trans_Power = 0.5 + (1.0 - 0.5) * (clampedPos - sub1) / 40;
                } else {
                    Motor_Trans_Power = 1.0;
                }
                lockedMode = false;
            } else if (eState.equals("exitingSub")) {
                if (state.equals("highBasket") || state.equals("lowBasket")) {
                    double clampedPos = Math.min(highBasket1, arm1Pos / ticks_in_degree_1);
                    if (arm1Pos / ticks_in_degree_1 > highBasket1 - 40) {
                        Motor_Trans_Power = 0.5 + (1.0 - 0.5) * (highBasket1 - clampedPos) / 40;
                    } else {
                        Motor_Trans_Power = 1.0;
                    }
                }
//            else if (state.equals("highRung_Up")) {
//                Motor_Trans_Power = 0.5;
//            }
                else {
                    Motor_Trans_Power = 1.0;
                }
                lockedMode = false;
            }
        }
        else if (mode.equals("specimenCycle")||mode.equals("specimenCollect")){
            Motor_Trans_Power = 0.5;
            lockedMode = false;
        }
        if (hanging){
            Motor_Trans_Power = 1.0;
            Motor_Rot_Power = 2.0;
            lockedMode = false;
        }
    }
    private void ARM_PID_Control(){
        if (!state.equals("floor")){
            Lift_Power = 1;
        }
        double theta1_actual = Math.toRadians(target1pid + ARM1_OFFSET);
        double theta2_actual = Math.toRadians(target1pid + ARM1_OFFSET + target2pid + ARM2_OFFSET);
        controller1.setPID(p1,i1,d1);
//        arm1Pos = ARM1.getCurrentPosition();
        double pid1 = controller1.calculate(arm1Pos,(int)(target1pid*ticks_in_degree_1)); //PID calculation
        double ff1 = (m1 * x1 * Math.cos(theta1_actual) + m2 * (L1 * Math.cos(theta1_actual) + x2 * Math.cos(theta2_actual))) * f1;
        double power1 = pid1 + ff1;
        ARM1.setPower(power1*Lift_Power); //set the power

        controller2.setPID(p2,i2,d2);
//        arm2Pos = ARM2.getCurrentPosition();
        double pid2 = controller2.calculate(arm2Pos, (int)(target2pid*ticks_in_degree_2));
        double ff2 = m2 * x2 * Math.cos(theta2_actual) * f2;
        double power2 = pid2 + ff2;
        ARM2.setPower(power2);
    }
    private boolean inSub() {
        // Check if X and Y are both between 48 and 96 inches
        return (Math.abs(x) <= 16) &&
                (Math.abs(y) <= 28);
    }
    private void ARM_Calibration(){
        //resets each motor to 0 when the touch sensor is pressed, and doesn't enter the if statement afterwards
        if (!ARM1Sensor.isPressed() && !ARM1calibrated) {
            ARM1.setPower(0);
            ARM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ARM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            ARM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target1 = 0;
            Target1 = target1;
            target1pid = target1;
//            arm1TargetPos = target1 * ticks_in_degree_1;
            ARM1calibrated = true;
        }
        else if (!ARM1calibrated && ARM1.getPower() != -0.2){
            ARM1.setPower(-0.2);
        }
        if (!ARM2Sensor.isPressed() && !ARM2calibrated) {
            ARM2.setPower(0);
            ARM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ARM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            ARM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target2 = 0;
            Target2 = target2;
            target2pid = target2;
//            arm2TargetPos = target2 * ticks_in_degree_2;
            ARM2calibrated = true;
        }
        else if (!ARM2calibrated && ARM2.getPower() != -0.2){
            ARM2.setPower(-0.2);
        }
    }
    private void ARM_SetTargets() {
        if (gamepad2.y) { // Basket Mode
            mode = "sample";
            eState = "a";
            // Motion profiling OFF
        } else if (gamepad2.a) { // Specimen Collection
            mode = "specimenCollect";
            eState = "a";
            // Motion profiling ON
        } else if (gamepad2.x) { // Specimen Cycle
            mode = "specimenCycle";
            eState = "cycling";
        }
        if (gamepad2.start){ //recalibrate the motors
            ARM1calibrated = false;
            ARM2calibrated = false;
            ARM1.setPower(-0.2);
            ARM2.setPower(-0.2);
        }
        if (gamepad2.left_bumper) { //prepare for hang (change to LB)
            target1 = 115.3484; //111.830920056; //111.330920056
            target2 = 172;
            hookUp = true;
        }
        if (gamepad2.right_bumper) { //hang (change to RB)
            hanging = true;
            hookDown = true;
            manual = true;
        }

        if (gamepad1.right_bumper && !rightBumperPressed && !mode.equals("specimenCycle")) { //hang (change to RB)
            double currentTime = getRuntime();

            // Reset count if too much time has passed since last press
            if (currentTime - lastRbPressTime > RB_PRESS_TIMEOUT) {
                rbPressCount = 0;
            }

            rbPressCount++;
            lastRbPressTime = currentTime;

            if (rbPressCount == 1) {
                // First press - go to sub position
                target1 = sub1; //vertSub1;
                target2 = sub2;
                state = "enterSub";
                manual = false;
                if (mode.equals("specimenCollect")){
                    eState = "exitingSub";
                    eManual = false;
                }
                stateTime = getRuntime();
            }
            else if (rbPressCount == 2) {
                eState = "exitingSub";
                eManual = false;
                manual = false;
                rbPressCount = 0;
                if (mode.equals("sample")) {
                    // Second press - go to basket position
                    target1 = highBasket1;
                    target2 = highBasket2;
                    state = "highBasket";
                }
                else if (mode.equals("specimenCollect")){
                    state = "enterSub";
                    target1 = sub1; //vertSub1;
                    target2 = sub2;
                }
                stateTime = getRuntime();
            }
        }
        rightBumperPressed = gamepad1.right_bumper;

//        if (gamepad1.right_bumper && mode.equals("sample")){
//            eState = "exitingSub";
//            state = "enterSub";
//            target1 = sub1;
//            target2 = sub2;
//            eManual = false;
//            manual = false;
//        }
//        if (eState.equals("exitingSub") && !state.equals("droppedHighBasket") && !inSub && mode.equals("sample") && !eManual){
//            state = "highBasket";
//            target1 = highBasket1;
//            target2 = highBasket2;
//        }
        if (state.equals("droppedHighBasket") && getRuntime() - stateTime > 0.5 && mode.equals("sample") && !eManual) {
            if (!state.equals("enterSub")) {
                eState = "enteringSub";
                state = "enterSub";
                stateTime = getRuntime();
                target1 = sub1;
                target2 = sub2;
            }
        }
        if (gamepad1.left_bumper && (mode.equals("sample") || mode.equals("specimenCollect"))){
            target1 = floor1;
            target2 = floor2;
            state = "floor";
            Lift_Power = 0.4;
        }
//        if (eState.equals("exitingSub") && !state.equals("droppedObs") && !inSub && mode.equals("specimenCollect") && !eManual){
//            state = "drop";
//            target1 = highRung1_2;
//            target2 = down2;
//        }
//        if (state.equals("droppedObs") && getRuntime() - stateTime > 0.5 && mode.equals("specimenCollect") && !eManual) {
//            eState = "enteringSub";
//            if (!state.equals("enterSub")) {
//                state = "enterSub";
//                target1 = sub1;
//                target2 = sub2;
//            }
//        }
        if (gamepad1.right_bumper && mode.equals("specimenCycle")){
            state = "highRung";
            target1 = highRung1_2;
            target2 = highRung2_2;
            stateTime = getRuntime();
            eManual = false;
            manual = false;
        }
        if (gamepad1.left_bumper && mode.equals("specimenCycle")){
            target1 = wall1_2;
            target2 = wall2_2;
            state = "wall";
            Lift_Power = 0.75;
            stateTime = getRuntime();
            manual = false;
            eManual = false;
        }
        if (gamepad1.x) {//high rung
//            target1 = highRung;
            target1 = highRung1_2;
//            target2 = highRung2;
            target2 = highRung2_2;
            stateTime = getRuntime();
            state = "highRung";
            manual = false;
            eManual = true;
        }
        if (gamepad1.dpad_left) {//low basket
//            target1 = highRung;
            target1 = lowBasket1;
//            target2 = highRung2;
            target2 = lowBasket2;
            stateTime = getRuntime();
            state = "lowBasket";
            manual = false;
            eManual = false;
        }
        if (gamepad1.dpad_right) {//high rung
//            target1 = highRung;
            target1 = highRung1_First;
//            target2 = highRung2;
            target2 = highRung2_First;
            stateTime = getRuntime();
            state = "highRung_Up";
            manual = false;
            eManual = false;
        }
        if (gamepad1.y) {//high basket
            target1 = highBasket1;
            target2 = highBasket2;
//            Targeting_Angle = -45; //(add this?)
            stateTime = getRuntime();
            state = "highBasket";
            manual = false;
            eManual = true;
        }
        if (gamepad1.a) {//floor
            if (intake_angle == intake_AngleVertical){
                target1 = vertFloor1;
                target2 = vertFloor2;
            }
            else {
                target1 = floor1;
                target2 = floor2;
                state = "floor";
            }
            Lift_Power = 0.4;
        }
        if (gamepad1.b) {//wall
//            target1 = wall;
            target1 = wall1_2;
//            target2 = wall2;
            target2 = wall2_2;
            stateTime = getRuntime();
            state = "wall";
            manual = false;
            eManual = true;
//            Targeting_Angle = -175 + initialHeading;
//            if (Targeting_Angle > 180) {
//                Targeting_Angle = Targeting_Angle - 360;
//            } else if (Targeting_Angle < -180) {
//                Targeting_Angle = Targeting_Angle + 360;
//            }
        }
        if (gamepad1.touchpad) { //into submersible (not needed bc wall preset?)
            if (intake_angle == intake_AngleVertical){
                target1 = sub1; //vertSub1;
                target2 = sub1; //vertSub2;
                state = "enterSubVert";
            }
            else {
                target1 = sub1; //6.4322;
                target2 = sub2;
                state = "enterSub";
            }
            stateTime = getRuntime();
            manual = false;
            eManual = true;
        }
        if (gamepad1.right_stick_button){ //retract both arms
            target1 = down1;
            target2 = down2;
        }
        if (gamepad1.right_stick_y < -0.95) { //manual control of ARM1
            target1 += 0.89667631977;
        } else if (gamepad1.right_stick_y > 0.95) {
            target1 -= 0.89667631977;
        }
        if (gamepad1.dpad_up) { //manual control of ARM2
            target2 += 2.40399638714/3;
        } else if (gamepad1.dpad_down) {
            target2 -= 2.40399638714/3;
        }
        target1pid = target1;
        target2pid = target2;
    }

    /**
     * Describe this function...
     */
    private void Initialization() {
        controller1 = new PIDController(p1, i1, d1);
        controller2 = new PIDController(p2, i2, d2);

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        W_FR.setDirection(DcMotor.Direction.FORWARD);
        W_FL.setDirection(DcMotor.Direction.REVERSE);
        W_BR.setDirection(DcMotor.Direction.FORWARD);
        W_BL.setDirection(DcMotor.Direction.REVERSE);
        W_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ARM1.setDirection(DcMotor.Direction.REVERSE);
        ARM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ARM1.setPower(0);
        target1 = ARM1.getCurrentPosition()/ticks_in_degree_1;
        Target1 = target1;
        target1pid = target1;
//        arm1TargetPos = target1 * ticks_in_degree_1;
        ARM2.setDirection(DcMotor.Direction.REVERSE);
        ARM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ARM2.setPower(0);
        target2 = ARM2.getCurrentPosition()/ticks_in_degree_2;
        Target2 = target2;
        target2pid = target2;
//        arm2TargetPos = target2 * ticks_in_degree_2;
        telemetry.addData("Claw",Claw.getPosition());
        Claw.scaleRange(clawScale0,clawScale1);
        Intake_Angle.scaleRange(intake_AngleScale0, intake_AngleScale1);
        Claw_Angle.scaleRange(claw_AngleScale0, claw_AngleScale1);
        Sweeper.scaleRange(sweeperScale0,sweeperScale1); //0.582


        Motor_Power = 1.0;
        Motor_Trans_Power = 1.0;
        Motor_Rot_Power = 1.0;

        Targeting_Angle = initialHeading;

        if (printStuff) {
            telemetry.addData("arm1 pos", ARM1.getCurrentPosition());
            telemetry.addData("arm2 pos", ARM2.getCurrentPosition());
            telemetry.addData("arm1 target", target1);
            telemetry.addData("arm2. target", target2);

            telemetry.update();
        }
        waitForStart();
    }

    /**
     * Describe this function...
     */
    private void Calculate_IMU_Rotation_Power() {
        double Angle_Difference;
        try {
            Heading_Angle = Math.toDegrees(pinpoint.getHeading()) + initialHeading; //degrees
        }
        catch (Exception e){
            Heading_Angle = Targeting_Angle;
        }
        if (Math.abs(gamepad1.right_stick_x) >= 0.01) {
            imu_rotation = 0;
            Targeting_Angle = Heading_Angle;
        } else {
            Angle_Difference = Heading_Angle - Targeting_Angle;
            if (Angle_Difference > 180) {
                Angle_Difference = Angle_Difference - 360;
            } else if (Angle_Difference < -180) {
                Angle_Difference = Angle_Difference + 360;
            }
            if (Math.abs(Angle_Difference) < 1) {
                imu_rotation = 0;
            } else if (Angle_Difference >= 1) {
                imu_rotation = (Angle_Difference * 0.0 /*+ 0.1*/);
            } else {
                imu_rotation = (Angle_Difference * -0.0 /*- 0.1*/);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Calculate_Motor_Power() {
        double Motor_FWD_input;
        double Motor_Side_input;
        double mag;
        double x;
        double y;
        if (gamepad2.dpad_up) {eManual = true; Motor_Trans_Power = 1.0;}
        if (gamepad2.dpad_down) {eManual = true; Motor_Trans_Power = 0.5;}
        if (gamepad2.dpad_right) {overrideSpeed = true; Motor_Trans_Power = 0.5; Motor_Rot_Power = 1.0;}
        if (gamepad2.dpad_left) {overrideSpeed = false;}
        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        mag = Math.sqrt(y*y + x*x);
        if (lockedMode && mag == 0){
//            holdRobotPosition(currentVel);
            return;
        }
        Motor_FWD_input = y * mag * mag;
        Motor_Side_input = -x * mag * mag;
        Motor_fwd_power = (Math.cos(Heading_Angle / 180 * Math.PI) * Motor_FWD_input - Math.sin(Heading_Angle / 180 * Math.PI) * Motor_Side_input) * Motor_Trans_Power;
        Motor_side_power = (Math.cos(Heading_Angle / 180 * Math.PI) * Motor_Side_input + Math.sin(Heading_Angle / 180 * Math.PI) * Motor_FWD_input) / 0.7736350635 * Motor_Trans_Power; //*1.5
        Motor_Rotation_power = gamepad1.right_stick_x * 0.35 * Motor_Rot_Power + imu_rotation; //0.7 //0.5
        Motor_power_BL = -(((Motor_fwd_power - Motor_side_power) - Motor_Rotation_power) * Motor_Power);
        Motor_power_BR = -((Motor_fwd_power + Motor_side_power + Motor_Rotation_power) * Motor_Power);
        Motor_power_FL = -(((Motor_fwd_power + Motor_side_power) - Motor_Rotation_power) * Motor_Power);
        Motor_power_FR = -(((Motor_fwd_power - Motor_side_power) + Motor_Rotation_power) * Motor_Power);
        double m = Math.max(Math.max(Math.abs(Motor_power_BL),Math.abs(Motor_power_BR)),Math.max(Math.abs(Motor_power_FL),Math.abs(Motor_power_FR)));
        if (m > 1){
            Motor_power_BL /= m;
            Motor_power_BR /= m;
            Motor_power_FL /= m;
            Motor_power_FR /= m;
        }
    }
}
