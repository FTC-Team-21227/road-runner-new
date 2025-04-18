package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.autons.PoseStorage;

//@TeleOp(name = "TeleOp2425_V3Robot_Pinpoint")
public class TeleOp2425_V3Robot_Pinpoint extends LinearOpMode {
    //PID controllers for ARM1 and ARM2
    private PIDController controller1;
    private PIDController controller2;
    //PIDF gains
    double p1 = TunePID.p1, i1 = TunePID.i1, d1 = TunePID.d1;
    double f1 = TunePID.f1;
    double p2 = TunePID.p2, i2 = TunePID.i2, d2 = TunePID.d2;
    double f2 = TunePID.f2;
    //ARM1, ARM2 target positions, in degrees
    double target1 = 0;
    double target2 = 0;
    //ticks to degrees conversion, very useful
    private final double ticks_in_degree_1 = 537.7*28/360; // = 41.8211111111
    private final double ticks_in_degree_2 = 145.1*28/360; // = 11.2855555556
    private final double L1 = 43.2;
    private final double L2 = 43.2;
    private final double x1 = 36.96;
    private final double x2 = 26.4;
    private final double m1 = 810;
    private final double m2 = 99.79;
    private final double highBasket1 = Subsystem_Constants.highBasket1;
    private final double highRung1 = Subsystem_Constants.highRung1;
    private final double highRung1_2 = Subsystem_Constants.highRung1_2;
    private final double wall1 = Subsystem_Constants.wall1;
    private final double wall1_2 = Subsystem_Constants.wall1_2;
    private final double lowBasket1 = Subsystem_Constants.lowBasket1;
    private final double floor1 = Subsystem_Constants.floor1;
    private final double down1 = Subsystem_Constants.down1;
    private final double sub1 = Subsystem_Constants.sub1;

    private final double highBasket2 = Subsystem_Constants.highBasket2_teleop;
    final double highRung2 = Subsystem_Constants.highRung2;
    final double highRung2_2 = Subsystem_Constants.highRung2_2;
    private final double wall2 = Subsystem_Constants.wall2;
    private final double wall2_2 = Subsystem_Constants.wall2_2;
    private final double lowBasket2 = Subsystem_Constants.lowBasket2;
    private final double floor2 = Subsystem_Constants.floor2;
    private final double down2 = Subsystem_Constants.down2;
    private final double sub2 = Subsystem_Constants.sub2;

    final double clawScale0 = Subsystem_Constants.clawScale0;
    final double clawScale1 = Subsystem_Constants.clawScale1;
    final double closeClaw = Subsystem_Constants.closeClaw;
    final double openClaw = Subsystem_Constants.openClaw;

    final double intake_AngleScale0 = Subsystem_Constants.intake_AngleScale0;
    final double intake_AngleScale1 = Subsystem_Constants.intake_AngleScale1;
    final double intake_AngleFloor = Subsystem_Constants.intake_AngleFloor;
    final double intake_AngleBasket = Subsystem_Constants.intake_AngleBasket_teleop;
    final double intake_AngleRung = Subsystem_Constants.intake_AngleRung;
    final double intake_AngleStart = Subsystem_Constants.intake_AngleStart;
    final double intake_AngleWall = Subsystem_Constants.intake_AngleWall;
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
    private DcMotor W_BL;
    private DcMotor W_BR;
    private DcMotor W_FR;
    private DcMotor W_FL;
    private GoBildaPinpointDriver pinpoint;
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
    double tim;
    boolean ARM1calibrated = true;
    boolean ARM2calibrated = true;
    boolean hanging = false;
    int arm1Pos;
    int arm2Pos;
    String state = "a";
    double stateTime;
    boolean manual = false;
    boolean rightTriggerPressed = false;
    boolean leftTriggerPressed = false;
    boolean rightBumperPressed = false;
    boolean leftStickPressed = false;
    boolean rightTrigger2Pressed = false;
    boolean hookUp = false;
    boolean hookDown = false;
    int claw = 0;
    double claw_angle = claw_AngleForward;
    double intake_angle = intake_AngleFloor;
    double sweeper = openSweeper;
    boolean servoReset = false;
    double servoResetTime;
    double Lift_Power = 1;
    double currTime = 0;
    double initialHeading;
    boolean back = true;
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException{
        //getting all the motors, servos, and sensors from the hardware map
        Pose2d initialPose;
        try {
            initialPose = PoseStorage.currentPose;
            telemetry.addData("Yay1!","Yay1!");
        }
        catch (Exception e){
            initialPose = new Pose2d(0,0,0);
            telemetry.addData("No!","No!");
        }
        initialHeading = Math.toDegrees(initialPose.heading.toDouble());
        W_BL = hardwareMap.get(DcMotor.class, "W_BL");
        W_BR = hardwareMap.get(DcMotor.class, "W_BR");
        W_FR = hardwareMap.get(DcMotor.class, "W_FR");
        W_FL = hardwareMap.get(DcMotor.class, "W_FL");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        ARM1 = hardwareMap.get(DcMotor.class, "ARM1");
        ARM2 = hardwareMap.get(DcMotor.class, "ARM2");
        Hook = hardwareMap.get(CRServo.class, "Hook");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Intake_Angle = hardwareMap.get(Servo.class,"Intake_Angle");
        Claw_Angle = hardwareMap.get(Servo.class,"Claw_Angle");
        Sweeper = hardwareMap.get(Servo.class,"Sweeper");
        ARM1Sensor = hardwareMap.get(TouchSensor.class, "ARM1Sensor");
        ARM2Sensor = hardwareMap.get(TouchSensor.class, "ARM2Sensor");

        // Put initialization blocks here.
        Initialization();
        if (opModeIsActive()) {
            Claw_Angle.setPosition(claw_AngleForward);
            Intake_Angle.setPosition(intake_AngleFloor);
//            Claw.setPosition(0);
//            Sweeper.setPosition(0);
            // Put run blocks here.
            while (opModeIsActive()) {
                currTime = getRuntime();
                pinpoint.update(/*GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING*/);
                // Put loop blocks here.
                Calculate_IMU_Rotation_Power(); //calculates each motor power based on IMU reading
                Calculate_Motor_Power(); //calculates translational and rotational motor power
                //set power to each wheel motor
                W_BL.setPower(Motor_power_BL);
                W_BR.setPower(Motor_power_BR);
                W_FR.setPower(Motor_power_FR);
                W_FL.setPower(Motor_power_FL);
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
                //reset imu if necessary
                if (gamepad1.back && !back) {
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
                back = gamepad1.back;
                //        telemetry.addData("ARM1Calibrated",ARM1calibrated);
                //        telemetry.addData("ARM2Calibrated", ARM2calibrated);
                //        telemetry.addData("where am i going", !(ARM1calibrated && ARM2calibrated));
                telemetry.addData("Heading", Heading_Angle);
                telemetry.addData("Initial Heading", initialHeading);
                telemetry.addData("Targeting Angle", Targeting_Angle);
                telemetry.addData("Motor Power", Motor_Power);
                telemetry.addData("Side Power", Motor_side_power);
                telemetry.addData("FWD Power", Motor_fwd_power);
                telemetry.addData("IMU_Rotation Power", imu_rotation);
                telemetry.addData("Rotation Power", Motor_Rotation_power);
                telemetry.addData("ODO_Left", W_FL.getCurrentPosition());
                telemetry.addData("ODO_Right", W_FR.getCurrentPosition());
                telemetry.addData("ODO_Center", W_BR.getCurrentPosition());
                telemetry.addData("ARM1Pos: ", arm1Pos/ticks_in_degree_1);
                telemetry.addData("ARM1Target: ", target1);
                telemetry.addData("ARM2 Current Angle: ", arm2Pos/ticks_in_degree_2);
                telemetry.addData("ARM2 Target Angle: ", target2);
                telemetry.addData("Claw", Claw.getPosition());
                telemetry.addData("Intake angle", Intake_Angle.getPosition());
                telemetry.addData("Claw angle", Claw_Angle.getPosition());
                telemetry.addData("ARM1 Power", ARM1.getPower());
                telemetry.addData("ARM2 Power", ARM2.getPower());
                telemetry.addData("Manual servo", manual);
                telemetry.addData("State time", stateTime);
                telemetry.addData("Run time", currTime);
                telemetry.addData("Loop time", getRuntime()-currTime);
                telemetry.addData("ARM1 Sensor", ARM1Sensor.isPressed());
                telemetry.addData("ARM2 Sensor", ARM2Sensor.isPressed());
                telemetry.update();
            }
        }
    }
    private void Control_Servo_States() {
        if (state.equals("highBasket")) { //Intake Angle 0, swivel Claw Angle to face the basket
            telemetry.addData("In high basket", "yes");
            if (getRuntime() - stateTime > 0.25) {
                if (intake_angle != intake_AngleBasket) {
                    Intake_Angle.setPosition(intake_AngleBasket);
                    intake_angle = intake_AngleBasket;
                }
                if (claw_angle != claw_AngleBackward) {
                    Claw_Angle.setPosition(claw_AngleBackward);
                    claw_angle = claw_AngleBackward;
                    telemetry.addData("fd", "whyu not rotating");
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
                telemetry.addData("In high rung", "yes");
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
                telemetry.addData("In wall", "yes");
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
                telemetry.addData("In enter sub", "yes");
            }
        }
    }
    private void Intake_Control(){
        if (gamepad1.right_trigger > 0.1 && !rightTriggerPressed) {
            claw = 1 - claw;
            Claw.setPosition(claw);
//            manual = true;
        }
        rightTriggerPressed = gamepad1.right_trigger > 0.1;
        if (gamepad1.left_trigger > 0.1 && !leftTriggerPressed && intake_angle < 3) {
            if (intake_angle == intake_AngleFloor){
                intake_angle = intake_AngleVertical;
            }
            else{
                intake_angle = intake_AngleFloor;
            }
            Intake_Angle.setPosition(intake_angle);
            manual = true;
        }
        leftTriggerPressed = gamepad1.left_trigger > 0.1;
        if (gamepad1.right_bumper && !rightBumperPressed) {
            if (claw_angle == claw_AngleForward){
                claw_angle = claw_AngleBackward;
            }
            else {
                claw_angle = claw_AngleForward;
            }
            Claw_Angle.setPosition(claw_angle);
            manual = true;
        }
        rightBumperPressed = gamepad1.right_bumper;
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
        if (gamepad2.right_bumper || hookDown) {
            tim = getRuntime();
            Hook.setPower(0.5);
            hookDown = false;
        }
        if (gamepad2.left_bumper || hookUp) {
            tim = getRuntime();
            Hook.setPower(-0.5);
            hookUp = false;
        }
        if (getRuntime() > tim+1.5) {
            Hook.setPower(0);
            if (hanging){
                target2 = highRung2-20;
            }
        }
        if (gamepad2.right_trigger > 0.1 && !rightTrigger2Pressed) {
            sweeper = openSweeper - sweeper;
            Sweeper.setPosition(sweeper);
//            manual = true;
        }
        rightTrigger2Pressed = gamepad2.right_trigger > 0.1;
    }
    private void ARM_PID_Control(){
        if (!state.equals("floor")){
            Lift_Power = 1;
        }
        controller1.setPID(p1,i1,d1);
        arm1Pos = ARM1.getCurrentPosition();
        double pid1 = controller1.calculate(arm1Pos,(int)(target1*ticks_in_degree_1)); //PID calculation
        double ff1 = (m1*Math.cos(Math.toRadians(target1))*x1 +
        m2*Math.cos(Math.atan(((x2*Math.sin(Math.toRadians(target1+target2)))+(L1*Math.sin(Math.toRadians(target1))))/((L1*Math.cos(Math.toRadians(target1)))+(x2*Math.cos(Math.toRadians(target1+target2))))))*
        Math.sqrt(Math.pow((x2*Math.sin(Math.toRadians(target1+target2))+L1*Math.sin(Math.toRadians(target1))),2)+Math.pow((x2*Math.cos(Math.toRadians(target1+target2))+L1*Math.cos(Math.toRadians(target1))),2))) * f1; // feedforward calculation, change when equation is derived
        double power1 = pid1 + ff1;
        //telemetry.addData("ff1",ff1);
        ARM1.setPower(power1*Lift_Power); //set the power

        controller2.setPID(p2,i2,d2);
        arm2Pos = ARM2.getCurrentPosition();
        double pid2 = controller2.calculate(arm2Pos, (int)(target2*ticks_in_degree_2));
        double ff2 = (m2*Math.cos(Math.toRadians(target1+target2))*x2) * f2; //feedforward calculation, change when equation is derived
        double power2 = pid2 + ff2;
        ARM2.setPower(power2);
    }
    private void ARM_Calibration(){
        //resets each motor to 0 when the touch sensor is pressed, and doesn't enter the if statement afterwards
        if (!ARM1Sensor.isPressed() && !ARM1calibrated) {
            ARM1.setPower(0);
            ARM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ARM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            ARM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target1 = 0;
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
            ARM2calibrated = true;
        }
        else if (!ARM2calibrated && ARM2.getPower() != -0.2){
            ARM2.setPower(-0.2);
        }
        telemetry.addData("We Are Heree","yesd");
    }
    private void ARM_SetTargets() {
        if (gamepad2.start){ //recalibrate the motors
            ARM1calibrated = false;
            ARM2calibrated = false;
            ARM1.setPower(-0.2);
            ARM2.setPower(-0.2);
        }
        if (gamepad2.a) { //prepare for hang
            target1 = 115.3484; //111.830920056; //111.330920056
            target2 = 172;
//            hanging = true;
            hookUp = true;
        }
        if (gamepad2.b) { //hang
            target1 = -5; //might need to change this, doesn't make sense to lower it past 0.
            //ARM2 must be moved up manually.
//            Claw_Angle.setPosition(0);
//            Intake_Angle.setPosition(1);
//            Claw.setPosition(1);
            hanging = true;
            hookDown = true;
            manual = true;
        }
        if (gamepad1.x && gamepad1.left_bumper){
//            target1 = highRung;
            target1 = highRung1_2;
//            target2 = highRung2;
            target2 = highRung2_2;
        }
        else if (gamepad1.x) {//high rung
//            target1 = highRung;
            target1 = highRung1_2;
//            target2 = highRung2;
            target2 = highRung2_2;
            stateTime = getRuntime();
            state = "highRung";
            manual = false;
        }
        if (gamepad1.y && gamepad1.left_bumper){
            target1 = highBasket1;
            target2 = highBasket2;
        }
        else if (gamepad1.y) {//high basket
            target1 = highBasket1;
            target2 = highBasket2;
//            Targeting_Angle = -45; //(add this?)
            stateTime = getRuntime();
            state = "highBasket";
            manual = false;
        }
        if (gamepad1.a) {//floor
            target1 = floor1;
            target2 = floor2;
            Lift_Power = 0.25;
            state = "floor";
        }
        if (gamepad1.b && gamepad1.left_bumper){
//            target1 = wall;
            target1 = wall1_2;
//            target2 = wall2;
            target2 = wall2_2;
        }
        else if (gamepad1.b) {//wall
//            target1 = wall;
            target1 = wall1_2;
//            target2 = wall2;
            target2 = wall2_2;
            stateTime = getRuntime();
            state = "wall";
            manual = false;
            Targeting_Angle = -175;
        }
        if (gamepad1.start && gamepad1.left_bumper){
            target1 = sub1;
            target2 = sub2;
        }
        else if (gamepad1.start) { //into submersible (not needed bc wall preset?)
            target1 = sub1; //6.4322;
            target2 = sub2;
            stateTime = getRuntime();
            state = "enterSub";
            manual = false;
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
        //SOFTWARE LIMITS:
//        if (ARM1.getCurrentPosition() > 12700 && !hanging) {
//            ARM1.setTargetPosition(12600);
//        }
//        else if (ARM1.getCurrentPosition()+ARM2.getCurrentPosition()>8000){
//            ARM2.setTargetPosition(ARM2.getCurrentPosition());
//        }
//        if (ARM1.getCurrentPosition() > 5055 && !hanging){
//            ARM2.setTargetPosition(7508);
//        }
//        if (ARM2.getCurrentPosition() > 11000 && ARM1.getCurrentPosition() > 6000){
//            ARM2.setTargetPosition(10700);
//        }
    }

    /**
     * Describe this function...
     */
    private void Initialization() {
        controller1 = new PIDController(p1, i1, d1);
        controller2 = new PIDController(p2, i2, d2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        W_FR.setDirection(DcMotor.Direction.REVERSE);
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
//        Intake_Angle.scaleRange(-0.064, 0.26); //0.26, 0.584
        ARM1.setDirection(DcMotor.Direction.REVERSE);
        ARM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ARM1.setPower(0);
        target1 = ARM1.getCurrentPosition()/ticks_in_degree_1;
        ARM2.setDirection(DcMotor.Direction.REVERSE);
        ARM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ARM2.setPower(0);
        target2 = ARM2.getCurrentPosition()/ticks_in_degree_2;
        telemetry.addData("Claw",Claw.getPosition());
        telemetry.update();
        Claw.scaleRange(clawScale0,clawScale1);
        Intake_Angle.scaleRange(intake_AngleScale0, intake_AngleScale1);
        Claw_Angle.scaleRange(claw_AngleScale0, claw_AngleScale1);
        Sweeper.scaleRange(sweeperScale0,sweeperScale1); //0.482

        Motor_Power = 0.5;

        pinpoint.resetPosAndIMU();
        // wait for pinpoint to finish calibrating
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
//        pinpoint.update();
        Targeting_Angle = initialHeading;

        waitForStart();
    }

    /**
     * Describe this function...
     */
    private void Calculate_IMU_Rotation_Power() {
        double Angle_Difference;

        Heading_Angle = pinpoint.getPosition().getHeading(AngleUnit.DEGREES)+initialHeading; //degrees
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
                imu_rotation = (Angle_Difference * 0.01 + 0.1);
            } else {
                imu_rotation = (Angle_Difference * 0.01 - 0.1);
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
//        if (!(gamepad1.right_trigger>0.1)) {
        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        mag = Math.sqrt(y*y + x*x);
        Motor_FWD_input = y * mag;
        Motor_Side_input = -x * mag;
        Motor_fwd_power = Math.cos(Heading_Angle / 180 * Math.PI) * Motor_FWD_input - Math.sin(Heading_Angle / 180 * Math.PI) * Motor_Side_input;
        Motor_side_power = (Math.cos(Heading_Angle / 180 * Math.PI) * Motor_Side_input + Math.sin(Heading_Angle / 180 * Math.PI) * Motor_FWD_input) / 0.7736350635; //*1.5
        Motor_Rotation_power = gamepad1.right_stick_x * 0.7 + imu_rotation; //0.5
        Motor_power_BL = -(((Motor_fwd_power - Motor_side_power) - Motor_Rotation_power) * Motor_Power);
        Motor_power_BR = -((Motor_fwd_power + Motor_side_power + Motor_Rotation_power) * Motor_Power);
        Motor_power_FL = -(((Motor_fwd_power + Motor_side_power) - Motor_Rotation_power) * Motor_Power);
        Motor_power_FR = (((Motor_fwd_power - Motor_side_power) + Motor_Rotation_power) * Motor_Power);
//        } else {
//            Motor_power_BR = 0;
//            Motor_power_BL = 0;
//            Motor_power_FL = 0;
//            Motor_power_FR = 0;
//        }
    }
}
