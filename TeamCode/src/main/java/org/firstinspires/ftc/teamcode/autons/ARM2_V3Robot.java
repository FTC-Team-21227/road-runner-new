package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem_Constants;
import org.firstinspires.ftc.teamcode.TunePID;
import org.firstinspires.ftc.teamcode.TunePID_MotionProfile;

public class ARM2_V3Robot {
    private DcMotor arm2;
    //PID controllers for ARM1 and ARM2
    private PIDController controller2;
    //PIDF gains
    double p2 = TunePID_MotionProfile.p2, i2 = TunePID_MotionProfile.i2, d2 = TunePID_MotionProfile.d2;
    double f2 = TunePID_MotionProfile.f2;
    //ticks to degrees conversion
    private final double ticks_in_degree_2 = TunePID_MotionProfile.ticks_in_degree_2;
    //length, COM, mass values for feedforward calculation (not even performed in arm2)
    private final double L1 = TunePID_MotionProfile.L1;
    private final double L2 = TunePID_MotionProfile.L2;
    private final double x1 = TunePID_MotionProfile.x1;
    private final double x2 = TunePID_MotionProfile.x2;
    private final double m1 = TunePID_MotionProfile.m1;
    private final double m2 = TunePID_MotionProfile.m2;
    private final double highBasket2 = Subsystem_Constants.highBasket2_auto;
    final double highRung2 = Subsystem_Constants.highRung2;
    final double highRung2_2 = Subsystem_Constants.highRung2_2;
    final double highRung2_First = Subsystem_Constants.highRung2_First;
    private final double wall2 = Subsystem_Constants.wall2;
    private final double wall2_2 = Subsystem_Constants.wall2_2;
    private final double wall2_First = Subsystem_Constants.wall2_First;
    private final double lowBasket2 = Subsystem_Constants.lowBasket2;
    private final double floor2 = Subsystem_Constants.floor2;
    private final double down2 = Subsystem_Constants.down2;
    private final double sub2 = Subsystem_Constants.sub2;
    private final double vertSub2 = Subsystem_Constants.vertSub2;
    private final double vertFloor2 = Subsystem_Constants.vertFloor2;
    // Motion Profile Variables (ADDED)
    private double arm2StartPos;
    private double arm2TargetPos;
    private double arm2DDec;
    private boolean arm2Decelerating = false;
    private final double V_MAX = TunePID_MotionProfile.V_MAX; // Encoder ticks/sec (≈ 100° /sec if 1 tick/degree)
    private final double A_DEC = TunePID_MotionProfile.A_DEC; // Ticks/sec² (adjust for smooth stopping)
    private final double LOOP_TIME = TunePID_MotionProfile.LOOP_TIME; // 20ms (typical FTC loop time)
    double ARM1_OFFSET = TunePID_MotionProfile.ARM1_OFFSET;
    double ARM2_OFFSET = TunePID_MotionProfile.ARM2_OFFSET;

    public ARM2_V3Robot(HardwareMap hardwareMap) {
        arm2 = hardwareMap.get(DcMotor.class, "ARM2");
        arm2.setDirection(DcMotor.Direction.REVERSE); //CHANGE BACK TO DCMOTORSIMPLE IF SOMETHING DOESN'T WORK
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller2 = new PIDController(p2, i2, d2);
    }
    public class LiftTarget implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double target2;
        double target2pid;
        double waitTime;
        double runTime;
        boolean startMove;
        double power = 1;

        public LiftTarget(double pos) {
            target2 = pos;
            start = false;
            startMove = false;
            waitTime = 0;
            runTime = 2;
        }

        public LiftTarget(double pos, double tim) {
            target2 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = 2;
        }

        public LiftTarget(double pos, double tim, double runtime) {
            target2 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
        }

        public LiftTarget(double pos, double tim, double runtime, double pwr) {
            target2 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
            power = pwr;
        }

        private void initArmProfile(double targetPos) {
            arm2StartPos = arm2.getCurrentPosition();
            arm2TargetPos = targetPos;
            double dTotal = Math.abs(arm2TargetPos - arm2StartPos);
            arm2DDec = (V_MAX * V_MAX) / (2 * A_DEC);
            arm2Decelerating = (dTotal <= arm2DDec);
        }

        private void updateArmProfile() {
            DcMotor motor;
            double currentPos, targetPos, dDec;
            boolean decelerating;
            motor = arm2;
            currentPos = motor.getCurrentPosition();
            targetPos = arm2TargetPos;
            decelerating = arm2Decelerating;
            dDec = arm2DDec;
            double dRemaining = Math.abs(targetPos - currentPos);
            double direction = Math.signum(targetPos - currentPos);

            // Check if we need to start decelerating
            if (!decelerating && dRemaining <= dDec) {
                decelerating = true;
                arm2Decelerating = true;
            }

            // Calculate desired velocity
            double vDesired;
            if (decelerating) {
                vDesired = Math.sqrt(2 * A_DEC * dRemaining);
                vDesired = Math.min(vDesired, V_MAX);
            } else {
                vDesired = V_MAX;
            }

            // Update target position
            double delta = vDesired * direction * LOOP_TIME;
            double newTarget = currentPos + delta;

            target2pid = newTarget / ticks_in_degree_2; // Convert to degrees
            PoseStorage.target2 = target2pid;
        }

        public double ARM_Control_PID(@NonNull TelemetryPacket packet) {
            double target1 = PoseStorage.target1; //NEW
            packet.addLine("target1pos2:"+target2pid); //NEW
            double theta1_actual = Math.toRadians(target1 + ARM1_OFFSET);
            double theta2_actual = Math.toRadians(target1 + ARM1_OFFSET + target2pid + ARM2_OFFSET);
            int arm2Pos = arm2.getCurrentPosition();
            double pid2 = controller2.calculate(arm2Pos, (int)(target2pid*ticks_in_degree_2));
            double ff2 = m2 * x2 * Math.cos(theta2_actual) * f2;
            return (pid2 + ff2) * power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            packet.addLine("time.seconds():"+(time.seconds()));
            packet.addLine("arm2pos:"+(arm2.getCurrentPosition()));
//            packet.addLine("target1pos:"+target1);
            packet.addLine("target2pos:"+(int)(target2*ticks_in_degree_2));

            if (time.seconds() < runTime + waitTime/* && Math.abs(arm2.getCurrentPosition() - arm2TargetPos) > 5*/) {
                if (time.seconds() > waitTime) {
                    if (!startMove){
                        PoseStorage.target2 = target2;
                        initArmProfile(target2 * ticks_in_degree_2);
                        startMove = true;
                    }
                    updateArmProfile();
                    double power = ARM_Control_PID(packet /*new*/);
                    packet.addLine("power2:" + power);
                    arm2.setPower(power);                }
                return true;
            } else {
                packet.addLine("Arm 2 time:" + time.seconds());
                arm2.setPower(0);
                return false;
            }
        }
    }

    public Action liftHighBasket() {return new LiftTarget(highBasket2);}
    public Action liftRung() {return new LiftTarget( highRung2);}
    public Action liftRung2() {return new LiftTarget( highRung2_2);}
    public Action liftRung2_First() {return new LiftTarget( highRung2_First);}
    public Action liftWall() {return new LiftTarget(wall2);}
    public Action liftWall2() {return new LiftTarget(wall2_2);}
    public Action liftWall2_First() {return new LiftTarget(wall2_First);}
    public Action liftLowBasket() {return new LiftTarget(lowBasket2);} //not tested i think
    public Action liftFloor() {return new LiftTarget(floor2);}
    public Action liftDown() {return new LiftTarget(down2);}
    public Action liftSub() {return new LiftTarget(sub2);}
    public Action liftVertSub() {return new LiftTarget(vertSub2);}
    public Action liftVertFloor() {return new LiftTarget(vertFloor2);}
    public Action liftHighBasket(double waitseconds) {return new LiftTarget(highBasket2,waitseconds);}
    public Action liftRung(double waitseconds) {return new LiftTarget(highRung2,waitseconds);}
    public Action liftRung2(double waitseconds) {return new LiftTarget(highRung2_2,waitseconds);}
    public Action liftRung2_First(double waitseconds) {return new LiftTarget(highRung2_First,waitseconds);}
    public Action liftWall(double waitseconds) {return new LiftTarget(wall2,waitseconds);}
    public Action liftWall2(double waitseconds) {return new LiftTarget(wall2_2,waitseconds);}
    public Action liftWall2_First(double waitseconds) {return new LiftTarget(wall2_First,waitseconds);}
    public Action liftLowBasket(double waitseconds) {return new LiftTarget(lowBasket2,waitseconds);} //not tested i think
    public Action liftFloor(double waitseconds) {return new LiftTarget(floor2,waitseconds);}
    public Action liftDown(double waitseconds) {return new LiftTarget(down2,waitseconds);}
    public Action liftSub(double waitseconds) {return new LiftTarget(sub2, waitseconds);}
    public Action liftVertSub(double waitseconds) {return new LiftTarget(vertSub2,waitseconds);}
    public Action liftVertFloor(double waitseconds) {return new LiftTarget(vertFloor2,waitseconds);}
    public Action liftHighBasket(double waitseconds, double seconds) {return new LiftTarget(highBasket2,waitseconds,seconds);}
    public Action liftRung(double waitseconds, double seconds) {return new LiftTarget(highRung2,waitseconds,seconds);}
    public Action liftRung2(double waitseconds, double seconds) {return new LiftTarget(highRung2_2,waitseconds,seconds);}
    public Action liftRung2_First(double waitseconds, double seconds) {return new LiftTarget(highRung2_First,waitseconds,seconds);}
    public Action liftRung2First(double waitseconds, double seconds) {return new LiftTarget(highRung2_2+0.3,waitseconds,seconds);}
    public Action liftWall(double waitseconds, double seconds) {return new LiftTarget(wall2,waitseconds,seconds);}
    public Action liftWall2(double waitseconds, double seconds) {return new LiftTarget(wall2_2,waitseconds,seconds);}
    public Action liftWall2_First(double waitseconds, double seconds) {return new LiftTarget(wall2_First,waitseconds,seconds);}
    public Action liftLowBasket(double waitseconds, double seconds) {return new LiftTarget(lowBasket2,waitseconds,seconds);} //not tested i think
    public Action liftFloor(double waitseconds, double seconds) {return new LiftTarget(floor2,waitseconds,seconds);}
    public Action liftDown(double waitseconds, double seconds) {return new LiftTarget(down2,waitseconds,seconds);}
    public Action liftSub(double waitseconds, double seconds) {return new LiftTarget(sub2, waitseconds, seconds);}
    public Action liftVertSub(double waitseconds,double seconds) {return new LiftTarget(vertSub2,waitseconds,seconds);}
    public Action liftVertFloor(double waitseconds,double seconds) {return new LiftTarget(vertFloor2,waitseconds,seconds);}
    public Action liftRung(double waitseconds,double seconds, double power) {return new LiftTarget(highRung2, waitseconds, seconds, power);}
    public Action liftRung2(double waitseconds,double seconds, double power) {return new LiftTarget(highRung2_2, waitseconds, seconds, power);}
    public Action liftRung2_First(double waitseconds,double seconds, double power) {return new LiftTarget(highRung2_First, waitseconds, seconds, power);}
    public Action liftRungFirst(double waitseconds,double seconds, double power) {return new LiftTarget(highRung2+0.75, waitseconds, seconds,power);}
}