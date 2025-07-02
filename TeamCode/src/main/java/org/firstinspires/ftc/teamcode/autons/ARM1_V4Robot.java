package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem_Constants;
import org.firstinspires.ftc.teamcode.TunePID_MotionProfile;
import org.firstinspires.ftc.teamcode.vision.ExcludePipeline;

public class ARM1_V4Robot {
    private DcMotor arm1;
    //PID controllers for ARM1 and ARM2
    private PIDController controller1;
    //PIDF gains
    double p1 = TunePID_MotionProfile.p1, i1 = TunePID_MotionProfile.i1, d1 = TunePID_MotionProfile.d1;
    double f1 = TunePID_MotionProfile.f1;
    //ticks to degrees conversion
    private final double ticks_in_degree_1 = TunePID_MotionProfile.ticks_in_degree_1;
    //length, COM, mass values for feedforward calculation
    private final double L1 = TunePID_MotionProfile.L1;
    private final double L2 = TunePID_MotionProfile.L2;
    private final double x1 = TunePID_MotionProfile.x1;
    private final double x2 = TunePID_MotionProfile.x2;
    private final double m1 = TunePID_MotionProfile.m1;
    private final double m2 = TunePID_MotionProfile.m2;
    private final double highBasket = Subsystem_Constants.highBasket1;
    private final double highRung = Subsystem_Constants.highRung1;
    private final double highRung2 = Subsystem_Constants.highRung1_2;
    private final double highRung_First = Subsystem_Constants.highRung1_First;
    private final double wall = Subsystem_Constants.wall1;
    private final double wallwall = Subsystem_Constants.wallwall1;
    private final double wall2 = Subsystem_Constants.wall1_2;
    private final double wall2_First = Subsystem_Constants.wall1_2_First;
    private final double wall_First = Subsystem_Constants.wall1_First;
    private final double lowBasket = Subsystem_Constants.lowBasket1;
    private final double floor = Subsystem_Constants.floor1;
    private final double down = Subsystem_Constants.down1;
    private final double sub = Subsystem_Constants.sub1;
    private final double vertSub1 = Subsystem_Constants.vertSub1;
    private final double vertSub1_auto = Subsystem_Constants.vertSub1_auto;
    private final double vertFloor1 = Subsystem_Constants.vertFloor1;
    double ARM1_OFFSET = TunePID_MotionProfile.ARM1_OFFSET;
    double ARM2_OFFSET = TunePID_MotionProfile.ARM2_OFFSET;
    // ARM1 Motion Profile Variables
    private double arm1StartPos;
    private double arm1TargetPos;
    private double arm1DDec;
    private boolean arm1Decelerating = false;

    // Profile Parameters (tune these!)
    private final double V_MAX = TunePID_MotionProfile.V_MAX; // Encoder ticks/sec (≈ 100° /sec if 1 tick/degree)
    private final double A_DEC = TunePID_MotionProfile.A_DEC; // Ticks/sec² (adjust for smooth stopping)
    private final double LOOP_TIME = TunePID_MotionProfile.LOOP_TIME; // 20ms (typical FTC loop time)

    public ARM1_V4Robot(HardwareMap hardwareMap) {
        arm1 = hardwareMap.get(DcMotor.class, "ARM1");
        arm1.setDirection(DcMotor.Direction.REVERSE); //CHANGE BACK TO DCMOTORSIMPLE IF SOMETHING DOESN'T WORK
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller1 = new PIDController(p1, i1, d1);
    }

    public class LiftTarget implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double target1;
        double target1pid;
        double waitTime;
        double runTime;
        boolean startMove;
        double power = 1;
        boolean profile = true;
        private double profileStartTime;
        private double profileStartPos;
        private double profileTargetPos;
        private double totalDistance;
        private double decelStartDistance;
        private boolean decelerating;
        private double currentVelocity;

        public LiftTarget(double pos) {
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = 0;
            runTime = 2;
        }

        public LiftTarget(double pos, double tim) {
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = 2;
        }

        public LiftTarget(double pos, double tim, double runtime) {
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
        }

        public LiftTarget(double pos, double tim, double runtime, double pwr) {
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
            power = pwr;
        }

        public LiftTarget(double pos, double tim, double runtime, double pwr, boolean prof) {
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
            power = pwr;
            profile = prof;
        }

        private void initMotionProfile(double targetPos) {
            profileStartPos = arm1.getCurrentPosition();
            profileTargetPos = targetPos;
            totalDistance = Math.abs(profileTargetPos - profileStartPos);
            decelStartDistance = (V_MAX * V_MAX) / (2 * A_DEC);
            decelerating = (totalDistance <= decelStartDistance);
            currentVelocity = 0;
        }

        private double updateMotionProfile(double elapsedTime) {
            if (elapsedTime <= 0) {
                return profileStartPos;
            }

            double currentPos;
            double direction = Math.signum(profileTargetPos - profileStartPos);
            double distanceCovered = Math.abs(currentVelocity * elapsedTime);
            double distanceRemaining = totalDistance - distanceCovered;

            // Check if we should start decelerating
            if (!decelerating && distanceRemaining <= decelStartDistance) {
                decelerating = true;
            }

            // Update velocity
            if (decelerating) {
                currentVelocity = Math.sqrt(2 * A_DEC * distanceRemaining);
                currentVelocity = Math.min(currentVelocity, V_MAX);
            } else {
                currentVelocity = V_MAX;
            }

            // Calculate new position
            double deltaPos = currentVelocity * elapsedTime * direction;
            currentPos = profileStartPos + deltaPos;

            // Apply final position when close enough
            if (Math.abs(profileTargetPos - currentPos) < 10) {
                return profileTargetPos;
            }
            return currentPos;
        }

        public double ARM_Control_PID(@NonNull TelemetryPacket packet /*NEW*/) {
            double target2 = PoseStorage.target2; //NEW
            packet.addLine("target2pos1:" + target2); //NEW
            double theta1_actual = Math.toRadians(target1pid + ARM1_OFFSET);
            double theta2_actual = Math.toRadians(target1pid + ARM1_OFFSET + target2 + ARM2_OFFSET);
            int arm1Pos = arm1.getCurrentPosition();
            double pid1 = controller1.calculate(arm1Pos, (int) (target1pid * ticks_in_degree_1)); //PID calculation
            double ff1 = (m1 * x1 * Math.cos(theta1_actual) + m2 * (L1 * Math.cos(theta1_actual) + x2 * Math.cos(theta2_actual))) * f1;
            return ((pid1 + ff1)) * power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            double currentTime = time.seconds();
            packet.addLine("time.seconds():" + (currentTime));
            packet.addLine("arm1pos:" + (arm1.getCurrentPosition()));
//            packet.addLine("target1pos:"+target1);
            packet.addLine("target1pos:" + (int) (target1 * ticks_in_degree_1));
            if (currentTime < runTime + waitTime/* && Math.abs(arm1.getCurrentPosition() - arm1TargetPos) > 5*/) {
//                packet.addLine("still running 1");
                if (time.seconds() > waitTime) {
                    //NEW
                    if (!startMove) {
                        PoseStorage.target1 = target1;
                        initMotionProfile(target1 * ticks_in_degree_1);
                        profileStartTime = currentTime;
                        startMove = true;
                    }
                    double elapsed = currentTime - profileStartTime;
                    double desiredPosition = updateMotionProfile(elapsed);
                    target1pid = desiredPosition / ticks_in_degree_1;
//                    updateArmProfile();
                    double power = ARM_Control_PID(packet /*new*/);
                    packet.addLine("power1:" + power);
                    arm1.setPower(power);
                }
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                return true;
            } else {
                packet.addLine("Arm 1 time:" + time.seconds());
                arm1.setPower(0);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                return false;
            }
        }
    }

    public Action liftHighBasket() {
        return new LiftTarget(highBasket);
    }

    public Action liftRung() {
        return new LiftTarget(highRung);
    }

    public Action liftRung2() {
        return new LiftTarget(highRung2);
    }

    public Action liftRung_First() {
        return new LiftTarget(highRung_First);
    }

    public Action liftWall() {
        return new LiftTarget(wall);
    }

    public Action waitLiftWallWall() {
        return new LiftTarget(wallwall);
    }

    public Action liftWall2() {
        return new LiftTarget(wall2);
    }

    public Action liftWall_First() {
        return new LiftTarget(wall_First);
    }

    public Action liftLowBasket() {
        return new LiftTarget(lowBasket);
    } //not tested i think

    public Action liftFloor() {
        return new LiftTarget(floor);
    }

    public Action liftDown() {
        return new LiftTarget(down);
    }

    public Action liftSub() {
        return new LiftTarget(sub);
    }

    public Action liftVertSub() {
        return new LiftTarget(vertSub1);
    }

    public Action liftVertFloor() {
        return new LiftTarget(vertFloor1);
    }

    public Action liftHighBasket(double waitseconds) {
        return new LiftTarget(highBasket, waitseconds);
    }

    public Action liftRung(double waitseconds) {
        return new LiftTarget(highRung, waitseconds);
    }

    public Action liftRung2(double waitseconds) {
        return new LiftTarget(highRung2, waitseconds);
    }

    public Action liftRung_First(double waitseconds) {
        return new LiftTarget(highRung_First, waitseconds);
    }

    public Action liftRung_FirstSecond(double waitseconds) {
        return new LiftTarget(highRung_First - 3, waitseconds);
    }

    public Action liftWall(double waitseconds) {
        return new LiftTarget(wall, waitseconds);
    }

    public Action waitLiftWallWall(double waitseconds) {
        return new LiftTarget(wallwall, waitseconds);
    }

    public Action liftWall2(double waitseconds) {
        return new LiftTarget(wall2, waitseconds);
    }

    public Action liftWall_First(double waitseconds) {
        return new LiftTarget(wall_First, waitseconds);
    }

    public Action liftLowBasket(double waitseconds) {
        return new LiftTarget(lowBasket, waitseconds);
    } //not tested i think

    public Action liftFloor(double waitseconds) {
        return new LiftTarget(floor, waitseconds);
    }

    public Action liftDown(double waitseconds) {
        return new LiftTarget(down, waitseconds);
    }

    public Action liftSub(double waitseconds) {
        return new LiftTarget(sub, waitseconds);
    }

    public Action liftVertSub(double waitseconds) {
        return new LiftTarget(vertSub1, waitseconds);
    }
    public Action liftVertSub_Auto(double waitseconds){
        return new LiftTarget(vertSub1_auto,waitseconds);
    }

    public Action liftVertFloor(double waitseconds) {
        return new LiftTarget(vertFloor1, waitseconds);
    }

    public Action liftHighBasket(double waitseconds, double seconds) {
        return new LiftTarget(highBasket, waitseconds, seconds);
    }

    public Action liftRung(double waitseconds, double seconds) {
        return new LiftTarget(highRung, waitseconds, seconds);
    }

    public Action liftRung2(double waitseconds, double seconds) {
        return new LiftTarget(highRung2, waitseconds, seconds);
    }
    public Action liftRung2(double waitseconds, double seconds,boolean profile) {
        return new LiftTarget(highRung2, waitseconds, seconds,1,profile);
    }

    public Action liftRung_First(double waitseconds, double seconds) {
        return new LiftTarget(highRung_First, waitseconds, seconds);
    }
    public Action liftRung_First(double waitseconds, double seconds, boolean profile) {
        return new LiftTarget(highRung_First, waitseconds, seconds,1,profile);
    }

    public Action liftWall(double waitseconds, double seconds) {
        return new LiftTarget(wall, waitseconds, seconds);
    }

    public Action liftWall2(double waitseconds, double seconds) {
        return new LiftTarget(wall2, waitseconds, seconds);
    }
    public Action liftWall2_First(double waitseconds, double seconds) {
        return new LiftTarget(wall2_First, waitseconds, seconds);
    }

    public Action liftWall_First(double waitseconds, double seconds) {
        return new LiftTarget(wall_First, waitseconds, seconds);
    }

    public Action liftLowBasket(double waitseconds, double seconds) {
        return new LiftTarget(lowBasket, waitseconds, seconds);
    } //not tested i think

    public Action liftFloor(double waitseconds, double seconds) {
        return new LiftTarget(floor, waitseconds, seconds);
    }

    public Action liftDown(double waitseconds, double seconds) {
        return new LiftTarget(down, waitseconds, seconds);
    }

    public Action liftSub(double waitseconds, double seconds) {
        return new LiftTarget(sub, waitseconds, seconds);
    }

    public Action liftVertSub(double waitseconds, double seconds) {
        return new LiftTarget(vertSub1, waitseconds, seconds);
    }

    public Action liftVertFloor(double waitseconds, double seconds) {
        return new LiftTarget(vertFloor1, waitseconds, seconds);
    }

    public Action liftFloor(double waitseconds, double seconds, double power) {
        return new LiftTarget(floor, waitseconds, seconds, power);
    }

    public Action liftSub(double waitseconds, double seconds, double power) {
        return new LiftTarget(sub, waitseconds, seconds, power);
    }

    public Action liftDown(double waitseconds, double seconds, double power) {
        return new LiftTarget(down, waitseconds, seconds, power);
    }

    public Action liftVertSub(double waitseconds, double seconds, double power) {
        return new LiftTarget(vertSub1, waitseconds, seconds, power);
    }

    public Action liftVertFloor(double waitseconds, double seconds, double power) {
        return new LiftTarget(vertFloor1, waitseconds, seconds, power);
    }

    public Action liftFloor(double waitseconds, double seconds, boolean profile) {
        return new LiftTarget(floor, waitseconds, seconds, 1, profile);
    }
}