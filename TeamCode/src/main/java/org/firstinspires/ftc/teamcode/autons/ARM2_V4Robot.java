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


/*
* Motion profiling moves the PID controller's reference point smoothly.
* Motion profiling maps out what position the motor should be at at a specific time.
* This creates intermediate target positions in between the start and target positions
* Without motion profiling: too much acceleration can cause slip and inaccuracies.
*/


public class ARM2_V4Robot {
    private DcMotor arm2;              // Second arm motor
    private PIDController controller2; // PID controller for ARM2

    // PIDF constants for ARM2
    double p2 = TunePID_MotionProfile.p2, i2 = TunePID_MotionProfile.i2, d2 = TunePID_MotionProfile.d2;
    double f2 = TunePID_MotionProfile.f2; // Feedforward gain

    // Conversion factor from ticks to degrees
    private final double ticks_in_degree_2 = TunePID_MotionProfile.ticks_in_degree_2;

    // Length, COM, mass values for feedforward calculation (not performed in arm2)
    private final double L1 = TunePID_MotionProfile.L1;
    private final double L2 = TunePID_MotionProfile.L2;
    private final double x1 = TunePID_MotionProfile.x1;
    private final double x2 = TunePID_MotionProfile.x2;
    private final double m1 = TunePID_MotionProfile.m1;
    private final double m2 = TunePID_MotionProfile.m2;

    // Target positions for different scoring+intaking levels from Subsystem_Constants
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
    private double arm2StartPos;              // Starting encoder pos when motion profiling begins
    private double arm2TargetPos;             // Target encoder position for motion profile
    private double arm2DDec;                  // Distance when deceleration starts (not used directly here)
    private boolean arm2Decelerating = false; // Flag if currently decelerating

    // Motion profile constants: max velocity, deceleration, loop timing (ticks/second)
    private final double V_MAX = TunePID_MotionProfile.V_MAX; // Encoder ticks/sec (≈ 100° /sec if 1 tick/degree)
    private final double A_DEC = TunePID_MotionProfile.A_DEC; // Rate of deceleration // Ticks/sec² (adjust for smooth stopping)
    private final double LOOP_TIME = TunePID_MotionProfile.LOOP_TIME; // 20ms (typical FTC loop time)

    double ARM1_OFFSET = TunePID_MotionProfile.ARM1_OFFSET; // Offsets to adjust angle measurements
    double ARM2_OFFSET = TunePID_MotionProfile.ARM2_OFFSET;

    // Constructor initializing ARM2
    public ARM2_V4Robot(HardwareMap hardwareMap) {
        arm2 = hardwareMap.get(DcMotor.class, "ARM2");
        arm2.setDirection(DcMotor.Direction.REVERSE); //CHANGE BACK TO DCMOTORSIMPLE IF SOMETHING DOESN'T WORK
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder position to zero
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Runs motor without encoder for velocity control
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller2 = new PIDController(p2, i2, d2); // Initialize PID controller
    }

    // Action class moving ARM2 to a target position
    public class LiftTarget implements Action {
        ElapsedTime time = new ElapsedTime(); // Timer tracks elapsed time arm is in action
        boolean start;
        double target2;                       // Target ARM2 position in degrees
        double target2pid;                    // Target position updated by motion profiling for PID controller
        double waitTime;                      // Wait time (seconds) before starting movement
        double runTime;                       // Maximum time to run this action
        boolean startMove;
        double power = 1;                     // Motor power scaling
        boolean profile = true;               // Whether to use motion profiling or just PID
        private double profileStartTime;
        private double profileStartPos;
        private double profileTargetPos;
        private double totalDistance;         // total distance to move (ticks)
        private double decelStartDistance;    // Distance to start deceleration
        private boolean decelerating;         // Whether currently decelerating
        private double currentVelocity;       // Current velocity

        public LiftTarget(double pos) {
            this(pos, 0, 2, 1, true);
            /*
            target2 = pos;
            start = false;
            startMove = false;
            waitTime = 0;
            runTime = 2;
             */
        }

        public LiftTarget(double pos, double tim) { // with wait
            this(pos, tim, 2, 1, true);
            /*
            target2 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = 2;
             */
        }

        public LiftTarget(double pos, double tim, double runtime) { // with wait and runtime
            this(pos, tim, runtime, 1, true);
            target2 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
        }

        public LiftTarget(double pos, double tim, double runtime, double pwr) { // with wait, runtime, and power
            target2 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
            power = pwr;
        }

        public LiftTarget(double pos, double tim, double runtime, double pwr, boolean prof) { // with wait, runtime, power, and profiling
            target2 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
            power = pwr;
            profile = prof;
        }

        // Initialize motion profiling at start of a move
        private void initMotionProfile(double targetPos) {
            profileStartPos = arm2.getCurrentPosition();
            profileTargetPos = targetPos;
            totalDistance = Math.abs(profileTargetPos - profileStartPos); // Distance to move
            decelStartDistance = (V_MAX * V_MAX) / (2 * A_DEC);           // Distance needed to decelerate
            decelerating = (totalDistance <= decelStartDistance);         // If total distance is small, start decelerating immediately
            currentVelocity = 0;                                          // Starting velocity at 0 to ramp up
        }

        // Update target postion based on elapsed time and motion profile
        private double updateMotionProfile(double elapsedTime) {
            if (elapsedTime <= 0) {
                return profileStartPos; // If time hasn't started, current position is the same as start position
            }

            double currentPos;
            double direction = Math.signum(profileTargetPos - profileStartPos); // +1 or -1 depending on direction
            double distanceCovered = Math.abs(currentVelocity * elapsedTime);   // distance traveled so far
            double distanceRemaining = totalDistance - distanceCovered;         // Remaining distance

            // Check if we should start decelerating
            if (!decelerating && distanceRemaining <= decelStartDistance) {
                decelerating = true;
            }

            // Adjusting velocity: accelerate up to V_MAX, then decelerate smoothly (square root based)
            if (decelerating) {
                currentVelocity = Math.sqrt(2 * A_DEC * distanceRemaining);
                currentVelocity = Math.min(currentVelocity, V_MAX);
            } else {
                currentVelocity = V_MAX; // Full speed while accelerating/cruising
            }

            // Calculate new current position
            double deltaPos = currentVelocity * elapsedTime * direction;
            currentPos = profileStartPos + deltaPos;

            // Snap to final position when close enough
            if (Math.abs(profileTargetPos - currentPos) < 10) {
                return profileTargetPos;
            }
            return currentPos;
        }

        // PID control loop for ARM2 using target2pid value made by profile and ff value
        public double ARM_Control_PID(@NonNull TelemetryPacket packet) {
            double target1 = PoseStorage.target1; //NEW
            packet.addLine("target1pos2: " + target2pid); //NEW
            double theta1_actual = Math.toRadians(target1 + ARM1_OFFSET);
            double theta2_actual = Math.toRadians(target1 + ARM1_OFFSET + target2pid + ARM2_OFFSET); // combined angles for feedforward
            int arm2Pos = arm2.getCurrentPosition();
            double pid2 = controller2.calculate(arm2Pos, (int)(target2pid*ticks_in_degree_2)); // PID output
            double ff2 = m2 * x2 * Math.cos(theta2_actual) * f2;                               // Feedforward to compensate
            return (pid2 + ff2) * power;
        }

        // Main loop running the LiftTarget Action
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            double currentTime = time.seconds();

            // Telemetry lines
            packet.addLine("time.seconds():"+(time.seconds()));
            packet.addLine("arm2pos:"+(arm2.getCurrentPosition()));
            packet.addLine("target2pos:"+(int)(target2*ticks_in_degree_2));

            // Run action while time is less than the wait and allowed run time
            if (time.seconds() < runTime + waitTime/* && Math.abs(arm2.getCurrentPosition() - arm2TargetPos) > 5*/) {
                if (time.seconds() > waitTime) {
                    if (!startMove){
                        PoseStorage.target2 = target2; // Store target globally to allow smooth tele-op transition
                        initMotionProfile(target2 * ticks_in_degree_2); // Initialize motion profile for this movement
                        profileStartTime = currentTime;
                        startMove = true;
                    }
                    double elapsed = currentTime - profileStartTime;
                    double desiredPosition = updateMotionProfile(elapsed); // Get the profiled position for this time.
                    target2pid = desiredPosition / ticks_in_degree_2;      // Converting back to degrees for our PID controller

                    double power = ARM_Control_PID(packet /*new*/); // Calculate motor power with PIDF
                    packet.addLine("power2:" + power);
                    arm2.setPower(power); // Set motor power
                }
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                return true; // Action still running
            } else {
                packet.addLine("Arm 2 time: " + time.seconds());
                arm2.setPower(0); // Stop motor after time ends
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                return false; // Action finished
            }
        }
    }

    // Helper methods for common preset positions
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
    public Action liftRung2(double waitseconds, double seconds,boolean profile) {return new LiftTarget(highRung2_2,waitseconds,seconds,1,profile);}
    public Action liftRung2_First(double waitseconds, double seconds) {return new LiftTarget(highRung2_First,waitseconds,seconds);}
    public Action liftRung2First(double waitseconds, double seconds) {return new LiftTarget(highRung2_2+0.3,waitseconds,seconds);}
    public Action liftRung2First(double waitseconds, double seconds,boolean profile) {return new LiftTarget(highRung2_2+0.3,waitseconds,seconds,1,profile);}
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
    public Action liftFloor(double waitseconds, double seconds, boolean profile) {return new LiftTarget(floor2,waitseconds,seconds, 1, profile);}
}