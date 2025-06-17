package org.firstinspires.ftc.teamcode.vision;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem_Constants;
import org.firstinspires.ftc.teamcode.TunePID_MotionProfile;
import org.firstinspires.ftc.teamcode.autons.PoseStorage;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

@Config
public class ExcludePipeline extends OpenCvPipeline {
    public static int retVal = 0;
    public static boolean isBlue = true;
    List<MatOfPoint> contours = new ArrayList<>();

    public static boolean printStuff= true;
    public static double RUH = 10, RLH = 150, RS = 90, RV = 170, BH = 100, BUH = 120, BS = 70, BV = 80, YH = 15 /*10*/ /*20*/ /*15*/ /*27*/, YUH = 40 /*30*/ /*40*/ /*30*/ /*33*/, YS = 80 /*150*/ /*85*/ /*80*/ /*100*/ /*80*/ /*100*/, YV = 210 /*243*/ /*250*/ /*150*/ /*100*/ /*150*/ /*51*/, AREA_RATIO_WEIGHT = -0.4, UPPIES = .5, MIN_AREA = 2500 /*7000*/,FOR_MULT=0.85,
            FOR_CONST = 3.6;
    public static int UPPER_THRESH = 280 /*120*/, LOWER_THRESH = 20 /*60*/, YUPPER_THRESH = 240, YLOWER_THRESH = 80, KERNEL_SIZE = 2, YELLOW_KERNEL_SIZE = 2;
    public static double horizontal_offset = 6.55, camera_tilt = Math.toRadians(36), forward_offset = 20;
    private final double ticks_in_degree_1 = TunePID_MotionProfile.ticks_in_degree_1;
    private final double ticks_in_degree_2 = TunePID_MotionProfile.ticks_in_degree_2;
    private final double L1 = TunePID_MotionProfile.L1;
    private final double L2 = TunePID_MotionProfile.L2;
    private final double x1 = TunePID_MotionProfile.x1;
    private final double x2 = TunePID_MotionProfile.x2;
    private final double m1 = TunePID_MotionProfile.m1;
    private final double m2 = TunePID_MotionProfile.m2;
    private final double vertFloor1 = Subsystem_Constants.vertFloor1;
    private final double vertFloor2 = Subsystem_Constants.vertFloor2;
    double ARM1_OFFSET = TunePID_MotionProfile.ARM1_OFFSET;
    double ARM2_OFFSET = TunePID_MotionProfile.ARM2_OFFSET;
    Mat hsv = new Mat();
    Mat mask = new Mat(), mask2 = new Mat(), closedEdges = new Mat(), edges = new Mat();
    Mat kernel = new Mat();
    Mat colorMask = new Mat();
    Mat colorMask2 = new Mat();

    Mat hierarchy = new Mat();
    Mat boundingImage = new Mat(), maskedImage = new Mat();

    public static double AREA_THRESH = .6 /*.82*/, FCL = 1, UP_TOLERANCE = 0.6, DOWN_TOLERANCE = 0.8, CLASSUP_TOL = 0.5, CLASSDOWN_TOL = 0.3;
    double objectWidth = 3.5;
    double objectHeight = 1.5;

    MatOfPoint3f objectPoints = new MatOfPoint3f(
            new Point3(objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, -objectHeight / 2, 0),
            new Point3(objectWidth / 2, -objectHeight / 2, 0));
    MatOfPoint3f objectPoints1 = new MatOfPoint3f(
            new Point3(objectWidth / 2, objectHeight / 2, -1),
            new Point3(-objectWidth / 2, objectHeight / 2, -1),
            new Point3(-objectWidth / 2, -objectHeight / 2, 0),
            new Point3(objectWidth / 2, -objectHeight / 2, 0));
    MatOfPoint3f objectPoints2 = new MatOfPoint3f(
            new Point3(objectWidth / 2, -objectHeight / 2, -1),
            new Point3(-objectWidth / 2, -objectHeight / 2, 0),
            new Point3(-objectWidth / 2, objectHeight / 2, 0),
            new Point3(objectWidth / 2, objectHeight / 2, -1)
    );

    MatOfPoint3f rlObjectPoints;

    Point[] orderedRectPoints;
    Mat rvec = new Mat();
    Mat tvec = new Mat();
    MatOfPoint2f imagePoints = new MatOfPoint2f(), contour2f = new MatOfPoint2f();
    private volatile double[] center = {0, 0, 0, 0};
    Double[] camCent = {0.0 , 0.0, 0.0, 0.0};

    double objX_cam = 0;
    double objY_cam = 0;
    double objZ_cam = 0;
    int color = 0;

    /*
     * Camera Calibration Parameters
     */
    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    MatOfDouble distCoeffs = new MatOfDouble();
    RotatedRect minAreaRect;
    Telemetry telemetry;


    public ExcludePipeline(Telemetry telemetry) {
        double fx = 1647 * FCL; // Replace with your camera's focal length in pixels
        double fy = 1647 * FCL;
        double cx = 746; // Replace with your camera's principal point x-coordinate (usually image width / 2)
        double cy = 439; // Replace with your camera's principal point y-coordinate (usually image height / 2)
        cameraMatrix.put(0, 0,
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1);
        distCoeffs = new MatOfDouble(0.08642896,  0.58342025,  0.00830023,  0.00885814, -3.45247042);
        this.telemetry = telemetry;
    }


    public void resetCenter() {
        center = new double[]{0, 0, 0, 0};
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Scalar rlFilt = new Scalar(RLH, RS, RV),
                ruFilt = new Scalar(180, 255, 255),
                rllFilt = new Scalar(0, RS, RV),
                rulFilt = new Scalar(RUH, 255, 255),
                blFilt = new Scalar(BH, BS, BV),
                buFilt = new Scalar(BUH, 255, 255),
                ylFilt = new Scalar(YH, YS, YV),
                yuFilt = new Scalar(YUH, 255, 255);

        input.copyTo(boundingImage);  // More memory-efficient
        if (color == 0) {
            Core.inRange(hsv, rlFilt, ruFilt, mask);
            Core.inRange(hsv, rllFilt, rulFilt, mask2);
            Core.bitwise_or(mask, mask2, colorMask);
        } else if (color == 1)
            Core.inRange(hsv, blFilt, buFilt, colorMask);
        else if(color == 2){
            if(isBlue){
                Core.inRange(hsv, blFilt, buFilt, colorMask);
            } else {
                Core.inRange(hsv, rlFilt, ruFilt, mask);
                Core.inRange(hsv, rllFilt, rulFilt, mask2);
                Core.bitwise_or(mask, mask2, colorMask);
            }
            Core.inRange(hsv, ylFilt, yuFilt, colorMask2);
        }
        else{
            Core.inRange(hsv, ylFilt, yuFilt, colorMask);
        }

        maskedImage = new Mat();
        Core.bitwise_and(input, input, maskedImage, colorMask);
        if(color==2) {
            Core.bitwise_and(input, input, maskedImage, colorMask2);
        }

        edges = new Mat();
        // Apply Canny edge detection
        if (color != 2) {
            Imgproc.Canny(maskedImage, edges, LOWER_THRESH, UPPER_THRESH);
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(KERNEL_SIZE, KERNEL_SIZE));
            closedEdges = new Mat();
            Imgproc.dilate(edges, closedEdges, kernel);
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(KERNEL_SIZE, KERNEL_SIZE));
            Imgproc.morphologyEx(closedEdges, edges, Imgproc.MORPH_CLOSE, kernel);
        } else {
            Imgproc.Canny(maskedImage, edges, YLOWER_THRESH, YUPPER_THRESH);
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(YELLOW_KERNEL_SIZE, YELLOW_KERNEL_SIZE));
            closedEdges = new Mat();
            Imgproc.dilate(edges, closedEdges, kernel);
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(YELLOW_KERNEL_SIZE, YELLOW_KERNEL_SIZE));
            Imgproc.morphologyEx(closedEdges, edges, Imgproc.MORPH_CLOSE, kernel);

        }

        contours = new ArrayList<>();
        Imgproc.findContours(closedEdges, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        ArrayList<Double[]> colorCoords = contoursToCoords();
        if (!contours.isEmpty()) {
            Double[] centerd = matchedCoords(colorCoords, colorCoords);
            if (centerd[0] != 100) center = convertToDoubleArray(centerd);
            telemetry.addLine("we have" + contours.size() + " contours :))");
            // After setting center
            telemetry.addData("Valid Center", center[0] != 0);
            telemetry.addData("Contour Count", contours.size());
            telemetry.addData("ColorCoords Count", colorCoords.size());
        }
        else{
            telemetry.addLine("contours are empty!!");
        }
        telemetry.update();
//        if(printStuff) {
//            if (retVal == 0)
//                boundingImage.copyTo(input);
//            else if (retVal == 1)
//                maskedImage.copyTo(input);
//            else if (retVal == 2)
//                return edges;
//            else
//                return closedEdges;
//        }
//        closedEdges.release();
//        colorMask.release();
//        edges.release();
//        hsv.release();
//        mask.release();
//        mask2.release();
//        maskedImage.release();
//        hierarchy.release();
//        hierarchy.release();
//        boundingImage.release();
//        return input;
        // Replace the final return section with:
        Mat result;
        if(printStuff) {
            if (retVal == 0) {
                result = boundingImage;
            } else if (retVal == 1) {
                result = maskedImage;
            } else if (retVal == 2) {
                result = edges;
            } else {
                result = closedEdges;
            }
        } else {
            result = input;
        }

// Only release Mats not being returned
        if (result != closedEdges) closedEdges.release();
        if (result != edges) edges.release();
        if (result != maskedImage) maskedImage.release();
        if (result != hsv) hsv.release();
        if (result != mask) mask.release();
        if (result != mask2) mask2.release();
        if (result != hierarchy) hierarchy.release();
        if (result != colorMask) colorMask.release();
        if (result != colorMask2) colorMask2.release();
        if (result != boundingImage) boundingImage.release();

        return result;
    }

    double[] convertToDoubleArray(Double[] wrapperArray) {
        double[] primitiveArray = new double[wrapperArray.length];

        for (int i = 0; i < wrapperArray.length; i++) {
            primitiveArray[i] = wrapperArray[i]; // Auto-unboxing
        }

        return primitiveArray;
    }

    public synchronized void setCenter(double[] newCenter) {
        center = newCenter;
    }

    public synchronized double[] getCenter(@NonNull TelemetryPacket packet) {
        packet.put("objX_cam", objX_cam);
        packet.put("objY_cam", objY_cam);
        packet.put("objZ_cam", objZ_cam);
        packet.put("CAM X", camCent[0]);
        packet.put("CAM Y", camCent[1]);
        packet.put("CAM Z", camCent[2]);

        return center;
    }

//    public Double[] matchedCoords(ArrayList<Double[]> colorCoords, ArrayList<Double[]> allCoords) {
//        ArrayList<Double[]> matchedCenters = new ArrayList<>();
//        double minDist = 1000;
//        int coord = 0;
//
//        // Get current arm angles from your teleop class
//        double arm1Angle = Math.toRadians(vertFloor1/ticks_in_degree_1 + ARM1_OFFSET);
//        double arm2Angle = Math.toRadians(vertFloor2/ticks_in_degree_2 + ARM2_OFFSET);
//
//        // Calculate end effector position using forward kinematics
//        double endEffectorX = L1 * Math.cos(arm1Angle) + L2 * Math.cos(arm1Angle + arm2Angle);
//        double endEffectorY = L1 * Math.sin(arm1Angle) + L2 * Math.sin(arm1Angle + arm2Angle);
//
//        for (int i = 0; i < colorCoords.size(); i++) {
//            matchedCenters.add(colorCoords.get(i));
//            double[] relCent = convertToDoubleArray(colorCoords.get(i));
//
//            // Transform camera coordinates to arm base coordinates
//            // This depends on how your camera is mounted relative to the arm base
//            double armBaseX = relCent[0] * FOR_MULT - FOR_CONST; // Adjust these constants
//            double armBaseY = relCent[1] * FOR_MULT;
//
//            // Calculate distance from current end effector position
//            double dx = armBaseX - endEffectorX;
//            double dy = armBaseY - endEffectorY;
//            double dist = dx*dx + dy*dy;
//
//            if (dist < minDist) {
//                coord = matchedCenters.size() - 1;
//                minDist = dist;
//            }
//        }
//
//        if (matchedCenters.isEmpty())
//            return new Double[]{100.0, 100.0, 100.0, 100.0};
//        return matchedCenters.get(coord);
//    }
    public Double[] matchedCoords(ArrayList<Double[]> colorCoords, ArrayList<Double[]> allCoords) {
        ArrayList<Double[]> matchedCenters = new ArrayList<>();
        double minDist = 1000;
        int coord = 0;

        // Get current arm angles in radians
        double arm1Angle = Math.toRadians((PoseStorage.target1) + ARM1_OFFSET);
        double arm2Angle = Math.toRadians((PoseStorage.target2) + ARM2_OFFSET);
        double totalAngle = arm1Angle + arm2Angle;

        double arm1FloorAngle = Math.toRadians(vertFloor1) + ARM1_OFFSET;
        double arm2FloorAngle = Math.toRadians(vertFloor2) + ARM2_OFFSET;
        double totalFloorAngle = arm1FloorAngle + arm2FloorAngle;

        // Camera position relative to ARM2 joint (measure these values)
        double CAM_OFFSET_X = 10.75;  // Distance along the arm segment (inches)
        double CAM_OFFSET_Z = -0.25;  // Height offset from arm plane (inches)

        // Calculate camera position in arm base frame
        double camX = L1 * Math.cos(arm1Angle) + CAM_OFFSET_X * Math.cos(totalAngle);
        double camY = L1 * Math.sin(arm1Angle) + CAM_OFFSET_X * Math.sin(totalAngle);
        double camZ = CAM_OFFSET_Z;  // Assuming camera is at constant height

        for (int i = 0; i < colorCoords.size(); i++) {
            Double[] relCent = colorCoords.get(i).clone();
            // Camera-relative coordinates
            objX_cam = relCent[0];  // Right/Left in camera view
            objY_cam = relCent[1];  // Down/Up in camera view
            objZ_cam = relCent[2];  // Forward in camera view


            // Transform to arm base coordinates
//            double objX_base = camX +
//                    objZ_cam * Math.cos(totalAngle) -
//                    objX_cam * Math.sin(totalAngle);
            double objX_base = objX_cam + horizontal_offset;

            double objY_base = objY_cam*Math.cos(camera_tilt) - objZ_cam*Math.sin(camera_tilt) + forward_offset;

            // Calculate distance to end effector
            double endEffectorX = 0;
            double endEffectorY = L1 * Math.cos(arm1FloorAngle) + L2 * Math.cos(totalFloorAngle);

            double dx = objX_base - endEffectorX;
            double dy = objY_base - endEffectorY;
            double dist = dx*dx + dy*dy;

            if (dist < minDist) {
                coord = i;
                minDist = dist;
                // Store transformed coordinates for later use
                relCent[0] = objX_base;
                relCent[1] = objY_base;
                matchedCenters.add(relCent);
            }
        }

        if (matchedCenters.isEmpty())
            return new Double[]{100.0, 100.0, 100.0, 100.0};
        else return matchedCenters.get(matchedCenters.size()-1);
    }

    public ArrayList<Double[]> contoursToCoords() {
        ArrayList<Double[]> centers = new ArrayList<>();
        // Set acceptable aspect ratio range
        double minAspectRatio = 3.5 / 1.5 - DOWN_TOLERANCE;
        double maxAspectRatio = 3.5 / 1.5 + UP_TOLERANCE;
        // Iterate over contours
        for (MatOfPoint contour : contours) {
            // Filter out small contours based on area
            if (Imgproc.contourArea(contour) < MIN_AREA) {
                continue;
            }

            // Approximate the contour to a polygon
            contour2f = new MatOfPoint2f(contour.toArray());
            minAreaRect = Imgproc.minAreaRect(contour2f);

            if (minAreaRect.size.width != 0 && minAreaRect.size.height != 0 && Imgproc.contourArea(contour) / (minAreaRect.size.height * minAreaRect.size.width) > AREA_THRESH) {
                Point[] box = new Point[4];
                minAreaRect.points(box);
                Point[] orded = orderPoints(box);
                if(printStuff) {
                    for (int j = 0; j < 4; j++) {
                        Imgproc.line(boundingImage, box[j], box[(j + 1) % 4], new Scalar(255, 0, 0), 2);
                    }
                }
                double[] distances = {distance(orded[0], orded[1]), distance(orded[1], orded[2]), distance(orded[0], orded[2])};
                Arrays.sort(distances);
                double width = distances[1];
                double height = distances[0];
                if (height != 0) {  // Avoid division by zero
                    double aspectRatio = width / height;
//                    if (minAspectRatio <= aspectRatio && aspectRatio <= maxAspectRatio) {
                        // Draw the bounding rectangle on the image

                        double rotRectAngle = minAreaRect.angle;
                        if (minAreaRect.size.width < minAreaRect.size.height) {
                            rotRectAngle += 90;
                        }

                        // Compute the angle and store it
                        double angle = (rotRectAngle);


                        // Order the image points in the same order as object points
                        orderedRectPoints = orded;

                        int classify = 0;
                        if (aspectRatio < 3.5 / 1.5 - CLASSDOWN_TOL) {
                            classify = 1;
                        } else if (aspectRatio > 3.5 / 1.5 + CLASSUP_TOL) {
                            classify = 2;
                        }
                        if (classify == 2) {
                            rlObjectPoints = objectPoints2;
                        } else if (classify == 1) {
                            rlObjectPoints = objectPoints1;
                        } else {
                            rlObjectPoints = objectPoints;
                            orderedRectPoints = orderCorner(orderedRectPoints);
                        }
                        imagePoints = new MatOfPoint2f(orderedRectPoints);

                        boolean success = Calib3d.solvePnP(
                                rlObjectPoints, // Object points in 3D
                                imagePoints,  // Corresponding image points
                                cameraMatrix,
                                distCoeffs,
                                rvec,
                                tvec,
                                false,
                                Calib3d.SOLVEPNP_P3P
                        );
                        if(printStuff) {
                            for (int j = 0; j < 4; j++) {
                                Imgproc.line(boundingImage, box[j], box[(j + 1) % 4], new Scalar(0, 0, 255), 2);
                            }
                        }
                        if (success) {
                            // After solvePnP success
                            double[] coords = new double[3];
                            tvec.get(0, 0, coords);

// Calculate empirical distance scaling
                            double multiplia = sqrt(aspectRatio / (3.5 / 1.5));
                            if (multiplia < 1) multiplia = 1 / multiplia;

                            double areaRatio = Imgproc.contourArea(contour) /
                                    (minAreaRect.size.height * minAreaRect.size.width);
                            double distScale = 1.16 * pow(areaRatio, AREA_RATIO_WEIGHT) * multiplia;

// Apply scaling only to distance estimate
                            double distance = coords[2] * distScale;

                            camCent = new Double[]{coords[0],  // Raw camera-relative X
                                    coords[1],  // Raw camera-relative Y
                                    coords[2],  // Raw camera-relative Z
                                    angle};       // Detection angle
// Store raw coordinates - we'll transform in matchedCoords
                            centers.add(camCent);

                            if (center != null) {
//                                packet.put("CAM X", center[0]);
//                                packet.put("CAM y", center[1]);
//                                packet.put("CAM Z", center[2]);
//                                packet.put("classif", classify);
//                                packet.put("aspectRatio", aspectRatio);
//                                packet.put("area", Imgproc.contourArea(contour));
//                                packet.put("color", color);
                            }

//                        }
                    }
                }
            }
        }
        return centers;
    }


    private static double distance(Point p1, Point p2) {
        return Math.sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
    }

    public static Point[] orderCorner(Point[] pts) {
        if (pts.length != 4) {
            throw new IllegalArgumentException("Exactly four points are required.");
        }
        Point[] sortedByDistance = {pts[0], null, null};
        double d1 = distance(pts[0], pts[1]);
        double d2 = distance(pts[0], pts[2]);
        double d3 = distance(pts[0], pts[3]);
        Point[] orderedPts = new Point[4];
        if (d1 < d2 && d1 < d3) {
            //  3 0 1 2, 3 1 0 2, 2 1 0 3, 2 0 1 3
            int[] shorts = {0, 1};
            int[] longs = {2, 3};
            if (isCC(pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]]};
            } else if (isCC(pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]])) {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]]};
            } else if (isCC(pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]]};
            } else {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[1]], pts[shorts[0]], pts[longs[0]]};
            }
        } else if (d2 < d3) {
            int[] shorts = {0, 2};
            int[] longs = {1, 3};
            if (isCC(pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]]};
            } else if (isCC(pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]])) {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]]};
            } else if (isCC(pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]]};
            } else {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[1]], pts[shorts[0]], pts[longs[0]]};
            }
        } else {
            int[] shorts = {0, 3};
            int[] longs = {1, 2};
            if (isCC(pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[0]], pts[shorts[1]], pts[longs[1]]};
            } else if (isCC(pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]])) {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[0]], pts[shorts[1]], pts[longs[0]]};
            } else if (isCC(pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]])) {
                orderedPts = new Point[]{pts[longs[0]], pts[shorts[1]], pts[shorts[0]], pts[longs[1]]};
            } else {
                orderedPts = new Point[]{pts[longs[1]], pts[shorts[1]], pts[shorts[0]], pts[longs[0]]};
            }
        }


        return orderedPts;
    }

    // Helper method to calculate the dot product of two vectors
    double dotProduct(Point p1, Point p2) {
        return p1.x * p2.x + p1.y * p2.y;
    }

    // Helper method to calculate the magnitude of a vector
    double magnitude(Point p) {
        return Math.sqrt(p.x * p.x + p.y * p.y);
    }

    // Helper method to calculate the angle between two vectors represented by Points
    double calculateAngle(Point p1, Point p2) {
        double dotProd = dotProduct(p1, p2);
        double magP1 = magnitude(p1);
        double magP2 = magnitude(p2);
        double cosTheta = dotProd / (magP1 * magP2);
        return Math.acos(cosTheta) * (180.0 / Math.PI); // Return the angle in degrees
    }

    public static Point[] orderPoints(Point[] pts) {
        if (pts.length != 4) {
            throw new IllegalArgumentException("Exactly four points are required.");
        }

        // Calculate the center of the frame
        double centerX = 320;
        double centerY = 240;
        Point center = new Point((int) centerX, (int) centerY);

        // Calculate distances from each point to the center
        double[] distances = new double[4];
        for (int i = 0; i < 4; i++) {
            distances[i] = distance(center, pts[i]);
        }

        // Sort points by proximity to center
        Point[] sortedByDistance = Arrays.copyOf(pts, 4);
        Arrays.sort(sortedByDistance, Comparator.comparingDouble(p -> distance(center, p)));

        // Start with the two closest points
        Point[] orderedPts = new Point[4];
        orderedPts[0] = sortedByDistance[0]; // Closest point
        orderedPts[1] = sortedByDistance[1]; // Second closest point

        // Remaining points
        Point thirdPoint = sortedByDistance[2];
        Point fourthPoint = sortedByDistance[3];

        // Determine the order of the remaining points in clockwise manner
        if (isCC(thirdPoint, orderedPts[0], orderedPts[1], fourthPoint)) {
            orderedPts = new Point[]{thirdPoint, orderedPts[0], orderedPts[1], fourthPoint};
        } else if (isCC(thirdPoint, orderedPts[1], orderedPts[0], fourthPoint)) {
            orderedPts = new Point[]{thirdPoint, orderedPts[1], orderedPts[0], fourthPoint};

        } else if (isCC(fourthPoint, orderedPts[1], orderedPts[0], thirdPoint)) {
            orderedPts = new Point[]{fourthPoint, orderedPts[1], orderedPts[0], thirdPoint};

        } else {
            orderedPts = new Point[]{fourthPoint, orderedPts[0], orderedPts[1], thirdPoint};

        }

        return orderedPts;
    }


    public static boolean isCC(Point p1, Point p2, Point p3, Point p4) {
        return isCCW(p1, p2, p3) && isCCW(p1, p3, p4) &&
                isCCW(p2, p3, p4) && isCCW(p1, p2, p4);
    }

    private static boolean isCCW(Point a, Point b, Point c) {
        // Calculate the cross product of vector AB and AC
        double crossProduct = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        return crossProduct > 0; // Returns true if the points are in counter-clockwise order
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat, String color) {
        Scalar colorScalar = getColorScalar(color);

        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        rect.center.x - 50,  // x anchor point
                        rect.center.y + 25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1, // Font size
                colorScalar, // Font color
                1); // Font thickness
    }


    static Scalar getColorScalar(String color) {
        switch (color) {
            case "Blue":
                return new Scalar(0, 0, 255);
            case "Yellow":
                return new Scalar(255, 255, 0);
            default:
                return new Scalar(255, 0, 0);
        }
    }

    public void setColor(int color) {
        this.color = color;
    }

    public int getColor() {
        return color;
    }
}