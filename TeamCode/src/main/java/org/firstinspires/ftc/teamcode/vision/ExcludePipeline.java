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
    boolean chamberPos;

    public static boolean printStuff= false;
    public static double RUH = 10, RLH = 150, RS = 90, RV = 200 /*175*/ /*210*/, BH = 110 /*100*/, BUH = 130 /*120*/, BS = 70, BV = 190 /*250*/ /*80*/, YH = 15 /*10*/ /*20*/ /*15*/ /*27*/, YUH = 40 /*30*/ /*40*/ /*30*/ /*33*/, YS = 80 /*150*/ /*85*/ /*80*/ /*100*/ /*80*/ /*100*/, YV = 210 /*243*/ /*250*/ /*150*/ /*100*/ /*150*/ /*51*/, AREA_RATIO_WEIGHT = -0.4, UPPIES = .5, MIN_AREA = 2500 /*7000*/;
    public static int UPPER_THRESH = 280 /*120*/, LOWER_THRESH = 20 /*60*/, YUPPER_THRESH = 240, YLOWER_THRESH = 80, KERNEL_SIZE = 2, YELLOW_KERNEL_SIZE = 2;
    public static double horizontal_offset, camera_tilt, forward_offset, inchPerPixel_x, inchPerPixel_y,k,MIN_DIST = 36;
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

    MatOfPoint2f contour2f = new MatOfPoint2f();
    private volatile double[] center = {0, 0, 0, 0};
    Double[] camCent = {0.0 , 0.0, 0.0, 0.0};

    double objX_cam = 0;
    double objY_cam = 0;
    double objZ_cam = 0;
    double objX_base = 0;
    double objY_base = 0;
    int color = 0;

    /*
     * Camera Calibration Parameters
     */
    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    MatOfDouble distCoeffs = new MatOfDouble();
    RotatedRect minAreaRect;
    Telemetry telemetry;


    public ExcludePipeline(Telemetry telemetry, boolean chamberPos) {
        if (chamberPos) {horizontal_offset = -9.5 /*5*/ /*-11.5*/; camera_tilt = Math.toRadians(36); forward_offset = 0; inchPerPixel_x = 17.0/640; inchPerPixel_y = 18.0/480; k = Math.log(14.0/2)/Math.log(14.17/3.68);} //30; //45; //50 /*30*/;}
        else {horizontal_offset = 7 /*-8*/ /*5*/ /*-9.5*/; camera_tilt = Math.toRadians(0); forward_offset = -6.5 /*-2.5*/ /*-3*/; inchPerPixel_x = 10.5/640; inchPerPixel_y = 7.0/480; k=1;
        } //30; //45; //50 /*30*/;}
//        double fx = 1647 * FCL; // Replace with your camera's focal length in pixels
//        double fy = 1647 * FCL;
//        double cx = 746; // Replace with your camera's principal point x-coordinate (usually image width / 2)
//        double cy = 439; // Replace with your camera's principal point y-coordinate (usually image height / 2)
//        cameraMatrix.put(0, 0,
//                fx, 0, cx,
//                0, fy, cy,
//                0, 0, 1);
//        distCoeffs = new MatOfDouble(0.08642896,  0.58342025,  0.00830023,  0.00885814, -3.45247042);
        this.telemetry = telemetry;
        this.chamberPos = chamberPos;
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
            if (printStuff){
            telemetry.addLine("we have" + contours.size() + " contours :))");
            // After setting center
            telemetry.addData("Valid Center", center[0] != 0);
            telemetry.addData("Contour Count", contours.size());
            telemetry.addData("ColorCoords Count", colorCoords.size());}
        }
        else if (printStuff){
            telemetry.addLine("contours are empty!!");
        }
        if (printStuff) telemetry.update();
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
        if (printStuff){
        packet.put("objX_cam", objX_cam);
        packet.put("objY_cam", objY_cam);
        packet.put("objZ_cam", objZ_cam);
        packet.put("CAM X", camCent[0]);
        packet.put("CAM Y", camCent[1]);
        packet.put("CAM Z", camCent[2]);
        packet.put("angle", camCent[3]);
        packet.put("objX_base", objX_base);
        packet.put("objY_base", objY_base);}
        return center;
    }
    public Double[] matchedCoords(ArrayList<Double[]> colorCoords, ArrayList<Double[]> allCoords) {
        ArrayList<Double[]> matchedCenters = new ArrayList<>();
        double minDist = MIN_DIST;
        int coord = 0;

        for (int i = 0; i < colorCoords.size(); i++) {
            Double[] relCent = colorCoords.get(i).clone();
            // Camera-relative coordinates
            objX_cam = relCent[0];  // Right/Left in camera view
            objY_cam = relCent[1];  // Down/Up in camera view
            objZ_cam = relCent[2];  // Forward in camera view
            double angle = relCent[3];

            if (chamberPos) {
                if (PoseStorage.grabColorPose.position.x < 5) horizontal_offset += 6;
                objX_base = objX_cam + horizontal_offset;
            }
            else
                objX_base = -objX_cam + horizontal_offset;
            if (chamberPos)
                objY_base = 18.0*Math.pow((-objY_cam+18.0)/18.0,k) + forward_offset;
            else
                objY_base = 7.0*Math.pow((objY_cam)/7.0,k) + forward_offset;
            // Calculate distance to end effector
            double endEffectorX = 0;
            double endEffectorY;
            if (chamberPos) {
                endEffectorY = PoseStorage.grabColorPose.position.y + 45;
            }
            else{
                endEffectorY = PoseStorage.grabYellowPose.position.y - 94;
            }
            double dx = objX_base - endEffectorX;
            double dy = objY_base - endEffectorY;
            double dist = dx*dx + dy*dy;

            if (dist < minDist && angle > 50 && angle < 120) {
                coord = i;
                minDist = dist;
                // Store transformed coordinates for later use
                relCent[0] = objX_base;
                relCent[1] = objY_base;
                matchedCenters.add(relCent);
            }
        }

        if (matchedCenters.isEmpty()){
            return new Double[]{100.0, 100.0, 100.0, 100.0};}
        else {
            return matchedCenters.get(matchedCenters.size()-1);
        }
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
//                  if (minAspectRatio <= aspectRatio && aspectRatio <= maxAspectRatio) {
                        // Draw the bounding rectangle on the image

                    double rotRectAngle = minAreaRect.angle;
                    if (minAreaRect.size.width < minAreaRect.size.height) {
                        rotRectAngle += 90;
                    }

                    // Compute the angle and store it
                    double angle = (rotRectAngle);

                    if(printStuff) {
                        for (int j = 0; j < 4; j++) {
                            Imgproc.line(boundingImage, box[j], box[(j + 1) % 4], new Scalar(0, 0, 255), 2);
                        }
                    }
                    double[] coords = new double[3];
                    coords[0] = (orded[0].x+orded[1].x+orded[2].x+orded[3].x)/4.0 * inchPerPixel_x;
                    coords[1] = (orded[0].y+orded[1].y+orded[2].y+orded[3].y)/4.0 * inchPerPixel_y;
                    coords[2] = (1000);

                    camCent = new Double[]{coords[0],  // Raw camera-relative X
                            coords[1],  // Raw camera-relative Y
                            coords[2],  // Raw camera-relative Z
                            angle};       // Detection angle
// Store raw coordinates - we'll transform in matchedCoords
                    centers.add(camCent);
                }
            }
        }
        return centers;
    }


    private static double distance(Point p1, Point p2) {
        return Math.sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
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