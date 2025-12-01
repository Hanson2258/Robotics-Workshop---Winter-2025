package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

/**
 * This program allows you to cycle through the different filters in a EasyOpenCV Pipeline.
 *
 * To use, press A on Gamepad 1 to toggle to the next filter. Please note that you will not be able
 * to cycle to the next one while in the camera stream. Return to the main interface (where you can
 * see telemetry) to cycle.
 *
 * It currently detects for a colour blob.
 */
@TeleOp(name="OpenCV Colour Example", group="OpenCV")
public class OpenCVColourExample extends OpMode
{
    // Variable declaration
    OpenCvCamera camera;
    ColourBlobPipeline colourBlobPipeline;
    long lastButtonPress = 0;

    public static Scalar PURPLE_HSV_RANGE_LOW = new Scalar(120.0, 60.0, 0.0);
    public static Scalar PURPLE_HSV_RANGE_HIGH = new Scalar(190.0, 255.0, 255.0);
    public static Scalar GREEN_HSV_RANGE_LOW = new Scalar(70.0, 60.0, 25.0);
    public static Scalar GREEN_HSV_RANGE_HIGH = new Scalar(110.0, 255.0, 255.0);

    /**
     * Stages (filters) in this EasyOpenCV Pipeline.
     */
    enum Stage
    {
        RAW_IMAGE,
        RAW_IMAGE_TO_HSV,
        THRESHOLD,
        DILATE,
        ERODE,
        CONTOURS_OVERLAY_ON_FRAME,
        CONTOURS_AFTER_REMOVING_INNER_CONTOUR,
        BOUNDING_BOX
    }

    // Pipeline Stage Variable Declaration
    private static Stage stageToRenderToViewport = Stage.RAW_IMAGE;
    private final Stage[] stages = Stage.values();
    int currentStageNum = stageToRenderToViewport.ordinal();


    /**
     * Sets up Pipeline and Camera Stream
     */
    @Override
    public void init()
    {
        colourBlobPipeline = new ColourBlobPipeline();

        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(colourBlobPipeline);
            }

            @Override public void onError(int errorCode) {
                telemetry.addData("Failed to open camera due to error code", errorCode);
                telemetry.update();
            }
        });
    }

    /**
     * Infinite loop until hit run to allow cycling through stages.
     */
    @Override
    public void init_loop()
    {
        telemetry.addData("Blob location is", ColourBlobPipeline.getBlobLocation());
        telemetry.addData("Current filter", ColourBlobPipeline.getCurrentStage());

        if (gamepad1.a && (System.currentTimeMillis() - lastButtonPress) > 200)
        {
            int nextStageNum = currentStageNum + 1;
            lastButtonPress = System.currentTimeMillis();

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];

            currentStageNum = nextStageNum;
        }
        else if (gamepad1.b && (System.currentTimeMillis() - lastButtonPress) > 200)
        {
            int previousStageNum = currentStageNum - 1;
            lastButtonPress = System.currentTimeMillis();

            if(previousStageNum < 0)
            {
                previousStageNum = stages.length - 1;
            }

            stageToRenderToViewport = stages[previousStageNum];

            currentStageNum = previousStageNum;
        }

        telemetry.update();
    }

    @Override public void loop() {}


    /**
     * EasyOpenCV Pipeline to detect Colour Blob.
     */
    static class ColourBlobPipeline extends OpenCvPipeline
    {
        public static int CAMERA_WIDTH = 320;

        // Declaring which is defined as left and right of screen
        public static double LEFT_X  = 0.25 * (double) CAMERA_WIDTH;
        public static double RIGHT_X = 0.75 * (double) CAMERA_WIDTH;

        // Variable to store blob location
        public static String blobLocation;

        // Dilating the thresholded area to remove small bits that may be thought as separate.
        public static int DILATE_PASSES = 2;

        // The more Erode passes you do, the more noise will be removed (but if you do too much, you
        // may erode your object. A good balance is key - Remember you don't have to remove all
        // noise. Part of the pipeline is to select the biggest blob, removing noise simply reduces
        // computational time.
        public static int ERODE_PASSES = 2;

        // Erode setup
        private static final Point CV_ANCHOR        = new Point(-1, -1);
        private static final Scalar CV_BORDER_VALUE = new Scalar(-1);
        private static final int CV_BORDER_TYPE     = Core.BORDER_CONSTANT;

        // Colour of the Bounding Rectangle
        public static volatile Scalar BOUNDING_RECTANGLE_COLOR = new Scalar(0, 255, 0);

        // Range for a Colour Blob
        public static Scalar targetHSVRangeLow = GREEN_HSV_RANGE_LOW;
        public static Scalar targetHSVRangeHigh = GREEN_HSV_RANGE_HIGH;

        // Mat object initialization
        private final Mat hsvMat    = new Mat(),
                hierarchy           = new Mat(),
                hierarchyExternal   = new Mat(),
                cvDilateKernel      = new Mat(),
                cvErodeKernel       = new Mat(),
                thresholdOutput     = new Mat(),
                dilateOutput        = new Mat(),
                erodeOutput         = new Mat(),
                contoursOutput      = new Mat(),
                bigContoursOutput   = new Mat(),
                output              = new Mat();

        /**
         * Pipeline to process the frame.
         *
         * @param input from the camera.
         * @return current stage of pipeline (normally you only return the final stage)
         */
        @Override
        public Mat processFrame(Mat input)
        {
            // Convert color to HSV
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Checks if the image is in range
            Core.inRange(hsvMat, targetHSVRangeLow, targetHSVRangeHigh, thresholdOutput);

            // Dilate to combine
            Imgproc.dilate(
                    thresholdOutput,
                    dilateOutput,
                    cvDilateKernel,
                    CV_ANCHOR,
                    DILATE_PASSES,
                    CV_BORDER_TYPE,
                    CV_BORDER_VALUE);

            // Erode to remove noise
            Imgproc.erode(
                    dilateOutput,
                    erodeOutput,
                    cvErodeKernel,
                    CV_ANCHOR,
                    ERODE_PASSES,
                    CV_BORDER_TYPE,
                    CV_BORDER_VALUE);

            // Finds the contours of the image
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(erodeOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // Filter out all inner contours
            List<MatOfPoint> externalContours = new ArrayList<>();
            Imgproc.findContours(
                    erodeOutput,
                    externalContours,
                    hierarchyExternal,
                    Imgproc.RETR_EXTERNAL,
                    Imgproc.CHAIN_APPROX_SIMPLE
            );

            // Compute bounding boxes only from EXTERNAL contours
            Rect biggestBoundingBox = new Rect(0, 0, 0, 0);

            for (MatOfPoint contour : externalContours)
            {
                Rect rect = Imgproc.boundingRect(contour);

                if (rect.area() > biggestBoundingBox.area())
                {
                    biggestBoundingBox = rect;
                }
            }
            if (biggestBoundingBox.area() != 0)
            { // If blob is detected
                if (biggestBoundingBox.x < LEFT_X)
                { // Check to see if the bounding box is on the left 25% of the screen
                    blobLocation = "LEFT";
                } else if (biggestBoundingBox.x > RIGHT_X)
                { // Check to see if the bounding box is on the right 25% of the screen
                    blobLocation = "RIGHT";
                } else
                { // If it isn't left or right and the blob is detected it must be in the center
                    blobLocation = "CENTER";
                }
            } else
            { // If blob is not detected
                blobLocation = "NONE";
            }

            // Select stage to return to viewport
            switch (stageToRenderToViewport)
            {
                default:
                case RAW_IMAGE:
                {
                    return textOverlay(input, "Original");
                }

                case RAW_IMAGE_TO_HSV:
                {
                    return textOverlay(hsvMat, "HSV");
                }

                case THRESHOLD:
                {
                    return textOverlay(thresholdOutput, "Threshold");
                }

                case DILATE:
                {
                    return textOverlay(dilateOutput, "Dilate");
                }

                case ERODE:
                {
                    return textOverlay(erodeOutput, "Erode");
                }

                case CONTOURS_OVERLAY_ON_FRAME:
                {
                    input.copyTo(contoursOutput);
                    Imgproc.drawContours(contoursOutput, contours, -1, new Scalar(0, 255, 0), 1, 8);

                    return textOverlay(contoursOutput, "Contours");
                }

                case CONTOURS_AFTER_REMOVING_INNER_CONTOUR:
                {
                    input.copyTo(bigContoursOutput);
                    Imgproc.drawContours(bigContoursOutput, externalContours, -1, new Scalar(0, 255, 0), 1, 8);

                    return textOverlay(bigContoursOutput, "Ext contours");
                }

                case BOUNDING_BOX:
                {
                    Imgproc.rectangle(input, biggestBoundingBox, BOUNDING_RECTANGLE_COLOR);
                    return textOverlay(input, "Bounding");
                }
            }
        }

        /**
         * Get location of blob.
         * @return A string with the location of blob.
         */
        public static String getBlobLocation()
        {
            return blobLocation;
        }

        /**
         * Name of the current stage.
         * @return string of the current stage name.
         */
        private static String getCurrentStage()
        {
            return stageToRenderToViewport.name();
        }

        private Mat textOverlay(Mat input, String label){
            // text properties
            int fontFace = Imgproc.FONT_HERSHEY_SIMPLEX;
            double fontScale = 0.5;
            int thickness = 1;
            int[] baseLine = new int[1];

            // measure text
            Size textSize = Imgproc.getTextSize(label, fontFace, fontScale, thickness, baseLine);

            // desired offsets from the right and top edges (pixels)
            int rightPadding = 10;
            int topPadding = 10;

            // compute origin so text is anchored at top-right
            int originX = (int) Math.round(input.cols() - rightPadding - textSize.width);
            int originY = (int) Math.round(topPadding + textSize.height);

            // clamp inside image
            originX = Math.max(0, originX);
            originY = Math.max((int)textSize.height, Math.min(input.rows(), originY));

            // optional background rectangle for readability
            Point rectTl = new Point(originX - 6, originY - textSize.height - 6); // top-left of rect
            Point rectBr = new Point(originX + textSize.width + 6, originY + 6);   // bottom-right of rect
            Imgproc.rectangle(input, rectTl, rectBr, new Scalar(0,0,0), -1);

            // draw text (origin is baseline-left)
            Imgproc.putText(input,
                    label,
                    new Point(originX, originY),
                    fontFace,
                    fontScale,
                    new Scalar(255, 255, 255),
                    thickness);

            return input;
        }
    }
}