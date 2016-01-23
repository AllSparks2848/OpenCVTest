
package org.usfirst.frc.team2850.robot;

import java.io.File;
import java.util.ArrayList;

import org.apache.commons.math3.stat.regression.SimpleRegression;
//import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.opencv.core.Core;
import org.opencv.core.Core.MinMaxLocResult;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser chooser;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    VideoCapture video;
    Joystick xbox;
    public void robotInit() {
    	System.load("/usr/local/lib/lib_OpenCV/java/libopencv_java2410.so");
//        System.out.println("Welcome to OpenCV " + Core.VERSION);
//        Mat m = new Mat(5, 10, CvType.CV_8UC1, new Scalar(0));
//        System.out.println("OpenCV Mat: " + m);
//        Mat mr1 = m.row(1);
//        mr1.setTo(new Scalar(1));
//        Mat mc5 = m.col(5);
//        mc5.setTo(new Scalar(5));
//        System.out.println("OpenCV Mat data:\n" + m.dump());
        
        try {
        video = new VideoCapture();
        video.open("http://10.28.60.20/mjpg/video.mjpg");
        }
        catch(CvException e) {
        	System.out.println(e.getMessage());
        }
        //Timer.delay(0.);
        video.grab();
        xbox = new Joystick(0);
        
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    	autoSelected = (String) chooser.getSelected();
//		autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	switch(autoSelected) {
    	case customAuto:
        //Put custom auto code here   
            break;
    	case defaultAuto:
    	default:
    	//Put default auto code here
            break;
    	}
    }

    /**
     * This function is called periodically during operator control
     */
    //boolean wasapressed = false;
    int framenum = 0;
    public void teleopPeriodic() {
        if(!video.isOpened()) {
        	System.out.println("Lost connection");
        	video.release();
        	Timer.delay(0.05);
            video.open("http://10.28.60.20/mjpg/video.mjpg");
            System.out.println("Refreshing connection");
        }
        video.grab();
        Timer.delay(0.1);
        if(xbox.getRawButton(1)) {
        	Mat image = new Mat();  
        	if(video.retrieve(image)){
        		long time = System.currentTimeMillis();
        		Mat hsv = new Mat();
        		Imgproc.cvtColor(image, hsv, Imgproc.COLOR_BGR2HSV);
        		ArrayList<Mat> channels = new ArrayList<Mat>();
        		Core.split(hsv, channels);
        		Mat hue = new Mat();
        		Mat value = new Mat();
        		Imgproc.threshold(channels.get(0), hue, 125, 255, Imgproc.THRESH_BINARY);
        		Imgproc.threshold(hue, hue, 150, 255, Imgproc.THRESH_TOZERO);
          		Imgproc.threshold(channels.get(2), value, 100, 255, Imgproc.THRESH_BINARY);	
        		Mat filtered = new Mat(image.rows(), image.cols(), CvType.CV_8UC1);
        		Core.min(hue, value, filtered);
        		filtered.convertTo(filtered, CvType.CV_8UC1);
        		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        		Mat hierarchy = new Mat();
        		Imgproc.findContours(filtered, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        		if(contours.size() > 0) {
	        		double maxarea = 0;
	        		int maxindex = 0;
	        		for(int i = 0; i < contours.size(); i++) {
	        			double area = Imgproc.contourArea(contours.get(i));
	        			if(area > maxarea) {
	        				maxarea = area;
	        				maxindex = i;
	        			}
	        		}
	        		
	        		Rect aabb = Imgproc.boundingRect(contours.get(maxindex));
	        		aabb.set(new double[]{aabb.x, aabb.y, aabb.width, aabb.height});
	        		int minx = aabb.x;
	        		int maxx = aabb.x+aabb.width;
	        		int miny = aabb.y;
	        		int maxy = aabb.y+aabb.height;
	        		
	        		Mat result = filtered.submat(aabb);
	        		
	        		if(Core.countNonZero(result) > 0) {
	        			Mat nonzeroes = new Mat();
	            		Core.findNonZero(result, nonzeroes);
	            		//System.out.println(nonzeroes.rows() + " " + nonzeroes.cols() + " " + nonzeroes.get(0,0).length);
		        		ArrayList<Point> left = new ArrayList<Point>();
		        		ArrayList<Point> right = new ArrayList<Point>();
		        		ArrayList<Point> south = new ArrayList<Point>();
		        		
		        		result.convertTo(result, CvType.CV_16UC1);
		        		Mat mask = Mat.zeros(result.rows(), result.cols(), CvType.CV_8UC1);
		        		for(int i = 0; i < nonzeroes.rows(); i++) {
		        			int y = (int) nonzeroes.get(i, 0)[1];
		        			int x = (int) nonzeroes.get(i, 0)[0];
		        			//System.out.println(x+ " " + y);
		        			result.put(y, x,1+ x + y);
		        			mask.put(y, x, 1);
		        			//System.out.println(mask.get(y,x)[0] + " " + x + " " + y);
		        		}
		        		
		        		for(int y = 0; y < result.rows(); y++) {
		        			Mat row = result.row(y);
		        			Mat rowmask = mask.row(y);
		        			MinMaxLocResult minmaxresult = Core.minMaxLoc(row, rowmask);
		        			
		        			if(minmaxresult.maxVal > 0)right.add(new Point(minmaxresult.maxLoc.x + minx, y + miny));
		        			if(minmaxresult.minVal > 0)left.add(new Point(minmaxresult.minLoc.x+ minx, y+ miny));
		        			//System.out.println((minmaxresult.minLoc.x) + " " + (y) + " " + minmaxresult.minVal);
		        			
		        		}
		        		
		        		for(int x = 0; x < result.cols(); x++) {
		        			Mat col = result.col(x);
		        			Mat colmask = mask.col(x);
		        			MinMaxLocResult minmaxresult = Core.minMaxLoc(col, colmask);
		        			if(minmaxresult.maxVal > 0)south.add(new Point(x+ minx, minmaxresult.maxLoc.y + miny));
		        			
		        		}
		        		//System.out.println(left.size() + " " + right.size() + "" + south.size());
		        		SimpleRegression lregression = new SimpleRegression();
		        		SimpleRegression rregression = new SimpleRegression();
		        		SimpleRegression sregression = new SimpleRegression();
		        		//Imgproc.cvtColor(result, result, Imgproc.COLOR_GRAY2BGR);
		        		//Highgui.imwrite("/home/lvuser/image1.png", result);
		        		Core.line(image, new Point(minx, miny), new Point(minx, maxy), new Scalar(0,0,255));
		        		Core.line(image, new Point(minx, maxy), new Point(maxx, maxy), new Scalar(0,0,255));
		        		Core.line(image, new Point(maxx, maxy), new Point(maxx, miny), new Scalar(0,0,255));
		        		Core.line(image, new Point(maxx, miny), new Point(minx, miny), new Scalar(0,0,255));
		        		double maxright = 0;
		        		double minyright = image.rows();
		        		double minleft = image.cols();
		        		double minyleft = image.rows();
		        		double maxsouth = 0;
		        		for(int i = 0; i < right.size(); i++) {
		        			if(right.get(i).x > maxright) maxright = right.get(i).x;
		        		}
		        		for(int i = 0; i < right.size(); i++) {
		        			if(maxright-right.get(i).x < 5) {
		            			Core.line(image, right.get(i), right.get(i), new Scalar(0, 255, 0));
		            			rregression.addData(right.get(i).y, right.get(i).x);
		            			if(right.get(i).y < minyright) minyright = right.get(i).y;
		        			}
		        		}
		        		for(int i = 0; i < left.size(); i++) {
		        			if(left.get(i).y < minleft) minleft = left.get(i).x;
		        		}
		        		for(int i = 0; i < left.size(); i++) {
		        			if(Math.abs(left.get(i).x-minleft) < 5) {
		            			Core.line(image, left.get(i), left.get(i), new Scalar(255, 255, 0));
		            			lregression.addData(left.get(i).y, left.get(i).x);
		            			if(left.get(i).y < minyleft) minyleft = left.get(i).y;
		        			}
		        		}
		        		for(int i = 0; i < south.size(); i++) {
		        			if(south.get(i).y > maxsouth) maxsouth = south.get(i).y;
		        		}
		        		for(int i = 0; i < south.size(); i++) {
		        			if(maxsouth-south.get(i).y < 10) {
		        				Core.line(image, south.get(i), south.get(i), new Scalar(0, 255, 255));
		        				sregression.addData(south.get(i).x, south.get(i).y);
		        			}
		        		}
		        		
		        		lregression.regress();
		        		rregression.regress();
		        		sregression.regress();
		        		
		        		double m = sregression.getSlope();
		        		double n = lregression.getSlope();
		        		double o = rregression.getSlope();
		        		double a = sregression.getIntercept();
		        		double b = lregression.getIntercept();
		        		double c = rregression.getIntercept();
		        		
		        		double leftminy = (m*b + a)/(1-m*n);
		        		double leftminx = n*leftminy + b;
		        		Point leftmin = new Point(leftminx, leftminy);
		        		
		        		double rightminy = (m*c + a)/(1-m*o);
		        		double rightminx = o*rightminy + c;
		        		Point rightmin = new Point(rightminx, rightminy);
		        		
		        		Point leftmax = new Point(n*minyleft + b, minyleft);
		        		Point rightmax = new Point(o*minyright + c, minyright);
		        		
		        		Core.line(image, leftmax, leftmin, new Scalar(255,255,0), 1);
		        		Core.line(image, leftmin, rightmin, new Scalar(255,255,0), 1);
		        		Core.line(image, rightmax, rightmin, new Scalar(255,255,0), 1);
		        		Highgui.imwrite("/home/lvuser/image" + framenum + ".png", image);
		        		framenum++;
	        		}
	
	        		System.out.println(System.currentTimeMillis()-time);
	        		
	        		System.out.println("Image made");
	        	}
        	}
        	else {
        	}
        }
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
