package org.usfirst.frc.team2485.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Scanner;
import java.text.SimpleDateFormat;

import org.usfirst.frc.team2485.robot.Robot;

/**
 * Static class to interface IO between the RoboRio and the Driver Station. Used
 * to save constants to a file rather than being hard coded.
 * 
 * @author Ben Clark
 * @author Jeremy McCulloch
 */
public class ConstantsIO {
	public static final String ROBO_RIO_CONSTANTS_FILE_PATH = "/home/lvuser/constants.txt";

	public static HashMap<String, String> data;

	public static String lastModified;

	public static double accelerationMax;

	public static double kDrift;
	public static double kPath;

	public static double driveTrainUpRamp;
	public static double driveTrainDownRamp;

	public static double kPMax_Distance;

	public static double kP_DriveVelocity;
	public static double kI_DriveVelocity;
	public static double kD_DriveVelocity;
	public static double kF_DriveVelocity;
	public static double kV_DriveVelocity;
	
	public static double kP_DriveAngle;
	public static double kI_DriveAngle;
	public static double kD_DriveAngle;
	
	public static double kP_DriveAngVel;
	public static double kI_DriveAngVel;
	public static double kD_DriveAngVel;
	public static double kF_DriveAngVel;
	public static double kV_DriveAngVel;
	
	public static double teleopUpRamp;
	public static double teleopDownRamp;
	public static double kUpRamp_Velocity;
	public static double kDownRamp_Velocity;
	public static double kUpRamp_AngVelocity;
	public static double kDownRamp_AngVelocity;
	
	public static double kP_elevatorDistance;
	public static double kI_elevatorDistance;
	public static double kD_elevatorDistance;
	public static double kF_elevatorDistance;
	public static double levitateElevatorDistance;
	public static double kElevatorEncoderFilterCoefficient;
	public static double kElevatorDistanceOutputFilterCoefficient;
	

	public static double elevatorDistanceSetpointDownRamp;
	public static double elevatorDistanceSetpointUpRamp;
	public static double armDistanceSetpointDownRamp;
	public static double armDistanceSetpointUpRamp;
	public static double armDistanceSetpointDownRampClose;
	public static double armDistanceSetpointUpRampClose;
	
	public static double kP_cargoArmDistance;
	public static double kI_cargoArmDistance;
	public static double kD_cargoArmDistance;
	public static double kF_cargoArmDistance;
	public static double levitateCargo;
	public static double kArmEncoderFilterCoefficient;
	public static double kDistanceOutputFilterCoefficient;
	
	public static double cargoRollersIMax;
	public static double elevatorIMax;
	//public static double cargoArmIMax;
	public static double cargoArmIMaxUp;
	public static double cargoArmIMaxDown;
	public static double driveTrainIMax;

	public static void init() {

		System.out.println("ConstantsIO .class file loc: " + ConstantsIO.class.getResource("").getPath());

		if (Robot.isSimulation()) {

			String constantsFile = findConstantsFile();

			try {
				data = parseLoadFile(readLocalFile(constantsFile));
			} catch (IOException e1) {
				e1.printStackTrace();
			}
		} else {
			try {
				data = parseLoadFile(readLocalFile(ROBO_RIO_CONSTANTS_FILE_PATH));
			} catch (IOException e1) {
				e1.printStackTrace();
			}
		}

		accelerationMax = Double.parseDouble(data.get("accelerationMax"));

		kPath = Double.parseDouble(data.get("kPath"));
		kDrift = Double.parseDouble(data.get("kDrift"));

		driveTrainUpRamp = Double.parseDouble(data.get("driveTrainUpRamp"));
		driveTrainDownRamp = Double.parseDouble(data.get("driveTrainDownRamp"));

		kPMax_Distance = Double.parseDouble(data.get("kPMax_Distance"));

		kP_DriveVelocity = Double.parseDouble(data.get("kP_DriveVelocity"));
		kI_DriveVelocity = Double.parseDouble(data.get("kI_DriveVelocity"));
		kD_DriveVelocity = Double.parseDouble(data.get("kD_DriveVelocity"));
		kF_DriveVelocity = Double.parseDouble(data.get("kF_DriveVelocity"));
		
		kP_DriveAngle = Double.parseDouble(data.get("kP_DriveAngle"));
		kI_DriveAngle = Double.parseDouble(data.get("kI_DriveAngle"));
		kD_DriveAngle = Double.parseDouble(data.get("kD_DriveAngle"));
		
		kP_DriveAngVel = Double.parseDouble(data.get("kP_DriveAngVel"));
		kI_DriveAngVel = Double.parseDouble(data.get("kI_DriveAngVel"));
		kD_DriveAngVel = Double.parseDouble(data.get("kD_DriveAngVel"));
		kF_DriveAngVel = Double.parseDouble(data.get("kF_DriveAngVel"));
		kV_DriveAngVel = Double.parseDouble(data.get("kV_DriveAngVel"));

		teleopUpRamp = Double.parseDouble(data.get("teleopUpRamp"));		
		teleopDownRamp = Double.parseDouble(data.get("teleopDownRamp"));		
		kUpRamp_Velocity = Double.parseDouble(data.get("kUpRamp_Velocity"));
		kDownRamp_Velocity = Double.parseDouble(data.get("kDownRamp_Velocity"));
		kUpRamp_AngVelocity = Double.parseDouble(data.get("kUpRamp_AngVelocity"));
		kDownRamp_AngVelocity = Double.parseDouble(data.get("kDownRamp_AngVelocity"));
		
		kP_elevatorDistance = Double.parseDouble(data.get("kP_elevatorDistance"));
		kI_elevatorDistance = Double.parseDouble(data.get("kI_elevatorDistance"));
		kD_elevatorDistance = Double.parseDouble(data.get("kD_elevatorDistance"));
		kF_elevatorDistance = Double.parseDouble(data.get("kF_elevatorDistance"));
		kElevatorEncoderFilterCoefficient = Double.parseDouble(data.get("kElevatorEncoderFilterCoefficient"));
		kElevatorDistanceOutputFilterCoefficient = Double.parseDouble(data.get("kElevatorDistanceOutputFilterCoefficient"));
	
		elevatorDistanceSetpointDownRamp = Double.parseDouble(data.get("elevatorDistanceSetpointDownRamp"));
		elevatorDistanceSetpointUpRamp = Double.parseDouble(data.get("elevatorDistanceSetpointUpRamp"));

		armDistanceSetpointDownRamp = Double.parseDouble(data.get("armDistanceSetpointDownRamp"));
		armDistanceSetpointUpRamp = Double.parseDouble(data.get("armDistanceSetpointUpRamp"));
		armDistanceSetpointDownRampClose = Double.parseDouble(data.get("armDistanceSetpointDownRampClose"));
		armDistanceSetpointUpRampClose = Double.parseDouble(data.get("armDistanceSetpointUpRampClose")); 
		
		kP_cargoArmDistance = Double.parseDouble(data.get("kP_cargoArmDistance"));
		kI_cargoArmDistance = Double.parseDouble(data.get("kI_cargoArmDistance"));
		kD_cargoArmDistance = Double.parseDouble(data.get("kD_cargoArmDistance"));
		kF_cargoArmDistance = Double.parseDouble(data.get("kF_cargoArmDistance"));
		levitateCargo = Double.parseDouble(data.get("levitateCargo"));
		kArmEncoderFilterCoefficient = Double.parseDouble(data.get("kArmEncoderFilterCoefficient"));
		kDistanceOutputFilterCoefficient = Double.parseDouble(data.get("kDistanceOutputFilterCoefficient"));
		
		elevatorIMax = Double.parseDouble(data.get("elevatorIMax"));
		//cargoArmIMax = Double.parseDouble(data.get("cargoArmIMax"));
		cargoArmIMaxUp = Double.parseDouble(data.get("cargoArmIMaxUp"));
		cargoArmIMaxDown = Double.parseDouble(data.get("cargoArmIMaxDown"));
		driveTrainIMax = Double.parseDouble(data.get("driveTrainIMax"));
	


	}

	@SuppressWarnings("unused")
	private static void createUnMatchedConstants() {
		Field[] fields = ConstantsIO.class.getDeclaredFields();

		for (int i = 0; i < fields.length; i++) {
			fields[i].getName().startsWith("k");

			if (!data.containsKey(fields[i].getName())) {

			}
		}
	}

	/**
	 * I'm so sorry Jeremy
	 */
	private static String findConstantsFile() {
		File curFolder = new File(ConstantsIO.class.getResource("").getPath());

		while (!curFolder.getName().equals("frc-2017")) {
			curFolder = curFolder.getParentFile();
			System.out.println("Backed out to: " + curFolder.getPath());
		}

		return new File(curFolder, "Constants.txt").getPath().substring(5);
	}

	/**
	 * Used to read a file locally.
	 * 
	 * @param filePath
	 */
	public static String readLocalFile(String filePath) throws IOException {
		File file = new File(filePath);

		System.out.println("Resolved file path: " + file.getPath());

		SimpleDateFormat sdf = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss");
		
		lastModified = sdf.format(file.lastModified());

		String fileString;

		StringBuilder fileContents = new StringBuilder((int) file.length());
		Scanner scanner = new Scanner(file);
		String lineSeperator = "\n";

		try {
			while (scanner.hasNextLine())
				fileContents.append(scanner.nextLine() + lineSeperator);
			fileString = fileContents.toString();
			// remove the added "\n"
			fileString = fileString.substring(0, fileString.length() - 1);
		} finally {
			scanner.close();
		}
		return fileString;
	}

	/**
	 * @param loadFileContents
	 * @return HashMap containing constant names and their values as declared in the
	 *         load file.
	 */
	public static HashMap<String, String> parseLoadFile(String fileContents) {

		HashMap<String, String> constantsMap = new HashMap<String, String>();
		Scanner scanner = new Scanner(fileContents);

		while (scanner.hasNextLine()) {

			String currLine = scanner.nextLine().trim();

			if (currLine.contains("=")) {

				String constantName = currLine.substring(0, currLine.indexOf("=")).trim();
				String constantValue = currLine.substring(currLine.indexOf("=") + 1).trim();

				constantsMap.put(constantName, constantValue);
			}
		}
		scanner.close();
		return constantsMap;
	}

	/**
	 * NEEDS TO BE WRITTEN AND DEPLOTED FROM ELSE WHERE: WIDGITS?
	 */
	public static void writeConstantsToRoboRio(String loadFileContents) {

		PrintWriter printWriter = null;

		try {
			printWriter = new PrintWriter(
					new FileOutputStream("ftp://roborio-2485-frc.local" + ROBO_RIO_CONSTANTS_FILE_PATH)); // definitely
			// won't
			// work
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}

		if (printWriter != null) {
			printWriter.write(loadFileContents);
			printWriter.flush();
			printWriter.close();
		} else {
			System.err.println("PrintWriting failed to init, unable to write constants.");
		}

	}

}