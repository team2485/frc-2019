package org.usfirst.frc.team2485.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.net.URL;
import java.text.SimpleDateFormat;
import java.util.HashMap;
import java.util.Scanner;
import org.usfirst.frc.team2485.robot.Robot;

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
    public static double kP_PixelCorrection;
    public static double kP_LimelightAngle;
    public static double kI_LimelightAngle;
    public static double kD_LimelightAngle;
    public static double kP_LimelightStationaryAngle;
    public static double kI_LimelightStationaryAngle;
    public static double kD_LimelightStationaryAngle;
    public static double limelightUpRampQ;
    public static double limelightUpRampL;
    public static double limelightDownRampQ;
    public static double limelightDownRampL;
    public static double teleopUpRamp;
    public static double teleopUpRampL;
    public static double teleopUpRampQ;
    public static double teleopDownRamp;
    public static double teleopDownRampL;
    public static double teleopDownRampQ;
    public static double teleopUpRampElevatorUp;
    public static double teleopUpRampLElevatorUp;
    public static double teleopUpRampQElevatorUp;
    public static double teleopDownRampElevatorUp;
    public static double teleopDownRampLElevatorUp;
    public static double teleopDownRampQElevatorUp;
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
    public static double kP_elevatorDownVelocity;
    public static double kI_elevatorDownVelocity;
    public static double kD_elevatorDownVelocity;
    public static double kP_elevatorUpVelocity;
    public static double kI_elevatorUpVelocity;
    public static double kD_elevatorUpVelocity;
    public static double elevatorMinVelocity;
    public static double elevatorMaxVelocity;
    public static double elevatorMaxVelocityClose;
    public static double elevatorMaxVelocityManual;
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
    public static double cargoArmMinVelocity;
    public static double cargoArmMaxVelocity;
    public static double cargoArmMinVelocityClose;
    public static double cargoArmMaxVelocityClose;
    public static double kP_cargoArmDownVelocity;
    public static double kI_cargoArmDownVelocity;
    public static double kD_cargoArmDownVelocity;
    public static double kP_cargoArmUpVelocity;
    public static double kI_cargoArmUpVelocity;
    public static double kD_cargoArmUpVelocity;
    public static double levitateCargo;
    public static double kArmEncoderFilterCoefficient;
    public static double kCargoArmDistanceOutputFilterCoefficient;
    public static double cargoRollersIMax;
    public static double elevatorIMax;
    public static double cargoArmIMaxUp;
    public static double cargoArmIMaxDown;
    public static double cargoArmIMax;
    public static double driveTrainIMax;
    


    public static void init() {
        System.out.println("ConstantsIO .class file loc: " + ConstantsIO.class.getResource("").getPath());
        if (Robot.isSimulation()) {
            String constantsFile = ConstantsIO.findConstantsFile();
            try {
                data = ConstantsIO.parseLoadFile(ConstantsIO.readLocalFile(constantsFile));
            }
            catch (IOException e1) {
                e1.printStackTrace();
            }
        } else {
            try {
                data = ConstantsIO.parseLoadFile(ConstantsIO.readLocalFile(ROBO_RIO_CONSTANTS_FILE_PATH));
            }
            catch (IOException e1) {
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
        kP_PixelCorrection = Double.parseDouble(data.get("kP_PixelCorrection"));
        kP_LimelightAngle = Double.parseDouble(data.get("kP_LimelightAngle"));
        kI_LimelightAngle = Double.parseDouble(data.get("kI_LimelightAngle"));
        kD_LimelightAngle = Double.parseDouble(data.get("kD_LimelightAngle"));
        kP_LimelightStationaryAngle = Double.parseDouble(data.get("kP_LimelightStationaryAngle"));
        kI_LimelightStationaryAngle = Double.parseDouble(data.get("kI_LimelightStationaryAngle"));
        kD_LimelightStationaryAngle = Double.parseDouble(data.get("kD_LimelightStationaryAngle"));
        teleopUpRamp = Double.parseDouble(data.get("teleopUpRamp"));
        teleopUpRampQ = Double.parseDouble(data.get("teleopUpRampQ"));
        teleopUpRampL = Double.parseDouble(data.get("teleopUpRampL"));
        teleopDownRamp = Double.parseDouble(data.get("teleopDownRamp"));
        teleopDownRampQ = Double.parseDouble(data.get("teleopDownRampQ"));
        teleopDownRampL = Double.parseDouble(data.get("teleopDownRampL"));
        teleopUpRampElevatorUp = Double.parseDouble(data.get("teleopUpRampElevatorUp"));
        teleopUpRampQElevatorUp = Double.parseDouble(data.get("teleopUpRampQElevatorUp"));
        teleopUpRampLElevatorUp = Double.parseDouble(data.get("teleopUpRampLElevatorUp"));
        teleopDownRampElevatorUp = Double.parseDouble(data.get("teleopDownRampElevatorUp"));
        teleopDownRampQElevatorUp = Double.parseDouble(data.get("teleopDownRampQElevatorUp"));
        teleopDownRampLElevatorUp = Double.parseDouble(data.get("teleopDownRampLElevatorUp"));
        kUpRamp_Velocity = Double.parseDouble(data.get("kUpRamp_Velocity"));
        kDownRamp_Velocity = Double.parseDouble(data.get("kDownRamp_Velocity"));
        kUpRamp_AngVelocity = Double.parseDouble(data.get("kUpRamp_AngVelocity"));
        kDownRamp_AngVelocity = Double.parseDouble(data.get("kDownRamp_AngVelocity"));
        limelightUpRampQ = Double.parseDouble(data.get("limelightUpRampQ"));
        limelightUpRampL = Double.parseDouble(data.get("limelightUpRampL"));
        limelightDownRampQ = Double.parseDouble(data.get("limelightDownRampQ"));
        limelightDownRampL = Double.parseDouble(data.get("limelightDownRampL"));
        kP_elevatorDistance = Double.parseDouble(data.get("kP_elevatorDistance"));
        kI_elevatorDistance = Double.parseDouble(data.get("kI_elevatorDistance"));
        kD_elevatorDistance = Double.parseDouble(data.get("kD_elevatorDistance"));
        kF_elevatorDistance = Double.parseDouble(data.get("kF_elevatorDistance"));
        kElevatorEncoderFilterCoefficient = Double.parseDouble(data.get("kElevatorEncoderFilterCoefficient"));
        kElevatorDistanceOutputFilterCoefficient = Double.parseDouble(data.get("kElevatorDistanceOutputFilterCoefficient"));
        kP_elevatorDownVelocity = Double.parseDouble(data.get("kP_elevatorDownVelocity"));
        kI_elevatorDownVelocity = Double.parseDouble(data.get("kI_elevatorDownVelocity"));
        kD_elevatorDownVelocity = Double.parseDouble(data.get("kD_elevatorDownVelocity"));
        kP_elevatorUpVelocity = Double.parseDouble(data.get("kP_elevatorUpVelocity"));
        kI_elevatorUpVelocity = Double.parseDouble(data.get("kI_elevatorUpVelocity"));
        kD_elevatorUpVelocity = Double.parseDouble(data.get("kD_elevatorUpVelocity"));
        elevatorMaxVelocity = Double.parseDouble(data.get("elevatorMaxVelocity"));
        elevatorMaxVelocityClose = Double.parseDouble(data.get("elevatorMaxVelocityClose"));
        elevatorMaxVelocityManual = Double.parseDouble(data.get("elevatorMaxVelocityManual"));
        elevatorMinVelocity = Double.parseDouble(data.get("elevatorMinVelocity"));
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

        kP_cargoArmDownVelocity = Double.parseDouble(data.get("kP_cargoArmDownVelocity"));
        kI_cargoArmDownVelocity = Double.parseDouble(data.get("kI_cargoArmDownVelocity"));
        kD_cargoArmDownVelocity = Double.parseDouble(data.get("kD_cargoArmDownVelocity"));
        kP_cargoArmUpVelocity = Double.parseDouble(data.get("kP_cargoArmUpVelocity"));
        kI_cargoArmUpVelocity = Double.parseDouble(data.get("kI_cargoArmUpVelocity"));
        kD_cargoArmUpVelocity = Double.parseDouble(data.get("kD_cargoArmUpVelocity"));
        cargoArmMaxVelocity = Double.parseDouble(data.get("cargoArmMaxVelocity"));
        cargoArmMinVelocity = Double.parseDouble(data.get("cargoArmMinVelocity"));
        cargoArmMinVelocityClose = Double.parseDouble(data.get("cargoArmMinVelocityClose"));
        cargoArmMaxVelocityClose = Double.parseDouble(data.get("cargoArmMaxVelocityClose"));

        levitateCargo = Double.parseDouble(data.get("levitateCargo"));
        kArmEncoderFilterCoefficient = Double.parseDouble(data.get("kArmEncoderFilterCoefficient"));
        kCargoArmDistanceOutputFilterCoefficient = Double.parseDouble(data.get("kCargoArmDistanceOutputFilterCoefficient"));
        elevatorIMax = Double.parseDouble(data.get("elevatorIMax"));
        cargoArmIMaxUp = Double.parseDouble(data.get("cargoArmIMaxUp"));
        cargoArmIMaxDown = Double.parseDouble(data.get("cargoArmIMaxDown"));
        cargoArmIMax = Double.parseDouble(data.get("cargoArmIMax"));
        driveTrainIMax = Double.parseDouble(data.get("driveTrainIMax"));
    }

    private static void createUnMatchedConstants() {
        Field[] fields = ConstantsIO.class.getDeclaredFields();
        for (int i = 0; i < fields.length; ++i) {
            fields[i].getName().startsWith("k");
            if (data.containsKey(fields[i].getName())) continue;
        }
    }

    private static String findConstantsFile() {
        File curFolder = new File(ConstantsIO.class.getResource("").getPath());
        while (!curFolder.getName().equals("frc-2017")) {
            curFolder = curFolder.getParentFile();
            System.out.println("Backed out to: " + curFolder.getPath());
        }
        return new File(curFolder, "Constants.txt").getPath().substring(5);
    }

    /*
     * WARNING - Removed try catching itself - possible behaviour change.
     */
    public static String readLocalFile(String filePath) throws IOException {
        String fileString;
        File file = new File(filePath);
        System.out.println("Resolved file path: " + file.getPath());
        SimpleDateFormat sdf = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss");
        lastModified = sdf.format(file.lastModified());
        StringBuilder fileContents = new StringBuilder((int)file.length());
        Scanner scanner = new Scanner(file);
        String lineSeperator = "\n";
        try {
            while (scanner.hasNextLine()) {
                fileContents.append(scanner.nextLine() + lineSeperator);
            }
            fileString = fileContents.toString();
            fileString = fileString.substring(0, fileString.length() - 1);
        }
        finally {
            scanner.close();
        }
        return fileString;
    }

    public static HashMap<String, String> parseLoadFile(String fileContents) {
        HashMap<String, String> constantsMap = new HashMap<String, String>();
        Scanner scanner = new Scanner(fileContents);
        while (scanner.hasNextLine()) {
            String currLine = scanner.nextLine().trim();
            if (!currLine.contains("=")) continue;
            String constantName = currLine.substring(0, currLine.indexOf("=")).trim();
            String constantValue = currLine.substring(currLine.indexOf("=") + 1).trim();
            constantsMap.put(constantName, constantValue);
        }
        scanner.close();
        return constantsMap;
    }

    public static void writeConstantsToRoboRio(String loadFileContents) {
        PrintWriter printWriter = null;
        try {
            printWriter = new PrintWriter(new FileOutputStream("ftp://roborio-2485-frc.local/home/lvuser/constants.txt"));
        }
        catch (FileNotFoundException e) {
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