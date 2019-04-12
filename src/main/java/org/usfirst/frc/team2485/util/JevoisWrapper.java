package org.usfirst.frc.team2485.util;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * @author Nathan Sariowan
 */

public class JevoisWrapper {
    private static SerialPort jevois = null;
    private int loopCount;
    private UsbCamera jevoisCamera;
    private MjpegServer jevoisServer;

    private static SerialPort.Port jevoisSerialPort;
    
    public JevoisWrapper(SerialPort.Port jevoisSerialPort) {
        this.jevoisSerialPort = jevoisSerialPort;

        init();
    }

	public static void init() {
        int tryCount = 0;

        while (tryCount < 3) {
            try {
                System.out.println("Creating Jevois SerialPort...");
                jevois = new SerialPort(9600, jevoisSerialPort);
                tryCount = 99;
                System.out.println("Jevois SerialPort created.");
            } catch (Exception e) {
                System.out.println("Jevois SerialPort failed.");
                tryCount++;
            }

        } while (tryCount < 3);

        if (tryCount == 99) {
            
        }
    }
    
    public String checkJeVois() {
        if (jevois == null) return "";
        //@todo parse json
        if (jevois.getBytesReceived() > 0) {
            System.out.println("JEVOIS: " + jevois.readString());
            return jevois.readString();
        }
        return "";
    }

    public void writeJevois(String cmd) {
        if (jevois == null) return;
        int bytes = jevois.writeString(cmd);
        System.out.println("Wrote " + bytes + "/" + cmd.length() + " bytes");
    }
}
