package robot.utils;

import java.net.*;
import java.io.*;

public class MyUdpClient
{
    private DatagramSocket mSocket;
    private int mServerPort ; 
    private InetAddress mServerAddr ;
    private byte[] buf;
 
    public MyUdpClient(String serverName, int port) {
        mServerPort = port ;
        try {
            mServerAddr = InetAddress.getByName(serverName) ;
            // use any local port for the socket
            mSocket = new DatagramSocket() ;
        }
        catch (UnknownHostException e) {
            System.out.println("Unknown host: "+serverName) ;
        }
        catch (SocketException e) {
            System.out.println("Socket creation failure") ;
        }
    }
 
    public void sendMessage(String msg) {
        buf = msg.getBytes();
        // create a packet to send to the server and send it
        DatagramPacket packet = new DatagramPacket(buf, buf.length, mServerAddr, mServerPort);
        try {
            mSocket.send(packet);
        }
        catch (IOException e) {
            System.out.println("FAILED Sending " + msg) ;
        }
    }
 
    public void close() {
        mSocket.close();
    }
}