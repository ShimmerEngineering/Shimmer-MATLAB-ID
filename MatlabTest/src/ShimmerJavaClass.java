import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JTextField;

import com.shimmerresearch.bluetooth.ShimmerBluetooth.BT_STATE;
import com.shimmerresearch.driver.BasicProcessWithCallBack;
import com.shimmerresearch.driver.CallbackObject;
import com.shimmerresearch.driver.ObjectCluster;
import com.shimmerresearch.driver.ShimmerMsg;
import com.shimmerresearch.driver.Configuration.COMMUNICATION_TYPE;
import com.shimmerresearch.driverUtilities.ChannelDetails;
import com.shimmerresearch.exceptions.ShimmerException;
import com.shimmerresearch.pcDriver.ShimmerPC;

public class ShimmerJavaClass {
    private ShimmerPC shimmer;
    private String comPort;
    private SensorDataReceived sdr = new SensorDataReceived();
    private String[] channelNames;
    private final Queue<float[]> dataQueue = new ConcurrentLinkedQueue<float[]>();
    public static void main(String[] args) {
        ShimmerJavaClass example = new ShimmerJavaClass();
        example.createAndShowGUI();
    }

    public void createAndShowGUI() {
        JFrame frame = new JFrame("Shimmer Device Controller");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(400, 200);

        JPanel panel = new JPanel();

        JTextField inputComPort = new JTextField("COM3");
        JButton connectButton = new JButton("Connect");
        JButton disconnectButton = new JButton("Disconnect");
        JButton startButton = new JButton("Start Streaming");
        JButton stopButton = new JButton("Stop Streaming");
        
        connectButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	comPort = inputComPort.getText();
                connectDevice(comPort);
            }
        });

        disconnectButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                disconnectDevice();
            }
        });

        startButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                startStreaming();
                
              while(true) {
            	if(!dataQueue.isEmpty()) {
                    readData();
            	}
            }
            }
        });

        stopButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                stopStreaming();
            }
        });

        panel.add(inputComPort);
        panel.add(connectButton);
        panel.add(disconnectButton);
        panel.add(startButton);
        panel.add(stopButton);
        frame.add(panel);
        
        frame.setVisible(true);
    }

    public void connectDevice(String comPort) {
        shimmer = new ShimmerPC(comPort);
        shimmer.connect(comPort, "");
        sdr.setWaitForData(shimmer);
    }

    public void disconnectDevice() {
        try {
            shimmer.disconnect();
        } catch (ShimmerException e) {
            e.printStackTrace();
        }
    }

    public void startStreaming() {
        try {
            shimmer.startStreaming();
        } catch (ShimmerException e) {
            e.printStackTrace();
        }
    }

    public void stopStreaming() {
        shimmer.stopStreaming();
    }

    public String[] retrieveSensorChannels() {
        LinkedHashMap<String, ChannelDetails> mapOfAllChannels = shimmer.getMapOfAllChannelsForStoringToDB(COMMUNICATION_TYPE.BLUETOOTH, null, false, false);
        List<ChannelDetails> listOfChannelDetails = new ArrayList<ChannelDetails>(mapOfAllChannels.values());

        List<String> channelNamesList = new ArrayList<String>();
        for (ChannelDetails channel : listOfChannelDetails) {
            channelNamesList.add(channel.mObjectClusterName);
        }

        return channelNamesList.toArray(new String[0]);
    }

    public float receiveData(String channel) { //Receive data for specific Object Cluster
        float[] data = dataQueue.poll();
		for (int i = 0; i < channelNames.length; i++) {
		    if (channelNames[i].equals(channel)) {
		        return data[i];
		    }
		}
        return Float.NaN;
    }
    
    public float[] receiveData() { //Receive data for all Object Cluster
        float[] data = dataQueue.poll();
        return data;
    }

    private void sendData(float[] data) {
        dataQueue.add(data);
    }
    
    public void readData() { //Test receive data for all Object Cluster
    	float[] data = receiveData();
    	System.out.println("Timestamp : " + data[4] + "\nAccel_LN_X : " + data[0]);
    }

    public class SensorDataReceived extends BasicProcessWithCallBack {

        @Override
        protected void processMsgFromCallback(ShimmerMsg shimmerMSG) {
            int ind = shimmerMSG.mIdentifier;
            Object object = shimmerMSG.mB;

            if (ind == ShimmerPC.MSG_IDENTIFIER_STATE_CHANGE) {
                CallbackObject callbackObject = (CallbackObject) object;

                if (callbackObject.mState == BT_STATE.CONNECTING) {
                	
                } else if (callbackObject.mState == BT_STATE.CONNECTED) {
                	channelNames = retrieveSensorChannels();
                	for(String channel : channelNames) {
                		System.out.println(channel);
                	}
                } else if (callbackObject.mState == BT_STATE.DISCONNECTED || callbackObject.mState == BT_STATE.CONNECTION_LOST) {

                }
            } else if (ind == ShimmerPC.MSG_IDENTIFIER_NOTIFICATION_MESSAGE) {

            } else if (ind == ShimmerPC.MSG_IDENTIFIER_DATA_PACKET) {
                ObjectCluster objc = (ObjectCluster) shimmerMSG.mB;

                float[] data = new float[channelNames.length];
                for (int i = 0; i < channelNames.length; i++) {
                    data[i] = (float) objc.getFormatClusterValue(channelNames[i], "CAL");
                }

                sendData(data);
            }
        }
    }
}
