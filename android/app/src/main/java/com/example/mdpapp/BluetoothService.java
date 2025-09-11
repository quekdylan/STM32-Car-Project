package com.example.mdpapp;

import android.Manifest;
import android.app.Service;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;

import androidx.annotation.RequiresPermission;
import androidx.core.app.ActivityCompat;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.UUID;

public class BluetoothService extends Service {
    private static final String TAG = "BluetoothService";
    private static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private final BluetoothAdapter bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
    private ConnectThread connectThread;
    private AcceptThread acceptThread;
    private ConnectedThread connectedThread;

    private final List<OnMessageReceivedListener> listeners = new ArrayList<>();

    private final IBinder binder = new LocalBinder();

    public class LocalBinder extends Binder {
        public BluetoothService getService(){
            return BluetoothService.this;
        }
    }

    @Override
    public IBinder onBind(Intent intent){
        return binder;
    }

    // --- Listener interface ---
    public interface OnMessageReceivedListener {
        void onMessageReceived(String message);
    }

    public void addListener(OnMessageReceivedListener listener) {
        if (!listeners.contains(listener)) {
            listeners.add(listener);
        }
    }

    public void removeListener(OnMessageReceivedListener listener) {
        listeners.remove(listener);
    }

    private void notifyMessageReceived(String message) {
        for (OnMessageReceivedListener listener : listeners) {
            listener.onMessageReceived(message);
        }
    }


    // Start client connection
    public void connect(BluetoothDevice device) {
        if (connectThread != null) {
            connectThread.cancel();
        }
        connectThread = new ConnectThread(device);
        connectThread.start();
    }

    public interface MessageConstants {
        public static final int MESSAGE_READ = 0;
        public static final int MESSAGE_WRITE = 1;
        public static final int MESSAGE_TOAST = 2;

    }

    public Set<BluetoothDevice> getPairedDevices(Context context) {
        if (bluetoothAdapter != null) {
            if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT)
                    != PackageManager.PERMISSION_GRANTED) {
                // Permission not granted: return empty set, let Activity handle request
                return new HashSet<>();
            }
            return bluetoothAdapter.getBondedDevices();
        } else {
            return new HashSet<>();
        }
    }

    public class ConnectedThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        public ConnectedThread(BluetoothSocket socket) {
            mmSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            // Get input and output stream from the socket
            try {
                tmpIn = socket.getInputStream();
            } catch (IOException e) {
                Log.e(TAG, "Error occurred when creating input stream", e);
            }
            try {
                tmpOut = socket.getOutputStream();
            } catch (IOException e) {
                Log.e(TAG, "Error occurred when creating output stream", e);
            }

            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }

        public void run() {
            byte[] buffer = new byte[1024];
            int bytes;
            while (true) {
                try {
                    bytes = mmInStream.read(buffer);
                    if (bytes > 0) {
                        String message = new String(buffer, 0, bytes);
                        notifyMessageReceived(message);
                    }
                } catch (IOException e) {
                    Log.d(TAG, "Input stream disconnected", e);
                    break;
                }
            }
        }

        // Call this from main activity to send data to remote device
        public void write(byte[] bytes) {
            try {
                mmOutStream.write(bytes);
            } catch (IOException e) {
                Log.e(TAG, "Error occurred when sending data", e);

            }
        }

        // Call this method to shut down the connection
        public void cancel() {
            try {
                mmSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "Could not close the connect socket", e);
            }
        }
    }

    // The logic for connecting as client needs to be implemented
    // This is done in a separate thread to avoid blocking the main UI thread
    private class ConnectThread extends Thread {
        private static final String TAG = "ConnectThread";
        private final BluetoothSocket mmSocket;
        private final BluetoothDevice mmDevice;
        public ConnectThread(BluetoothDevice device) {
            // Use a temporary object that is later assigned to mmSocket
            // because mmSocket is final
            BluetoothSocket tmp = null;
            mmDevice = device;

            try {
                // Get the socket from selected device.
                tmp = device.createRfcommSocketToServiceRecord(MY_UUID);
            } catch (IOException e) {
                Log.e(TAG, "Socket's create() method failed", e);
            }
            mmSocket = tmp;
        }

        @RequiresPermission(allOf = {Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT})
        public void run() {
            // Stop discovery because it otherwise slows down the connection.
            // Note: don't check permissions inside ConnectThread
            bluetoothAdapter.cancelDiscovery();

            try {
                // Connect through the socket.
                mmSocket.connect();
                Log.d(TAG, "Connected to " + mmDevice.getName());
            } catch (IOException connectException) {
                // Failed to connect
                Log.e(TAG, "Connection failed", connectException);
                try {
                    mmSocket.close();
                } catch (IOException closeException) {
                    Log.e(TAG, "Could not close the client socket", closeException);
                }
                return;
            }

            // The connection attempt succeeded. Perform work associated with
            // the connection in a separate thread.
            manageMyConnectedSocket(mmSocket);


        }

        public void cancel() {
            try {
                mmSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "Could not close the client socket", e);
            }
        }
    }

    // Add this getter
    public ConnectedThread getConnectedThread() {
        return connectedThread;
    }

    // Thread for listening as a server.
    // Get a BluetoothServerSocket, call accept() to get the socket
    // Close connection.
    private class AcceptThread extends Thread {
        private static final String TAG = "AcceptThread";
        private final BluetoothServerSocket mmServerSocket;

        @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
        public AcceptThread() {
            // Use a temporary object that is later assigned to mmServerSocket
            // because mmServerSocket is final
            BluetoothServerSocket tmp = null;
            try {
                tmp = bluetoothAdapter.listenUsingRfcommWithServiceRecord("MDPApp", MY_UUID);
            } catch (IOException e) {
                Log.e(TAG, "Socket's listen() method failed", e);
            }
            mmServerSocket = tmp;
        }

        public void run() {
            BluetoothSocket socket = null;
            // Keep listening until exception occurs or a socket is returned
            while (true) {
                try {
                    socket = mmServerSocket.accept();
                } catch (IOException e) {
                    Log.e(TAG, "Socket's accept() method failed", e);
                    break;
                }

                if (socket != null) {
                    // A connection was accepted. Perform work associated with
                    // the connection in a separate thread.
                    manageMyConnectedSocket(socket);
                    try {
                        mmServerSocket.close();
                    } catch (IOException e) {
                        Log.e(TAG, "Could not close the server socket", e);
                    }
                    break;
                }
            }

        }

        // Closes the connect socket and causes the thread to finish
        public void cancel() {
            try {
                mmServerSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "Could not close the server socket", e);
            }
        }
    }

    public void startAcceptThread() {
        if (acceptThread != null && acceptThread.isAlive()) {
            // Already running, no need to start again
            return;
        }

        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT)
                != PackageManager.PERMISSION_GRANTED) {
            return;
        }

        acceptThread = new AcceptThread();
        acceptThread.start();
    }

    private void manageMyConnectedSocket(BluetoothSocket socket) {
        if (connectedThread != null) {
            connectedThread.cancel();
        }

        connectedThread = new ConnectedThread(socket);
        connectedThread.start();
    }

    public boolean isBluetoothEnabled() {
        return bluetoothAdapter != null && bluetoothAdapter.isEnabled();
    }
    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    public void enableBluetooth() {
        if (bluetoothAdapter != null && !bluetoothAdapter.isEnabled()) {
            bluetoothAdapter.enable();
        }
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    public void disableBluetooth() {
        if (bluetoothAdapter != null && bluetoothAdapter.isEnabled()) {
            bluetoothAdapter.disable();
        }
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_SCAN)
    public void startScan() {
        if (bluetoothAdapter == null) return;

        if (bluetoothAdapter.isDiscovering()) {
            bluetoothAdapter.cancelDiscovery();
        }

        bluetoothAdapter.startDiscovery();

        boolean started = bluetoothAdapter.startDiscovery();
        Log.d(TAG, "Bluetooth discovery started: " + started);

        // Notify listeners about scanning status
        notifyMessageReceived("STATUS:Scanning for devices...");
    }



}
