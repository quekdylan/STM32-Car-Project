package com.example.clonemdp;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.Switch;
import android.widget.Toast;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

public class ConnectDeviceActivity extends AppCompatActivity {
    private static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    private static final int REQUEST_ENABLE_BT = 1;
    private static final int REQUEST_BLUETOOTH_PERMISSION = 2;
    private static final int REQUEST_DISCOVERABLE_BT = 3;
    private BluetoothAdapter bluetoothAdapter;
    private Switch switchBluetooth;
    private ArrayAdapter<String> devicesAdapter;
    private final List<BluetoothDevice> devicesList = new ArrayList<>();

    private BluetoothSocket clientSocket;
    private BluetoothServerSocket serverSocket;

    // Receiver to sync switch when Bluetooth state changes
    private final BroadcastReceiver bluetoothStateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (BluetoothAdapter.ACTION_STATE_CHANGED.equals(action)) {
                int state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, BluetoothAdapter.ERROR);
                switch (state) {
                    case BluetoothAdapter.STATE_OFF:
                        switchBluetooth.setChecked(false);
                        break;
                    case BluetoothAdapter.STATE_ON:
                        switchBluetooth.setChecked(true);
                        break;
                }
            }
        }
    };

    private final BroadcastReceiver discoveryReceiver = new BroadcastReceiver(){
        @Override
        public void onReceive(Context context, Intent intent){
            String action = intent.getAction();
            if(BluetoothDevice.ACTION_FOUND.equals(action)){
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (device != null){
                    if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT)
                            != PackageManager.PERMISSION_GRANTED) {

                        return;
                    }
                    String deviceName = (device.getName() != null) ? device.getName() : "Unknown";
                    String deviceInfo = deviceName + "\n" + device.getAddress();

                    if(!devicesList.contains(device)){
                        devicesList.add(device);
                        devicesAdapter.add(deviceInfo);
                        devicesAdapter.notifyDataSetChanged();
                    }

                }

            } else if (BluetoothAdapter.ACTION_DISCOVERY_FINISHED.equals(action)){
                Toast.makeText(context, "Discovery finished", Toast.LENGTH_SHORT).show();
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_connect_device);

        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        switchBluetooth = findViewById(R.id.switchBluetooth);
        Button btnDiscoverable = findViewById(R.id.btnDiscoverable);
        ListView listViewDevices = findViewById(R.id.listViewDevices);
        devicesAdapter = new ArrayAdapter<>(this, android.R.layout.simple_list_item_1);
        listViewDevices.setAdapter(devicesAdapter);
        Button btnScan = findViewById(R.id.btnScan);

        if (bluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth not supported", Toast.LENGTH_SHORT).show();
            finish();
            return;
        }

        // Set initial switch state
        switchBluetooth.setChecked(bluetoothAdapter.isEnabled());

        // Register Bluetooth state receiver
        IntentFilter stateFilter = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
        registerReceiver(bluetoothStateReceiver, stateFilter);

        // Switch toggle listener
        switchBluetooth.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                if (checkBluetoothPermission()) {
                    if (!bluetoothAdapter.isEnabled()) {
                        Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                        startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
                    }
                } else {
                    requestBluetoothPermission();
                }
            } else {
                if (checkBluetoothPermission()) {
                    bluetoothAdapter.disable();
                    Toast.makeText(this, "Bluetooth turned off", Toast.LENGTH_SHORT).show();
                } else {
                    requestBluetoothPermission();
                }
            }
        });

        btnDiscoverable.setOnClickListener(v->{
            if (bluetoothAdapter != null && bluetoothAdapter.isEnabled()) {
                if (checkBluetoothPermission()) {
                    // Ask system to make device discoverable for 120 seconds
                    Intent discoverableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
                    discoverableIntent.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 120);
                    startActivityForResult(discoverableIntent, REQUEST_DISCOVERABLE_BT);
                } else {
                    requestBluetoothPermission();
                }
            } else {
                Toast.makeText(this, "Enable Bluetooth first", Toast.LENGTH_SHORT).show();
            }
        });

        // Register receiver for device discovery
        IntentFilter discoveryFilter = new IntentFilter();
        discoveryFilter.addAction(BluetoothDevice.ACTION_FOUND);
        discoveryFilter.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
        registerReceiver(discoveryReceiver, discoveryFilter);

        btnScan.setOnClickListener(v->{
            if(bluetoothAdapter.isEnabled()){
                startDiscovery();
            } else{
                Toast.makeText(this, "Enable Bluetooth first", Toast.LENGTH_SHORT).show();
            }
        });

        // On list item click: connect to the selected device
        listViewDevices.setOnItemClickListener((parent, view, position, id)->{
            BluetoothDevice selectedDevice = devicesList.get(position);
            connectToDevice(selectedDevice);
        });

    }

    private void startDiscovery(){
        // Reset discovery when rescanning.
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN)
                != PackageManager.PERMISSION_GRANTED) {
            return;
        }
        if(bluetoothAdapter.isDiscovering()){
            bluetoothAdapter.cancelDiscovery();
        }

        devicesList.clear();
        devicesAdapter.clear();
        devicesAdapter.notifyDataSetChanged();

        bluetoothAdapter.startDiscovery();
        Toast.makeText(this, "Discovery started", Toast.LENGTH_SHORT).show();
    }
    private boolean checkBluetoothPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            return ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT)
                    == PackageManager.PERMISSION_GRANTED;
        }
        return true; // Older versions don't need runtime permission
    }

    private void requestBluetoothPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.BLUETOOTH_CONNECT},
                    REQUEST_BLUETOOTH_PERMISSION);
        }
    }
    // For client connection
    private void connectToDevice(BluetoothDevice device){
        new Thread(() -> {
            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED)
                return;

            bluetoothAdapter.cancelDiscovery();

            try {
                clientSocket = device.createRfcommSocketToServiceRecord(MY_UUID);
                clientSocket.connect();

                runOnUiThread(() -> Toast.makeText(this, "Connected to " + device.getName(), Toast.LENGTH_SHORT).show());

                // Start listening/reading from socket
                listenToSocket(clientSocket);

            } catch (IOException e){
                runOnUiThread(() -> Toast.makeText(this, "Connection failed", Toast.LENGTH_SHORT).show());
                e.printStackTrace();
            }
        }).start();
    }


    @Override
    protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        if (requestCode == REQUEST_DISCOVERABLE_BT) {
            if (resultCode != RESULT_CANCELED) {
                Toast.makeText(this, "Device is now discoverable for " + resultCode + " seconds", Toast.LENGTH_SHORT).show();

                // Start server to accept incoming connections
                startServer();
            } else {
                Toast.makeText(this, "Discoverability request canceled", Toast.LENGTH_SHORT).show();
            }
        }

        else if (requestCode == REQUEST_ENABLE_BT) {
            switchBluetooth.setChecked(bluetoothAdapter.isEnabled());
        }
    }


    // For server
    private void startServer() {
        new Thread(() -> {
            try {
                if (!checkBluetoothPermission()) return;

                serverSocket = bluetoothAdapter.listenUsingRfcommWithServiceRecord("MDPAppServer", MY_UUID);

                runOnUiThread(() -> Toast.makeText(this, "Server started, waiting for connection...", Toast.LENGTH_SHORT).show());

                clientSocket = serverSocket.accept(); // blocks until a client connects

                runOnUiThread(() ->
                        Toast.makeText(this, "Incoming connection from: " + clientSocket.getRemoteDevice().getName(), Toast.LENGTH_SHORT).show()
                );

                listenToSocket(clientSocket); // handle I/O

            } catch (IOException e) {
                e.printStackTrace();
            }
        }).start();
    }

    // Example: listening to the socket
    private void listenToSocket(BluetoothSocket socket){
        new Thread(() -> {
            try {
                byte[] buffer = new byte[1024];
                int bytes;

                while (true) {
                    bytes = socket.getInputStream().read(buffer);
                    if (bytes > 0) {
                        String msg = new String(buffer, 0, bytes);
                        runOnUiThread(() -> Toast.makeText(this, "Received: " + msg, Toast.LENGTH_SHORT).show());
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }).start();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        unregisterReceiver(bluetoothStateReceiver);
        unregisterReceiver(discoveryReceiver);
    }
}
