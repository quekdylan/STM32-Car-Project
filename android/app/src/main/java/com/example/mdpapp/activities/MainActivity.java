package com.example.mdpapp.activities;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.IBinder;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.Switch;
import android.widget.Toast;

import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.RequiresApi;
import androidx.annotation.RequiresPermission;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.example.mdpapp.R;
import com.example.mdpapp.services.BluetoothService;

import java.util.ArrayList;
import java.util.Set;

public class MainActivity extends AppCompatActivity {

    private BluetoothService bluetoothService;
    private boolean isServiceBound = false;

    private Switch switchBluetooth;
    private Button btnDiscoverable, btnScan, btnChat, btnArena;
    private ListView listPairedDevices, listNearbyDevices;

    private ArrayList<String> pairedDeviceList, nearbyDeviceList;
    private ArrayList<BluetoothDevice> pairedDeviceListObjects, nearbyDeviceListObjects;
    private ArrayAdapter<String> pairedAdapter, nearbyAdapter;
    private final int discoverable_duration = 300; // 300 seconds = 5 minutes
    // BroadcastReceiver to listen for Bluetooth state changes

    private final BroadcastReceiver bluetoothStateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();

            if (BluetoothAdapter.ACTION_STATE_CHANGED.equals(action)) {
                int state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, BluetoothAdapter.ERROR);

                switch (state) {
                    case BluetoothAdapter.STATE_ON:
                        switchBluetooth.setChecked(true);
                        break;

                    case BluetoothAdapter.STATE_OFF:
                        switchBluetooth.setChecked(false);
                        break;
                }
            }
        }
    };

    private final BroadcastReceiver deviceFoundReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (BluetoothDevice.ACTION_FOUND.equals(action)) {
                // Discovery has found a device. Get the BluetoothDevice
                // object and its info from the Intent.
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.BLUETOOTH_CONNECT)
                        != PackageManager.PERMISSION_GRANTED) {
                    return;
                }
                if (device != null && !nearbyDeviceListObjects.contains(device)) {
                    nearbyDeviceListObjects.add(device);
                    nearbyDeviceList.add(device.getName() + "\n" + device.getAddress());
                    nearbyAdapter.notifyDataSetChanged();
                }
            } else if (BluetoothAdapter.ACTION_DISCOVERY_FINISHED.equals(action)) {
                Toast.makeText(context, "Scan complete", Toast.LENGTH_SHORT).show();
            }
        }
    };

    private final ServiceConnection serviceConnection = new ServiceConnection() {
        @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            BluetoothService.LocalBinder binder = (BluetoothService.LocalBinder) service;
            bluetoothService = binder.getService();
            isServiceBound = true;

            // Set switch state
            switchBluetooth.setChecked(bluetoothService.isBluetoothEnabled());

            // Load paired devices
            Set<BluetoothDevice> pairedDevices = bluetoothService.getPairedDevices(MainActivity.this);
            if (!pairedDevices.isEmpty()) {
                for (BluetoothDevice device : pairedDevices) {
                    String deviceName = device.getName();
                    String deviceHardwareAddress = device.getAddress();
                    pairedDeviceList.add(deviceName + "\n" + deviceHardwareAddress);
                    pairedDeviceListObjects.add(device);
                }
                pairedAdapter.notifyDataSetChanged();
            }


        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            bluetoothService = null;
            isServiceBound = false;
        }
    };

    @RequiresApi(api = Build.VERSION_CODES.S)
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Initialize UI
        switchBluetooth = findViewById(R.id.switchBluetooth);
        btnDiscoverable = findViewById(R.id.btnDiscoverable);
        btnScan = findViewById(R.id.btnScan);
        btnChat = findViewById(R.id.btnChat);
        btnArena = findViewById(R.id.btnArena);

        listPairedDevices = findViewById(R.id.listPairedDevices);
        listNearbyDevices = findViewById(R.id.listNearbyDevices);

        // Initialize lists and adapters
        pairedDeviceList = new ArrayList<>();
        nearbyDeviceList = new ArrayList<>();
        pairedDeviceListObjects = new ArrayList<>();
        nearbyDeviceListObjects = new ArrayList<>();

        pairedAdapter = new ArrayAdapter<>(this, android.R.layout.simple_list_item_1, pairedDeviceList);
        nearbyAdapter = new ArrayAdapter<>(this, android.R.layout.simple_list_item_1, nearbyDeviceList);

        listPairedDevices.setAdapter(pairedAdapter);
        listNearbyDevices.setAdapter(nearbyAdapter);

        // Bind the BluetoothService
        Intent serviceIntent = new Intent(this, BluetoothService.class);
        bindService(serviceIntent, serviceConnection, BIND_AUTO_CREATE);

        // Switch listener
        switchBluetooth.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (!isServiceBound) return;

            if (isChecked) {
                if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) ==
                        PackageManager.PERMISSION_GRANTED) {
                    bluetoothService.enableBluetooth();
                } else {
                    requestBluetoothEnableLauncher.launch(Manifest.permission.BLUETOOTH_CONNECT);
                }
            } else {
                bluetoothService.disableBluetooth();
            }
        });

        // Discoverable button
        btnDiscoverable.setOnClickListener(v -> {
            if (!isServiceBound) return;

            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT)
                    != PackageManager.PERMISSION_GRANTED) {
                requestBluetoothEnableLauncher.launch(Manifest.permission.BLUETOOTH_CONNECT);
            } else {
                Intent discoverableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
                discoverableIntent.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, discoverable_duration);
                requestDiscoverableLauncher.launch(discoverableIntent);
            }
        });

        // Scan button
        btnScan.setOnClickListener(v -> {
            if (!isServiceBound) return;

            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN)
                    == PackageManager.PERMISSION_GRANTED) {
                bluetoothService.startScan();
            } else {
                requestBluetoothScanLauncher.launch(Manifest.permission.BLUETOOTH_SCAN);
            }
        });

        // Listeners for paired devices
        listPairedDevices.setOnItemClickListener((parent, view, position, id) -> {
            if (!isServiceBound) return;

            BluetoothDevice selectedDevice = pairedDeviceListObjects.get(position);
            bluetoothService.connect(selectedDevice);
            Toast.makeText(this, "Connecting to " + selectedDevice.getName(), Toast.LENGTH_SHORT).show();
        });

        // Listeners for nearby devices
        listNearbyDevices.setOnItemClickListener((parent, view, position, id) -> {
            if (!isServiceBound) return;

            BluetoothDevice selectedDevice = nearbyDeviceListObjects.get(position);
            bluetoothService.connect(selectedDevice);
            Toast.makeText(this, "Connecting to " + selectedDevice.getName(), Toast.LENGTH_SHORT).show();
        });

        // Chat button
        btnChat.setOnClickListener(v -> {
            startActivity(new Intent(MainActivity.this, ChatActivity.class));
        });

        //Arena button
        btnArena.setOnClickListener(v -> {
            startActivity(new Intent(MainActivity.this, ArenaActivity.class));
        });
    }

    @Override
    protected void onStart() {
        super.onStart();

        // Register receivers for Bluetooth state and discovery
        IntentFilter stateFilter = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
        registerReceiver(bluetoothStateReceiver, stateFilter);

        IntentFilter discoveryFilter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
        registerReceiver(deviceFoundReceiver, discoveryFilter);

        discoveryFilter = new IntentFilter(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
        registerReceiver(deviceFoundReceiver, discoveryFilter);
    }

    @Override
    protected void onStop() {
        super.onStop();
        unregisterReceiver(bluetoothStateReceiver);
        unregisterReceiver(deviceFoundReceiver);
    }

    @SuppressLint("MissingPermission")
    private final ActivityResultLauncher<String> requestBluetoothEnableLauncher =
            registerForActivityResult(new ActivityResultContracts.RequestPermission(), isGranted -> {
                if (isGranted) {
                    bluetoothService.enableBluetooth();
                } else {
                    Toast.makeText(this, "Permission denied", Toast.LENGTH_SHORT).show();
                    switchBluetooth.setChecked(false);
                }
            });

    @SuppressLint("MissingPermission")
    private final ActivityResultLauncher<String> requestBluetoothScanLauncher =
            registerForActivityResult(new ActivityResultContracts.RequestPermission(), isGranted -> {
                if (isGranted) {
                    if (bluetoothService != null) {
                        bluetoothService.startScan();
                    }
                } else {
                    Toast.makeText(this, "Permission denied", Toast.LENGTH_SHORT).show();
                }
            });

    private final ActivityResultLauncher<Intent> requestDiscoverableLauncher =
            registerForActivityResult(new ActivityResultContracts.StartActivityForResult(), result -> {
                if (result.getResultCode() == RESULT_CANCELED) {
                    Toast.makeText(this, "Device cannot be made discoverable", Toast.LENGTH_SHORT).show();
                } else {
                    Toast.makeText(this, "Device is discoverable for " + discoverable_duration + " seconds", Toast.LENGTH_SHORT).show();

                    if (bluetoothService != null) {
                        bluetoothService.startAcceptThread();
                    }
                }
            });
}
