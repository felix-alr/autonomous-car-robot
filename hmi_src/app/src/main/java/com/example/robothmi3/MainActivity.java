package com.example.robothmi3;

import android.Manifest;
import android.app.AlertDialog;
import android.bluetooth.*;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.*;
import android.util.Log;
import android.view.View;
import android.widget.*;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresPermission;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;
import java.lang.Object;
import java.util.concurrent.TimeUnit;

public class MainActivity extends AppCompatActivity {

    //MapView
    private MapView mapView;
    // Konstanten
    private static final int REQUEST_ENABLE_BT = 1;

    // Bluetooth-Komponenten
    private BluetoothAdapter bluetoothAdapter;  // Schnittstelle zum Bluetooth-Adapter
    private BluetoothSocket socket;             // Kommunikationskanal zu einem Gerät
    private OutputStream outputStream;          // Ausgabestream (zum Senden von Daten)
    private BufferedReader reader;              // Eingabestream (zum Empfangen von Daten)

    // UI-Elemente
    private TextView textViewStatus, textViewPosX, textViewPosY, textViewPhi, textViewModus, textViewText;
    private Button buttonConnect, buttonIdle, buttonScout, buttonParking, buttonSetup;
    private Button parkSlotButton1;

    // Handler für wiederkehrende Positionsabfragen
    private Handler handler = new Handler(Looper.getMainLooper());
    private Runnable positionUpdater;            // Aufgabe, die regelmäßig ausgeführt wird
    private boolean keepUpdating = true;        // Flag, um den Loop zu stoppen

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mapView = findViewById(R.id.mapView);

        // UI-Elemente verknüpfen
        buttonConnect = findViewById(R.id.buttonConnect);
        textViewStatus = findViewById(R.id.textViewStatus);
        textViewPosX = findViewById(R.id.textViewPosX);
        textViewPosY = findViewById(R.id.textViewPosY);
        textViewPhi = findViewById(R.id.textViewPhi);
        textViewModus = findViewById(R.id.textViewModus);
        textViewText = findViewById(R.id.textViewText);

        buttonIdle = findViewById(R.id.buttonIdle); // Roboter Ruht
        buttonScout = findViewById(R.id.buttonScout); //Pfad Erkunden
        buttonParking = findViewById(R.id.buttonParking); //Parkplatz anzeigen
        buttonSetup = findViewById(R.id.buttonSetup); //Kalibrieren
        //parkSlotButton1 = findViewById(R.id.parkSlotButton1);

        // Aktionen für Steuerungsbuttons
        buttonIdle.setOnClickListener(v -> sendCommand("z\n"));
        buttonIdle.setOnClickListener(v -> textViewModus.setText("Modus: Idle"));
        buttonScout.setOnClickListener(v -> sendCommand("y\n"));
        buttonScout.setOnClickListener(v -> textViewModus.setText("Modus: Scout"));
        buttonParking.setOnClickListener(v -> anfrageParkingSpots());
        buttonParking.setOnClickListener(v -> textViewModus.setText("Modus: Parking"));
        //parkSlotButton1.setOnClickListener(v -> sendCommand("3\n"));
        buttonSetup.setOnClickListener(v -> sendCommand("r\n"));
        buttonSetup.setOnClickListener(v -> textViewModus.setText("Modus: SetUp"));

        // Bluetooth-Adapter holen (null, falls Gerät kein Bluetooth unterstützt)
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // "Verbinden"-Button
        buttonConnect.setOnClickListener(view -> {
            if (bluetoothAdapter == null) {
                showToast("Bluetooth nicht verfügbar");
                return;
            }
            // Bluetooth einschalten, falls es deaktiviert ist
            if (!bluetoothAdapter.isEnabled()) {
                Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
            } else {
                selectDevice(); // Gerät auswählen
            }
        });
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    @Override
    protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        // Falls Bluetooth erfolgreich aktiviert wurde, Gerät auswählen
        if (requestCode == REQUEST_ENABLE_BT && resultCode == RESULT_OK) {
            selectDevice();
        }
    }

    /**
     * Zeigt eine Liste der gekoppelten Geräte an und lässt den Nutzer ein Gerät auswählen.
     */
    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    private void selectDevice() {
        Set<BluetoothDevice> pairedDevices = bluetoothAdapter.getBondedDevices();
        if (pairedDevices.isEmpty()) {
            showToast("Keine gekoppelten Geräte gefunden");
            return;
        }

        // Namen und Objekte in Arrays speicher
        String[] names = new String[pairedDevices.size()];
        BluetoothDevice[] devices = new BluetoothDevice[pairedDevices.size()];
        int index = 0;
        for (BluetoothDevice device : pairedDevices) {
            names[index] = device.getName();
            devices[index] = device;
            index++;
        }

        // Dialog anzeigen, um Gerät auszuwählen
        new AlertDialog.Builder(this)
                .setTitle("Wähle Gerät")
                .setItems(names, (dialog, which) -> {
                    Log.d("BluetoothDebug", "Gerät gewählt: " + names[which]);
                    connectToDevice(devices[which]);  // Verbindung starten
                })
                .show();
    }

    /**
     * Baut eine Bluetooth-Verbindung zum gewählten Gerät auf.
     */
    private void connectToDevice(BluetoothDevice device) {
        new Thread(() -> {
            try {
                runOnUiThread(() -> textViewStatus.setText("Verbinde mit " + device.getName() + "..."));
                Log.d("BluetoothDebug", "Starte Verbindungsaufbau...");

                // UUID für SPP (Serial Port Profile)
                UUID uuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
                socket = device.createRfcommSocketToServiceRecord(uuid);

                // Berechtigungsprüfung
                if (checkSelfPermission(Manifest.permission.BLUETOOTH) != PackageManager.PERMISSION_GRANTED) {
                    runOnUiThread(() -> textViewStatus.setText("Bluetooth-Berechtigung fehlt!"));
                    return;
                }

                bluetoothAdapter.cancelDiscovery();  // Suche abbrechen für stabile Verbindung
                socket.connect();                    // Verbindung aufbauen (blockiert bis erfolgreich oder fehlgeschlagen)

                Log.d("BluetoothDebug", "Verbindung erfolgreich!");
                runOnUiThread(() -> textViewStatus.setText("Verbindung erfolgreich"));

                // Ein- und Ausgabekanäle öffnen
                outputStream = socket.getOutputStream();
                reader = new BufferedReader(new InputStreamReader(socket.getInputStream(), StandardCharsets.UTF_8));

                // Erste Positionsanfrage senden
                outputStream.write("1\n".getBytes(StandardCharsets.UTF_8));

                runOnUiThread(() -> {
                    textViewStatus.setText("Verbunden mit " + device.getName());
                    startPositionUpdates();  // Automatische Positionsabfragen starten

                });
            } catch (IOException e) {
                e.printStackTrace();
                String msg = e.getMessage() != null ? e.getMessage() : "Unbekannter Fehler";
                Log.e("BluetoothDebug", "Verbindungsfehler: " + msg);
                runOnUiThread(() -> textViewStatus.setText("Fehler: " + msg));
            }
        }).start();
    }

    /**
     * Zeigt eine kurze Nachricht auf dem Bildschirm.
     */
    private void showToast(String message) {
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show();
    }
    /**
     * Startet eine wiederkehrende Aufgabe, die jede Sekunde die Position vom Roboter abfragt.
     */
    private void startPositionUpdates() {
        positionUpdater = new Runnable() {
            @Override
            public void run() {
                if (socket != null && socket.isConnected()) {
                    try {
                        // Positionsanfrage senden
                        outputStream.write("1\n".getBytes(StandardCharsets.UTF_8));

                        // Nur weiter verarbeiten, wenn "p" empfangen wird
                        String line;
                        do {
                            line = reader.readLine();
                        } while (line != null && !line.equals("p"));

                        if (line == null) return;  // Verbindung abgebrochen

                        // Positionswerte lesen
                        String posX = reader.readLine();
                        String posY = reader.readLine();
                        String phi = reader.readLine();

                        // UI mit neuer Position aktualisieren
                        runOnUiThread(() -> {
                            textViewPosX.setText("X: " + posX);
                            textViewPosY.setText("Y: " + posY);
                            textViewPhi.setText("Phi: " + phi);

                            // position updaten
                            float x = Float.parseFloat(posX);
                            float y = Float.parseFloat(posY);
                            float p = Float.parseFloat(phi);
                            y = -y; // Koordinatensystem umdrehen

                            //Umrechnung cm -> pixel/dp
                            float scale = 1.13f; // umrechnung vgl Karte
                            x = x*scale;
                            y = y*scale;
                            p = p*scale;

                            mapView.updateRobotPose(x, y, p);
                        });

                    } catch (IOException e) {
                        runOnUiThread(() -> textViewStatus.setText("Fehler beim Lesen: " + e.getMessage()));
                        keepUpdating = false; // Schleife beenden
                        return;
                    }
                }

                // Nächste Positionsabfrage planen
                if (keepUpdating) {
                    handler.postDelayed(this, 100); // 1000 ms = 1 Sekunde
                }
            }
        };

        handler.post(positionUpdater);
    }

    /**
     * Stoppt Hintergrundaufgaben, wenn die Activity beendet wird.
     */
    @Override
    protected void onDestroy() {
        super.onDestroy();
        keepUpdating = false;
        handler.removeCallbacks(positionUpdater);
    }

    /**
     * Sendet einen Befehl als String an den Roboter.
     */
    private void sendCommand(String command) {
        new Thread(() -> {
            try {
                if (outputStream != null) {
                    outputStream.write(command.getBytes(StandardCharsets.UTF_8));
                    Log.d("BluetoothDebug", "Befehl gesendet: " + command.trim());
                }
            } catch (IOException e) {
                e.printStackTrace();
                runOnUiThread(() -> textViewStatus.setText("Fehler beim Senden: " + e.getMessage()));
            }
        }).start();
    }

    private void anfrageParkingSpots() {
        new Thread(() -> {
            if (socket != null && socket.isConnected()) {
                try {
                    keepUpdating = false; // Positionen pausieren
                    handler.removeCallbacks(positionUpdater);

                    synchronized (reader) {
                        outputStream.write("2\n".getBytes(StandardCharsets.UTF_8));

                        ArrayList<ParkingSpot> spots = readParkingSpots();

                        runOnUiThread(() -> {
                            mapView.updateParkingSpots(spots);
                            textViewModus.setText("Parkplätze empfangen: " + spots.size());
                            textViewText.setText(parkingSpotsToText(spots));

                            // Positionen wieder aktivieren
                            keepUpdating = true;
                            handler.post(positionUpdater);
                        });
                    }
                } catch (Exception e) {
                    runOnUiThread(() -> textViewStatus.setText("Fehler: " + e.getMessage()));
                    keepUpdating = true;
                    handler.post(positionUpdater);
                }
            }
        }).start();
    }

    private String parkingSpotsToText(ArrayList<ParkingSpot> spots) {
        StringBuilder sb = new StringBuilder();

        for (ParkingSpot p : spots) {

            float x1 = p.x1;
            float y1 = p.y1;
            float x2 = p.x2;
            float y2 = p.y2;

            sb.append(
                    "Spot: x1=" + x1 +
                            ", y1=" + y1 +
                            ", x2=" + x2 +
                            ", y2=" + y2 +
                            ", id=" + p.id +
                            "\n"
            );
        }
        return sb.toString();
    }

    private ArrayList<ParkingSpot> readParkingSpots() throws IOException {
        ArrayList<ParkingSpot> spots = new ArrayList<>();

        // 1) Warten bis "s" kommt
        String line;
        do {
            line = reader.readLine();    // blockiert wie bei Positionsdaten
            if (line == null) return spots;
        } while (!line.equals("s"));

        // 2) Zeilen sammeln bis "end"
        StringBuilder buffer = new StringBuilder();
        while (true) {
            line = reader.readLine();    // blockiert
            if (line == null) break;
            if (line.equals("end")) break;

            buffer.append(line).append(";");
        }

        // 3) Tokens parsen
        String[] tokens = buffer.toString().split(";");

        for (int i = 0; i + 5 < tokens.length; i += 6) {
            float scale = 1.13f;

            float id = Float.parseFloat(tokens[i]) * scale;
            float x1 = Float.parseFloat(tokens[i + 1]);
            float y1 = Float.parseFloat(tokens[i + 2]);
            float x2 = Float.parseFloat(tokens[i + 3]);
            float y2 = Float.parseFloat(tokens[i + 4]);
            float suitable = Float.parseFloat(tokens[i + 5]);

            spots.add(new ParkingSpot(id, x1, y1, x2, y2, suitable));
        }
        return spots;
    }
}