package com.example.robothmi3;

import android.Manifest;
import android.app.AlertDialog;
import android.bluetooth.*;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.*;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.*;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresPermission;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Set;
import java.util.UUID;
import java.lang.Object;
import java.util.concurrent.TimeUnit;

public class MainActivity extends AppCompatActivity
        implements MapView.OnParkingSpotClickListener{

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
    private TextView textViewStatus, textViewPosX, textViewPosY, textViewPhi, textViewModus, textViewText, textViewDst, textViewEcke;
    private Button buttonConnect, buttonIdle, buttonScout, buttonParking, buttonSetup;

    // Handler für wiederkehrende Positionsabfragen
    private Handler handler = new Handler(Looper.getMainLooper());
    private Runnable positionUpdater;            // Aufgabe, die regelmäßig ausgeführt wird
    private boolean keepUpdating = true;        // Flag, um den Loop zu stoppen

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    @Override
    /**
     * Funktion Die Alle UI-Elemente (Button, Text, MapView) Initialisiert Und Mit Dem Layout Verknüpft
     * Zusätzlich: Aktionen Für Buttons Definieren -> Senden Von Befehl An Roboter
     * Bluetooth Verbindung Wird Vorbereitet
     */
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mapView = findViewById(R.id.mapView);
        mapView.setOnParkingSpotClickListener(this);

        // UI-Elemente verknüpfen
        buttonConnect = findViewById(R.id.buttonConnect);

        // Texte
        textViewStatus = findViewById(R.id.textViewStatus);
        textViewPosX = findViewById(R.id.textViewPosX);
        textViewPosY = findViewById(R.id.textViewPosY);
        textViewPhi = findViewById(R.id.textViewPhi);
        textViewModus = findViewById(R.id.textViewModus);
        textViewDst = findViewById(R.id.textViewDst);

        // Button
        buttonIdle = findViewById(R.id.buttonIdle); // Roboter Ruht
        buttonScout = findViewById(R.id.buttonScout); //Pfad Erkunden
        buttonParking = findViewById(R.id.buttonParking); //Parkplatz anzeigen
        buttonSetup = findViewById(R.id.buttonSetup); //Kalibrieren

        // Aktiven Button Farbig Setzen (Start Zustand)
        activeMode(buttonIdle);
        textViewModus.setText("Modus: Idle");

        // Aktionen für Steuerungsbuttons
        // Idle
        buttonIdle.setOnClickListener(v -> {
            sendCommand("z\n");                     // Befehl An Roboter Senden
            textViewModus.setText("Modus: Idle");   // Modus Anzeigen
            activeMode(buttonIdle);                 // Aktiven Button Farbig Setzen
        });

        // Scout
        buttonScout.setOnClickListener(v -> {
            sendCommand("y\n");                                 // Befehl An Roboter Senden
            textViewModus.setText("Modus: Scout");              // Modus Anzeigen
            activeMode(buttonScout);                            // Aktiven Button Farbig Setzen
            ArrayList<ParkingSpot> spots = new ArrayList<>();   // Leeres Array An ParkingSpots Definieren
            mapView.updateParkingSpots(spots);                  // Neues Zeichnen Der "leeren" Parklücken -> Bei Scout Sollen Keine Parklücken Gezeichnet Werden
        });

        // Parking
        buttonParking.setOnClickListener(v -> {
            anfrageParkingSpots();                      // Befehl Zur Anfrage An Roboter Senden
            textViewModus.setText("Modus: Parking");    // Modus Anzeigen
            activeMode(buttonParking);                  // Aktiven Button Farbig Setzen
        });

        //SetUp
        buttonSetup.setOnClickListener(v -> {
            sendCommand("r\n");                         // Befehl An Roboter Senden
            textViewModus.setText("Modus: SetUp");      // Modus Anzeigen
            activeMode(buttonSetup);                    // Aktiven Button Farbig Setzen

            // SetUp Wird Innerhalb Von 3 Sekunden Ausgeführt, Danach Direkt Wieder Idle
            handler.postDelayed(() -> {
                sendCommand("z\n");                     // Befehl An Roboter Senden
                textViewModus.setText("Modus: Idle");   // Modus Anzeigen
                activeMode(buttonIdle);                 // Aktiven Button Farbig Setzen
            }, 3_000);                        // Zeit Die Gewartet Werden Soll
        });

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
     *
     * Auslesen, Ausgeben Und Umrechnen Der Positionsdaten
     * Wichitg: Koordinatensystem Des Roboters Wird Gedreht (Karte Und Roboter Haben Unterschiedliche Ursprünge)
     * Daten Werden Der MapVie Übergeben
     */
    private void startPositionUpdates() {
        positionUpdater = new Runnable() {
            @Override
            public void run() {
                if (socket != null && socket.isConnected()) {
                    try {
                        // Positionsanfrage Senden
                        outputStream.write("1\n".getBytes(StandardCharsets.UTF_8));

                        // Nur Weiter Verarbeiten, Wenn "p" Empfangen Wird
                        String line;
                        do {
                            line = reader.readLine();
                        } while (line != null && !line.equals("p"));

                        if (line == null) return;  // Verbindung abgebrochen

                        // Positionswerte Auslesen
                        String posX = reader.readLine();
                        String posY = reader.readLine();
                        String phi = reader.readLine();
                        String dst = reader.readLine();

                        // UI Mit Neuer Position Aktualisieren
                        runOnUiThread(() -> {
                            textViewPosX.setText("X: " + posX);
                            textViewPosY.setText("Y: " + posY);
                            textViewPhi.setText("Phi: " + phi);
                            textViewDst.setText("Dst: " + dst);

                            // Position Updaten Und In Float Umrechnen
                            float x = Float.parseFloat(posX);
                            float y = Float.parseFloat(posY);
                            float p = Float.parseFloat(phi);
                            float d = Float.parseFloat(dst);
                            y = -y; // Koordinatensystem umdrehen
                            float dx = 0;
                            float dy = 0;

                            // Position Abstand Je Nach Winkel Berechnen
                            if (p <= 45.0f && p > -45.0f){
                                dx = x;
                                dy = d + y + 30;
                            } else if (p <= 135.0f && p > 45.0f) {
                                dx = d + x + 30;
                                dy = y;
                            } else if (p <= 225.0f && p > 135.0f) {
                                dx = x;
                                dy = y - d - 30;
                            } else if (p <= 315.0f && p > 225) {
                                dx = x - d - 30;
                                dy = y;
                            } else if (p <= -45.0f && p > -135.0f) {
                                dx = x - d - 30;
                                dy = y;
                            }

                            //Umrechnung cm -> pixel/dp
                            float scaleX = 1.1167f; // umrechnung vgl Karte
                            float scaleY = 1.109167f; // umrechnung vgl Karte
                            x = x*scaleX;
                            y = y*scaleY;
                            dx = dx*scaleX;
                            dy = dy*scaleY;

                            // Übergabe Daten An MapView
                            mapView.updateRobotPose(x, y, p, dx, dy);
                        });

                    } catch (IOException e) {
                        runOnUiThread(() -> textViewStatus.setText("Fehler beim Lesen: " + e.getMessage()));
                        keepUpdating = false; // Schleife beenden
                        return;
                    }
                }

                // Nächste Positionsabfrage Planen
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

    /**
     * Wird Aufgerufen Beim Drücken Des Parking Buttons
     * Positionsabfrage Des Roboters Stoppen Um Anfrage An ParkingSpots Zu Senden
     * Befehl Wird Gesendet Und Empfangene Spots In Array Gespeichert
     * Ausgabe Anzahl ParkingSpots
     */
    private void anfrageParkingSpots() {
        new Thread(() -> {
            if (socket != null && socket.isConnected()) {
                try {
                    keepUpdating = false; // Positionen pausieren
                    handler.removeCallbacks(positionUpdater);

                    // Befehl Senden Und Daten Empfangen
                    synchronized (reader) {
                        outputStream.write("2\n".getBytes(StandardCharsets.UTF_8));

                        ArrayList<ParkingSpot> spots = readParkingSpots();

                        runOnUiThread(() -> {
                            mapView.updateParkingSpots(spots);
                            textViewModus.setText("Parkplätze empfangen: " + spots.size());

                            // Positionen Wieder Aktivieren
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

    /**
     * Wird Aufgerufen Von anfrageParkingSpots
     * Lesen Der ParkingSpots Von Empfangenen Daten
     * Wartet Bis "s" Kommt -> Sammelt Und Liest Daten -> Bis "end"
     * Empfangene Daten Trennen Nach ";"
     * -> 6 Empfangene Daten Bilden Einen ParkingSPot
     * Nacheinander Spots Generieren Und Diese Zur Liste Hinzufügen
     * @return Liste Der Empfangenen ParkingSpots
     * @throws IOException FÜr Fehler
     */
    private ArrayList<ParkingSpot> readParkingSpots() throws IOException {
        ArrayList<ParkingSpot> spots = new ArrayList<>();

        // Warten bis "s" kommt
        String line;
        do {
            line = reader.readLine();    // blockiert wie bei Positionsdaten
            if (line == null) return spots;
        } while (!line.equals("s"));

        // Zeilen Sammeln Bis "end"
        StringBuilder buffer = new StringBuilder();
        while (true) {
            line = reader.readLine();    // blockiert
            if (line == null) break;
            if (line.equals("end")) break;

            buffer.append(line).append(";");
        }

        // Tokens Parsen
        String[] tokens = buffer.toString().split(";");

        // Alle 6 Parameter Bilden Einen ParkingSpot
        for (int i = 0; i + 5 < tokens.length; i += 6) {

            spots.add(createParkingSpot(Arrays.copyOfRange(tokens, i, i+6)));
        }
        return spots;
    }

    /**
     * Aufgerufen Von readParkingSpots
     * Einem ParkingSpot Seine Daten Zuordnen
     * @param data 6 Daten Bilden Eine Parklücke -> Werden Den Start Und End Werten Zugeordnet
     * @return Fertige Parklücke
     */
    private ParkingSpot createParkingSpot(String[] data){
        // Daten lesen Und In FLoat Wandeln
        float id = Float.parseFloat(data[0]);
        float x1 = Float.parseFloat(data[1]);
        float y1 = Float.parseFloat(data[2]);
        float x2 = Float.parseFloat(data[3]);
        float y2 = Float.parseFloat(data[4]);
        float suitable = Float.parseFloat(data[5]);

        // Verhältnis Karte
        float scaleX = 1.1167f;
        float scaleY = 1.09167f;

        // Start Und Endkoordinaten
        float xStart = x1;
        float yStart = y1;
        float xEnd   = x2;
        float yEnd   = y2;

        // Offset Der Karte
        float offsetX = 250;
        float offsetY = 775;

        // Parklücke Definieren -> Koordinaten Definieren Für MapView Zum Zeichnen
        if (yStart == yEnd) {// Horizontale Parklücke Nach Unten
            xStart = offsetX + xStart * scaleX;
            yStart = offsetY - yStart * scaleY;
            xEnd = offsetX + xEnd * scaleX;
            yEnd = offsetY - (yEnd - 125) * scaleY;
        } else if (xStart == xEnd && yStart < yEnd) { // Vertikale Parklücke Nach Rechts
            xStart = offsetX+xStart*scaleX;
            yStart = offsetY-y2*scaleY;
            xEnd   = offsetX+(xEnd+125)*scaleX;
            yEnd   = offsetY-y1*scaleY;
        } else if (xStart == xEnd && yStart> yEnd) { // Vertikale Parklücke Nach Links
            xStart = offsetX+(xStart-125)*scaleX;
            yStart = offsetY-yStart*scaleY;
            xEnd   = offsetX+xEnd*scaleX;
            yEnd   = offsetY-yEnd*scaleY;
        }

        return new ParkingSpot(id, xStart, yStart, xEnd, yEnd, suitable);
    }

    /**
     * Ausgeführt Wenn In MapView Tippen Erkannt Wurde
     * Sender Der Gefundenen Id
     * @param spotId
     */
    public void onParkingSpotClicked(float spotId) {
        sendSpotId(spotId);
        System.out.println("onParkingSpotClicked");
    }

    /**
     * Senden Der ID Aus onParkingSpotClicked Als Command mit ID
     * @param id Der Parklücke
     * Ausgabe der ID
     */
    public void sendSpotId(float id){
        int ID = (int) id;
        sendCommand("3 "+ ID +"\r");
        textViewModus.setText("Einparken: " + ID);
        System.out.println("sendSpotId");
    }

    /**
     * Aufgerufen in onCreat Beim Drücken Der Buttons
     * Setzt Aktiven Button (Button Der Gedrückt Wurde) Auf Aktive Farbe
     * Alle Anderen Buttons Werden Auf Inaktive Farbe Gesetzt
     * @param activeButton
     */
    private void activeMode(Button activeButton) {
        // Alle Buttons Zurücksetzen
        buttonIdle.setBackgroundTintList(
                getColorStateList(R.color.inactive));
        buttonScout.setBackgroundTintList(
                getColorStateList(R.color.inactive));
        buttonParking.setBackgroundTintList(
                getColorStateList(R.color.inactive));
        buttonSetup.setBackgroundTintList(
                getColorStateList(R.color.inactive));

        // Aktiven Button Hervorheben
        activeButton.setBackgroundTintList(
                getColorStateList(R.color.active));
    }

}