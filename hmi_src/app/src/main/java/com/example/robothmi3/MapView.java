package com.example.robothmi3;

import android.content.Context;
import android.graphics.*;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

import androidx.annotation.Nullable;
import androidx.annotation.StyleableRes;
import androidx.lifecycle.DefaultLifecycleObserver;

import java.util.ArrayList;

public class MapView extends View {

    // On Spot Clicked Event
    public interface OnParkingSpotClickListener {
        void onParkingSpotClicked(float spotId);
    }
    // Listener
    private OnParkingSpotClickListener listener;


    private Paint parkingPaintSuitable;
    private Paint parkingPaintNotSuitable;
    private Paint distanceLineColor;
    private Bitmap mapBitmap;
    private Bitmap robotBitmap;

    // Roboterposition
    private float robotX = 0;
    private float robotY = 0;
    private float robotPhi = 0;

    private float robotDx = 0;
    private float robotDy = 0;

    public float offsetX = 250;
    public float offsetY = 775;
    public float offsetPhi = -90;

    // Pfadliste
    private final ArrayList<PointF> pathPoints = new ArrayList<>();

    //Parklückenliste
    private ArrayList<ParkingSpot> parkingSpots = new ArrayList<>();

    private final Paint pathPaint;

    public MapView(Context ctx, @Nullable AttributeSet attrs) {
        super(ctx, attrs);

        // Bitmap definition
        mapBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.karte_roboter);
        robotBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.auto);

        // ParkingSpot-Farben definition
        parkingPaintSuitable = new Paint();
        //parkingPaintSuitable.setColor(Color.GREEN);
        parkingPaintSuitable.setColor(Color.parseColor("#27C93F")); // Grün
        parkingPaintSuitable.setAlpha(255);

        parkingPaintNotSuitable = new Paint();
        //parkingPaintNotSuitable.setColor(Color.RED);
        parkingPaintNotSuitable.setColor(Color.parseColor("#FF5F57")); // Rot
        parkingPaintNotSuitable.setAlpha(255);

        // Pfad definition
        pathPaint = new Paint();
        //pathPaint.setColor(Color.RED);
        pathPaint.setColor(Color.parseColor("#FF5F57")); // Rot
        pathPaint.setStrokeWidth(5f);
        pathPaint.setStyle(Paint.Style.STROKE);

        distanceLineColor = new Paint();
        //distanceLineColor.setColor(Color.BLUE);
        distanceLineColor.setColor(Color.parseColor("#1E6AFF")); // Blau
        distanceLineColor.setStrokeWidth(5f);
        distanceLineColor.setStyle(Paint.Style.STROKE);

    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        // Karte zeichnen
        Rect dst = new Rect(0, 0, getWidth(), getHeight());
        canvas.drawBitmap(mapBitmap, null, dst, null);

        // Pfad zeichnen
        if (pathPoints.size() > 1) {
            for (int i = 0; i < pathPoints.size() - 1; i++) {
                PointF p1 = pathPoints.get(i);
                PointF p2 = pathPoints.get(i + 1);
                canvas.drawLine(p1.x, p1.y, p2.x, p2.y, pathPaint);
            }
        }
        //canvas.drawRect(250.0f,120.0f,585.0f,775.0f,parkingPaintSuitable);

        // Parklücken zeichnen
        if (!parkingSpots.isEmpty()){ //parkingSpots.size() > 0

            for (ParkingSpot parkingSpot : parkingSpots) {
                Paint fillPaint = (parkingSpot.suitable == 1)
                        ? parkingPaintSuitable
                        : parkingPaintNotSuitable;

                // Fläche
                //canvas.drawRect(left, top, right, bottom, fillPaint);
                canvas.drawRect(parkingSpot.x1, parkingSpot.y1, parkingSpot.x2, parkingSpot.y2, fillPaint);

                // Parkplatz ID zeichnen
                Paint textPaint = new Paint();
                textPaint.setColor(Color.BLACK);
                textPaint.setTextSize(35);
                canvas.drawText("ID " + parkingSpot.id, parkingSpot.x1 + 10, parkingSpot.y1 + 40, textPaint);
            }
        }

        // Roboter drehen + zeichnen
        canvas.save();
        canvas.translate(robotX, robotY);
        canvas.rotate(-robotPhi);
        canvas.scale(0.2f, 0.2f);
        canvas.drawLine(robotX, robotY, robotDx, robotDy, distanceLineColor);


        // Mittelpunkt des Roboters richtig setzen
        canvas.drawBitmap(robotBitmap,
                -robotBitmap.getWidth() / 2f,
                -robotBitmap.getHeight() / 2f,
                null);

        canvas.restore();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        float posX = event.getX();
        float posY = event.getY();

        int action = event.getAction();
        if (action == MotionEvent.ACTION_DOWN) {
            ParkingSpot clickedSpot = checkParkingSpot(posX,posY);
            if (clickedSpot != null){
                System.out.println("Spot Valid And Clicked");
                listener.onParkingSpotClicked(clickedSpot.id);
            }
            return true;
        }
        return false;
    }

    private ParkingSpot checkParkingSpot(float x, float y) {
        for (ParkingSpot parkingSpot: parkingSpots){
            if (isBetween(x, parkingSpot.x1, parkingSpot.x2) && isBetween(y, parkingSpot.y1, parkingSpot.y2)) {
                if (parkingSpot.suitable != 1) {
                    return null;
                }
                return parkingSpot;
            }
        }
        return null;
    }

    private boolean isBetween(float value, float v1, float v2){
        if(v1 >= v2) {
            return (value < v1 && value > v2);
        }else {
            return (value > v1 && value < v2);
        }

    }

    /**
     * Wird von der MainActivity aufgerufen, wenn neue Positionen eintreffen.
     */
    public void updateRobotPose(float x, float y, float phi, float dx, float dy) {
        x += offsetX;
        y += offsetY;
        phi += offsetPhi;
        dx += offsetX;
        y += offsetY;

        robotX = x;
        robotY = y;
        robotPhi = phi;
        robotDx = dx;
        robotDy = dy;

        // Pfadpunkt hinzufügen
        pathPoints.add(new PointF(x, y));

        invalidate();  // View neu zeichnen
    }
    /**
     * Wird von der MainActivity aufgerufen, wenn die ParkingSports eintreffen.
     */
    public void updateParkingSpots(ArrayList<ParkingSpot> parkingSpots) {
        this.parkingSpots = parkingSpots;

        invalidate();
    }

    /**
     * Verbindet den gefragten listener in dem Skript
     * @param listener
     */
    public void setOnParkingSpotClickListener(OnParkingSpotClickListener listener) {
        this.listener = listener;
    }
}

