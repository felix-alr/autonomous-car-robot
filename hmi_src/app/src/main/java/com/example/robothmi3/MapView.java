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
    private Bitmap mapBitmap;
    private Bitmap robotBitmap;

    // Roboterposition
    private float robotX = 0;
    private float robotY = 0;
    private float robotPhi = 0;

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
        parkingPaintSuitable.setColor(Color.GREEN);
        parkingPaintSuitable.setAlpha(140);
        parkingPaintNotSuitable = new Paint();
        parkingPaintNotSuitable.setColor(Color.RED);
        parkingPaintNotSuitable.setAlpha(140);

        // Pfad definition
        pathPaint = new Paint();
        pathPaint.setColor(Color.RED);
        pathPaint.setStrokeWidth(5f);
        pathPaint.setStyle(Paint.Style.STROKE);

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

                //float scaleX = 1.1167f;
                //float scaleY = 1.09167f;
                //Koordinaten übernehmen
                //float xStart =  parkingSpot.x1;
                //float yStart = parkingSpot.y1;
                //float xEnd   = parkingSpot.x2;
                //float yEnd   = parkingSpot.y2;

                //if (yStart == yEnd){// Horizontale Parklücke Nach Unten
                //    xStart = offsetX+xStart*scaleX;
                //    yStart = offsetY-yStart*scaleY;
                //    xEnd   = offsetX+xEnd*scaleX;
                //    yEnd   = offsetY-(yEnd-125)*scaleY;
                //    canvas.drawRect(xStart, yStart, xEnd, yEnd, fillPaint);
                //} else if (xStart == xEnd && yStart < yEnd) { // Vertikale Parklücke Nach Rechts
                //    xStart = offsetX+xStart*scaleX;
                //    yStart = offsetY-parkingSpot.y2*scaleY;
                //    xEnd   = offsetX+(xEnd+125)*scaleX;
                //    yEnd   = offsetY-parkingSpot.y1*scaleY;
                //    canvas.drawRect(xStart, yStart, xEnd, yEnd, fillPaint);
                //} else if (xStart == xEnd && yStart> yEnd) { // Vertikale Parklücke Nach Links
                //    xStart = offsetX+(xStart-125)*scaleX;
                //    yStart = offsetY-yStart*scaleY;
                //    xEnd   = offsetX+xEnd*scaleX;
                //    yEnd   = offsetY-yEnd*scaleY;
                //    canvas.drawRect(xStart, yStart, xEnd, yEnd, fillPaint);
                //}

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
                listener.onParkingSpotClicked(clickedSpot.id);
            }
            return true;
        }
        return false;
    }

    private ParkingSpot checkParkingSpot(float x, float y) {
        for (ParkingSpot parkingSpot: parkingSpots){
            if (isBetween(x, parkingSpot.x1, parkingSpot.x2) && isBetween(y, parkingSpot.y1, parkingSpot.y2)) {
                return parkingSpot;
            }
        }
        return null;
    }

    private boolean isBetween(float test, float max, float min){
        return (test < max && test > min);
    }

    /**
     * Wird von der MainActivity aufgerufen, wenn neue Positionen eintreffen.
     */
    public void updateRobotPose(float x, float y, float phi) {
        x += offsetX;
        y += offsetY;
        phi += offsetPhi;

        robotX = x;
        robotY = y;
        robotPhi = phi;

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

