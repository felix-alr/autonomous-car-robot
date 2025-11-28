package com.example.robothmi3;

import android.content.Context;
import android.graphics.*;
import android.util.AttributeSet;
import android.view.View;

import androidx.annotation.Nullable;
import androidx.annotation.StyleableRes;
import androidx.lifecycle.DefaultLifecycleObserver;

import java.util.ArrayList;

public class MapView extends View {
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

        // Parklücken zeichnen
        if (!parkingSpots.isEmpty()){ //parkingSpots.size() > 0

            for (ParkingSpot parkingSpot : parkingSpots) {
                Paint fillPaint = (parkingSpot.suitable == 1)
                        ? parkingPaintSuitable
                        : parkingPaintNotSuitable;

                float scale = 1.13f;
                //Koordinaten übernehmen
                float xStart =  parkingSpot.x1;
                float yStart = parkingSpot.y1;
                float xEnd   = parkingSpot.x2;
                float yEnd   = parkingSpot.y2;

                if (yStart == yEnd){// Horizontale Parklücke Nach Unten
                    xStart = offsetX+xStart*scale;
                    yStart = offsetY-yStart*scale;
                    xEnd   = offsetX+xEnd*scale;
                    yEnd   = offsetY-(yEnd-125)*scale;
                    canvas.drawRect(xStart, yStart, xEnd, yEnd, fillPaint);
                } else if (xStart == xEnd && yStart < yEnd) { // Vertikale Parklücke Nach Rechts
                    xStart = offsetX+xStart*scale;
                    yStart = offsetY-parkingSpot.y2*scale;
                    xEnd   = offsetX+(xEnd+125)*scale;
                    yEnd   = offsetY-parkingSpot.y1*scale;
                    canvas.drawRect(xStart, yStart, xEnd, yEnd, fillPaint);
                } else if (xStart == xEnd && yStart> yEnd) { // Vertikale Parklücke Nach Links
                    xStart = offsetX+(xStart-125)*scale;
                    yStart = offsetY-yStart*scale;
                    xEnd   = offsetX+xEnd*scale;
                    yEnd   = offsetY-yEnd*scale;
                    canvas.drawRect(xStart, yStart, xEnd, yEnd, fillPaint);
                }

                // Fläche
                //canvas.drawRect(left, top, right, bottom, fillPaint);

                // Parkplatz ID zeichnen
                Paint textPaint = new Paint();
                textPaint.setColor(Color.BLACK);
                textPaint.setTextSize(35);
                canvas.drawText("ID " + parkingSpot.id, xStart + 10, yStart + 40, textPaint);
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
}

