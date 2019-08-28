package app.example.cardiosafe;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Gravity;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import com.google.firebase.database.ChildEventListener;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.scichart.charting.ClipMode;
import com.scichart.charting.model.dataSeries.XyDataSeries;
import com.scichart.charting.modifiers.AxisDragModifierBase;
import com.scichart.charting.modifiers.ModifierGroup;
import com.scichart.charting.visuals.SciChartSurface;
import com.scichart.charting.visuals.annotations.HorizontalAnchorPoint;
import com.scichart.charting.visuals.annotations.TextAnnotation;
import com.scichart.charting.visuals.annotations.VerticalAnchorPoint;
import com.scichart.charting.visuals.axes.IAxis;
import com.scichart.charting.visuals.pointmarkers.EllipsePointMarker;
import com.scichart.charting.visuals.renderableSeries.IRenderableSeries;
import com.scichart.core.annotations.Orientation;
import com.scichart.core.framework.UpdateSuspender;
import com.scichart.drawing.utility.ColorUtil;
import com.scichart.extensions.builders.SciChartBuilder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class GraphActivity extends AppCompatActivity {

    private DatabaseReference databaseReferencia = FirebaseDatabase.getInstance().getReference();
    private DatabaseReference temperaturaRef = databaseReferencia.child("Temp");
    private DatabaseReference sinal = databaseReferencia.child("Sinal");

    Float[] array = new Float[9];
    Integer[] xArray = new Integer[9];
    int i = 0, x = 0, cont = 0;
    String data = new String();
    Boolean flag = false;
    Boolean flag_init = false;
    String key = new String();

    SciChartSurface surface;
    XyDataSeries lineData;
    XyDataSeries scatterData;
    IRenderableSeries lineSeries;
    EllipsePointMarker pointMarker;
    IRenderableSeries scatterSeries;

    Button iniciar;
    Button pausar;
    Button parar;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_graph);

        surface = findViewById(R.id.layout);

        iniciar = (Button) findViewById(R.id.iniciarGraph);
        pausar = findViewById(R.id.pausar);
        parar = (Button) findViewById(R.id.parar);
        // Get a layout declared in "activity_main.xml" by id
        // LinearLayout chartLayout = (LinearLayout) findViewById(R.id.layout);

        // Add the SciChartSurface to the layout
        // chartLayout.addView(surface);
        xArray[0] = 0;
        xArray[8] = 0;
        // Initialize the SciChartBuilder
        SciChartBuilder.init(this);

        // Obtain the SciChartBuilder instance
        final SciChartBuilder sciChartBuilder = SciChartBuilder.instance();

        // Create a numeric X axis
        final IAxis xAxis = sciChartBuilder.newNumericAxis()
                .withAxisTitle("Tempo em s")
                .withVisibleRange(0, 100)
                .build();

        // Create a numeric Y axis
        final IAxis yAxis = sciChartBuilder.newNumericAxis()
                .withAxisTitle("Posição").withVisibleRange(0, 110).build();

        // Create a TextAnnotation and specify the inscription and position for it
        TextAnnotation textAnnotation = sciChartBuilder.newTextAnnotation()
                .withX1(5.0)
                .withY1(55.0)
                .withHorizontalAnchorPoint(HorizontalAnchorPoint.Center)
                .withVerticalAnchorPoint(VerticalAnchorPoint.Center)
                .withFontStyle(20, ColorUtil.White)
                .build();

        // Create interactivity modifiers
        ModifierGroup additionalModifiers = sciChartBuilder.newModifierGroup()

                .withPinchZoomModifier().build()

                .withZoomPanModifier().withReceiveHandledEvents(true).build()

                .withZoomExtentsModifier().withReceiveHandledEvents(true).build()

                .withXAxisDragModifier().withReceiveHandledEvents(true).withDragMode(AxisDragModifierBase.AxisDragMode.Scale).withClipModex(ClipMode.None).build()

                .withYAxisDragModifier().withReceiveHandledEvents(true).withDragMode(AxisDragModifierBase.AxisDragMode.Pan).build()

                .build();

        // Add the Y axis to the YAxes collection of the surface
        Collections.addAll(surface.getYAxes(), yAxis);

        // Add the X axis to the XAxes collection of the surface
        Collections.addAll(surface.getXAxes(), xAxis);

        // Add the annotation to the Annotations collection of the surface
        Collections.addAll(surface.getAnnotations(), textAnnotation);

        // Add the interactions to the ChartModifiers collection of the surface
        Collections.addAll(surface.getChartModifiers(), additionalModifiers);

// Set FIFO capacity to 500 on DataSeries
        final int fifoCapacity = 200;

        lineData = sciChartBuilder.newXyDataSeries(Integer.class, Float.class)
                .withFifoCapacity(fifoCapacity)
                .build();


        // Create and configure a line series
        lineSeries = sciChartBuilder.newLineSeries()
                .withDataSeries(lineData)
                .withStrokeStyle(ColorUtil.LightBlue, 2f, true)
                .build();


        // Create an Ellipse PointMarker for the Scatter Series
        pointMarker = sciChartBuilder
                .newPointMarker(new EllipsePointMarker())
                .withFill(ColorUtil.LightBlue)
                .withStroke(ColorUtil.Green, 2f)
                .withSize(10)
                .build();

// Create and configure a scatter series
        scatterSeries = sciChartBuilder.newScatterSeries()
                .withPointMarker(pointMarker)
                .build();

        scatterData = sciChartBuilder.newXyDataSeries(Integer.class, Double.class).build();

        // Create a LegendModifier and configure a chart legend
        ModifierGroup legendModifier = sciChartBuilder.newModifierGroup()
                .withLegendModifier()
                .withOrientation(Orientation.HORIZONTAL)
                .withPosition(Gravity.CENTER_HORIZONTAL | Gravity.BOTTOM, 10)
                .build()
                .build();

        // Add the LegendModifier to the SciChartSurface
        surface.getChartModifiers().add(legendModifier);

        // Create and configure a CursorModifier
        ModifierGroup cursorModifier = sciChartBuilder.newModifierGroup()
                .withCursorModifier().withShowTooltip(true).build()
                .build();

        // Add the CursorModifier to the SciChartSurface
        surface.getChartModifiers().add(cursorModifier);
    }


    protected void onResume(){
        super.onResume();

        iniciar.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                flag = true;
                sinal.setValue(flag);

                Log.i("Flag_init", flag.toString());
            }
        });

        pausar.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                flag = false;
            }
        });

        parar.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                flag = false;
                xArray[0] = 0;
                temperaturaRef.child(key).removeValue();
                sinal.setValue(flag);
                lineData.clear();
            }
        });

    //   if (flag_init == true) {
            ChildEventListener childEventListener = new ChildEventListener() {
                @Override
                public void onChildAdded(@NonNull DataSnapshot dataSnapshot, @Nullable String s) {
                    data = dataSnapshot.getValue().toString();

                    key = dataSnapshot.getKey();
                    Log.i("data", "entrou no listener");
                    final String[] dataSplit = data.split(",");
                    cont = dataSplit.length;
                    System.out.println(cont);
                    converteDados(dataSplit, cont);

                    UpdateSuspender.using(surface, new Runnable() {    //This updater graphs the values
                        @Override
                        public void run() {
                            if (flag == true) {
                                Log.i("Thread", "entrou na thread ");
                                lineData.append(xArray, array);
                                surface.zoomExtents();
                                xArray[0] = xArray[8];
                            }


                           // flag = false;
                        }
                    });

                }


                @Override
                public void onChildChanged(@NonNull DataSnapshot dataSnapshot, @Nullable String s) {

                }

                @Override
                public void onChildRemoved(@NonNull DataSnapshot dataSnapshot) {

                }

                @Override
                public void onChildMoved(@NonNull DataSnapshot dataSnapshot, @Nullable String s) {

                }

                @Override
                public void onCancelled(@NonNull DatabaseError databaseError) {

                }

            };
            temperaturaRef.addChildEventListener(childEventListener);

            surface.getRenderableSeries().add(lineSeries);
           // surface.getRenderableSeries().add(scatterSeries);
        }


 //   }
    private void converteDados(String[] data, int cont){
        //System.out.println(contador);
        for (int i = 0; (i < cont); i++){
            if (data[i] != null)
                array[i] = Float.valueOf(data[i]);
            xArray[i] = xArray[0] + i;
        }

      //  calculaPosicao(array);
       // flag = true;
    }

    /*
    private void calculaPosicao(ArrayList<Float> vetor){
        float valorAnterior;
        float valorAtual;


    }
*/
}