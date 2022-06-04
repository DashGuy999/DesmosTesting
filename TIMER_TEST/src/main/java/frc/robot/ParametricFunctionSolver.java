// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This is the main paper I referenced when creating this file:
//https://people.cs.clemson.edu/~dhouse/courses/405/notes/splines.pdf
package frc.robot;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.awt.Desktop;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

/** Add your docs here. */
public class ParametricFunctionSolver {

    private final double[] endChunk = {-1,-1};
    private final double[] endPath = {-2,-2};

    /*
    private double[][] allCoordinates = 
    {{5,1, 0}, {2,-9, 1}, {-3,-3, 2}, {-10,10, 3}, endChunk,
    {-10,10, 4}, {0,0, 5}, {4,-7, 6}, {9,4, 7}, endChunk,
    endPath};
    */
    private double[][] allCoordinates = 
    {{0,0, 0}, {1,2, 1}, endChunk, 
    {1,2, 2}, {1,-4, 3}, {3,-5, 4}, endChunk,
    endPath};

    private double[][] chunk = new double[4][3];
    private double[] coordinate = new double[3];
    private int chunkLength = 0;

    private SimpleMatrix A;
    private SimpleMatrix B;
    private SimpleMatrix C;
    


  public ParametricFunctionSolver() {  
        
        SmartDashboard.putString("Trip1", "Before Loop");

        for(int outerLoop = 0; coordinate != endPath; outerLoop++){

            SmartDashboard.putNumber("outerLoop", outerLoop);
            
            coordinate = allCoordinates[outerLoop];

            if(coordinate != endChunk){
                SmartDashboard.putString("Trip2", "Filling Chunks");
                chunk[chunkLength] = coordinate;
                chunkLength++;
                continue;
            }

            A = new SimpleMatrix(chunkLength, chunkLength);
            B = new SimpleMatrix(chunkLength, 1);
            C = new SimpleMatrix(chunkLength,2);

            for(int i = 0; i < chunkLength; i++){
                SmartDashboard.putString("Trip3", "Inner Loop 1");
                for(int j = 0; j < chunkLength; j++){
                    A.set(i, j, Math.pow(chunk[i][2], j));
                }
                
            }
        
            
            for(int i = 0; i < 2; i++){
                SmartDashboard.putString("Trip4", "Inner Loop 2");
                for(int j = 0; j < chunkLength; j++){
                    B.set(j, 0, chunk[j][i]);
                }
                SimpleMatrix invA = A.invert();

                C.insertIntoThis(0, i, invA.mult(B)); 
            }

            try{
                SmartDashboard.putNumber("AX", C.get(0, 0));
                SmartDashboard.putNumber("BX", C.get(1, 0));
                SmartDashboard.putNumber("CX", C.get(2, 0));
                //SmartDashboard.putNumber("DX", C.get(3, 0));
                
                SmartDashboard.putNumber("AY", C.get(0, 1));
                SmartDashboard.putNumber("BY", C.get(1, 1));
                SmartDashboard.putNumber("CY", C.get(2, 1));
                //SmartDashboard.putNumber("DY", C.get(3, 1));
            }
            catch(Exception exception){

            }

            
            
            chunkLength = 0;
        }
        SmartDashboard.putString("Trip", "After Loop");
        //Opening Desmos

        
        try {

            URI desmosURI = new URI("https://dashguy999.github.io/DesmosTesting/");

            
            String queryString = "value= 3t^3+2t^2";

            //SmartDashboard.putString("desmosURI.getQuery before", desmosURI.getQuery());

            String newQuery = desmosURI.getQuery();
            if (newQuery == null) {
                newQuery = queryString;
            } else {
                newQuery += "&" + queryString;  
            }
            
            URI newDesmosURI = new URI(desmosURI.getScheme(), desmosURI.getAuthority(),
            desmosURI.getPath(), newQuery, desmosURI.getFragment());

            SmartDashboard.putString("Generated URL", newDesmosURI.toString());
            SmartDashboard.putString("desmosURI.getQuery after", newDesmosURI.getQuery());
            
            Desktop.getDesktop().browse(newDesmosURI);
            


        } 
        catch (URISyntaxException e) {
            SmartDashboard.putString("ERROR", "Something Went Wrong (catch1)");
        }
        catch(IOException e){
            SmartDashboard.putString("ERROR", "Something Went Wrong (catch2)");
        }
        
        

        

    }

}