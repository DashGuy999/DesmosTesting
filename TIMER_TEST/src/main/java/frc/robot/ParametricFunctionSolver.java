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
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.net.URI;
import java.net.URISyntaxException;

/** Add your docs here. */
public class ParametricFunctionSolver {

    private final double[] endChunk = {-1,-1};
    private final double[] endPath = {-2,-2};


    private double[][] allCoordinates = 
    {{0,4}, {2,-3}, {-5,1}, {6,3}, endChunk,
    {6,3}, {5,0}, {0,-3}, {2,-5}, endChunk,
    endPath};

    private double[][] chunk = new double[4][3];
    private double[] coordinate = new double[3];
    private int chunkLength = 0;
    private int uImprint = 0;

    private SimpleMatrix A;
    private SimpleMatrix B;
    private SimpleMatrix C;
    
    private String queryString = "";


  public ParametricFunctionSolver() {  
        

        for(int outerLoop = 0; coordinate != endPath; outerLoop++){

            
            
            coordinate = allCoordinates[outerLoop];

            if(coordinate != endChunk){
                SmartDashboard.putString("Trip2", "Filling Chunks");
                chunk[chunkLength][0] = coordinate[0];
                chunk[chunkLength][1] = coordinate[1];
                chunk[chunkLength][2] = uImprint;
                uImprint++;
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

            

            //Setting up query string
            //Testing query string: ?u=1&x=1&x=2&y=1&y=2&br&u=4&x=2&x=3&y=4&y=2&
            
            queryString = queryString + "u=" + chunk[chunkLength-1][2] + "&";

            for(int i = 0; i < C.numRows(); i++){
                BigDecimal rounder = new BigDecimal(C.get(i, 0));
                BigDecimal rounded = rounder.setScale(4, RoundingMode.FLOOR);
                queryString = queryString + "x=" + rounded + "&";
            }
            for(int i = 0; i < C.numRows(); i++){
                BigDecimal rounder = new BigDecimal(C.get(i, 1));
                BigDecimal rounded = rounder.setScale(4, RoundingMode.FLOOR);
                queryString = queryString + "y=" + rounded + "&";
            }
            queryString = queryString + "br&";
            
            uImprint--;

            chunkLength = 0;
        }
        SmartDashboard.putString("Trip", "After Loop");
        
        //Shave off extra br&
        queryString = queryString.substring(0,queryString.length()-3);
        
        //Opening Desmos
        
        try {

            URI desmosURI = new URI("https://dashguy999.github.io/DesmosTesting/");

            //SmartDashboard.putString("desmosURI.getQuery before", desmosURI.getQuery());

            
            
            URI newDesmosURI = new URI(desmosURI.getScheme(), desmosURI.getAuthority(),
            desmosURI.getPath(), queryString, desmosURI.getFragment());

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