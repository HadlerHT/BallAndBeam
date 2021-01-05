/*=============================================================================
  CENTRO FEDERAL DE EDUCAÇÃO TECNOLÓGICA DE MINAS GERAIS - CEFET-MG
  Departamento de Mecatrônica
  Laboratório de Automação e Sistemas Embarcados - LASE
  
  LASE Ball and Beam kit.
  (c) 2019-2020 LASE - CEFET MG.

  Author:         Hadler Henrique Tempesta
  Orientation:    Dr. Juliano Coelho Miranda
  Co-Orientation: Ms. Daniel Soares de Alcâtara

  ----------------------------------------------------------------------------

  Ball And Beam Class Functions
    
================================================================================*/

#include "BallAndBeam.h"

BallAndBeam::BallAndBeam () {
       
}

void BallAndBeam::initialize (void) {

    Serial.begin (USART_BAUDRATE);
    
    //-- SERVO --
    servo.attach (MOTOR_CONTROL_PIN);
    writeBeamAngle (0);
    
    bootSensor ();

    delay (10);
}

#ifdef SENSOR_VL53L0X
bool BallAndBeam::bootSensor (void) {
    
    pinMode      (SENSOR_RESET_PIN, OUTPUT);
    digitalWrite (SENSOR_RESET_PIN,   HIGH);
    
    Wire.begin ();
    delay (1);
    
    laserSensor.setTimeout(SENSOR_TIMEOUT_MS);
    
    short try_to_boot;
    
    for (try_to_boot = 0; try_to_boot < ATTEMPTS_TO_BOOT_SENSOR && !laserSensor.init(); try_to_boot++) {
        Serial.println ("Retring to boot sensor...");
        
        digitalWrite (SENSOR_RESET_PIN,  LOW);
        delayMicroseconds (10);
        digitalWrite (SENSOR_RESET_PIN, HIGH);
        
        delay (10);
    }
    
    if (try_to_boot == ATTEMPTS_TO_BOOT_SENSOR) {
        Serial.println ("\nCritical Failure! Sensor couldn't be accessed.\nPlease check your hardware and then reset.");
        while (1);
    }
    
    else
        laserSensor.startContinuous ();
    
    return 0;
}

float BallAndBeam::readSensor (bool useKalman) {
    
    float measure;
    
    measure = laserSensor.readRangeContinuousMillimeters();
    
    if (laserSensor.timeoutOccurred()) {
        laserSensor.stopContinuous ();
        laserSensor.startContinuous ();
    }

    if (useKalman) 
        runKalmanFilter (&measure);

    //Output unit: mm
    return (measure);
}
#endif

#ifdef SENSOR_HSR04 
bool BallAndBeam::bootSensor (void) {
    
    pinMode (ECCHO_PIN  ,  INPUT);
    pinMode (TRIGGER_PIN, OUTPUT);
    
    digitalWrite (TRIGGER_PIN,  LOW);
    
    return 0;
}

float BallAndBeam::readSensor (bool useKalman) {

    float measure;
    
    digitalWrite (TRIGGER_PIN, HIGH);
    delayMicroseconds (10);
    digitalWrite (TRIGGER_PIN,  LOW);

    measure = pulseIn (ECCHO_PIN, SENSOR_TIMEOUT_MS, HIGH) / 58.84; 
    
    if (useKalman)
        runKalmanFilter (&measure);

    //Output unit: cm
    return measure; 
}
#endif

#ifdef SENSOR_GENERIC_ANALOG
bool BallAndBeam::bootSensor (void) {

    pinMode (ANALOG_INPUT_PIN, INPUT);

    return 0;
}

float BallAndBeam::readSensor (bool useKalman) {

    float measure;

    measure = analogRead (ANALOG_INPUT_PIN);
    
    if (useKalman)
        runKalmanFilter (&measure);

    //Output unit: unknown
    return measure; 
}
#endif

void BallAndBeam::plotPID (bool justOutput) {

    //Must be used with the Arduino Serial Ploter at the correct baudrate
    
    if (!justOutput) {
        Serial.print (proportional, 3); Serial.print ("\t");
        Serial.print (integrative , 3); Serial.print ("\t");
        Serial.print (derivative  , 3); Serial.print ("\t");   
    }

    Serial.println (outputPID, 3);
}

void BallAndBeam::runInteract (void) {

    if (Serial.available ()) {
        char data = Serial.read ();

        //Randomizer
        if (data == 'r'  || data == 'R') {
            Serial.println ("Random");
            
            writeBeamAngle (random (0.5 * beamHysteresis, beamHysteresis) * (random (2) ? 1 : -1));
            delay (random (300, 500));
        }
        
        //Pause  
        if (data == 'p' || data == 'P') {
            data = 0;
            
            writeBeamAngle (0);
            Serial.println ("Paused");
           
            do
                if (Serial.available ())
                    data = Serial.read ();
                    
            while (!(data == 'p' || data == 'P'));
            
            Serial.println ("Unpaused");
        }
    }  
}

void BallAndBeam::runKalmanFilter (float *handledVariable) {

   /*  KALMAN ALGORITHIM DETAILS
    *  
    *  Used/modified by Hadler H. Tempesta, Semptember, 12th, 2020.   
    *  
    *  SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
    *  Created by Denys Sene, January, 1st, 2017.
    *  
    *  Released under MIT License:
    *  -Permission is hereby granted, free of charge, to any person obtaining a copy
    *   of this software and associated documentation files (the "Software"), to deal
    *   in the Software without restriction, including without limitation the rights
    *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    *   copies of the Software, and to permit persons to whom the Software is
    *   furnished to do so, subject to the following conditions:
    *  
    *  -THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    *   SOFTWARE.  
    */

    gain = estimatedError / (estimatedError + measurementError);
    *handledVariable = lastKalmanOutput + (gain * (*handledVariable - lastKalmanOutput));
    estimatedError = ((1.0 - gain) * estimatedError) + (processNoise * fabs (*handledVariable - lastKalmanOutput));

    lastKalmanOutput = *handledVariable;
}

void BallAndBeam::runPID (void) {
    
    /*   -- Proportional --
     *   Given in any time by the difference between the current error (in this case the sensor reading)
     *   and the target value (represented in this setup by the set point), been therefore able to respond
     *   accuratelly to different levels of system error.
     *   
     *   -- Integrative --
     *   Given by the cumulative sum of all errors (proportional values) that have been registred 
     *   in constante periods of time troughout the whole control execution, resembeling a Rieman's 
     *   sum, with the objective of recording past erros that can be considered on the final output. 
     *   
     *   -- Derivative --
     *   Given by the difference between two error values adquired in distincs moments of time, 
     *   with the objective of estimating the curve variation rate at that period and 
     *   therefor try to predict future errors.
     */
      
    proportional = readSensor (true) - setPoint;
    
    if ((millis () - integrativeTimeCounter) > integrativeRefreshTime) {
        integrative += ki * proportional;
        
        integrativeTimeCounter = millis ();
    }

    if ((millis () - derivativeTimeCounter) > derivativeRefreshTime) {
        derivative = kd * (proportional - derivativeBuffer);      
        derivativeBuffer = proportional;
        
        derivativeTimeCounter = millis ();
    }
       
    proportional *= kp;

    outputPID = proportional + integrative + derivative;  
    outputPID = 0.1 * round (10 * outputPID);

    // If the output module is less than ERROR_ADMISSION of the maximum output, it is considered as 0 (zero).
    // The integrative error is also written as 0 (zero) for stabilization and noise canceling purposes.
    if (fabs (outputPID) < (ERROR_ADMISSION * beamHysteresis)) {
        outputPID = 0;
        integrative = 0; 
    }

    writeBeamAngle (outputPID); 
}

void BallAndBeam::setPIDParameters (float _kp, float _ki, float _kd) {
  
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

void BallAndBeam::writeBeamAngle (float beamAngle) {
    
    //The input angle gets limited to avoid mechanical incompatibilities and absurd inputs in the next equation
    beamAngle = constrain (beamAngle, -beamHysteresis, beamHysteresis);
    
    float motorAngle = motorOffset + MOTOR_ORIENTATION * (beamAngle * beamRadius / motorRadius);
   
    servo.write (round (motorAngle));
}
