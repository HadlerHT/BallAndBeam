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

  Ball And Beam Class
    
================================================================================*/

#ifndef BallAndBeam_h
#define BallAndBeam_h

//======= REFERENCES ========
#include "Configuration.h"
#include "ConfigurationCheck.h"

//======== LYBRARIES ========
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>

class BallAndBeam {
    
    public :
    
        BallAndBeam ();

        void  initialize (void);
        float readSensor (bool useKalman);
        void  runPID (void); 

        void  setPIDParameters (float _kp, float _ki, float _kd);
        
        void  plotPID (bool justOutput);
        void  runInteract (void);
        
    private :
    
        VL53L0X laserSensor;
        Servo   servo;
        
        bool bootSensor (void);
        void runKalmanFilter (float *handledVariable);
        void writeBeamAngle (float beamAngle);
                
        float    estimatedError         = KALMAN_ESTIMATED_ERROR, 
                 measurementError       = KALMAN_MEASUREMENT_ERROR, 
                 processNoise           = KALMAN_PROCESS_NOISE, 
                 lastKalmanOutput       = 0,
                 gain;
                 
        unsigned integrativeRefreshTime = PID_INTEGRATIVE_REFRESH_MS,
                 derivativeRefreshTime  = PID_DERIVATIVE_REFRESH_MS;
        float    setPoint               = PID_SETPOINT,
                 kp                     = PID_KP,
                 ki                     = PID_KI, 
                 kd                     = PID_KD,
                 proportional           = 0, 
                 integrative            = 0, 
                 derivative             = 0,
                 derivativeBuffer       = 0, 
                 outputPID              = 0;
        int32_t  integrativeTimeCounter = 0,
                 derivativeTimeCounter  = 0;
        
        float    beamRadius             = BEAM_RADIUS_MM,
                 beamLength             = BEAM_LENGTH_MM, 
                 beamHysteresis         = BEAM_HYSTERESIS_DEG,
                 motorRadius            = MOTOR_RADIUS_MM ,
                 motorOffset            = MOTOR_OFFSET_DEG;                     
};
#endif
