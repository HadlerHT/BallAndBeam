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

  Configuration Section
  
  This section grants access to basic settings, such as:
    - PID paramenters
    - Sensor configuration
    - Kalman filter parameters
    - Mechanical and hardware definitions
    - Firmware preferences
    
================================================================================*/

//==========================================
//============== PID Settings ==============
//==========================================
#define PID_SETPOINT                     320

#define PID_KP                       0.02800
#define PID_KI                       0.00019
#define PID_KD                       0.04950

#define PID_INTEGRATIVE_REFRESH_MS         8
#define PID_DERIVATIVE_REFRESH_MS         50

#define ERROR_ADMISSION                 0.10


//==========================================
//================ HARDWARE ================
//==========================================
#define MOTOR_CONTROL_PIN                  9

#define SENSOR_VL53L0X
//#define SENSOR_HSR04
//#define SENSOR_GENERIC_ANALOG

#ifdef SENSOR_VL53L0X 
  #define SENSOR_RESET_PIN                 8
  #define ATTEMPTS_TO_BOOT_SENSOR          3
  #define SENSOR_TIMEOUT_MS              500         
#endif

#ifdef SENSOR_HSR04
  #define ECCHO_PIN                        2
  #define TRIGGER_PIN                      3 
  #define SENSOR_TIMEOUT_MS              500  
#endif

#ifdef SENSOR_GENERIC_ANALOG 
  #define ANALOG_INPUT_PIN                A0
#endif


//==========================================
//=============== MECHANICAL ===============
//==========================================
#define BEAM_RADIUS_MM                   175
#define BEAM_LENGTH_MM                   500
#define MOTOR_RADIUS_MM                   40

#define BEAM_HYSTERESIS_DEG                8
#define MOTOR_OFFSET_DEG                65.5

#define MOTOR_ORIENTATION               LEFT


//==========================================
//================ FIRMWARE ================
//==========================================
#define USART_BAUDRATE                115200


//==========================================
//================= KALMAN =================
//==========================================
#define KALMAN_ESTIMATED_ERROR          0.50
#define KALMAN_MEASUREMENT_ERROR        0.20
#define KALMAN_PROCESS_NOISE            0.01
