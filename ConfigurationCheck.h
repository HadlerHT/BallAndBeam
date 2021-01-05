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

  Configuration Check Section
  
  This section is based on preprocessing features, it assures that the inputs 
  inserted on the Configuration.h file are not incompatible with the code.
  It also defines some useful macros throughout the code.
  
  Obs.: There is no need of changing anything in here.
    
================================================================================*/

//======================== MACROS ========================
#define VALIDATE(x, a, b) (x < a) || (x > b) || (x == MOTOR_CONTROL_PIN)
#define RIGHT  1
#define LEFT  -1

//================== COMPILER CHECKING ===================
#ifndef __AVR_ATmega328P__
  #warning "This code was developed to be used on a Atmega 328P based board, using another microcontroler may cause incompatibilities"
#endif

//=============== CONFIGURATION.H CHECKING ===============
#if MOTOR_CONTROL_PIN != 9 && MOTOR_CONTROL_PIN != 10
  #error "The chosen pin to the motor is not compatible. Try 9 or 10."
#endif

#if defined(SENSOR_VL53L0X) + defined(SENSOR_HSR04) + defined(SENSOR_GENERIC_ANALOG) > 1
  #error "Only one sensor can be used at a time."
#elif defined(SENSOR_VL53L0X) + defined(SENSOR_HSR04) + defined(SENSOR_GENERIC_ANALOG) == 0
  #error "No sensors have been defined" 
#endif

#ifdef SENSOR_VL53L0X 
  #if VALIDATE (SENSOR_RESET_PIN, 2, 13)
    #error "The chosen pin to sensor reset is not available or doesn't exist."
  #endif
#endif

#ifdef SENSOR_HSR04
  #if (TRIGGER_PIN == ECCHO_PIN)
    #error "Trigger and eccho pins can't be the same"
  #elif VALIDATE (TRIGGER_PIN, 2, 13) || VALIDATE (ECCHO_PIN, 2, 13)
    #error "The chosen pins to sensor eccho or trigger are not available or don't exist."
  #endif
#endif

#ifdef SENSOR_GENERIC_ANALOG
  #if VALIDATE (ANALOG_INPUT_PIN, 0, 5)
    #error "The chosen analog pin is not available or doesn't exist"
  #endif
#endif
