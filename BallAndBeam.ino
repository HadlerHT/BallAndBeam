/*=============================================================================
  CENTRO FEDERAL DE EDUCAÇÃO TECNOLÓGICA DE MINAS GERAIS - CEFET-MG
  Departamento de Mecatrônica
  Laboratório de Automação e Sistemas Embarcados - LASE
  
  LASE Ball and Beam kit.
  (c) 2019-2020 LASE - CEFET MG.

  Author:         Hadler Henrique Tempesta
  Orientation:    Dr. Juliano Coelho Miranda
  Co-Orientation: Ms. Daniel Soares de Alcâtara
================================================================================*/

#include "BallAndBeam.h"

BallAndBeam bnb;

void setup() {
    bnb.initialize (); 
}  

void loop() {

    bnb.runPID ();
    //bnb.plotPID (false);
    //bnb.runInteract ();
}
