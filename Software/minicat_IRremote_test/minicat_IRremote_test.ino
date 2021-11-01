/*

======================== ATELIER ROBOTIQUE : CatMini ========================
==                              Premiers pas                               ==
==                        Test capteur infrarouge                          ==
=============================================================================

Copyright (c) 2020 QI Informatique.



Test du montage avec le capteur infrarouge. 

Ce code permet de voir le code du bouton reçu par le capteur. 

*/




#include <IRremote.h> //on inclut la librairie nécéssaire pour les capteurs IR

#define RECV_IR 5 //déclaration du PIN du capteur 


IRrecv irrec(RECV_IR); //on dit au programme que ce Pin est un capteur IR
decode_results result ; //on crée une variable result pour recevoir les informations
//du capteur 

void setup() {

  Serial.begin(57600);
  irrec.enableIRIn(); //initialisation du capteur IR (il attend de recevoir un signal)
}

void loop() {
  
   if (irrec.decode(&result)){ //si une valeur est captée
      Serial.println(result.value, HEX); //On affiche la valeur du signal capté (HEX pour hexadecimal)  
      //comme cela on identifie le code de chacun des boutons 
      translateIR();
      irrec.resume(); //pour recevoir une nouvelle valeur 
       
   }
      

    
}

void translateIR() {//fonction pour associer une action à chaque bouton

  switch(result.value){ //switch permet de lister tous les cas possibles et 
    //de leur associer une action

    //on va donc lister tous les codes et retourner leur nom sur la télécommande

    //AV: FOR MINICAT1, "Carmp3" RemoteControl
    //AV: almost all original MiniCat RemoteControl codes are working except for the "Carmp3"remote 9 7 3 1 since those buttons are not on the "black" remote control

    //AV: original MiniCat SW :
    case 0xFF18E7: Serial.println(" 2 (grey1) or 2 (grey2) or UP (black)"); break;
    //case 0x3D9AE3F7: Serial.println(" HAUT"); break;

    case 0xFF10EF: Serial.println(" 4 (grey1) or 4 (grey2) or LEFT (black) or Mode (grey2)");    break;
    //case 0x8C22657B: Serial.println(" GAUCHE");    break;
    
    case 0xFF38C7: Serial.println(" 5 (grey1) or 5 (grey2) or OK (black)");    break;
    //case 0x488F3CBB: Serial.println(" -OK-");    break;
    
    case 0xFF5AA5: Serial.println(" 6 (grey1) or 6 (grey2) or RIGHT (black)");   break;
    //case 0x449E79F: Serial.println(" DROITE");   break;

    case 0xFF4AB5: Serial.println(" 8 (grey1) or 8 (grey2) or DOWN (black)"); break;
    //case 0x1BC0157B: Serial.println(" BAS"); break;
    
    case 0xFFA25D: Serial.println(" CH- (grey1) or ON (grey2) or 1 (black)");    break;
    //case 0xE318261B: Serial.println(" 1");    break;
 
    case 0xFF629D: Serial.println(" CH (grey1) or Mode (grey2) or 2 (black)");    break;
    //case 0x511DBB: Serial.println(" 2");    break;

    case 0xFFE21D: Serial.println(" CH+ (grey1) or LoudspeakerOFF (grey2) or 3 (black)");    break;
    //case 0xEE886D7F: Serial.println(" 3");    break;

    case 0xFF22DD: Serial.println(" << (grey1) or >|| (grey2) or 4 (black)");    break;
    //case 0x52A3D41F: Serial.println(" 4");    break;
    
    case 0xFF02FD: Serial.println(" >> (grey1) or |<< (grey2) or 5 (black)");    break;
    //case 0xD7E84B1B: Serial.println(" 5");    break;
    
    case 0xFFC23D: Serial.println(" >|| (grey1) or >>| (grey2) 6 (black)");    break;
    //case 0x20FE4DBB: Serial.println(" 6");    break;
    
    case 0xFFE01F: Serial.println(" - (grey1)  or EQ (grey2) or 7 (black)");    break;
    //case 0xF076C13B: Serial.println(" 7");    break;
    
    case 0xFFA857: Serial.println(" + (grey1) or VOL- (grey2) or 8 (black)");    break;
    //case 0xA3C8EDDB: Serial.println(" 8");    break;
    
    case 0xFF906F: Serial.println(" EQ (grey1) or VOL+ (grey2) or 9 (black)");    break;
    //case 0xE5CFBD7F: Serial.println(" 9");    break;
    
    case 0xFF6897: Serial.println(" 0 (grey1) or 0 (grey2) or * (black)");    break;
    //case 0xC101E57B: Serial.println(" *");    break;
    
    case 0xFF9867: Serial.println(" 100+ (grey1) or RPT (grey2) or 0 (black)");    break;
    //case 0x97483BFB: Serial.println(" 0");    break;
    
    case 0xFFB04F: Serial.println(" 200+ (grey1) or clock (grey2) or # black");    break;
    //case 0xF0C41643: Serial.println(" #");    break;
    
    case 0xFFFFFFFF: Serial.println(" REPEAT");break;  

  default: //si le signal ne correspond à aucun de ces codes
    Serial.println("autre bouton ");

  }
  delay(500); //on doit attendre un peu pour éviter d'appuyer 2 fois
}
 
