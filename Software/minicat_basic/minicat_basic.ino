/*

======================== ATELIER ROBOTIQUE : CatMini ========================
==                           Programme principal                           ==
==                          Faire bouger MiniCat                           ==
=============================================================================

Ecrit par Rongzong Li, Petoi 
Modifié et commenté par Lilie Boizumault, QI Informatique 
2020 


*/


// Définition de raccourcis 

#define SPT(s) Serial.print(s)
#define SPTL(s) Serial.println(s)
#define SPTF(s) Serial.print(F(s))
#define SPLF(s) Serial.println(F(s))



// Variables pour le module MPU6050 
// ---------------------------------------------------------------------------------

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;



bool dmpReady = false;  
uint8_t mpuIntStatus;  
uint8_t devStatus;     
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 


Quaternion q;           
VectorFloat gravity;   
float ypr[3], yprLag[2][3];        
bool overFlow = false;



volatile bool mpuInterrupt = false;     
void dmpDataReady() {
  mpuInterrupt = true;
}

// ---------------------------------------------------------------------------------



// Définition des fonctions beep() et melody() (comme vu lors du montage)

// ---------------------------------------------------------------------------------

#include <EEPROM.h> // bibliothèque pour accéder à la mémoire EEPROM 

#define BUZZER 4
#define MELODY 1023
void beep(byte note, float duration = 10, int pause = 0, byte repeat = 1 ) {
  if (note == 0) {
    analogWrite(BUZZER, 0);
    delay(duration);
    return;
  }
  int freq = 220 * pow(1.059463, note);
  float period = 1000000.0 / freq / 2.0;
  for (byte r = 0; r < repeat; r++) {
    for (float t = 0; t < duration * 1000; t += period * 2) {
      analogWrite(BUZZER, 150);      
      delayMicroseconds(period);         
      analogWrite(BUZZER, 0);     
      delayMicroseconds(period);          
    }
    delay(pause);
  }
}
void playMelody(int start) {  // on a stocké la mélodie dans l'EEPROM 
  byte len = (byte)EEPROM.read(start) / 2;
  for (int i = 0; i < len; i++)
    beep(EEPROM.read(start - 1 - i), 1000 / EEPROM.read(start - 1 - len - i), 100);
}

// ---------------------------------------------------------------------------------



// Définition des actions à associer à chaque bouton de la télécommande IR 

// ---------------------------------------------------------------------------------


#include "IRremote.h"
int receiver = 5; 

IRrecv irrecv(receiver);     
decode_results results;      
String translateIR() 
{                                     
  switch (results.value) { // cette fois l'action est de renvoyer l'abréviation de la 
    // posture ou du mouvement demandé


    
                 //abbreviation of gaits      key on IR remote                gait/posture names
    case 0xFF18E7: return (F("vt"));        //Serial.println(" FORWARD");   //stepping on spot, "mark time"
    case 0xFF10EF: return (F("sit"));       //Serial.println(" LEFT");      //sit
    case 0xFF38C7: return (F("bd"));        //Serial.println(" -OK-");      //bound
    case 0xFF5AA5: return (F("balance"));   //Serial.println(" RIGHT");     //standing
    case 0xFF4AB5: return (F("d"));         //Serial.println(" REVERSE");   //shut down servos
    case 0xFFA25D: return (F("trL"));       //Serial.println(" 1");         //trot left
    case 0xFF629D: return (F("tr"));        //Serial.println(" 2");         //trot 
    case 0xFFE21D: return (F("trR"));       //Serial.println(" 3");         //trot right
    case 0xFF22DD: return (F("wkL"));       //Serial.println(" 4");         //walk left
    case 0xFF02FD: return (F("wk"));        //Serial.println(" 5");         //walk 
    case 0xFFC23D: return (F("wkR"));       //Serial.println(" 6");         //walk right
    case 0xFFE01F: return (F("crL"));       //Serial.println(" 7");         //crawl left
    case 0xFFA857: return (F("cr"));        //Serial.println(" 8");         //crawl
    case 0xFF906F: return (F("crR"));       //Serial.println(" 9");         //crawl right
    case 0xFF6897: return (F("bkL"));       //Serial.println(" *");         //back left
    case 0xFF9867: return (F("bk"));        //Serial.println(" 0");         //back
    case 0xFFB04F: return (F("bkR"));       //Serial.println(" #");         //back right
    case 0xFFFFFFFF: return (""); //Serial.println(" REPEAT");

    default:
      return ("");                      //Serial.println("null");
  }
}

// ---------------------------------------------------------------------------------



// Définition des constantes pour les servomoteurs 

// ---------------------------------------------------------------------------------

#define DOF 16

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 
#define SERVOMAX  600 
#define BOUND 1/8
#define RANGE (SERVOMAX - SERVOMIN)


int8_t servoCalibs[DOF] = {}; // variable pour stocker la position de référence en degrés
char currentAng[DOF] = {}; // variable pour stocker la position actuelle de chaque servomoteur 


// Définition des tolérances de rotation (roulis, tangage, lacet) sur chaque articulation
#define FLAT 5
#define panF 1 / 2.0
#define tF 1.0
#define sXF 2
#define sYF 1/4.0
#define uF 0.8
#define dF -0.8
float coeff[16][2] = {
  { -panF, 0}, {0, -tF}, {0, 0}, {0, 0},
  {sXF, -sYF}, { -sXF, -sYF}, { -sXF, sYF}, {sXF, sYF},
  { uF, 1}, { -uF, 1}, { -uF , -1 }, { uF , -1 },
  { dF, dF}, { -dF, dF}, { -dF, -dF}, { dF, -dF}
};

// ---------------------------------------------------------------------------------


int calibratedDuty0[DOF]; // variable pour stocker la position de référence en longeur de pulsation

#include "modes.h" // on inclut le fichier de définition des postures et mouvements 


// Définition des fonctions pour écrire et lire dans l'EEPROM

// ---------------------------------------------------------------------------------

#define PIN 0  // définition des emplacements de stockage des variables 
#define CALIB 16
#define MIDSHIFT 32
#define ROTDIRECTION 48
#define RANGERATIO 64
#define MPUCALIB 80



void EEPROMWriteInt(int p_address, int p_value) // écriture d'une variable à une certaine 
// adresse de l'EEPROM
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);
  EEPROM.update(p_address, lowByte);
  EEPROM.update(p_address + 1, highByte);
}


int EEPROMReadInt(int p_address) // lecture de la valeur d'une variable stockée à 
// une certaine adresse de l'EEPROM
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);
  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}


byte pin(byte idx) {  // retourne la valeur du pin sur lequel est branché le servomoteur
// numéro "idx"
  return (byte)EEPROM.read(PIN + idx);
}


int8_t middleShift(byte idx) { // retourne la valeur de middleshift correspondant au 
// servomoteur numéro "idx"
  return (int8_t)EEPROM.read( MIDSHIFT + idx);
}


byte rangeRatio(byte idx) { // retourne la valeur de rangeRatio correspondant au 
// servomoteur numéro "idx"
  return (byte)EEPROM.read(RANGERATIO + idx);
}


int8_t rotationDirection(byte idx) { // retourne la valeur de rotationDirection 
// correspondant au servomoteur numéro "idx"
  return (int8_t)EEPROM.read(ROTDIRECTION + idx);
}


int8_t servoCalib(byte idx) { // retourne la valeur de l'offset sur la position de  
// référence correspondant au servomoteur numéro "idx"
  return (int8_t)EEPROM.read( CALIB + idx);
}


void saveCalib(int8_t *var) { // écriture des bonnes valeurs d'offset pour la position
// de référence => sauvegarde de l'étalonnage 
  for (byte i = 0; i < DOF; i++) {
    EEPROM.update(CALIB + i, var[i]);
    calibratedDuty0[i] = SERVOMIN + RANGE / 2 + float(middleShift(i) + var[i]) * RANGE * rotationDirection(i) / rangeRatio(i);
  }
}

// ---------------------------------------------------------------------------------


// Définition des fonctions de mouvement

// ---------------------------------------------------------------------------------

void calibratedPWM(byte i, int dt) { // fonction qui règle le servomoteur i à l'ange dt
  char duty = max(-128, min(127, dt)); // on force l'angle dt à être entre -128 et 127 degrés 
  if (i > 3 && i < 8) // si l'indice du servomoteur est entre 4 et 7
    duty = max(-5, duty); // l'angle doit être plus grand que -5 degrés
  currentAng[i] = duty; // stockage de la valeur de l'angle pour le servomoteur i 
  dt = calibratedDuty0[i] + float(duty) * RANGE * rotationDirection(i) / rangeRatio(i); // conversion en longueur de pulsation
  dt = max(SERVOMIN + RANGE / 2 * BOUND, min(SERVOMAX - RANGE / 2 * BOUND, dt)); // on contraint la longeur de pulsation
  // à être entre les bonnes valeurs 
  pwm.setPWM(pin(i), 0, dt); // commande du servomoteur 
}


void allCalibratedPWM(char * dutyAng) { // fonction qui place tous les servomoteurs 
  // aux bons angles 
  for (byte i = 0; i < DOF; i++) {
    calibratedPWM(i, dutyAng[i]); // envoi de chaque servomoteur à l'angle voulu 
  }
}


void printList(char * arr, byte len = DOF) { // affiche les valeurs d'une liste sur le moniteur 
  // série
  for (byte i = 0; i < len; i++) {
    SPT(int(arr[i]));
    SPT('\t');
  }
  SPTL();
}


void transform( char * target, byte offset = 0, float speedRatio = 0.5) { // fonction qui 
  // place les servomoteurs aux angles cibles, en prenant en compte la position actuelle
  // Indique de combien chaque servomoteur doit tourner pour se placer à l'angle voulu

  char *diff = new char[DOF - offset], maxDiff = 0; // initialisation de la variable diff
  
  for (byte i = offset; i < DOF; i++) { // pour chaque servomoteur 
    diff[i - offset] =   currentAng[i] - target[i - offset]; // on calcule la différence
    // entre l'angle actuel et l'angle voulu  
    maxDiff = max(maxDiff, abs( diff[i - offset])); // maxDiff devient une valeur positive (0 ou plus) 
  }
  
  byte steps = byte(round(maxDiff / 3.0/*degreeStep*/ / speedRatio)); // conversion de la différence 
  // en nombre de "pas" pour la rotation 
  for (byte s = 0; s < steps; s++) // pour chaque pas de rotation 
    for (byte i = offset; i < DOF; i++) { // pour chaque servomoteur 
      char duty = (target[i - offset] + (1 + cos(M_PI * s / steps)) / 2 * diff[i - offset]); // calcul de l'angle 
      // à envoyer 
      calibratedPWM(i,  duty); // Envoi au servomoteur i 
    }
  delete [] diff; // effacer diff
  
}


void shutServos() { // fonction pour éteindre les servomoteurs 
  delay(250);
  for (byte i = 0; i < DOF; i++) {
    pwm.setPWM(i, 0, 4096);
  }
}


int8_t idxOfGait(const char * gait) { // retourne l'indice de la posture ou du mouvement 
  // demandé
  for (byte i = 0; i < sizeof(gaits) / sizeof(char*); i++) // on parcourt la liste 
  // des mouvements  et postures
    if (!strcmp(gaits[i], gait)) // quand on trouve le bon nom dans la liste 
      return i; // on retourne i 
  return -1;
}


//AV: target angles for the servomotors are taken from a certain position or movement target which is sent by the remote control or terminal :

int pgmCpy(char * dutyAng, const char * gait) { // fonction qui copie dans dutyAng les angles 
  // voulus pour réaliser une certaine posture (ou mouvement) 
  dutyAng--;
  strcpy_P(dutyAng, (char*)pgm_read_word(&(gaits_table[idxOfGait(gait)]))); // on cherche l'indice 
  // de la posture (ou mouvement)dans la liste des postures 
  // on récupère le contenu (angles voulus) dans le fichier modes.h
  // on copie ces angles dans dutyAng
  int period = dutyAng[0]; // on récupère le nombre de postures ( 0 -> posture fixe, sinon -> mouvement)
  dutyAng++;
  return period;
}

// AV: this is where the gyroscope latest measurement is taken in account to adjust the balance of the robot : 

int adjust(int i) { // calculer un coefficient de rotation, pour l'équilibre
  if (i>7)
    return coeff[i][0] * ypr[2] * 180 / M_PI + coeff[i][1] * ypr[1] * 180 / M_PI;
  else
    return coeff[i][0] * ypr[2] * 180 / M_PI + coeff[i][1] * ypr[1] * 180 / M_PI;
}


// ---------------------------------------------------------------------------------


// Initialisation des variables nécéssaires 
int t = 0;
int tPeriod = 1;
byte hold = 0;
byte offset;
char token;
#define CMD_LEN 10
char lastCmd[CMD_LEN] = {};



/* =================================================================================
 * ==                                                                             ==
 * ==                           SETUP - INITIALISATION                            == 
 * ==                                                                             == 
 * =================================================================================
 */


void setup() {

// Initialisation MPU6050

// ---------------------------------------------------------------------------------
 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(57600);
  Serial.setTimeout(10);
  while (!Serial);
  while (Serial.available() && Serial.read()); 
  delay(100);
  SPLF("* Starting *");
  SPLF("Initializing I2C");
  mpu.initialize();
  SPLF("Testing connections...");
  Serial.println(mpu.testConnection() ? F("MPU successful") : F("MPU failed"));
  Serial.println(F("Initializing DMP..."));
  //devStatus = mpu.dmpInitialize();//AV, RoboticsForMakers: commenting this line, minicat_basic.ino software not running stable with this line
  mpu.setZAccelOffset(EEPROMReadInt(MPUCALIB + 4));
  mpu.setXGyroOffset(EEPROMReadInt(MPUCALIB + 6));
  mpu.setYGyroOffset(EEPROMReadInt(MPUCALIB + 8));
  mpu.setZGyroOffset(EEPROMReadInt(MPUCALIB + 10));
  
  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.println(F("Enabling interrupt detection"));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready!"));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.println(F("DMP failed (code "));
    SPT(devStatus);
    SPLF(")");
  }

// ---------------------------------------------------------------------------------
  

  if (WalkingDOF == 8)
  {
    pinMode(BUZZER, OUTPUT);
    playMelody(MELODY); // Musique d'initialisation 
    Serial.println("DEBUG : Init Melody played"); //AV: DEBUG : check if we reach this line
    
  }

  {
    irrecv.enableIRIn(); // Initialisation pour l'IR
    Serial.println("DEBUG : Init IR done"); //AV: DEBUG : check if we reach this line
  }

  // Initialisation pour les servomoteurs
  { pwm.begin();
    pwm.setPWMFreq(60);  
    delay(200);
    char cmd[CMD_LEN] = {};
   

    strcpy(lastCmd, "rest");
    pgmCpy(dutyAng, "rest"); // on charge les angles de la position "rest" dans dutyAng
    
    for (byte i = 0; i < DOF; i++) {
      servoCalibs[i] = servoCalib(i); // Stockage des valeurs des positions de références (en degrés)
      calibratedDuty0[i] =  SERVOMIN + RANGE / 2 + float(middleShift(i) + servoCalibs[i]) * RANGE  * rotationDirection(i) / rangeRatio(i);
      // Stockage des valeurs des positions de références (en longueur de pulsation)
      calibratedPWM(i, dutyAng[i]); // On place les servomoteurs aux bons angles pour la position "rest"
    }
   
    shutServos(); // on éteint les servomoteurs 
    token = 'd'; 
    Serial.println("DEBUG : Init Servos done"); //AV: DEBUG : check if we reach this line
  }
  beep(30);
}

/* =================================================================================
 * ==                                                                             ==
 * ==                           LOOP - BOUCLE PRINCIPALE                          == 
 * ==                                                                             == 
 * =================================================================================
 */



void loop() {
  
// MPU6050

// ---------------------------------------------------------------------------------
  {
    if (!dmpReady) return;
    Serial.println("DEBUG : DMP=MPU init did NOT fail"); //AV: DEBUG : check if we reach this line
    while (!mpuInterrupt && fifoCount < packetSize) ;
    
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 256) { //1024) {
      mpu.resetFIFO();
      overFlow = true;
    }
    else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
    if (overFlow) {
      for (byte g = 0; g < 3; g++) {
        ypr[g] = yprLag[0][g];
        yprLag[1][g] = yprLag[0][g];
      }
      overFlow = false;
    }
    else {
      for (byte g = 0; g < 3; g++) {
        float temp = ypr[g];
        ypr[g] = yprLag[0][g];
        yprLag[0][g] = yprLag[1][g];
        yprLag[1][g] = temp;
      }
    }
  }

// ---------------------------------------------------------------------------------
  
  for (byte i = 1; i < 3; i++) // on convertit les mesures du gyroscope en angles (degrés)
    ypr[i] = ((ypr[i] > 0) - (ypr[i] < 0)) * max(fabs(ypr[i]) - FLAT*M_PI/180.0, 0);
  ypr[2] = -ypr[2];

  char cmd[CMD_LEN] = {};
  byte newCmd = 0;


// block "accident" 
  
// ---------------------------------------------------------------------------------

  if (fabs(ypr[1] * 180 / M_PI) > 60) { // Si l'angle de rotation du robot est trop important 
    if (!hold) {
      token = 'p';
      strcpy(cmd, ypr[1] * 180 / M_PI > 60 ? "lifted" : "dropped"); // c'est qu'il est tenu en l'air (lifted)
      // ou tombé (dropped)
      newCmd = 1;
    }
    hold = 10;
  }
  // Dans ce cas : se relever 
  else if (hold ) {
    if (hold == 10) {
      token = 'p';
      strcpy(cmd, "balance"); // le robot se met debout 
      newCmd = 1;
    }
    hold --;
    if (!hold) {
      char temp[CMD_LEN];
      strcpy(temp, cmd);
      strcpy(cmd, lastCmd);
      strcpy(lastCmd, temp); // stockage de la commande "balance" dans la mémoire (variable "lastCmd")
      newCmd = 1;
    }
  }

// ---------------------------------------------------------------------------------

// Block de commande 

// ---------------------------------------------------------------------------------

    if (irrecv.decode(&results)) { // si on reçoit un signal IR 
      if (translateIR() != "") {
        strcpy(cmd, translateIR().c_str()); // on copie dans "cmd" la commande reçue par le capteur IR 
        if (!strcmp(cmd, "d")) // si l'ordre est de couper les servomoteurs 
          token = 'd';
        else  // si on demande de réaliser une posture ou un mouvement 
          token = 'g';
        newCmd = 2;
      }
      irrecv.resume(); // on se prépare pour recevoir un autre ordre 
    }
    
    if ( Serial.available() > 0) { // si le moniteur série est ouvert 
      token = Serial.read(); // token devient la première lettre de l'ordre entré dans le moniteur 
      newCmd = 3;
    }

    

  if (newCmd) { // si on a reçu un ordre 
    SPTL(token); // affichage du token 
    beep(newCmd * 10);
    
    // détail des actions selon la valeur de "token" 
    
    if (token == 'h')
      SPLF("** Help Information **"); // affichage de l'aide | n'éxiste pas |

    
    else if (token == 'd' ) { // si le "token" est "d" => on charge la posture "rest"
      // puis on éteint les servomoteurs
      pgmCpy(dutyAng, "rest"); // on charge les angles voulus dans dutyAng
      transform( dutyAng); // on envoie les servomoteurs dans la bonne position (en fonction de leur position précédente)
      shutServos(); // on éteint les servomoteurs
    }
    
    else if (token == 's') { // si le "token" est "s" => on sauvegarde les angles de compensation pour l'étalonnage
      saveCalib(servoCalibs);
    }
    
    else if (token == 'a') { // si le "token" est "a" => on abandonne l'étalonnage
      for (byte i = 0; i < DOF; i++) {
        servoCalibs[i] = servoCalib( i);
      }
    }
    
    else if (token == 'l' ) { // commande qui permet de commander les servomoteurs un par un 
      int len = Serial.read();
      SPT(len);

      char *inBuffer = new char[len];
      for (byte i = 0; i < len; i++){
        inBuffer[i] = Serial.read();
        SPT(inBuffer[i]);
        SPTL();
      }
      if (len == DOF)
        allCalibratedPWM(inBuffer);
      else
        for (byte i = 0; i < len / 2; i++)
          calibratedPWM(inBuffer[i * 2], inBuffer[i * 2 + 1]);
      delete [] inBuffer;
    }

    
    else if (token == 'c' || token == 't') { // token pour l'étalonnage ou pour bouger un servomoteur 
      int target[2] = {};
      String inBuffer = Serial.readStringUntil('\n'); // on récupère les informations suivantes 
      // de type "indice_servomoteur angle"
      strcpy(cmd, inBuffer.c_str());
      char *pch;
      pch = strtok (cmd, " ,");
      for (int c = 0; pch != NULL; c++)
      {
        target[c] = atoi(pch);
        pch = strtok (NULL, " ,");
      }

      if (token == 'c') { // si on étalonne
        
        if (strcmp(lastCmd, "c")) { // si on commence l'étalonnage
          pgmCpy(dutyAng, "calib"); // on place le robot en position "étalonnage"
          transform( dutyAng);
        }
        SPT('\n');  // affichage des indices des servomoteurs 
        for (byte i = 0; i < DOF; i++) {
          SPT(i);
          SPT(",\t");
        }
        SPT('\n');
        for (byte i = 0; i < DOF; i++) { // affichage des angles de compensation de chaque servomoteur 
          SPT(servoCalibs[i]);
          SPT(",\t");
        }
        SPT('\n');
        yield();
        servoCalibs[target[0]] = target[1];
      }

      
      else if (token == 't') { // token "t" pour bouger un servomoteur à un certain angle 
        dutyAng[target[0]] = target[1];
      }

      SPT(target[0]); // affichage de l'indice du servomoteur à bouger 
      SPT(",\t");
      SPT(target[1]); // affichage de l'angle du servomoteur à atteindre

      // conversion de l'angle (en degrés) en longueur de pulsation  
      int duty = SERVOMIN + RANGE / 2 + float(middleShift(target[0])  + servoCalibs[target[0]] + dutyAng[target[0]]) * RANGE * rotationDirection(target[0]) / rangeRatio(target[0]);
      pwm.setPWM(pin(target[0]), 0,  duty); // commande du servomoteur 
    }



    else if (Serial.available() > 0) { // si le token n'est aucun de ceux précédemment listés 
      // => si on demande une posture ou un mouvement 
      
      String inBuffer = Serial.readStringUntil('\n'); // lecture de tout ce qui est écrit après le token
      // => le nom de la posture ou du mouvement 
      strcpy(cmd, inBuffer.c_str()); // copie du nom dans "cmd"
    }


    if (strcmp(cmd, "") && strcmp(lastCmd, cmd) ) { // si "cmd" n'est pas vide, et n'est pas la même que la commande
      // précédente

      if (token == 'w'); // on n'utilise pas ce token pour le moment 

      if (token == 'p' || token == 'g') { // token correspondants à une commande de posture ou mouvement 
        if (idxOfGait(cmd) >= 0) { // on vérifie que la commande est dans la liste des postures / mouvements 
          tPeriod =  pgmCpy(dutyAng, cmd); // on copie les angles voulus dans dutyAng et on récupère le nombre de posture
          t = 0;
          if (strcmp(cmd, "balance") && strcmp(cmd, "lifted") && strcmp(cmd, "dropped") )
            strcpy(lastCmd, cmd); // on stocke la commande dans lastCmd
          offset = (tPeriod == 1) ? 0 : DOF - WalkingDOF; // si c'est une posture on considère les 16 articulations 
          // si c'est un mouvement on ne prend que les 8 des jambes 

          //AV : here all the positions of servos for a given target robot position or movement (see modes.h) are implemented by the servos :
          transform( dutyAng, offset, 1); // on envoie les servomoteurs dans la bonne posture OU dans la première posture 
          // du mouvement voulu 
          if (!strcmp(cmd, "rest")) { // si la commande est "rest"
            shutServos(); // on éteint les servomoteurs 
            token = 'd'; // token devient d 
          }
        }
        else
          SPLF("wrong key!");
      }
      else { // si c'est encore un autre token : on ne fait rien 
        lastCmd[0] = token;
        memset(lastCmd + 1, '\0', CMD_LEN - 1); 
      }
    }
  }

// ---------------------------------------------------------------------------------

// Block de mouvement : seulement pour les mouvements (suites de postures)

// ---------------------------------------------------------------------------------

  { // éxécution de la suite des postures qui composent le mouvement 
    if (token == 'p' || token == 'g') {
      for (int i = offset; i < DOF; i++) { // pour les articulations des jambes 
        int dutyIdx =  t * WalkingDOF - offset + i; // l'indice de ligne (de posture) change 
        calibratedPWM(i, (dutyAng[dutyIdx] + adjust(i)) / (1 + sqrt(fabs(ypr[1] * ypr[2])) / M_PI * 2) ); // envoi des ordres aux servomoteurs 
        // Remarque : les angles sont ajustés pour l'équilibre 
      }
      t = (t + 1) % tPeriod; // t permet de changer de ligne et est compris entre 0 et le nombre de postures du mouvement
      
      
      byte pause = 0;
      if (lastCmd[0] == 'w' && lastCmd[1] == 'k')
        pause = 5;
      else
        pause = 0;
      delay(pause); 
    }
  }
}
