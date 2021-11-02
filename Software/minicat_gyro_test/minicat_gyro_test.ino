/*

======================== ATELIER ROBOTIQUE : CatMini ========================
==                              Premiers pas                               ==
==                         Test du module MPU-6050                         ==
==                          Centrale inertielle                            ==
=============================================================================





Test du module MPU-6050, centrale inertielle.

Ce code permet d'afficher l'orientation du module, et l'accélération selon
les 3 axes 3D.  

*/

/* =========================================================================
 *  Références : (Clique sur le "-" à gauche pour cacher cette note)
 *  ========================================================================
 *  
 *  
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

// calibration: http://42bots.com/tutorials/arduino-script-for-mpu-6050-auto-calibration/

 ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================

*/


//on inclut les librairies nécéssaires
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu; //définition du module MPU

// variables pour le contrôle et le statut du MPU 
bool dmpReady = false;  // true si MPU est correctement initialisé
uint8_t mpuIntStatus;   // contient le statut d'interruption du MPU
uint8_t devStatus;      // retourne 0 si l'opération est un succès et !0 si erreur 
uint16_t packetSize;    // taille du paquet qui stocke les données (defining size from Gyro/Accelerometer)
uint16_t fifoCount;     // compte de tous les octets stockés dans FIFO (stacking successive measures from sensor into buffer)
uint8_t fifoBuffer[64]; // buffer de stockage FIFO 


// Variables pour l'orientation et l'accélération du module
Quaternion q;           // [w, x, y, z]         quaternion pour reccueillir les mesures 
VectorInt16 acc;         // [x, y, z]           mesures d'accélération directement des capteurs (avec gravité)
VectorInt16 accReal;     // [x, y, z]           mesures d'accélération sans gravité
VectorFloat gravity;    // [x, y, z]            vecteur de gravité
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll (roulis/tangage/lacet)


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===   
// ================================================================

volatile bool mpuInterrupt = false;     //indique si le pin d'interruption du MPU est HIGH 
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


void setup() {
    //réglage selon les libraires : ne pas porter attention 
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    //communication série
    Serial.begin(57600);
    while (!Serial); //on attend le moniteur série


    // initialisation du module
    Serial.println(F("Initialisation I2C..."));
    mpu.initialize();

    // vérification des connections 
    Serial.println(F("Vérification des connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection réussie") : F("MPU6050 connection échouée"));


    // configuration 
    Serial.println(F("Configuration..."));
    devStatus = mpu.dmpInitialize();


    //offsets pour le gyroscope (valeurs après tests) (gyro/acc offset calibration value at 0 position got after test of the MPU chip)
    mpu.setXGyroOffset(135);
    mpu.setYGyroOffset(34);
    mpu.setZGyroOffset(-30);
    mpu.setZAccelOffset(1736);

    
    if (devStatus == 0) { // vérification que tout a fonctionné
        // on allume le module 
        Serial.println(F("Allumage..."));
        mpu.setDMPEnabled(true);

        // on permet à Arduino de détecter le signal d'interruption 
        Serial.println(F("Permission de détecter le signal d'interruption..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // module prêt 
        Serial.println(F("Module prêt..."));
        dmpReady = true;

        // on récupère la taille du paquet de stockage 
        packetSize = mpu.dmpGetFIFOPacketSize();
        
    } else { //si erreur 
        // ERREUR!
        // 1 = erreur de chargement dans la mémoire
        // 2 = erreur de configuration 
        Serial.print(F("ERREUR (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (!dmpReady) return; //si le module n'est pas prêt, on ne fait rien 

  mpuInterrupt = false; 
  mpuIntStatus = mpu.getIntStatus();  //récupération du statut du module 

  fifoCount = mpu.getFIFOCount(); //récupération du nombre de mesures dans le buffer FIFO

  // vérification que le buffer n'est pas plein (ne devrait jamais arriver)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset pour recommencer proprement 
        mpu.resetFIFO();
        Serial.println(F("FIFO surchargé!"));

    // sinon, on attend un signal du module pour signifier qu'il est prêt
    } else if (mpuIntStatus & 0x02) {
        // on attend que toutes les mesures soient faites (donc que le buffer soit de 
        // la bonne taille) 
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // lecture du FIFO (once FIFO is full one can unstack the values in the FIFO)
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        
        fifoCount -= packetSize;


        // affichage des angles de roulis, tangage, lacet 
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);//(Converting yaw, pitch, roll, angles in degrees)
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
       

       
        // affichage de l'accélération, sans la gravité
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&acc, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&accReal, &acc, &gravity);
            Serial.print("accreal\t");
            Serial.print(accReal.x);
            Serial.print("\t");
            Serial.print(accReal.y);
            Serial.print("\t");
            Serial.println(accReal.z);
       

    }
}
