/*
  rotation levels (16)
                |
                |
                |
                |
                |
                |
                level 2 (rotateLs2)(resetLevel) (positive pulse)
  (15)------- level 1 (rotateLs1)(negative pulse)


  Platform Levels

  --------------(17) level 2 (platformLs2) (negative pulse)

    ---------   (Sub level for each ring in total 10 rings)
    ---------
    ---------

  -------------- (14)level 1 (platformLs1)(resetLevel) (positive pulse)
*/
#include <EspNow.h>
#include <Motor.h>
#include <positionalnew.h>

Peer remote;
Peer dataesp;
JSONVar feedback;
JSONVar datapick;
JSONVar setdata;
Motor rotationMotor(22, 23);
Motor platformMotor(33, 32);
UniversalEncoder rotationEncoder(19, 18, 1);
UniversalEncoder platformEncoder(25, 26, -1);
positionalnew rMPID(&rotationMotor);
positionalnew pMPID(&platformMotor);

double AggKpRotation = 1.4 , AggKiRotation = 0.0, AggKdRotation = 0;
double SoftKpRotation = 0.4, SoftKiRotation = 0.0, SoftKdRotation = 0;
double AggKpPlatform = 1.8, AggKiPlatform = 0.0, AggKdPlatform = 0;
double SoftKpPlatform = 1, SoftKiPlatform = 0, SoftKdPlatform = 0;

int rotateLs1 = 15, rotateLs2 = 16, platformLs1 = 14, platformLs2 = 17;
// int pneumaticPin = 13;

int rotateLevel = 0, platformLevel = 0, platformSubLevel = 0;
int rInternalLvl = -1, pInternalLvl = -1;
double rotationPulseOffset = 0.045, platformPulseOffset = 0.1, signOffsetRotation = 0, signOffsetPlatform = 0;
static long start = 0;
long rotatePulse = 0, platformPulse = 0, resetPulse = 50000;
long rLvl2Pulse = 0, pLvl1Pulse = 0, subLevel1 = 0, oneRingPulse = 0, rotationExtraPulse = 0, platformExtraPulse = 0, setRing = 0;
int setOffset, setPid, moveMsg;
bool init_ = false;
bool rLs1 = false, rLs2 = false, pLs1 = false, pLs2 = false;
bool allRings = true;
bool pOpen = false;
bool setSubLevel = false;
void setup()
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  //  delay(5000);
  rotationMotor.setEncoder(&rotationEncoder);
  platformMotor.setEncoder(&platformEncoder);

  rMPID.setThreshold(100);
  rMPID.setOutputLimits(-35, 35);
  rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
  rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);

  pMPID.setThreshold(100);
  pMPID.setOutputLimits(-90, 90);
  pMPID.setAggTunings(AggKpPlatform, AggKiPlatform, AggKdPlatform);
  pMPID.setSoftTunings(SoftKpPlatform, SoftKiPlatform, SoftKdPlatform);

  pinMode(rotateLs1, INPUT_PULLUP);
  pinMode(rotateLs2, INPUT_PULLUP);
  pinMode(platformLs1, INPUT_PULLUP);
  pinMode(platformLs2, INPUT_PULLUP);

  setId("PICKE");
  remote.init("TenZZ");
  //  dataesp.init("DATAZ");
  remote.setOnRecieve(rotationLvl1, "rLvl1");
  remote.setOnRecieve(rotationLvl2, "rLvl2");
  remote.setOnRecieve(platformLvl1, "pLvl1");
  // remote.setOnRecieve(platformSubLvl2, "pLvl2");
  remote.setOnRecieve(setRotateExtraPulse, "exRo");
  remote.setOnRecieve(setPlatformExtraPulse, "exPl");
  remote.setOnRecieve(resetAll, "Erst");
  remote.setOnRecieve(pneumaticMove, "pMove");
  remote.setOnRecieve(setPID, "sPID");
  remote.setOnRecieve(resetPID, "resP");
  setdata["type"] = "setup";
  //  remote.send(setdata);
}
void loop()
{

  rotatePulse = rotationMotor.getReadings();
  platformPulse = platformMotor.getReadings();
  Serial.println("ROTATION=" + String(rotatePulse) + ", PLATFORM= " + String(platformPulse));
  pLs1 = !(bool)digitalRead(platformLs1);
  pLs2 = !(bool)digitalRead(platformLs2);
  rLs1 = !(bool)digitalRead(rotateLs1);
  rLs2 = !(bool)digitalRead(rotateLs2);
  if (Serial.available() > 0)
  {
    rotateLevel = Serial.readStringUntil(',').toInt();
    platformLevel = Serial.readStringUntil('\n').toInt();
  }

  if (!init_)
  {
    if (rInternalLvl != 3)
    {
      if (rInternalLvl == -1) // initially goto level 1
      {
        rMPID.setOutputLimits(-20, 20);
        // Serial.println("goto level 1");
        rMPID.setPulse( resetPulse); // postive pulse for lvl 1 fix
        rInternalLvl = 0;
      }
      else if (rLs1 && rInternalLvl == 0) // reached level 1
      {
        //Serial.println("reached level 1");
        rInternalLvl = 1;
        rotationMotor.reset();
        rMPID.setPulse(0);
        //Serial.println(rInternalLvl);
      }
      else if (!rLs2 && rInternalLvl == 1) // goto level 2
      {
        rMPID.setOutputLimits(-25, 25);
        //Serial.println("goto level 2  " + String(rInternalLvl));
        rMPID.setPulse(-resetPulse);// positive pulse for lvl 2 fix
        rInternalLvl = 2;
      }
      else if (rLs2 && rInternalLvl == 2) // reached level 2
      {
        //Serial.println("reached level 2  " + String(rInternalLvl));
        rLvl2Pulse = rotatePulse;
        rotationExtraPulse = rLvl2Pulse * rotationPulseOffset;
        //Serial.println(rLvl2Pulse);
        //Serial.println(rLvl2Pulse - rLvl2Pulse * 0.051);
        rMPID.setPulse(rLvl2Pulse - rLvl2Pulse * 0.051); // move the rotation slightly towards lvl 1 to free Limit Switch
        rInternalLvl = 3;
      }
    }
    else if (rInternalLvl == 3)
    {
      if (!pLs2 && pInternalLvl == -1) // initially goto level 2
      {
        //Serial.println("goto level 2");
        pMPID.setPulse(-resetPulse); // negative pulse for lvl 2 fix
        pInternalLvl = 0;
      }
      else if (pLs2 && pInternalLvl == 0) // reached level 2
      {
        //Serial.println("reached level 2");
        // rMPID.setPulse(rotatePulse);
        pInternalLvl = 2;
        platformMotor.reset();
        pMPID.setPulse(0);
      }
      else if (!pLs1 && pInternalLvl == 2) // goto level 1
      {
        //Serial.println("goto level 1");
        pMPID.setPulse(resetPulse); // positive pulse for lvl 1 fix
        pInternalLvl = 1;
      }
      else if (pLs1 && pInternalLvl == 1) // reached level 1
      {
        //Serial.println("reached level 2");
        pLvl1Pulse = platformPulse;
        subLevel1 = pLvl1Pulse * 0.85;
        //        platformExtraPulse = (pLvl1Pulse - subLevel1) * 0.05;
        oneRingPulse = subLevel1 / 9.8;    //495
        // oneRingPulse = pLvl1Pulse / 12.5;
        pMPID.setPulse(pLvl1Pulse - pLvl1Pulse * 0.0225); // move the platform slightly towards lvl 2 to free Limit Switch
        pInternalLvl = 3;
        Serial.println(String(pLvl1Pulse) + ", SUB=" + String(subLevel1) + ", EXTRA=" + String(platformExtraPulse) + ", ONE=" + String(oneRingPulse));
      }
    }
  }
  else if (init_)
  {
    if (rLs2 || rLs1) // whenever rotation limit switch pressed move the rotation towards opposite level slightly for safety purpose
    {

      if (rLs2)
      {
        // motorBack();
        //       rMPID.setPulse(rotatePulse - rLvl2Pulse * 0.03); // after this limit switch should be free
//       if (setSubLevel)
//       {
//        if (allRings)
//        {
//          if (setPid == 3)
//          {
//            Serial.println("SET PID FOR POLE 3");
//            //        AggKpPlatform = 5;   //change this value to 2
//            //        SoftKpPlatform = 5;
//            pMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
//            pMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);
//          }
//
//          setRing = 1000;
//          pMPID.setPulse(subLevel1 - setRing); //move to a sublevel where the 10th ring is loaded to pass
//          allRings = false;
//          platformSubLevel = 2;
//          setSubLevel = false;
//        }
//      }
      }

      else if (rLs1)
      {
        rMPID.setPulse(rotatePulse + rLvl2Pulse * 0.051); // after this limit switch should be free
      }
    }

    if (pLs2 || pLs1) // whenever platform limit switch pressed move the rotation towards opposite level slightly for safety purpose
    {
      if (pLs2)
      {
        pMPID.setPulse(platformPulse + pLvl1Pulse * 0.015); // after this limit switch should be free
      }

      else if (pLs1)
      {
        pMPID.setPulse(platformPulse - pLvl1Pulse * 0.016); // after this limit switch should be free
      }
    }
  }
  rMPID.compute();
  pMPID.compute();
}

//check threshold and output limit for each function of espNow

void rotationLvl1(JSONVar msg)
{
  //Serial.println("rotationLvl1");
  rMPID.setThreshold(200);
  rMPID.setOutputLimits(-25, 25);
  //  rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
  //  rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);

  rotateLevel = 1;
  init_ = true;

  datapick["rotate"] = "LEVEL 1";
  datapick["type"] = "rotation";
  //  dataesp.send(datapick);

  rMPID.setPulse(-20); // move to level 1
}

void rotationLvl2(JSONVar msg)
{
  //Serial.println("rotationLvl2");

  rMPID.setThreshold(200);
  rMPID.setOutputLimits(-57, 57);
  //  rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
  //  rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);

  datapick["rotate"] = "LEVEL 2";
  datapick["type"] = "rotation";
  //  dataesp.send(datapick);

  rotateLevel = 2;
  init_ = true;

  rMPID.setPulse(rLvl2Pulse); // rotate to level 2
//  if(!setSubLevel)
//  {
//    allRings=true;
//    setSubLevel=true;
//  }
}

void platformLvl1(JSONVar msg)
{
  //Serial.println("platform Lvl 1");
  pMPID.setThreshold(100);
  pMPID.setOutputLimits(-100, 100);
  // pMPID.setAggTunings(AggKpPlatform, AggKiPlatform, AggKdPlatform);
  // pMPID.setSoftTunings(SoftKpPlatform, SoftKiPlatform, SoftKdPlatform);

  datapick["platform"] = "LEVEL 1";
  datapick["type"] = "plat";
  //  dataesp.send(datapick);

  platformLevel = 1;
  allRings = true;
  platformSubLevel = 0;
  init_ = true;
//    setSubLevel=true;
  pMPID.setPulse(pLvl1Pulse - pLvl1Pulse * 0.015); //move platform to level 1
}


void setRotateExtraPulse(JSONVar msg)
{
  rMPID.setThreshold(30);
  AggKpPlatform = 0.8;
  SoftKpPlatform = 0.15;
  rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
  rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);
  //  rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
  //  rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);

  int extraOffset = (int)msg["offset"]; // for rotation towards lvl 1(offset = -1) & towards lvl 2(offset = 1) // set this values on remote reciever
  rMPID.setOutputLimits(-22, 22);
  setOffset = rotationMotor.getReadings() + (extraOffset * rotationExtraPulse);

  // Serial.println(JSON.stringify(msg));
  // Serial.println("setOffset: " + String(setOffset) + "RotaEP: " + String(rotationExtraPulse));

  rMPID.setPulse(setOffset); // rotate a bit

  init_ = true;
  datapick["rExtra"] = rotationExtraPulse;
  datapick["setOffset"] = setOffset;
  datapick["type"] = "rotation";
  //  dataesp.send(datapick);
}
void setPID(JSONVar msg)
{
  setPid = msg["spid"];
  Serial.println(JSON.stringify(msg));
}
void setPlatformExtraPulse(JSONVar msg) // move platform up for one ring on each up press and move platform down for one ring on each down press
{
  int temp = msg["side"];
  // int extraOffset = (int)msg["offset"];
  if (temp == 1)
  {
    pMPID.setThreshold(50);
    pMPID.setOutputLimits(-100, 100);
    platformLevel = 2;
    //Serial.println("platformSubLvl2");
    init_ = true;

        if (allRings)
        {
          if (setPid == 3)
          {
            Serial.println("SET PID FOR POLE 3");
//                AggKpPlatform = 5;
//                SoftKpPlatform = 5;
            pMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
            pMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);
          }
          //      pMPID.setPulse(signOffsetPlatform * subLevel1);
    //      setRing=oneRingPulse*2;
            setRing=1200;
//          pMPID.setPulse(subLevel1-setRing);//move to a sublevel where the 10th ring is loaded to pass
          pMPID.setPulse(subLevel1);
          Serial.println("lvl1: " + (String)pLvl1Pulse+ " Sublvl: " + (String)(subLevel1)+", PLATFORM= " + String(platformPulse));
    
          allRings = false;
          platformSubLevel=0;
        }
     else if (!allRings)
    {
      if (setPid == 3)
      {
        Serial.println("SET PID FOR POLE 3");
        //        AggKpPlatform = 5;
        //        SoftKpPlatform = 5;
        pMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
        pMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);
      }
      platformSubLevel++;
      platformSubLevel = platformSubLevel > 11 ? 11 : platformSubLevel;
      //      pMPID.setPulse(signOffsetPlatform * (subLevel1 - platformSubLevel * oneRingPulse));
      //pMPID.setPulse(platformMotor.getReadings() - oneRingPulse);

      setOffset = (subLevel1 - platformSubLevel * oneRingPulse);
      pMPID.setPulse((subLevel1 - platformSubLevel * oneRingPulse));//move to a sublevel for each ring after 10th ring

      Serial.println("lvl1: " + (String)pLvl1Pulse + " UpOffset2: " + (String) int(setOffset) + ", PLATFORM= " + String(platformPulse) + " Sublvl: " + (String)subLevel1);
    }
    datapick["platform"] = "LEVEL 2";
    datapick["sublevel"] = platformSubLevel;
    datapick["type"] = "plat";
    dataesp.send(datapick);
  }
  else if (temp == -1)
  {
    if (setPid == 3)
    {
      Serial.println("SET PID FOR POLE 3");
      //      AggKpPlatform = 5;
      //      SoftKpPlatform = 5;
      pMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
      pMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);
    }
    // int extraOffset = (int)msg["offset"];
    if ( platformSubLevel == 0)
    {
      setOffset = (subLevel1 - platformSubLevel * oneRingPulse);
       pMPID.setPulse(setOffset);
    }
    else
    {
      platformSubLevel--;

      //platformSubLevel = platformSubLevel < 1 ? 1 : platformSubLevel;
      platformSubLevel = platformSubLevel < 0 ? 0 : platformSubLevel;

//      allRings = platformSubLevel < 0? !allRings : allRings;

      // Serial.println(JSON.stringify(msg));
      Serial.println(" Off: " + (String)setOffset + ", PLATFORM= " + String(platformPulse) + "lvl1: " + (String)pLvl1Pulse + "platformSubLevel: " + String(platformSubLevel));

      datapick["pExtra"] = platformExtraPulse;
      datapick["setOffset"] = setOffset;
      datapick["type"] = "plat";
      dataesp.send(datapick);
      //setOffset = (platformMotor.getReadings() + oneRingPulse); // offset to move platform down OneRing

      setOffset = (subLevel1 - platformSubLevel * oneRingPulse);
      pMPID.setPulse(setOffset);//move platform down OneRing
      Serial.println("platformSubLevel: " + String(platformSubLevel) + " Off: " + ((String)setOffset) );
      init_ = true;
      
    }

  }
}
//void setPlatformExtraPulse(JSONVar msg) // move platform up for one ring on each up press and move platform down for one ring on each down press
//{
//  int temp = msg["side"];
//  if (temp == 1)
//  {
//    pMPID.setThreshold(50);
//    pMPID.setOutputLimits(-100, 100);
//    init_ = true;
//
//    if (allRings)
//    {
//      platformSubLevel++;
//      if (setPid == 3)
//      {
//        Serial.println("SET PID FOR POLE 3");
//        AggKpPlatform = 5;
//        SoftKpPlatform = 5;
//        rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
//        rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);
//      }
//      pMPID.setPulse(pLvl1Pulse - platformSubLevel * oneRingPulse);
//      Serial.println("lvl1: " + (String)pLvl1Pulse + " UpOffset: " + (String) int(pLvl1Pulse - platformSubLevel * oneRingPulse) + "One ring= " + String(oneRingPulse));
//    }
//  }
//  else if (temp == -1)
//  {
//    if (setPid == 3)
//    {
//      Serial.println("SET PID FOR POLE 3");
//      AggKpPlatform = 5;
//      SoftKpPlatform = 5;
//      rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
//      rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);
//    }
//    platformSubLevel--;
//    platformSubLevel = platformSubLevel < 1 ? 1 : platformSubLevel;
//    pMPID.setPulse(pLvl1Pulse - platformSubLevel * oneRingPulse); //move platform down OneRing
//    Serial.println("lvl1: " + (String)pLvl1Pulse + " DownOffset: " + (String) int(pLvl1Pulse - platformSubLevel * oneRingPulse) + "One ring= " + String(oneRingPulse));
//    init_ = true;
//  }
//}
void resetPID(JSONVar msg)
{
  setPid = 0;
}
void resetAll(JSONVar msg)
{
  // Serial.println("resetAll");
  init_ = false;
  allRings = true;
  setSubLevel = true;
  rInternalLvl = -1;
  rotateLevel = 0;

  pInternalLvl = -1;
  platformLevel = 0;
  platformSubLevel = 0;
  datapick["res"] = "reset";
  datapick["type"] = "reset";
  //  dataesp.send(datapick);
}
void pneumaticMove(JSONVar msg)
{
  //  if (pOpen)
  //  {
  //    digitalWrite(13, HIGH);
  //    Serial.println("close");
  //    pOpen = false;
  //  }
  //  else if (!pOpen)
  //  {
  //    digitalWrite(13, LOW);
  start = millis();
  while (millis() - start < 1000)
  {
    digitalWrite(13, HIGH);
    //    Serial.println("close");
  }
  digitalWrite(13, LOW);

}
//void pneumaticClose(JSONVar msg)
//{
//  //Serial.println("Pneumatic Close");
//  digitalWrite(13, HIGH);
//}
//void pneumaticOpen(JSONVar msg)g
//{
//  //Serial.println("Pneumatic Open");
//  digitalWrite(13, LOW);
//}
