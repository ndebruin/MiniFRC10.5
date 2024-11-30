#include <Arduino.h>

#include "coProcCom.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

bool updateCoProc = false;

CoProcStructTX txDataStruct;
CoProcStructRX rxDataStruct;

void loop() {
  // if we have data from the coproc
  updateFromCoProc(&rxDataStruct);

  

}

