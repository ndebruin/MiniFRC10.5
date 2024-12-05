#ifndef COPROCCOM_H
#define COPROCCOM_H

// #include <Arduino.h>

// struct CoProcStructTX {
//   bool camFlash;    // 1 byte
//   // total: 1 byte
// };

struct CoProcStructRX {
    int64_t posX;   // 8 bytes
    int64_t posY;   // 8 bytes
    float yaw;      // 4 bytes
    // total: 20 bytes
};

CoProcStructRX COPROCRXDATA;

bool coProcReceive(CoProcStructRX* table)
{
  return (Serial.readBytes((char*)table, sizeof(CoProcStructRX)) == sizeof(CoProcStructRX)); // 25 bytes.
}

bool updateFromCoProc(CoProcStructRX* table)
{   
    if(Serial.available() > sizeof(struct CoProcStructRX))
    {
        if(!coProcReceive(&COPROCRXDATA)){
        //                                            ERROR DEBUG STATE
        }
        *table = COPROCRXDATA;
        return true; // updated
    }
    return false; // didn't update
}

#endif // COPROCCOM_H