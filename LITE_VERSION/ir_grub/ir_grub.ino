#include <IRremote.hpp>

#define btn A2

void setup() {
  pinMode(btn, INPUT_PULLUP);
  
  IrReceiver.enableIRIn();
  IrReceiver.begin(A0);
  IrSender.begin(A1);

// Разкомментируй, если нужен вывод поступающих на датчик пакетов
//  Serial.begin(9600);
}

void loop() {
  int val = !digitalRead(btn);

  if (val == HIGH) {
    IrSender.enableIROut(38);
// Разкомментируй, если нужен вывод поступающих на датчик пакетов
//    Serial.println("Send IR packet!");
//    Serial.println("=========INFO============");

    sendIRPacket(&IrReceiver.decodedIRData, 1);
    delay(100);
  }

  if (IrReceiver.decode()) {
// Разкомментируй, если нужен вывод поступающих на датчик пакетов
//    debug();
    delay(100);

    IrReceiver.resume();
    IrReceiver.enableIRIn();
  }
}

// Разкомментируй, если нужен вывод поступающих на датчик пакетов
//void debug() {
//  Serial.print("Protocol String: ");
//  Serial.println(getProtocolString(IrReceiver.decodedIRData.protocol));
//  
//  Serial.print("Protocol Int: ");
//  Serial.println(IrReceiver.decodedIRData.protocol);
//  
//  Serial.print("Command: ");
//  Serial.println(IrReceiver.decodedIRData.command, HEX);
//  
//  Serial.print("Address: ");
//  Serial.println(IrReceiver.decodedIRData.address, HEX);
//  
//  Serial.print("Raw-Data: ");
//  Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
//  
//  Serial.print("Receive Data: "); 
//  IrReceiver.printIRResultShort(&Serial, &IrReceiver.decodedIRData, true);
//  IrReceiver.compensateAndPrintIRResultAsCArray(&Serial, true);
//  
//  IrReceiver.printIRSendUsage(&Serial);
//  
//  Serial.println("===================================");
//}

void sendRawPacket(uint16_t aAddress, uint16_t aCommand, 
    uint16_t aExtra, uint16_t aNumberOfBits, uint32_t aDecodedRawData
) {
  unsigned int tHeaderMarkMicros = (aExtra >> 8) * MICROS_PER_TICK;
  unsigned int tHeaderSpaceMicros = (aExtra & 0xFF) * MICROS_PER_TICK;
  unsigned int tOneMarkMicros = (aAddress >> 8) * MICROS_PER_TICK;
  unsigned int tOneSpaceMicros = (aAddress & 0xFF) * MICROS_PER_TICK;
  unsigned int tZeroMarkMicros = (aCommand >> 8) * MICROS_PER_TICK;
  unsigned int tZeroSpaceMicros = (aCommand & 0xFF) * MICROS_PER_TICK;

  boolean tProtocolIs;
  if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_MSB_FIRST) {
    tProtocolIs = PROTOCOL_IS_MSB_FIRST;
  } else {
    tProtocolIs = PROTOCOL_IS_LSB_FIRST;
  }

#if __INT_WIDTH__ < 32
   uint_fast8_t tNumberOfArrayData = ((aNumberOfBits - 1) / 32) + 1;
   if(tNumberOfArrayData > 1) {
      uint32_t tRawData[tNumberOfArrayData] = {};
#else
   uint_fast8_t tNumberOfArrayData = ((aNumberOfBits - 1) / 64) + 1;
   if(tNumberOfArrayData > 1) {
      uint64_t tRawData[tNumberOfArrayData] = {};
#endif
      for (uint_fast8_t i = 0; i < tNumberOfArrayData; ++i) {
          tRawData[i] = IrReceiver.decodedIRData.decodedRawDataArray[i];
      }
      
      IrSender.sendPulseDistanceWidthFromArray(38, tHeaderMarkMicros, 
          tHeaderSpaceMicros, tOneMarkMicros, tOneSpaceMicros, 
          tZeroMarkMicros, tZeroSpaceMicros, &tRawData[0],
          aNumberOfBits, tProtocolIs, SEND_STOP_BIT, 0, NO_REPEATS);
   } else {
       IrSender.sendPulseDistanceWidth(38, tHeaderMarkMicros, 
          tHeaderSpaceMicros, tOneMarkMicros, tOneSpaceMicros, 
          tZeroMarkMicros, tZeroSpaceMicros, aDecodedRawData,
          aNumberOfBits, tProtocolIs, SEND_STOP_BIT, 0, NO_REPEATS);
    }
}

void sendIRPacket(IRData *aIRSendData, int_fast8_t aNumberOfRepeats) {
     decode_type_t tProtocol = aIRSendData->protocol;
     uint16_t tAddress = aIRSendData->address;
     uint16_t tCommand = aIRSendData->command;
     uint32_t tDecodedRawData = aIRSendData->decodedRawData;
     uint16_t tNumberOfBits = aIRSendData->numberOfBits;
     bool tIsRepeat = (aIRSendData->flags & IRDATA_FLAGS_IS_REPEAT);
     
     if (tIsRepeat) {
         aNumberOfRepeats = -1;
     }
     
     switch (tProtocol) {
        case PULSE_DISTANCE:
           sendRawPacket(tAddress, tCommand, aIRSendData->extra, 
              tNumberOfBits, tDecodedRawData);
           break;
        case NEC:
           IrSender.sendNEC(tAddress, tCommand, aNumberOfRepeats);
           break;
        case SAMSUNG:
           IrSender.sendSamsung(tAddress, tCommand, aNumberOfRepeats);
           break;
        case SAMSUNG48:
           IrSender.sendSamsung48(tAddress, tCommand, aNumberOfRepeats);
           break;
       case SAMSUNG_LG:
           IrSender.sendSamsungLG(tAddress, tCommand, aNumberOfRepeats);
           break;
       case SONY:
           IrSender.sendSony(tAddress, tCommand, aNumberOfRepeats, tNumberOfBits);
           break;
       case PANASONIC:
           IrSender.sendPanasonic(tAddress, tCommand, aNumberOfRepeats);
           break;
       case DENON:
           IrSender.sendDenon(tAddress, tCommand, aNumberOfRepeats);
           break;
       case SHARP:
           IrSender.sendSharp(tAddress, tCommand, aNumberOfRepeats);
           break;
       case LG:
           IrSender.sendLG(tAddress, tCommand, aNumberOfRepeats);
           break;
       case JVC:
           IrSender.sendJVC((uint8_t) tAddress, 
              (uint8_t) tCommand, aNumberOfRepeats);
              
           break;
       case RC5:
           IrSender.sendRC5(tAddress, tCommand, aNumberOfRepeats, !tIsRepeat);
           break;
       case RC6:
           IrSender.sendRC6(tAddress, tCommand, aNumberOfRepeats, !tIsRepeat);
           break;
       case KASEIKYO_JVC:
           IrSender.sendKaseikyo_JVC(tAddress, tCommand, aNumberOfRepeats);
           break;
       case KASEIKYO_DENON:
           IrSender.sendKaseikyo_Denon(tAddress, tCommand, aNumberOfRepeats);
           break;
       case KASEIKYO_SHARP:
           IrSender.sendKaseikyo_Sharp(tAddress, tCommand, aNumberOfRepeats);
           break;
       case KASEIKYO_MITSUBISHI:
           IrSender.sendKaseikyo_Mitsubishi(tAddress, tCommand, aNumberOfRepeats);
           break;
       case NEC2:
           IrSender.sendNEC2(tAddress, tCommand, aNumberOfRepeats);
           break;
       case ONKYO:
           IrSender.sendOnkyo(tAddress, tCommand, aNumberOfRepeats);
           break;
       case APPLE:
           IrSender.sendApple(tAddress, tCommand, aNumberOfRepeats);
           break;
     }
}
