#include <LiquidCrystal.h>
#include <GyverButton.h>
#include <IRremote.hpp>

#define MAX_NUMBER_SAVE_PACKETS 5 // Увеличь число, если требуется хранить большее кол-во пакетов
 
// Для Экрана
#define RS 13
#define EN 12
#define D4 11
#define D5 10
#define D6 9
#define D7 8

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

GButton btnSend(A2);
GButton btnSetIrPacket(A3);

IRData packets[MAX_NUMBER_SAVE_PACKETS];

byte numberWritePacket = 0;
byte currentPacket = 0;
bool isSend = false;
bool modWrite = true;

void lcdWrite(String text, byte colm, boolean mod, byte arg=0) {
  lcd.setCursor(0, colm);
  if (mod) {
    lcd.print(text);lcd.print(arg);
  } else {
    lcd.print(text);
  }
}

void setup() {
  IrReceiver.enableIRIn();
  IrReceiver.begin(A0);
  IrSender.begin(A1);

  lcd.begin(16, 2);
  
  lcdWrite("Welcome to", 0, false);
  lcdWrite("IRGrub!", 1, false);
}

void loop() {
  btnSend.tick();
  btnSetIrPacket.tick();

  if (btnSend.isPress() && packets[currentPacket].protocol != 0) {
    IrSender.enableIROut(38);
    isSend = true;

    sendIRPacket(&packets[currentPacket], 1);

    lcd.clear();
    lcdWrite("Send IR Packet!", 0, false);
    lcdWrite(getProtocolString(packets[currentPacket].protocol), 1, false);
    delay(40);
  }

  if (btnSetIrPacket.isPress()) {
    currentPacket++; 
    if (currentPacket > MAX_NUMBER_SAVE_PACKETS-1) {
      currentPacket = 0;
    }

    lcd.clear();
    lcdWrite(getProtocolString(packets[currentPacket].protocol), 0, false);
    lcdWrite("Packet: ", 1, true, currentPacket);
  } 
  if (btnSetIrPacket.isHolded()) {
    modWrite = !modWrite;
    lcd.clear();
    lcdWrite("Mod Write: ", 1, true, modWrite ? 1 : 0);
  }
  
  if (IrReceiver.decode()) {
    delay(400);

    if (modWrite && !isSend) {
      lcd.clear();
      lcdWrite(getProtocolString(IrReceiver.decodedIRData.protocol), 0, false);
      lcdWrite("Write: ", 1, true, numberWritePacket);
      
      packets[numberWritePacket] = IrReceiver.decodedIRData;
      numberWritePacket++;
        
      if (numberWritePacket > MAX_NUMBER_SAVE_PACKETS-1) {
        numberWritePacket = 0;
      }
    } else {
      isSend = false;
    }

    IrReceiver.resume();
    IrReceiver.enableIRIn();
  }
}

void sendPulseDistancePacket(uint16_t aAddress, uint16_t aCommand, 
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
           sendPulseDistancePacket(tAddress, tCommand, aIRSendData->extra, 
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
