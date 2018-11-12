#include "hidjoystickrptparser.h"
#include <util/crc16.h>

JoystickReportParser::JoystickReportParser(JoystickEvents *evt) :
joyEvents(evt),
oldHat(0xDE),
oldButtons(0) {
        for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
                oldPad[i] = 0xD;
}

unsigned char reverse(unsigned char b, bool bo, bool bo2) {
   if(bo){
     b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   }
   //b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   if(bo2){
     b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   }
   return b;
}

void JoystickReportParser::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
        bool match = true;

        // Checking if there are changes in report since the method was last called
        for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
                if (buf[i] != oldPad[i]) {
                        match = false;
                        break;
                }
                uint8_t hat = (buf[5] & 0xF);
        // Calling Game Pad event handler
        uint16_t buttons = (0x0000 | buf[6]);
        buttons <<= 4;
        buttons |= (buf[5] >> 4);
        uint16_t changes = (buttons ^ oldButtons);
        if (!match && joyEvents || hat != oldHat || changes) {
                if (hat != oldHat && joyEvents) {
                    oldHat = hat;
                }

                oldButtons = buttons;
        }
                joyEvents->OnGamePadChanged((const GamePadEventData*)buf, oldHat, buttons);
                
                for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++) oldPad[i] = buf[i];
        }

        

//        // Calling Hat Switch event handler
//        if (hat != oldHat && joyEvents) {
//                joyEvents->OnHatSwitch(hat);
//                oldHat = hat;
//        }
//
//        uint16_t buttons = (0x0000 | buf[6]);
//        buttons <<= 4;
//        buttons |= (buf[5] >> 4);
//        uint16_t changes = (buttons ^ oldButtons);
//
//        // Calling Button Event Handler for every button changed
//        if (changes) {
//                for (uint8_t i = 0; i < 0x0C; i++) {
//                        uint16_t mask = (0x0001 << i);
//
//                        if (((mask & changes) > 0) && joyEvents) {
//                                if ((buttons & mask) > 0)
//                                        
//                                else
//                                        
//                        }
//                }
//                oldButtons = buttons;
//        }
//}
uint8_t a;
uint8_t b;
void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt,uint8_t hat,uint8_t but) {
        Serial.write(0x5A);
        if(evt->Rz != 0x08){
          switch((evt->Rz|0xF0)^0xF0){
              case 0x00: a = 0xFF^0x10; break;
              case 0x02: a = 0xFF^0x20; break;
              case 0x01: a = 0xFF^0x30; break;
              case 0x04: a = 0xFF^0x40; break;
              case 0x03: a = 0xFF^0x60; break;
              case 0x06: a = 0xFF^0x80; break;
              case 0x07: a = 0xFF^0x90; break;
              case 0x05: a = 0xFF^0xC0; break;
              case 0x08: a = 0xFF^0x00; break;
              default: a = (0xFF^(0xFF^((reverse(evt->Rz, 1, 1)|0x0F)^0xF0))); break;
          }
        }else{
            a = 0xFF;
        }
        if(but != 0x00){
          switch (0xFF^((but|0xF0)^0x0F)){
            case 0x01: a = a^0x01;break;
            case 0x02: a = a^0x08;break;
            case 0x03: a = a^0x09;break;
            case 0x04: a = a^0x02;break;
            case 0x05: a = a^0x03;break;
            case 0x06: a = a^0x0A;break;
            case 0x07: a = a^0x0B;break;
            case 0x08: a = a^0x04;break;
            case 0x09: a = a^0x05;break;
            case 0x0A: a = a^0x0C;break;
            case 0x0B: a = a^0x0D;break;
            case 0x0C: a = a^0x06;break;
            case 0x0D: a = a^0x07;break;
            case 0x0E: a = a^0x0E;break;
            case 0x0F: a = a^0x0F;break;
          }
       }
       Serial.write(a);

       
       if(evt->Rz != 0x08){
            b = (0xFF^(0xFF^((reverse(evt->Rz, 0, 1)|0x0F)^0xF0)));
       }else{
            b = 0xFF;
       }
       if(hat != 0x00){
            b = b^reverse(hat, 0, 0);
       }
        Serial.write(b);
        
        

        Serial.write(evt->Z1);
        Serial.write(evt->Z2);
        Serial.write(evt->X);
        Serial.write(evt->Y);
          //PrintHex<uint8_t > (evt->Rz, 0x80);
}
