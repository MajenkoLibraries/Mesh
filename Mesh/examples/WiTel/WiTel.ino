/*
 * Copyright (c) 2015, Majenko Technologies
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 * * Neither the name of Majenko Technologies nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* This example sketch creates a mesh of nRF24L01+ devices and allows
 * any node to dial any other node just like calling a dialup modem.
 *
 * Supported AT commands:
 *
 * AT           Confirm it's working - returns OK
 * ATI / ATI1   Display mesh information
 * ATS1=[x]     Set the ID of this node to [x] (stores in EEPROM)
 * ATDT[x]      Dial a remote node [x]
 * ATH          Hang up the current call
 * +++          Escape from the current call to command mode (so you can ATH)
 *
 * Special code:  ATB - Reset the board into bootloader mode (on boards
 *                      that support it - great for remote programming)
 */

#include <nRF24L01.h>
#include <EEPROM.h>
#include <Mesh.h>

// This is to auto-generate a random hardware address at compile time.
// It only works on UECIDE. If you are working in any other environment
// then you will need to define the macro ADDRESS and make sure it's
// unique for every node in range.

#pragma parameter extra.flags=-DADDRESS=${random:0,255},${random:0,255},${random:0,255},${random:0,255},${random:0,255}::-DCDCACM_PROD="WiTel USB Adapter"::-DCDCACM_MAN="Majenko Technologies"::-DCDCACM_SER="${uecide:uuid}"
const uint8_t myAddress[5] = { ADDRESS };

// Define the SPI port and pins to communicate with the radio module
DSPI0 spi;
nRF24L01 rf(spi, 3, 18, 0);

// Our user packet types:

// Data packets
#define SERIAL      0x01

// Requests
#define RING        0x10
#define ANSWER      0x11
#define HANGUP      0x12

// Positive responses
#define ANSWER_OK   0x21
#define HANGUP_OK   0x22

// Negative responses
#define BUSY        0x30
#define ANSWER_NO   0x31
#define HANGUP_NO   0x32


// Create the mesh environment
Mesh mymesh;

uint16_t myID = 0xFFFF; // Not assigned yet

// States for our state machine
#define MODE_COMMAND 0
#define MODE_RINGING 1
#define MODE_CONNECTED 2
#define MODE_HANGUP 3

int mode = MODE_COMMAND;
int lastRinger = 0;
int connectedHost = 0;

// The callback to deal with incoming packets that aren't internal Mesh messages
void processPacket(uint16_t sender, uint8_t type, uint8_t *data, uint8_t len) {
    switch (type) {
        case RING: // Incoming "Ring Ring"
            if (mode == MODE_CONNECTED) {
                mymesh.sendPacket(sender, BUSY, 0, 0);
            } else {
                Serial.println("RING");
                lastRinger = sender;
            }
            break;

        case HANGUP: // Hang up the line
            mymesh.sendPacket(connectedHost, HANGUP_OK, 0, 0);
        case HANGUP_OK:
            Serial.println("NO CARRIER");
            connectedHost = 0;
            mode = MODE_COMMAND;
            break;

        case ANSWER_NO:
            lastRinger = 0;
            Serial.println("ERR");
            break;

        case ANSWER: // I want to answer you.
            if (mode != MODE_RINGING) {
                mymesh.sendPacket(sender, ANSWER_NO, 0, 0);
            } else if (connectedHost == NULL) {
                mymesh.sendPacket(sender, ANSWER_NO, 0, 0);
            } else if (sender != connectedHost) {
                mymesh.sendPacket(sender, ANSWER_NO, 0, 0);
            } else {
                mymesh.sendPacket(sender, ANSWER_OK, 0, 0);
                Serial.println("CONNECTED 115200");
                mode = MODE_CONNECTED;
            }
            break;
        case ANSWER_OK: // You may answer me.
            connectedHost = sender;
            Serial.println("CONNECTED 115200");
            mode = MODE_CONNECTED;
            break;
            
        case SERIAL: // Serial data received
            if (mode == MODE_CONNECTED) {
                if (sender == connectedHost) {
                    Serial.write(data, len);
                }
            }
            break;

        case BUSY: // Busy, go away!
            Serial.println("BUSY");
            mode = MODE_COMMAND;
            connectedHost = 0;
            break;
    }    
}

int readline(int readch, char *buffer, int len) {
  static int pos = 0;
  int rpos;

  if (readch > 0) {
    switch (readch) {
      case '\n': // Ignore new-lines
        break;
      case '\r': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        return rpos;
      default:
        if (pos < len - 1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}

void setup() {
    Serial.begin(115200);
    spi.begin();
    rf.begin(ADDRESS, 0);

    mymesh.addDevice(rf);
    mymesh.addUnicastCallback(processPacket);
    pinMode(PIN_LED1, OUTPUT);
    myID = (EEPROM.read(0) << 8) | EEPROM.read(1);
    mymesh.setLEDPin(PIN_LED1);
    mymesh.setID(myID);
}

void loop() {

    mymesh.process();

    switch (mode) {
        case MODE_HANGUP:
        case MODE_COMMAND:
            processSerialCommands();
            break;
        case MODE_RINGING:
            ringHost();
            break;
        case MODE_CONNECTED:
            transferData();
            break;
    }
}

void processSerialCommands() {
    static boolean willEcho = false;
    static char commandBuffer[80];
    
    if (!Serial.available()) {
        return;
    }

    int ch = Serial.read();
    Serial.write(ch);
    int len = readline(ch, commandBuffer, 80);
    if (len > 0) {
        Serial.println();
        if (len < 2) {
            Serial.println("ERR");
            return;
        }

        // All commands start with AT regardless.  Anything else must be an error.
        if ((commandBuffer[0] != 'A') && (commandBuffer[0] != 'a')) {
            Serial.println("ERR");
            return;
        }
        if ((commandBuffer[1] != 'T') && (commandBuffer[1] != 't')) {
            Serial.println("ERR");
            return;
        }

        if (len == 2) { // Simple AT
            Serial.println("OK");
            return;
        }

        switch (commandBuffer[2]) {
            // Enter bootloader
            case 'B':
            case 'b':
                executeSoftReset(USE_VIRTUAL_PROGRAM_BUTTON);
                while(1);
            // Information
            case 'I':
            case 'i':
                switch (commandBuffer[3]) {
                    case '\0':
                    case '1':
                        Serial.print(mymesh);
                        Serial.println("OK");
                        return;
                    default:
                        Serial.println("ERR");
                        return;
                }
                return;
            // Dial
            case 'D':
            case 'd':
                dialNode(atoi(commandBuffer + 4));
                return;
            case 'S':
            case 's':
                if (commandBuffer[4] != '=') {
                    Serial.println("ERR");
                    return;
                }
                switch (commandBuffer[3]) {
                    case '1':
                        myID = atoi(commandBuffer + 5);
                        mymesh.setID(myID);
                        EEPROM.write(0, myID >> 8);
                        EEPROM.write(1, myID & 0xFF);
                        Serial.println("OK");
                        return;
                    default:
                        Serial.println("ERR");
                        return;
                }
                return;
            case 'A':
            case 'a':
                if (mode == MODE_HANGUP) {
                    mode = MODE_CONNECTED;
                    Serial.println("CONNECTED 115200");
                } else {
                    answerPhone();
                }
                return;
            case 'H':
            case 'h':
                if (mode == MODE_HANGUP) {
                    mymesh.sendPacket(connectedHost, HANGUP, 0, 0);
                } else {
                    Serial.println("ERR");
                }
        }
    }
}

void ringHost() {
    static uint32_t lastRing = 0;
    if (millis() - lastRing > 5000) {
        lastRing = millis();
        if (!mymesh.sendPacket(connectedHost, RING, 0, 0)) {
            Serial.println("NO ANSWER");
            mode = MODE_COMMAND;
            connectedHost = 0;
        }
    }
    // Cancel on a keypress
    if (Serial.available()) {
        Serial.read(); // Consume the pressed key
        connectedHost = 0;
        Serial.println("OK");
        mode = MODE_COMMAND;
    }
}


void transferData() {
    static int plusCount = 0;
    static uint32_t lastChar = 0;

    if (millis() - lastChar > 500) {
        if (plusCount == 3) {
            mode = MODE_HANGUP;
            Serial.println("OK");
            return;
        }
    }
    
    if (!Serial.available()) {
        return;
    }    

    if (connectedHost == NULL) {
        Serial.println("ERR");
        return;
    }

    uint8_t data[Mesh::MTU];

    int i = 0;
    while (Serial.available() && i < Mesh::MTU) {
        int c = Serial.read();

        lastChar = millis();

        if (c == '+') {
            plusCount++;
        } else {
            plusCount = 0;
        }
        
        data[i] = c;
        i++;
    }
    if (!mymesh.sendPacket(connectedHost, SERIAL, data, i)) {
        Serial.println("NO CARRIER");
        connectedHost = 0;
        mode = MODE_COMMAND;
    }
}

void dialNode(int n) {
    if (n <= 0 || n >= 65535) {
        Serial.println("ERR");
        return;
    }
    if (!mymesh.knowHost(n)) {
        Serial.println("ERR");
        return;
    }
    connectedHost = n;
    mode = MODE_RINGING;
}

void answerPhone() {
    if (lastRinger == 0) {
        Serial.println("ERR");
        return;
    }

    mymesh.sendPacket(lastRinger, ANSWER, 0, 0);
}
