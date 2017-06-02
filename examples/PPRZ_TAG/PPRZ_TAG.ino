/**
 * TAG for use for trilateration use with Paparazzi UAV System
 *
 * Tag is placed on the mobile robots and are sending each new measured distance
 *
 */
#include <SPI.h>
#include "DW1000Ranging.h"

#define DEBUG 0

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

void setup() {
  Serial.begin(115200);
  delay(1000);
  //init the configuration
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
#if DEBUG
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
#endif
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);

  //we start the module as a tag
  DW1000Ranging.startAsTag("00:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

void loop() {
  DW1000Ranging.loop();
}

static uint8_t* float_to_buf(float f, uint8_t* b) {
  memcpy (b, (uint8_t*)(&f), sizeof(float));
  return b;
}

static uint8_t* uint16_to_buf(uint16_t u, uint8_t* b) {
  memcpy (b, (uint8_t*)(&u), sizeof(uint16_t));
  return b;
}

static uint8_t buf[6];

void newRange() {
  uint16_t address = DW1000Ranging.getDistantDevice()->getShortAddress();
  float dist = DW1000Ranging.getDistantDevice()->getRange();

  uint16_to_buf(address, buf);
  float_to_buf(dist, buf+2);

  uint8_t checksum = 0x00;
  for (int i = 0; i < 6; i++) {
    checksum += buf[i];
  }

#if DEBUG
  Serial.print("from: "); Serial.print(address, HEX);
  Serial.print("\t Range: "); Serial.print(dist); Serial.print(" m");
  Serial.print("\t RX power: "); Serial.print(DW1000Ranging.getDistantDevice()->getRXPower()); Serial.println(" dBm");
#else
  Serial.write(0xFE);
  Serial.write(buf, 6);
  Serial.write(checksum);
#endif
}

#if DEBUG
void newDevice(DW1000Device* device) {
  Serial.print("ranging init; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
#endif

