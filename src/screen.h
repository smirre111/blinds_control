/*

SSD1306 - Screen module

Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>


This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <Wire.h>
#include "SSD1306Wire.h"
#include "OLEDDisplay.h"
#include "images.h"
#include "fonts.h"
#include "axp20x.h"

#define SCREEN_HEADER_HEIGHT 14

class Screen
{

public:
  Screen(AXP20X_Class &axp, bool axp192_found)
      : _axp(axp),
        _axp192_found(axp192_found),
        _screen_line(SCREEN_HEADER_HEIGHT - 1)
  {
  }

  ~Screen()
  {
    delete (_display);
  }

  void header()
  {
    if (!_display)
      return;

    char buffer[20];

    // // Message count
    // snprintf(buffer, sizeof(buffer), "#%03d", ttn_get_count() % 1000);
    // display->setTextAlignment(TEXT_ALIGN_LEFT);
    // display->drawString(0, 2, buffer);

    // Datetime (if the axp192 PMIC is present, alternate between powerstats and time)
    if (_axp192_found && millis() % 8000 < 3000)
    {
      snprintf(buffer, sizeof(buffer), "%.1fV %.0fmA", _axp.getBattVoltage() / 1000, _axp.getBattChargeCurrent() - _axp.getBattDischargeCurrent());
    }
    // else {
    //     gps_time(buffer, sizeof(buffer));
    // }

    _display->setTextAlignment(TEXT_ALIGN_CENTER);
    _display->drawString(_display->getWidth() / 2, 2, buffer);

    // Satellite count
    // display->setTextAlignment(TEXT_ALIGN_RIGHT);
    // display->drawString(display->getWidth() - SATELLITE_IMAGE_WIDTH - 4, 2, itoa(gps_sats(), buffer, 10));
    // display->drawXbm(display->getWidth() - SATELLITE_IMAGE_WIDTH, 0, SATELLITE_IMAGE_WIDTH, SATELLITE_IMAGE_HEIGHT, SATELLITE_IMAGE);
  }

  void showLogo()
  {
    if (!_display)
      return;

    uint8_t x = (_display->getWidth() - TTN_IMAGE_WIDTH) / 2;
    uint8_t y = SCREEN_HEADER_HEIGHT + (_display->getHeight() - SCREEN_HEADER_HEIGHT - TTN_IMAGE_HEIGHT) / 2 + 1;
    _display->drawXbm(x, y, TTN_IMAGE_WIDTH, TTN_IMAGE_HEIGHT, TTN_IMAGE);
  }

  void screenOff()
  {
    if (!_display)
      return;

    _display->displayOff();
  }

  void screenOn()
  {
    if (!_display)
      return;

    _display->displayOn();
  }

  void clear()
  {
    if (!_display)
      return;

    _display->clear();
  }

  void printText(const char *text, uint8_t x, uint8_t y, uint8_t alignment)
  {
    DEBUG_MSG(text);

    if (!_display)
      return;

    _display->setTextAlignment((OLEDDISPLAY_TEXT_ALIGNMENT)alignment);
    _display->drawString(x, y, text);
  }

  void printText(const char *text, uint8_t x, uint8_t y)
  {
    printText(text, x, y, TEXT_ALIGN_LEFT);
  }

  void printText(const char *text)
  {
    Serial.printf("Screen: %s\n", text);
    if (!_display)
      return;

    _display->print(text);
    if (_screen_line + 8 > _display->getHeight())
    {
      // scroll
    }
    _screen_line += 8;
    loop();
  }

  void printAck()
  {
    _display->clear();
    _display->setFont(ArialMT_Plain_10);
    _display->setTextAlignment(TEXT_ALIGN_LEFT);
    _display->drawString(3, 5, F("Rcvr: packet "));
    _display->display();
  }

  void printString(const String &msg)
  {
    Serial.println("In printString");
    _display->clear();
    _display->setFont(ArialMT_Plain_10);
    _display->setTextAlignment(TEXT_ALIGN_LEFT);
    _display->drawString(28, 22, msg);
    _display->display();
  }

  void update()
  {
    if (_display)
      _display->display();
  }

  void setup()
  {
    // Display instance
    _display = new SSD1306Wire(SSD1306_ADDRESS, I2C_SDA, I2C_SCL);
    _display->init();
    _display->flipScreenVertically();
    _display->setFont(Custom_ArialMT_Plain_10);

    // Scroll buffer
    _display->setLogBuffer(5, 30);
  }

  void setupAndPrint()
  {
    _display->init();
    _display->flipScreenVertically();
    _display->setFont(ArialMT_Plain_10);
    _display->setTextAlignment(TEXT_ALIGN_LEFT);
    _display->drawString(5, 5, "LoRa Receiver");
    _display->display();
  }

  void loop()
  {
    if (!_display)
      return;

#ifdef T_BEAM_V10
    if (_axp192_found && pmu_irq)
    {
      pmu_irq = false;
      _axp.readIRQ();
      if (_axp.isChargingIRQ())
      {
        baChStatus = "Charging";
      }
      else
      {
        baChStatus = "No Charging";
      }
      if (_axp.isVbusRemoveIRQ())
      {
        baChStatus = "No Charging";
      }
      Serial.println(baChStatus); //Prints charging status to screen
      digitalWrite(2, !digitalRead(2));
      _axp.clearIRQ();
    }
#endif

    _display->clear();
    header();
    _display->drawLogBuffer(0, SCREEN_HEADER_HEIGHT);
    _display->display();
  }

  SSD1306Wire *_display;
  AXP20X_Class &_axp;
  bool _axp192_found;
  uint8_t _screen_line;
};
