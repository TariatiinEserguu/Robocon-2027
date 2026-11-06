#pragma once
// Bluepad32 gamepad connection + timeout (from r1-controller).
#include <Bluepad32.h>
#include "config.h"

namespace Ctl {
  ControllerPtr controller = nullptr;
  uint32_t lastUpdatedTime = 0;

  void onConnectedController(ControllerPtr ctl) {
    Serial.printf("Connecting controller: %s\n", ctl->getModelName().c_str());
    if (!controller) {
      controller = ctl;
      Serial.println("Connected");
    }
  }

  void onDisconnectedController(ControllerPtr ctl) {
    if (controller == ctl) controller = nullptr;
    if (!controller) Serial.println("Disconnected.");
  }

  // true = controller connected and data is fresh
  bool update() {
    bool dataUpdated = BP32.update();          // true when new gamepad data arrived
    bool connected = controller && controller->isConnected();
    if (!connected) return false;
    if (dataUpdated) lastUpdatedTime = millis();
    return millis() - lastUpdatedTime <= (uint32_t)Cfg::CONTROLLER_TIMEOUT_MS;
  }

  void setup() {
    lastUpdatedTime = millis();
    BP32.setup(&onConnectedController, &onDisconnectedController);
#ifdef CONFIG_BLUEPAD32_MAX_DEVICES
    BP32.enableVirtualDevice(false);         // no virtual mouse from DS4/DualSense touchpad
#endif
  }
}
