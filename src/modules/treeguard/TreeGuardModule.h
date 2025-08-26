#pragma once
#include "mesh/SinglePortModule.h" // <- IMPORTANT: use SinglePortModule

class TreeGuardModule : public SinglePortModule
{
  public:
    // Register as a single-port module; we use it only to TX text
    TreeGuardModule();
    // Run once after boot, then we deep sleep.
    int32_t runOnce() override;

  private:
    void sendText(const char *message);
    void sampleAccelBlock();
    void processTimerWake();
    void processVibrationWake();
    void goToDeepSleep();
};
