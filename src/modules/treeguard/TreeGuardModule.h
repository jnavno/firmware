#pragma once
#include "mesh/SinglePortModule.h" // <- IMPORTANT: use SinglePortModule
#include "Observer.h"
#include "concurrency/OSThread.h"

class TreeGuardModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    // Register as a single-port module; we use it only to TX text
    TreeGuardModule();

  protected:
    // Run once after boot, then we deep sleep.
    virtual int32_t runOnce() override;

  private:
    void sendText(const char *message);
    void sampleAccelBlock();
    void processTimerWake();
    void processVibrationWake();
    void goToDeepSleep();
    
    // Observer callback for deep sleep preparation
    int prepareDeepSleep(void *unused);
    CallbackObserver<TreeGuardModule, void *> notifyDeepSleepObserver =
        CallbackObserver<TreeGuardModule, void *>(this, &TreeGuardModule::prepareDeepSleep);
};
