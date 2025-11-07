#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "Observer.h"
#include "concurrency/OSThread.h"
#include "mesh/SinglePortModule.h"

// we still include these, that's fine
#include "modules/treeguard/fft/fft_config.h"

// ── 1) define the constants ONCE here ─────────────────────────────
static constexpr uint16_t TG_SAMPLE_HZ = 333;
static constexpr uint16_t TG_CAPTURE_SEC = 12;
static constexpr uint16_t TG_NUM_SAMPLES = TG_SAMPLE_HZ * TG_CAPTURE_SEC;

struct Features;

class TreeGuardModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    TreeGuardModule();

    // we can keep this public, it’s just a plain POD
    struct TG_VibCapture {
        int16_t ax[TG_NUM_SAMPLES];
        int16_t ay[TG_NUM_SAMPLES];
        int16_t az[TG_NUM_SAMPLES];
    };

  protected:
    int32_t runOnce() override;

  private:
    void sendText(const char *message);
    void sampleAccelBlock();
    void processTimerWake();
    void processVibrationWake();
    void goToDeepSleep();

    // 12 s @ 333 Hz capture
    bool captureVibration333(TG_VibCapture &cap);

    // turn that capture into the 1024-sample FFT classification
    String classifyFromCapture(const TG_VibCapture &cap, Features *outFeatures, float &hf, float &r1, float &r2, int &sc);

    int prepareDeepSleep(void *unused);
    CallbackObserver<TreeGuardModule, void *> notifyDeepSleepObserver =
        CallbackObserver<TreeGuardModule, void *>(this, &TreeGuardModule::prepareDeepSleep);
};
