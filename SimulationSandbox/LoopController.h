#pragma once
/*
 * LoopController.h — Rate-limited loop with live-adjustable target Hz and
 *                    measured Hz reporting.
 *
 * Usage:
 *   LoopController lc(60.0f);    // 60 Hz target
 *   while (running) {
 *       float dt = lc.beginFrame();
 *       // ... do work ...
 *       lc.endFrame();            // sleeps + spins to hit target Hz
 *   }
 *
 * Thread-safety:
 *   setTargetHz() / getTargetHz() / getMeasuredHz() are safe to call from any
 *   thread.  beginFrame() / endFrame() must be called from the owning thread.
 *
 * Rate limiting:
 *   endFrame() sleeps most of the remaining budget, then busy-spins for the
 *   final short slice. At high target rates, yielding can overshoot by a whole
 *   scheduler quantum and halve the measured Hz.
 *   If targetHz is 0 the loop runs uncapped.
 */

#include <atomic>
#include <chrono>
#include <thread>

class LoopController
{
public:
    explicit LoopController(float targetHz = 60.0f)
        : m_targetHz(targetHz), m_measuredHz(0.0f)
    {
    }

    // ---- Live-adjustable target ---------------------------------------------
    void  setTargetHz(float hz) { m_targetHz.store(hz, std::memory_order_relaxed); }
    float getTargetHz()   const { return m_targetHz.load(std::memory_order_relaxed); }

    // ---- Measured Hz (updated every ~0.5 s) ---------------------------------
    float getMeasuredHz() const { return m_measuredHz.load(std::memory_order_relaxed); }

    // ---- Frame control ------------------------------------------------------

    // Call at the TOP of the loop body.
    // Returns the actual elapsed time (seconds) since the previous beginFrame().
    // Returns 0 on the very first call (initialisation only).
    float beginFrame()
    {
        using Clock = std::chrono::steady_clock;
        using Dsec  = std::chrono::duration<double>;

        Clock::time_point now = Clock::now();

        if (!m_initialized)
        {
            m_frameStart     = now;
            m_fpsWindowStart = now;
            m_initialized    = true;
            return 0.0f;
        }

        double dt = Dsec(now - m_frameStart).count();
        m_frameStart = now;

        // Update measured Hz over a 0.5-second window
        ++m_fpsFrameCount;
        double windowElapsed = Dsec(now - m_fpsWindowStart).count();
        if (windowElapsed >= 0.5)
        {
            float hz = (windowElapsed > 0.0)
                ? static_cast<float>(m_fpsFrameCount / windowElapsed)
                : 0.0f;
            m_measuredHz.store(hz, std::memory_order_relaxed);
            m_fpsFrameCount  = 0;
            m_fpsWindowStart = now;
        }

        return static_cast<float>(dt);
    }

    // Call at the END of the loop body (after work is done).
    // Sleeps + spins so that the loop runs at targetHz.
    // If targetHz <= 0, returns immediately (uncapped).
    void endFrame()
    {
        using Clock = std::chrono::steady_clock;
        using Dsec  = std::chrono::duration<double>;

        float hz = m_targetHz.load(std::memory_order_relaxed);
        if (hz <= 0.0f) return;

        double targetPeriod = 1.0 / static_cast<double>(hz);

        // Sleep for most of the remaining budget. At high rates the whole
        // period can be only 1-2 ms, and Windows sleep_for can overshoot by
        // enough to cap a 1000 Hz loop around 500 Hz. For those rates, spin
        // through the full wait instead.
        double elapsed = Dsec(Clock::now() - m_frameStart).count();
        if (hz < 500.0f)
        {
            constexpr double spinWindow = 0.001;
            double toSleep = targetPeriod - elapsed - spinWindow;
            if (toSleep > 0.0)
            {
                std::this_thread::sleep_for(Dsec(toSleep));
            }
        }

        // Busy-spin for the final short slice. Do not call yield() here:
        // on Windows it can hand away the CPU long enough to miss 1000 Hz.
        while (Dsec(Clock::now() - m_frameStart).count() < targetPeriod)
        {
        }
    }

private:
    std::atomic<float> m_targetHz;
    std::atomic<float> m_measuredHz;

    using TimePoint = std::chrono::steady_clock::time_point;
    TimePoint m_frameStart{};
    TimePoint m_fpsWindowStart{};
    int  m_fpsFrameCount = 0;
    bool m_initialized   = false;
};
