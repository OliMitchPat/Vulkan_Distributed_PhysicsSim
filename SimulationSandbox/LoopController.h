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
 *   endFrame() sleeps most of the remaining budget, then spin-yields for the
 *   final sub-millisecond to improve accuracy without burning 100% CPU.
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
    // Sleeps + spin-yields so that the loop runs at targetHz.
    // If targetHz <= 0, returns immediately (uncapped).
    void endFrame()
    {
        using Clock = std::chrono::steady_clock;
        using Dsec  = std::chrono::duration<double>;

        float hz = m_targetHz.load(std::memory_order_relaxed);
        if (hz <= 0.0f) return;

        double targetPeriod = 1.0 / static_cast<double>(hz);

        // Sleep for most of the remaining budget (leave 1 ms for spin)
        double elapsed = Dsec(Clock::now() - m_frameStart).count();
        double toSleep  = targetPeriod - elapsed - 0.001;
        if (toSleep > 0.0)
        {
            std::this_thread::sleep_for(Dsec(toSleep));
        }

        // Spin-yield for sub-millisecond accuracy
        while (Dsec(Clock::now() - m_frameStart).count() < targetPeriod)
        {
            std::this_thread::yield();
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
