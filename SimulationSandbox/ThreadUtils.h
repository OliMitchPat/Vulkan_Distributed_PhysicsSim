#pragma once
/*
 * ThreadUtils.h — Windows thread affinity and naming helpers.
 *
 * Core mapping (logical cores, 0-based):
 *   Core 0  -> Render / UI / main thread
 *   Core 1  -> Networking thread A
 *   Core 2  -> Networking thread B
 *   Core 3+ -> Simulation thread(s)
 *
 * If the process has fewer logical cores than requested, the helpers degrade
 * gracefully: they never crash and always leave the thread running.
 */

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

namespace ThreadUtils
{
    // ---- Core index constants ------------------------------------------------
    constexpr int CORE_RENDER = 0;
    constexpr int CORE_NET_0  = 1;
    constexpr int CORE_NET_1  = 2;
    constexpr int CORE_SIM_0  = 3;

    // ---- Helpers -------------------------------------------------------------

    // Returns the number of logical processors available to this process.
    inline int LogicalCoreCount()
    {
        DWORD_PTR processMask = 0, systemMask = 0;
        if (!GetProcessAffinityMask(GetCurrentProcess(), &processMask, &systemMask))
            return 1;

        int count = 0;
        for (DWORD_PTR m = processMask; m; m >>= 1)
            count += static_cast<int>(m & 1);
        return (count > 0) ? count : 1;
    }

    // Computes a safe affinity mask: intersects desiredMask with the process
    // affinity mask.  If the intersection is empty, falls back to the full
    // process mask (so we never pin to an unavailable core).
    inline DWORD_PTR SafeAffinityMask(DWORD_PTR desiredMask)
    {
        DWORD_PTR processMask = 0, systemMask = 0;
        if (!GetProcessAffinityMask(GetCurrentProcess(), &processMask, &systemMask))
            return desiredMask;

        DWORD_PTR result = desiredMask & processMask;
        return (result != 0) ? result : processMask;
    }

    // Returns a single-bit affinity mask for logical core index 'core',
    // adjusted to the actual number of available cores.
    inline DWORD_PTR CoreMask(int core)
    {
        const int numCores = LogicalCoreCount();
        // If the requested core exceeds available cores, wrap to the last one.
        int safeCore = (core < numCores) ? core : (numCores - 1);
        return SafeAffinityMask(static_cast<DWORD_PTR>(1) << safeCore);
    }

    // Pins the calling thread to the logical core(s) described by coreMask.
    // Returns true on success.
    inline bool PinCurrentThread(DWORD_PTR coreMask)
    {
        return SetThreadAffinityMask(GetCurrentThread(), coreMask) != 0;
    }

    // Convenience: pin the calling thread to a single logical core index.
    // Degrades gracefully if 'core' >= number of available cores.
    inline bool PinCurrentThreadToCore(int core)
    {
        return PinCurrentThread(CoreMask(core));
    }

    // Sets the calling thread's name (visible in debugger and ETW traces).
    // Uses SetThreadDescription (Windows 10+); silently no-ops on older OS.
    inline void SetCurrentThreadName(const char* name)
    {
        HMODULE kernel = GetModuleHandleA("kernel32.dll");
        if (!kernel) return;

        using Fn = HRESULT(WINAPI*)(HANDLE, PCWSTR);
        auto* fn = reinterpret_cast<Fn>(
            reinterpret_cast<void*>(GetProcAddress(kernel, "SetThreadDescription")));
        if (!fn) return;

        wchar_t wname[128] = {};
        MultiByteToWideChar(CP_UTF8, 0, name, -1, wname, 128);
        fn(GetCurrentThread(), wname);
    }
} // namespace ThreadUtils
