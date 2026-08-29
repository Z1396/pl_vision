#pragma once
// spin：自旋等待的体系结构提示。
//
// 纯自旋（while (!ready);）会让 CPU 乱序引擎满流水线冲刷 + 功耗墙 + 超线程邻居被饿死：
//   x86   _mm_pause —— 提示 CPU"我在自旋"，降低分支预测压力与功耗（~几 ns 代价）；
//   ARM   yield     —— 同语义的 WFE 类提示；
//   其它  退化 std::this_thread::yield 让出时间片。
//
// 真实项目中位于 crates/primitive/src/spin.hpp。

#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#define SPIN_HINT() _mm_pause()
#elif defined(__aarch64__) || defined(__arm__)
#define SPIN_HINT() __asm__ __volatile__("yield")
#else
#include <thread>
#define SPIN_HINT() std::this_thread::yield()
#endif
