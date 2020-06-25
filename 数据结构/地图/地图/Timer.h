#ifndef _TIMER_H_
#define _TIMER_H_

#include <windows.h>

class Timer {
private:
  __int64 freq, tStart, tStop;

public:
  Timer(){
    // Get the frequency of the hi-res timer
    QueryPerformanceFrequency((LARGE_INTEGER*)&freq);
  } //end-TimerClass

  void Start(){
    // Use hi-res timer
    QueryPerformanceCounter((LARGE_INTEGER*)&tStart);
  } //end-Start

  void Stop(){
    // Perform operations that require timing
    QueryPerformanceCounter((LARGE_INTEGER*)&tStop);
  } //end-Stop

  // Returns time in milliseconds
  double ElapsedTime(){
    // Calculate time difference in milliseconds
    return ((double)(tStop - tStart)/(double)freq)*1e3;
  } //end-Elapsed
};

#endif

/*包含文件：
#include "Timer.h"

调用代码：

  Timer timer;
  timer.Start();

  int noLines;
  LS *lines = 你的函数调用(srcImg, width, height, &noLines);

  timer.Stop();

  printf("Elapsed time is: <%4.2lf> ms\n", timer.ElapsedTime());
 */