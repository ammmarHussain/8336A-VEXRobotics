// prevent nesting
#pragma once

// include everything needed
#include "v5.h"
#include "v5_vcs.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// predefined vex function to wait
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

// predefined vex function to repeat
#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
