#include <cstdio>

#include "src/interp/interp.h"
#include "trap-with.h"

extern "C" void trapWith(int32_t r) {
  using namespace wabt::interp;
  Result result = static_cast<Result>(r);
  
  switch(result) {
    case Result::TrapIntegerDivideByZero :
      printf("trap: division by zero\n");
      break;
    default:
      break;
  }
}
