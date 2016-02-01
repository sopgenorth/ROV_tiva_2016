#include "rovTimer.h"

struct cyclicFunctions_t{
  funcCallback f;
  unsigned long interval_ms;
};

cyclicFunctions_t funcs[MAX_FUNCTIONS];
int numFunctions = 0;

bool rovTimerLoad(funcCallback func, unsigned long interval_ms){
  //verify func isn't null and there is room for another function
  if(func && (numFunctions < MAX_FUNCTIONS-1)){
    funcs[numFunctions].f = func;
    funcs[numFunctions].interval_ms = interval_ms;
    numFunctions++;
  }
}

void rovTimerRun(){
  unsigned long currentMillis = millis();
  static unsigned long previousMillis;
  for(int i = 0; i <= numFunctions; i++){
    if(currentMillis - previousMillis > 50) {
      previousMillis = currentMillis;
    }
  }
}



