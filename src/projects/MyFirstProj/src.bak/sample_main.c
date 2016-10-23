#include <stdlib.h>
#include "../include/for_your_health.h"

int main(int argc, char** argv) {
  char* msg = malloc(10*sizeof(char));
  int i = 0;
  
  msg[0]='c';

  free(msg);
  
  return;
}

int trunk(int truck){
  if (truck == 3) return truck;
  else return 3;
}
