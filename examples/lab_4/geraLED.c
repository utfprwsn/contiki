#include "project-conf.h"
#include <random.h>

char gerarLed()
{
    random_init(clock_time());
    int l = 1 + (random_rand() % 4);
    return (l % 2 == 0 ? 'R' : 'G');
}
