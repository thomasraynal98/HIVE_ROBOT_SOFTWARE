#include "00_function.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

//===================================================================
// DESCISION : Le thread qui va ecouter et noter tout les events internes.
//===================================================================

//===================================================================
// DEBUG : Affichage des informations pour un opérateur console.
//===================================================================

int main(int argc, char *argv[])
{
    int opt_reset = 0;
    if(argc == 2) opt_reset = std::atoi(argv[1]);

    if(opt_reset == 0)
    {
        init_redis_var(&redis);
    }
}