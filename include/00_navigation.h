#include <sw/redis++/redis++.h>

int auto_mode_available(sw::redis::Redis* redis);
int manual_mode_available(sw::redis::Redis* redis);
void map_manual_command(sw::redis::Redis* redis, double back_value, double front_value, double angle);