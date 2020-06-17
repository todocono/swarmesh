/*
Destinations.h - Library for Swarmesh Created 6/17/20
*/
#ifndef Destinations_h
#define Destinations_h
#include "Arduino.h"
#include <ArduinoJson.h>

class Destinations
{
  private:
    int **_dst_lst;
    int *_dst;
    int *_init_pos;
    int _lst_size;
    void _del_dst_lst(int idx);

  public:
    Destinations();
    void dst_init();
    void load_dst(DynamicJsonDocument &jTask, int *POS);
    void select_dst(int *POS);
    void proceed_dst(DynamicJsonDocument &jDst, int *POS);
    int* get_dst();
    int arrive_dst(int *POS);
};

#endif