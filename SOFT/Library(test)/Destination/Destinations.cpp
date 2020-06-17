#include "Arduino.h"
#include "Destinations.h"

Destinations::Destinations()
{
}

void Destinations::dst_init() {
  _dst = (int *)malloc(sizeof(int) * 2);
  _init_pos = (int *)malloc(sizeof(int) * 2);
  for (int i = 0; i < 2; i ++) _dst[i] = -1;
}

void Destinations::_del_dst_lst(int idx)
{
  // swap the value at index with the last element
  _dst_lst[idx][0] = _dst_lst[_lst_size - 1][0];
  _dst_lst[idx][1] = _dst_lst[_lst_size - 1][1];
  // modify _dst_lst and get rid of the smallest element
  _lst_size--;
  Serial.println(_lst_size);
  int **new_ptr = (int **)malloc(sizeof(int *) * (_lst_size));
  for (int i = 0; i < _lst_size; i++)
  {
    new_ptr[i] = (int *)malloc(sizeof(int) * 2);
    for (int j = 0; j < 2; j++)
    {
      new_ptr[i][j] = _dst_lst[i][j];
    }
    free(_dst_lst[i]);
  }
  free(_dst_lst);
  _dst_lst = new_ptr;
}

void Destinations::load_dst(DynamicJsonDocument &jTask, int *POS)
{
  for (int i = 0; i < 2; i ++) _init_pos[i] = POS[i];
  _lst_size = jTask["Num"];
  Serial.print("Task size: ");
  Serial.println(_lst_size);
  _dst_lst = (int **)malloc(sizeof(int *) * _lst_size);
  // load json destination coordinates into the two-dimensional array
  for (int i = 0; i < _lst_size; i++)
  {
    _dst_lst[i] = (int *)malloc(sizeof(int) * 2);
    for (int j = 0; j < 2; j++)
    {
      _dst_lst[i][j] = jTask["Task"][i][j];
    }
  }
  // automatically acquire the next destination by calling the function
  select_dst(POS);
}

void Destinations::select_dst(int *POS)
{
  // select the destination with the least manhattan distance
  int x = _dst_lst[0][0];
  int y = _dst_lst[0][1];
  int idx = 0;
  int dist = (abs(x - POS[0]) + abs(y - POS[1]));
  for (int i = 0; i < _lst_size; i++)
  {
    x = _dst_lst[i][0];
    y = _dst_lst[i][1];
    int new_dist = (abs(x - POS[0]) + abs(y - POS[1]));
    if (new_dist < dist)
    {
      dist = new_dist;
      idx = i;
    }
  }
  for (int i = 0; i < 2; i ++) _dst[i] = _dst_lst[idx][i];
  _del_dst_lst(idx);
}

void Destinations::proceed_dst(DynamicJsonDocument &jDst, int *POS)
{
  int *dst = (int *)malloc(sizeof(int) * 2);
  dst[0] = jDst["Pos"][0];
  dst[1] = jDst["Pos"][1];
  Serial.println(_lst_size);
  if (_lst_size > 0)
  {
    // proceed to the next least distance point
    if (dst[0] == _dst[0] && dst[1] == _dst[1])
    {
      select_dst(POS);
    }
    else
    {
      // get the id of the element
      int idx = -1;
      for (int i = 0; i < _lst_size; i++)
      {
        if (_dst_lst[i][0] == dst[0] && _dst_lst[i][1] == dst[1]) idx = i;
        Serial.println("matched");
      }
      //      filter out destination of robots going back to their origin
      if (idx >= 0) _del_dst_lst(idx);
    }
  }
  else
  {
    // run out of available Destinations
    // go back to original point
    //    only doing so when the last destination conflicts the destination taken by others
    if (dst[0] == _dst[0] && dst[1] == _dst[1]) {
      for (int i = 0; i < 2; i ++) _dst[i] = _init_pos[i];
    }
  }
}

int* Destinations::get_dst() {
  int* new_ptr = (int*) malloc(sizeof(int) * 2);
  for (int i = 0; i < 2; i ++) new_ptr[i] = _dst[i];
  return new_ptr;
}

int Destinations::arrive_dst(int *POS)
{
  if (POS[0] == _dst[0] && POS[1] == _dst[1]) return 1;
  return 0;
}