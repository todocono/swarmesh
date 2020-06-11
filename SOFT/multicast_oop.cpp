#include <ArduinoJson.h>

class Destinations
{
private:
    int **_dst_lst;
    int *_dst;
    int *_init_pos;
    int _lst_size;
    void _dst_lst_del(int idx);

public:
    Destinations();
    void dst_loader(DynamicJsonDocument &jTask, int ORI, int *POS) void dst_selector(int ORI, int *POS);
    void dst_proceed(DynamicJsonDocument &jDst);
    bool dst_arrive(int *POS);
};

Destinations::Destinations()
{
    int *_dst = (int *)malloc(sizeof(int) * 2);
    int *_init_pos = (int *)malloc(sizeof(int) * 2);
    _dst = {-1};
    _init_pos = {-1};
}

void Destinations::_dst_lst_del(int idx)
{
    _dst[0] = _dst_lst[idx][0];
    _dst[1] = _dst_lst[idx][1];
    // swap the value at index with the last element
    _dst_lst[idx][0] = _dst_lst[_lst_size - 1][0];
    _dst_lst[idx][1] = _dst_lst[_lst_size - 1][1];
    // modify _dst_lst and get rid of the smallest element
    _lst_size--;
    int **new_ptr = (int **)malloc(sizeof(int *) * (_lst_size));
    for (int i = 0; i < _lst_size; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            new_ptr[i][j] = _dst_lst[i][j];
            free(_dst_lst[i]);
        }
    }
    free(_dst_lst);
    _dst_lst = new_ptr;
}

void Destinations::dst_loader(DynamicJsonDocument &jTask, int ORI, int *POS)
{
    _init_pos[0] = ORI[0];
    _init_pos[1] = ORI[1];
    int _lst_size = jTask["Num"];
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
    dst_selector(ORI, POS);
}

void Destinations::dst_selector(int ORI, int *POS)
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
    _dst_lst_del(idx);
}

void Destinations::dst_proceed(DynamicJsonDocument &jDst, int ORI, int *POS);
{
    if (_lst_size > 0)
    {
        // proceed to the next least distance point
        int *dst = (int *)malloc(sizeof(int) * 2);
        dst[0] = jDst["Pos"][0];
        dst[1] = jDst["Pos"][1];
        if (dst[0] == _dst[0] && dst[1] == _dst[1])
        {
            dst_selector(ORI, POS);
        }
        else
        {
            // get the id of the element
            int idx;
            for (int i = 0; i < _lst_size; i++)
            {
                if (_dst_lst[i][0] == dst[0] && _dst_lst[i][1] == dst[1])
                {
                    idx = i;
                }
            }
            _dst_lst_del(idx);
        }
    }
    else
    {
        // run out of available Destinations
        // go back to original point
        _dst = _init_pos;
    }
}

bool Destinations::dst_arrive(int *POS, Async udp)
{
    if (POS[0] == _dst[0] && POS[1] == _dst[1])
    {
        // send out udp indicating itself has arrived
        StaticJsonDocument<200> doc;
        doc["Purpose"] = "Arrival";
        doc["Pos"] = [ POS[0], POS[1] ];
        const char *root;
        serializeJson(doc, root);
        udp.writeTo((const uint8_t *)root, strlen(root), IPAddress(224, 3, 29, 1), 10001);
        return true;
    }
    return false;
}