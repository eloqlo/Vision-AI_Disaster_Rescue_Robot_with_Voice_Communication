#include <iostream>
#include <stdlib.h> 
#include <unistd.h>
#include <cstdlib>

using namespace std;

class accelerator;
const int dummy = 0;

class engine {
    private:
    virtual void acceleration_output() = 0;
    virtual void reduce_output() = 0;
    friend class accelerator;
};

class ic_engine : public engine {
    private:
    virtual void acceleration_output() = 0;
    virtual void reduce_output() = 0;
};