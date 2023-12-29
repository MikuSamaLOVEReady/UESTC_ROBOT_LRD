#include <opencv2/opencv.hpp>
#include <iostream>
#include "ranging.h"


int main(int argc, char** argv)
{
    Ranging r(0);
    r.get_range();
    
    return 1;
}
 