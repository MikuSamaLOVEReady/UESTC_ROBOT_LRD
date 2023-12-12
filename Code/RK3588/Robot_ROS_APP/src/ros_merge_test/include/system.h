#ifndef SYSTEM_H
#define SYSTEM_H

#include <thread>
#include <string>
#include "mapping.h"
#include "uwb.h"
#include "senddata.h"
#include "align.h"
#include <iostream>

namespace uwb_slam{
    class System{

    public:
        System() {
        }
        void Run();
    public:
       
        std::shared_ptr<uwb_slam::Mapping> Mapping_;
        std::shared_ptr<uwb_slam::Uwb> Uwb_;
        std::shared_ptr<uwb_slam::Senddata> Sender_;
        std::shared_ptr<uwb_slam::Align> Align_;

        // Uwb* Uwb_ ;
        // Senddata* Sender_;
        // Mapping* Mapping_;
    };
}
#endif
