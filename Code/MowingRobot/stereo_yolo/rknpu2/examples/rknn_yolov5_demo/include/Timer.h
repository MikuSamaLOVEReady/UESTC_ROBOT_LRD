/**
 * @file Timer.h
 * @author hcy
 * @brief 用于计时
 * @version 1
 * @date 2023-04-10
 *
 */
 
 
#include <iostream>
#include <chrono>
 
class Timer
{
public:
    Timer()
    {
        start = std::chrono::high_resolution_clock::now();
    };
    ~Timer()
    {
        end = std::chrono::high_resolution_clock::now();
        duration = end - start;
        double ms = duration.count() * 1000.f;
        std::cout << "Timer took " << ms << "ms" << std::endl;
    };
 
private:
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end;
    std::chrono::duration<double> duration;
};