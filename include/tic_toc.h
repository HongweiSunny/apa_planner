// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>
#include <string>
namespace RL_PLANNER_NS
{
  

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

    void cout_time_used(std::string op)
    {
      std::cout << "spend_time in  " << op << ": " << toc()<< std::endl;
      }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
}; // end of class
}; // namespace RL_PLANNER_NS
