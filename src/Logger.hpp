#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <bits/stdc++.h>
#include <initializer_list>
#include <chrono>

class Logger
{
    public:
    std::ofstream logsFile;
    std::chrono::steady_clock::time_point prev_time;
    Logger(std::initializer_list<std::string> args);
    void writeMatrixCoords(double* double_array,std::string file_name);
    void writeFloat(double value, std::string file_name);
};