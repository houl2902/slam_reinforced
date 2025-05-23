#include "Logger.hpp"


Logger::Logger(std::initializer_list<std::string> args){
    for (auto& arg : args){
        logsFile.open(arg);
        logsFile.close();
    }   
}
void Logger::writeMatrixCoords(double* float_array, std::string file_name){
    auto time_diff = (std::chrono::steady_clock::now() - prev_time).count() / 100000.0 ;
    std::cout<< "TIME HERE" << std::endl;
    std::cout<< time_diff << std::endl;
    std::cout<< "!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    logsFile.open(file_name,std::ios::app);
    double vector = sqrt(pow(float_array[0],2)+pow(float_array[1],2));
    std::string vector_logs = std::to_string(vector) + " " + std::to_string(float_array[2]);
    logsFile <<  vector_logs <<std::endl;
    logsFile.close();
    prev_time = std::chrono::steady_clock::now();
};
void Logger::writeFloat(double value, std::string file_name){
    logsFile.open(file_name,std::ios::app);
    std::string float_log = std::to_string(value);
    logsFile <<  float_log <<std::endl;
    logsFile.close();
};
