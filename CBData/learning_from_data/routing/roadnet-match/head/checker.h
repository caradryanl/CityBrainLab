#ifndef CHECKER_H
#define CHECKER_H

#include <fstream>

#include "utils.h"

class Checker
{
    int threshold_;
public:
    Checker(int threshold);
    void Check(std::string input_path, std::string output_path);
};

#endif // CHECKER_H
