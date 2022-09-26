#include <iostream>
#include "head/matcher.h"
#include "head/filter.h"
#include "head/checker.h"

using namespace std;

int main()
{
    Filter filter;
    filter.Filtrate("../track", "../process");
    cout << Filter::max_time_ << endl;
    cout << Filter::min_time_ << endl;
    Matcher matcher("../road/OpenEngine-roadnet-shenzhen.txt");
    matcher.Match("../process", "../output");
    return 0;
}
