#include <iostream>
#include <string>
#include <unistd.h>

using namespace std;

int main(int argc, char ** argv)
{
    string s = "px7_54";
    
    string s_intera = s.substr(2,s.find_first_of("_")-2);
    string s_decimale = s.substr(s.find_first_of("_")+1);
    double intera =  stoi(s_intera) * 1.0;
    double decimale = stoi(s_decimale)  * 1.0;
    for(int i=0; i<s_decimale.length(); i++)
        decimale *= 0.1;
    double numero = intera + decimale; 
    cout << numero << endl << endl;


    double x = 7.5444;
    int int_x = (int)x;
    double d_dec_x = x - int_x;
    string s_dec_x = to_string(d_dec_x);
    s_dec_x = s_dec_x.substr(s_dec_x.find_first_of(".")+1, 2);
    int dec_x = (int)d_dec_x;
    cout << "px" << int_x << "_" <<  s_dec_x << endl; 
    return 0;
}