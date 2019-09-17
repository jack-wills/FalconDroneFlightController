#pragma once
#include <sstream>
#include <iomanip>

#include "math.h"

#include "pcserial.h"

class FCComm
{
public:
	FCComm(std::string identifier) : pcSerial(PCSerial(115200)), identifier(identifier) {
        printStream << identifier << " ";
    }

    ~FCComm() {
        pcSerial.println(printStream.str());
    }
    
    FCComm& operator<<(float const & value)
    {
        printStream << ftoa(value, 2);
        return *this;
    }
    FCComm& operator<<(double const & value)
    {
        printStream << ftoa(value, 2);
        return *this;
    }

    template <typename T>
    FCComm& operator<<(T const & value)
    {
        printStream << value;
        return *this;
    }
private:
    PCSerial pcSerial; 
    std::string identifier;
    std::stringstream printStream;
    // Converts a floating point number to string. 
    std::string ftoa(float n, int afterpoint) { 
        char res[30];
        bool negative = false;

        if (n < 0.0f) {
            n *= -1;
            negative = true;
        }

        // Extract integer part 
        int ipart = (int)n; 
    
        // Extract floating part 
        float fpart = n - (float)ipart; 
    
        // convert integer part to string 
        int i = intToStr(ipart, res, 1); 
    
        // check for display option after point 
        if (afterpoint != 0) { 
            res[i] = '.';  // add dot 
    
            // Get the value of fraction part upto given no. 
            // of points after dot. The third parameter is needed 
            // to handle cases like 233.007 
            fpart = fpart * pow(10, afterpoint); 
    
            intToStr((int)fpart, res + i + 1, afterpoint); 
        } 
        if (negative) {
            return "-" + std::string(res);
        }
        return std::string(res);
    } 
    int intToStr(int x, char str[], int d) { 
        int i = 0; 
        while (x) { 
            str[i++] = (x%10) + '0'; 
            x = x/10; 
        } 
    
        // If number of digits required is more, then 
        // add 0s at the beginning 
        while (i < d) 
            str[i++] = '0'; 
    
        reverse(str, i); 
        str[i] = '\0'; 
        return i; 
    } 
    void reverse(char *str, int len) { 
        int i=0, j=len-1, temp; 
        while (i<j) 
        { 
            temp = str[i]; 
            str[i] = str[j]; 
            str[j] = temp; 
            i++; j--; 
        } 
    } 
};

