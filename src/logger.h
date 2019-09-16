#pragma once
#include <sstream>
#include <iomanip>

#include "math.h"

#include "pcserial.h"

class Logger
{
public:
	Logger(std::string className) : pcSerial(PCSerial(115200)) {
        std::string nameTemp = "[" + className + "] ";
        for (int i = strlen(nameTemp.c_str()); i < 16; i++) {
            nameTemp = nameTemp + " ";
        }
        this->className = nameTemp;
    }

    Logger& info() {
        logLevel = "INFO ";
        return *this;
    }

    Logger& warn() {
        logLevel = "WARN ";
        return *this;
    }

    Logger& error() {
        logLevel = "ERROR";
        return *this;
    }

    Logger& operator<<(const char* const & value)
    {
        if (strcmp(value, "\n") != 0) {
            logStream << value;
        } else {
            print(logStream.str());
            logStream = std::stringstream();
        }
        return *this;
    }
    Logger& operator<<(float const & value)
    {
        logStream << ftoa(value, 2);
        return *this;
    }
    Logger& operator<<(double const & value)
    {
        logStream << ftoa(value, 2);
        return *this;
    }

    template <typename T>
    Logger& operator<<(T const & value)
    {
        logStream << value;
        return *this;
    }

    const char* flush = "\n"; 
private:
    PCSerial pcSerial; 
    std::string className;
    std::string logLevel;
    std::stringstream logStream;
	void print(std::string msg) {
        std::stringstream ss;
        int time = HAL_GetTick();
        int milli = time%1000;
        int sec = (time%60000)/1000;
        int min = time/60000;
        ss << "(" << std::setw(3) << std::setfill('0') << min 
           << ":" << std::setw(2) << std::setfill('0') << sec 
           << "." << std::setw(3) << std::setfill('0') << milli 
           << ") " << className << logLevel << ": " << msg;
        pcSerial.println(ss.str());
    }
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

