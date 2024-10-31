//
// Created by iahve on 16.07.24.
//

#ifndef BAMS_DRIVER_HPP
#define BAMS_DRIVER_HPP

#include <iostream>
#include <cstring>
using namespace std;

class driver
{
public:
    const char name[40];
    explicit driver(const char *_name): name(){
        size_t length = strnlen(_name, 39); // Определяем длину строки, но не более 39 символов
        strncpy(const_cast<char*>(name), _name, length);
        const_cast<char*>(name)[length] = '\0'; // Убедитесь, что строка завершается нулевым символом
    };
    virtual void dump()
    {

    }


protected:

};
#endif //BAMS_DRIVER_HPP
