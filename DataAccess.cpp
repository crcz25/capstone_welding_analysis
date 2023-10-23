#include <iostream>
#include <iomanip>
#include <sstream>
#include <conio.h>
#include <float.h>

// Include iCon classes etc.
#include "iconbuffer.h"
#include "ThreadSafePrint.h"

using namespace std;


// This function assumes that you don't know what type of traits sub-class to expect, hence the if cases.
// The function only prints a small selection of all available traits. See reference manual for a complete set of traits.
int printTraits(const icon::Component* component)
{
    // Easiest way to get a traits object of the correct sub-class is to try to cast it and see if it worked.
    // If dynamic_cast fails it returns a null pointer (that evaluates to false in an if statement).
    const icon::ImageTraits* imageTraits = dynamic_cast<const icon::ImageTraits*>(component->getTraits());
    const icon::SensorRangeTraits* sensorRangeTraits = dynamic_cast<const icon::SensorRangeTraits*>(component->getTraits());
    const icon::WorldRangeTraits* worldRangeTraits = dynamic_cast<const icon::WorldRangeTraits*>(component->getTraits());

    safeCout("\t" << left << component->getTraits()->getTraitsType().c_str() << " (selection):");
    // If the traits are Image Traits the dynamic_cast succeeded and imageTraits is not null (=> true).
    if (imageTraits) {
        safeCout("\t\t" << setw(10) << left << "SizeX: " << imageTraits->getSizeX());
        safeCout("\t\t" << setw(10) << left << "SizeY: " << imageTraits->getSizeY());
    }
    // If the traits are Sensor Range Traits the dynamic_cast succeeded and imageTraits is not null (=> true).
    else if (sensorRangeTraits) {
        safeCout("\t\t" << setw(10) << left << "OriginX: " << sensorRangeTraits->getOriginX() << " " << sensorRangeTraits->getCoordinates().c_str());
        safeCout("\t\t" << setw(10) << left << "OriginZ: " << sensorRangeTraits->getOriginZ() << " " << sensorRangeTraits->getCoordinates().c_str());
    }
    // If the traits are World Range Traits the dynamic_cast succeeded and imageTraits is not null (=> true).
    else if (worldRangeTraits) {
        safeCout("\t\t" << setw(15) << left << "LowerBoundX: " << worldRangeTraits->getLowerBoundX() << " " << worldRangeTraits->getCoordinateUnit().c_str());
        safeCout("\t\t" << setw(15) << left << "LowerBoundR: " << worldRangeTraits->getLowerBoundR() << " " << worldRangeTraits->getCoordinateUnit().c_str());
    }
    // Even if you know the expected traits type it is good practice to always perform a null-check.
    else {
        safeCout("\t\tUnknown traits type.");
        return -1;
    }

    return 0;
}


// The easiest way to get a printout of the complete data format is of course to use:
// cout << dataFormat->toString().c_str();
// ...but we extract some of the information manually here to illustrate the usage of the methods:
int printDataFormat(icon::IconBuffer* buffer)
{
    // Get handle to the buffers data format.
    const icon::DataFormat* dataFormat = buffer->getDataFormat();
    safeCout("Data format: " << dataFormat->getName().c_str());

    // Loop through all components in the buffer
    for (unsigned int compIndex = 0; compIndex < dataFormat->numComponents(); compIndex++)
    {
        // Get handle to component.
        const icon::Component* component = dataFormat->getComponent(compIndex);
        safeCout("Component " << compIndex << ": " << component->getName().c_str());
        printTraits(component);

        // Loop through all subcomponents of the component.
        for (unsigned int subCompIndex = 0; subCompIndex < component->numSubComponents(); subCompIndex++)
        {
            const icon::SubComponent* subComponent = component->getSubComponent(subCompIndex);
            safeCout("\tSubComponent " << subCompIndex << ": " << setw(20) << left << subComponent->getName().c_str() <<
                "Type: " << subComponent->getValueType().c_str());
        }
        safeCout(" ");
    }

    return 0;
}

// Simple example of accessing data from a known subcomponent.
// Returns -1 if subcomponent 'Hi3D 1 Range' does not exist.
int accessHi3DRange(icon::IconBuffer* buffer)
{
    double mean = 0;
    unsigned int sum = 0;
    unsigned int count = 0;
    unsigned int numberOfScans = buffer->getHeight();
    const unsigned short* data;        // Unsigned short since "Hi3D 1" "Range" contains 16 bit integer data.

    // A simple way to check that 'Hi3D 1 Range' exist is to try to access the data and see if it works.
    if (buffer->getReadPointer("Hi3D 1", "Range", 0, data) != icon::E_ALL_OK) {
        safeCout("Could not find 'Hi3D 1 Range' subcomponent in buffer.");
        return -1;
    }

    // Loop through all scans in the buffer.
    unsigned int subCompWidth = buffer->getDataFormat()->getNamedComponent("Hi3D 1")->getNamedSubComponent("Range")->getWidth();
    for (unsigned int scan = 0; scan < numberOfScans; scan++)
    {
        // Get pointer to beginning of scan number 'scan' of subComponent 'Hi3D 1 Range'.
        buffer->getReadPointer("Hi3D 1", "Range", scan, data);

        // Loop through all elements in each scan.
        for (unsigned int col = 0; col < subCompWidth; col++)
        {
            unsigned short val = *(data + col);
            if (val != 0)
            {
                // Do something with the data...
                sum += val;
                count++;
            }
        }
    }
    // Calculate the final mean value.
    mean = double(sum) / double(count);
    safeCout("Mean Hi3D 1 Range: " << mean);

    return 0;
}


// Accessing generic data using correct data type for each subcomponent
int accessData(icon::IconBuffer* buffer)
{
    unsigned int numberOfScans = buffer->getHeight();

    // Loop through all components in the buffer
    const icon::DataFormat* dataFormat = buffer->getDataFormat();
    if (dataFormat->getName().compare("IMAGE") == 0) {
        safeCout("This is an image buffer.");
        return 0;
    }

    for (unsigned int compIndex = 0; compIndex < dataFormat->numComponents(); compIndex++)
    {
        // Get handle to component.
        const icon::Component* component = dataFormat->getComponent(compIndex);

        // Loop through all subcomponents of the component.
        for (unsigned int subCompIndex = 0; subCompIndex < component->numSubComponents(); subCompIndex++)
        {
            const icon::SubComponent* subComponent = component->getSubComponent(subCompIndex);

            // Handle Mark subcomponents separately and only print mark value from the first line scan.
            // See the reference manual for a detailed description of the mark data format.
            if (subComponent->getName().compare("Mark") == 0)
            {
                const int* markData;
                buffer->getReadPointer(component->getName().c_str(), subComponent->getName().c_str(), 0, markData);
                int counter = *markData;
                unsigned int overtrig = ((*(markData + 1)) >> 16) & 0xff;
                unsigned int enable = ((*(markData + 1)) >> 30) & 0x1;
                safeCout(setw(16) << left << component->getName().c_str() << setw(16) << left << subComponent->getName().c_str() << "Counter: " << counter << "\tOvertrigs: " << overtrig << "\tEncoder enable: " << enable);
            }

            else {  // Calculate and print mean value for non-mark subcomponents.
                double sum = 0;
                unsigned int count = 0;
                unsigned int subCompWidth = subComponent->getWidth();

                // Since the data type is unknown in the general case it must first be determined so that the 
                // data pointer and value types can be declared correctly.
                switch (subComponent->getValueTypeEnum())
                {
                    // Subcomponent data are 8 bit unsigned integers.
                case icon::IVT_BYTE:
                    const unsigned char* byteData;
                    unsigned char byteValue;
                    buffer->getReadPointer(component->getName().c_str(), subComponent->getName().c_str(), 0, byteData);
                    for (unsigned int dataIndex = 0; dataIndex < (subCompWidth * numberOfScans); dataIndex++)
                    {
                        byteValue = *(byteData + dataIndex);
                        if (byteValue != 0)    // Missing data is represented by 0
                        {
                            sum += byteValue;
                            count++;
                        }
                    }
                    break;

                    // Subcomponent data are 16 bit unsigned integers.
                case icon::IVT_WORD:
                    const unsigned short* wordData;
                    unsigned short wordValue;
                    buffer->getReadPointer(component->getName().c_str(), subComponent->getName().c_str(), 0, wordData);
                    for (unsigned int dataIndex = 0; dataIndex < (subCompWidth * numberOfScans); dataIndex++)
                    {
                        wordValue = *(wordData + dataIndex);
                        if (wordValue != 0)    // Missing data is represented by 0
                        {
                            sum += wordValue;
                            count++;
                        }
                    }
                    break;

                    // Subcomponent data are 32 bit unsigned integers.
                case icon::IVT_DWORD:
                    const int* dwordData;
                    int dwordValue;
                    buffer->getReadPointer(component->getName().c_str(), subComponent->getName().c_str(), 0, dwordData);
                    for (unsigned int dataIndex = 0; dataIndex < (subCompWidth * numberOfScans); dataIndex++)
                    {
                        dwordValue = *(dwordData + dataIndex);
                        if (dwordValue != 0)    // Missing data is represented by 0
                        {
                            sum += dwordValue;
                            count++;
                        }
                    }
                    break;

                    // Subcomponent data are 32 bit signed integers.
                case icon::IVT_INT:
                    const int* intData;
                    int intValue;
                    buffer->getReadPointer(component->getName().c_str(), subComponent->getName().c_str(), 0, intData);
                    for (unsigned int dataIndex = 0; dataIndex < (subCompWidth * numberOfScans); dataIndex++)
                    {
                        intValue = *(intData + dataIndex);
                        if (intValue != 0)    // Missing data is represented by 0
                        {
                            sum += intValue;
                            count++;
                        }
                    }
                    break;

                    // Subcomponent data are 32 bit floating point numbers.
                case icon::IVT_FLOAT:
                    const float* floatData;
                    float floatValue;
                    buffer->getReadPointer(component->getName().c_str(), subComponent->getName().c_str(), 0, floatData);
                    for (unsigned int dataIndex = 0; dataIndex < (subCompWidth * numberOfScans); dataIndex++)
                    {
                        floatValue = *(floatData + dataIndex);
                        if (floatValue != 0)    // Missing data is represented by 0. Note that for ScanningRuler missing data is represented by NaN.
                        {
                            sum += floatValue;
                            count++;
                        }
                    }
                    break;

                    // Subcomponent data are 32 bit clusters of 8 bit color data organized like [B G R 0].
                case icon::IVT_RGB2:
                    const unsigned long* rgb2Data = NULL;
                    int rgb2Value;
                    buffer->getReadPointer(component->getName().c_str(), subComponent->getName().c_str(), 0, rgb2Data);
                    for (unsigned int dataIndex = 0; dataIndex < (subCompWidth * numberOfScans); dataIndex++)
                    {
                        rgb2Value = *(rgb2Data + dataIndex);
                        if (rgb2Value != 0)    // Missing data is represented by 0
                        {
                            unsigned int blue = (rgb2Value >> 24) & 0xff;
                            unsigned int green = (rgb2Value >> 16) & 0xff;
                            unsigned int red = (rgb2Value >> 8) & 0xff;
                            sum += (blue + green + red);
                            count = count + 3;
                        }
                    }
                    break;
                }

                double mean = double(sum) / double(count);
                safeCout(setw(16) << left << component->getName().c_str() << setw(16) << left << subComponent->getName().c_str() << "Mean: " << mean);
            }
        }
    }

    return 0;
}