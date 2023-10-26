/** dataformatexample.cpp

    Source file used by rangere_ex.cpp


    \par Copyright
    (c) 2007 SICK IVP AB

   $Revision: $
   $LastChangedDate: $
   $Author: $
   $HeadURL:  $
*/
// Include the icon api
#include "icon_api.h"
#include "dataformat.h"

// Include the EthernetCamera camera header
#include "ethernetcamera.h"
#include <iostream>
#include <sstream>

using namespace icon;
using namespace std;

void showDataFormat(Camera *cam0)
{
	icon::String format;
	int err = cam0->getDataFormat("", format);
	if(err != 0)
	{
		cout << "ERROR getting Data Format";
		return;
	}

	DataFormat *df;

	//  Have we received any data from camera?
	df = new DataFormat();
    err = df->init(format.c_str());
    if(err != DataFormat::E_ALL_OK)
	{
        cout << "Init data format failed with error code: " << err << endl;
        return;
    }

	ostringstream str;

	str << endl << "Data format: " << endl << endl;

	str << "Valutetype: " << df->getValueType().c_str() << endl;
	str << "Size: " << df->dataSize() << " bytes." << endl;
	str << "Number of components: " << df->numComponents() << endl << endl;

	int coffset = 0;
	for(unsigned int i=0; i < df->numComponents(); i++)
	{
		const Component *comp = df->getComponent(i);
		str << "Component name: " << comp->getName().c_str() << endl;
		str << "   Offset: " << coffset << " bytes." << endl;
		str << "   Value type: " << comp->getValueType().c_str() << "."<< endl;
		str << "   Height:" << comp->getHeight() << " rows" << endl;
		str << "   Number of subcomponents: " << comp->numSubComponents() << "." << endl;
		int soffset = 0;
		for(unsigned int j=0; j < comp->numSubComponents(); j++)
		{
			const SubComponent *subcomp = comp->getSubComponent(j);
			str << "   Subcomponent name: " << subcomp->getName().c_str() << "." << endl;
			str << "      Offset: " << coffset + soffset << " bytes." << endl;
			str << "      Value type: " << subcomp->getValueType().c_str() << "." << endl;
			str << "      Size: " << subcomp->getSize() << " bytes."<< endl;
			str << "      Width: " << subcomp->getWidth() << " " << subcomp->getValueType().c_str() << "S." << endl;

			soffset += subcomp->getSize();
		}
		coffset += comp->getSize();
		str << endl;
	}

	cout << str.str();
};