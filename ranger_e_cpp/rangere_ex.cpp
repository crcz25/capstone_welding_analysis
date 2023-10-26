/** rangere_ex.cpp

    This example covers the basic functions like setting up the camera and framegrabber classes,
    start and stop the camera, load parameter file. The example ends up with a polling mode operation
    to aquire frame grabber data.

    
    \par Copyright
    (c) 2007 SICK IVP AB

   $Revision: $
   $LastChangedDate: $
   $Author: $
   $HeadURL:  $
*/
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

// Include the iCon API
#include "icon_api.h"

#include <windows.h>

// Include the EthernetCamera camera header
#include "ethernetcamera.h"

// Include file for iCon Framegrabber support.
#include "framegrabber.h"

// For keypress detection.
#include <conio.h>

#include <iostream>
#include <sstream>
#include <conio.h>
#include <memory>   // For use of auto_ptr

// Include iCon Express classes etc.
#include "icamerasystem.h"
#include "icamerasystemsubscriber.h"

// Example program includes
// #include "ErrorHandling.h"
#include "DataAccess.h"
#include "ThreadSafePrint.h"


using namespace std;
using namespace icon;

// In file dataformatexample.cpp
void showDataFormat(Camera *cam0);

int closeDown(int err, EthernetCamera* cam, FrameGrabber *grabber);

// Construct an error callback by inheriting the
// icon::ErrorHandler class and override onError
class MyErrHandler : public icon::ErrorHandler {
	void onError(int errorLevel, const icon::String& err) { cout << err.c_str() << endl; };
};


bool saveRaw(std::string &filename, const unsigned char *data, int length)
{
	ofstream stream;

	stream.open(filename.c_str(), ios_base::out | ios_base::trunc | ios_base::binary);

	for(int i=0; i < length; i++)
	{
		stream << data[i];
	}

	stream.close();

	return 0;
}

/// Sets the process priority to REALTIME and the thread priority to IDLE
/// This implies priority 24-8=16 in ProcExplorer
void SetPriority(void)
{
	HANDLE CurrentProcess;
	HANDLE CurrentThread;
	CurrentProcess = GetCurrentProcess();
	CurrentThread = GetCurrentThread();
	//	SetPriorityClass(CurrentProcess, IDLE_PRIORITY_CLASS);
	SetPriorityClass(CurrentProcess, HIGH_PRIORITY_CLASS);
	//	SetPriorityClass(CurrentProcess, REALTIME_PRIORITY_CLASS);
	//	SetThreadPriority(CurrentThread, THREAD_PRIORITY_IDLE);
}

int main(int argc, char* argv[])
{
	// Construct & bind the error handler
	MyErrHandler err;
	setErrorHandler(&err);

	SetPriority();

	// Set the com-parameters. This must be done before the init function is called. 
	// If init fails, call close, adjust the parameters and try again.
	string ip;
	cout << endl << "Enter IP address of camera to use (enter 'l' for 127.0.0.1, enter 'd' for 192.168.0.92): ";
	cin >> ip;

	// shortcut key for localhost.
	if(ip == "d")
		ip = "192.168.0.92";

	// shortcut key for localhost.
	if(ip == "l")
		ip = "127.0.0.1";

	// Set how often an image is saved.
	int modulo;
	cout << endl << "Enter the interval between each saved image: ";
	cin >> modulo;

	cin.ignore();

	//********** BEGIN CAMERA CONTROL INIT

	// Call camera factory in the icon api. Ask for a EthernetCamera camera
	Camera* cam_generic = createCamera("EthernetCamera", "MyCamera");

	// Check result from factory and cast it to a EthernetCamera camera
	if(cam_generic == 0 || cam_generic->getCameraType() != EthernetCamera::cameraType) {
		delete cam_generic;
		return -1;
	}

	// Cast to the correct subclass.
	EthernetCamera* cam = static_cast<EthernetCamera*>(cam_generic);

	if(cam == NULL)
	{	
		/// The cast failed.
		delete cam_generic;
		return -1;
	}

	FrameGrabber *grabber = createFrameGrabber("FGEthernetFast", "MyGrabber");

	FGEthernetFastParameters* prms = dynamic_cast<FGEthernetFastParameters*>(grabber->getParameters()); 

	// UDP Port of the command channel on the camera.
    cam->setComParameters(ip.c_str(), prms->getFrameGrabberPort(), prms->getRedundancyPort(), EthernetCamera::HIGH_PERFORMANCE_DATA_CHANNEL);

	cam->setProtocolTimeout(3000); // 3 s timeout

	// Init camera
	cout << "Connecting ... " << endl;
	int ret = cam->connect();
	if(ret != EthernetCamera::E_ALL_OK) {
		cout << "Connect failed." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}
	cout << "connect ok." << endl;

	// Check camera status
	int status;
	ret = cam->checkCamStatus(status);
	if(ret == EthernetCamera::E_ALL_OK) {
		cout << "Status was: " << status << endl;
	}

	// We first disable data from the camera (if the last application has not cleaned up properly)

	ret = cam->stop();
	if(ret == EthernetCamera::E_ALL_OK) {
		cout << "Stop: Status was: " << status << endl;
	}

	// Get the available configurations
	icon::StringVector configs;
	ret = cam->getAvailableConfigurations(configs);
	if(ret != EthernetCamera::E_ALL_OK) {
		cout << "Get available configurations failed." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	cout << endl << "Configurations:" << endl;
	for(size_t i = 0; i < configs.size(); i++) {
		cout << i << " -- " << configs[i].c_str()  << endl;
	}

	// Download the current parameter file

	ret = cam->fileSaveParameters("DOWNLOAD.PRM");
	if(ret != EthernetCamera::E_ALL_OK) {
		cout << "Download of parameter file failed." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}  

	// Upload a new parameter file
	if(argc > 1)
		ret = cam->fileLoadParameters(argv[1]);
	else
		ret = 0;

	if(ret != EthernetCamera::E_ALL_OK) {
		cout << "Upload of parameter file failed." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	int confindex = 0;
	// Select a configuration
	cout << endl << "Select config (0-" << ((unsigned int)configs.size() - 1) << "): ";
	cin >> confindex;
	cin.ignore();

	if(confindex < 0 || confindex >= (int)configs.size()) {
		cout << "Invalid configuration." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	// Set the configuration
	ret = cam->setActiveConfiguration(configs.at(confindex));
	if(ret != EthernetCamera::E_ALL_OK) {
        cout << "Set active configuration failed with error code: " << ret << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	//********** END CAMERA CONTROL INIT

	//********** BEGIN FRAME GRABBER INIT

	cout << "Initiating Frame Grabber." << endl;


	if(prms == NULL)
	{
		cout << "Incorrect parameter type returned from getParameters()" << endl;
	}

	// IP address of the camera 
	prms->setCameraIP(ip.c_str());

	int dataPort;
	if(cam->getCameraDataPort(dataPort))
	{
		cout << "Get camera data port failed." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	// UDP source port of the stream data on the camera.
	prms->setCameraPort(dataPort);

	// Framegrabber buffer size in Megabytes.
	prms->setBufferSize(50);
	// Number of scans per IconBuffer.
	if (configs.at(confindex) == "Image")
    {   // Use 1 scan for Image, i.e. each buffer contains 1 complete image
        prms->setNoScans(1);
    }
    else
    {   // Use 50 scans for Measurement, i.e. each buffer contains 50 profiles
        prms->setNoScans(50);
    }

    if(cam->setBufferHeight(prms->getNoScans()))
    {
        cout << "Set buffer height failed." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
    }

    // How often is recovery packets sent.

	icon::String type, ver, serial;

	if(cam->getCameraInfo(type,ver,serial))
	{
		cout << "Get camera info failed." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	std::string path = "<ROOT><CAMERA name='";
	path += type.c_str();
	path += "'><MODULE name='Ethernet'>";
	const std::string name = "redundancy frequency";
	icon::String paramValue2;

	if(cam->getParameterValue(path.c_str(), name.c_str(), paramValue2))
	{
		cout << "Get recovery interval failed." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	prms->setRecoveryInterval(atoi(paramValue2.c_str()));

	// Polling mode of Framegrabber. FGCALLBACK is the other choice.
	prms->setMode(icon::FrameGrabber::FGPOLLING);
	// Manual release will be used.
	prms->setRelease(icon::FrameGrabber::MANUAL_RELEASE);
	// Pointer to a user supplied argument to the callback, not used.
	prms->setUserData(NULL);

	// Display the data format

	showDataFormat(cam);


	// Get the data format from the camera.
	icon::String dataformat;
	ret = cam->getDataFormat("", dataformat);
	if(ret != EthernetCamera::E_ALL_OK) {
		cout << "Get data format failed." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

    unsigned long packetSize;
   	ret = cam->getPacketSize(packetSize);
	if(ret != EthernetCamera::E_ALL_OK) {
		cout << "Get packet size failed." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	// Set the data format to same as the camera.
	prms->setDataFormat(dataformat, packetSize);


	
	ret = grabber->connect();
	if(ret != EthernetCamera::E_ALL_OK) {
		cout << "Framegrabber connect failed." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, grabber);
	}

 	ret = grabber->startGrab();
	if(ret != FrameGrabber::E_ALL_OK) {
		cout << "Framegrabber startGrab failed." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, grabber);
	}
	
	//********** END FRAME GRABBER INIT

	// Start the camera with the chosen configuration
	cout << "Starting camera... "; 
	ret = cam->start();
	if(ret != EthernetCamera::E_ALL_OK) {
		cout << " failed." << endl << endl;
        cout << "Press enter to exit application.";
		cin.get();
	}

	IconBuffer *buffer;
	int res;
	int oldtick = 0, count = 0;

	// Loop until key press.
	while(!_kbhit())
	{
		// Get a new scan in 'scan' and timeout if nothing is received in 1000 ms.
		res = grabber->getNextIconBuffer(&buffer, 1000);
		if(res == 0)
		{
			count += prms->getNoScans();
			if((GetTickCount()-oldtick) > 10000)
			{
				cout << count / 10 << " Hz." << endl;
				oldtick = GetTickCount();
				count = 0;
			}

			// cout << "Received scan with ID: " << buffer->getScanID(0) << " size: " << grabber->getDataFormat()->dataSize() << endl;
			
			/// Here we do some processing of the scan...
			if((buffer->getScanID(0) % modulo) == 0)
			{
				cout << " Saving..." << endl;
				ostringstream str, str2;
				/// Save the image in BMP format.
				str << "SCAN" << buffer->getScanID(0) << ".BMP";
				cout << "Saving image to " << str.str() << endl;
				buffer->saveImage(str.str().c_str());

				/// Save the image in RAW format.
				str2 << "SCAN" << buffer->getScanID(0) << ".RAW";
				cout << "Saving image to " << str2.str() << endl;
				saveRaw(str2.str(), buffer->getReadPointer(0), buffer->getScanSize());

				// Extract data from scan layout buffer
				const DataFormat *myDataFormat = buffer->getDataFormat();
				cout << "Data format: " << myDataFormat->toString().c_str() << endl; // This needs to be saved as XML to be able to use it later in the python script

				// Print the structure of the .dat file
				accessData(buffer);

			}

            // Release buffer (manual release mode)
            grabber->releaseIconBuffer();
		} 
		else
		{
			cout << "Timeout. " << endl;
		}
	}

	// Stop and exit
	cout << "stopping camera." << endl; 
	cam->stop();

	cout << "stopping frame grabber." << endl;
	grabber->stopGrab();
	grabber->disconnect();


	return closeDown(0, cam, grabber);
}

int
closeDown(int err, EthernetCamera* cam, FrameGrabber *grabber) {
	if(cam != 0) delete cam;
	if(grabber != 0) delete grabber;
	closeApi();
	return err;
}
