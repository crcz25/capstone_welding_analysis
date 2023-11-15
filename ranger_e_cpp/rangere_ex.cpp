/** calibration.cpp

	This example shows how to

	* Connect to an ethernet camera
	* Set up the camera configuration using a parameter file stored on disk
	* Set up a software frame grabber and configure it to listen to input from the ethernet camera
	* Retrieve scan data from the camera using polling
	* Filter the raw data to get calibrated data
	* Filter the calibrated data to get rectified data

	The example works with Ranger E and Ruler E cameras
	The camera needs to have a calibration LUT stored in flash. If you run the example on a Ruler E
	this is always the case since it comes shipped with a precalibrated LUT. If you use a Ranger E or D
	camera you first need to create a calibration LUT with the Coordinator tool

	To use the example with a Ranger D camera you need to remove all references to the Intensity
	subcomponent since Ranger D cameras only deliver range data

	\par Copyright
	(c) 2009 SICK IVP AB

   $Revision: $
   $LastChangedDate: $
   $Author: $
   $HeadURL:  $

   Argument list: <IP> <IN parameter file> <config mode>
   IP: IP address of camera
   IN parameter file: Parameter file to load into camera (.PRM)
   config mode: Configuration to use (Measurement or Image)
*/
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <locale>
#include <chrono>

// Include the icon api
#include "icon_api.h"

// Include the auto linker configuration for iCon API.
#include "icon_auto_link.h"

#include <windows.h>

// Include the EthernetCamera camera header
#include "ethernetcamera.h"

// Include file for iCon Framegrabber support.
#include "framegrabber.h"

// Include file for calibration filter.
#include "filter/calibration.h"

// Include file for rectification filter.
#include "filter/rectification.h"

// Example program includes
#include "DataAccess.h"
#include "ThreadSafePrint.h"


// For keypress detection.
#include <conio.h>

using namespace std;
using namespace icon;

// In file dataformatexample.cpp
//void showDataFormat(Camera *cam0);

int closeDown(int err, EthernetCamera* cam, FrameGrabber* grabber);
string currentFormattedTime();

// Construct an error callback by inheriting the
// icon::ErrorHandler class and override onError
class MyErrHandler : public icon::ErrorHandler {
	void onError(int errorLevel, const icon::String& err) { cout << err.c_str() << endl; };
};


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

bool saveRaw(std::string& filename, const unsigned char* data, int length) {
	ofstream stream;

	stream.open(filename.c_str(), ios_base::out | ios_base::trunc | ios_base::binary);

	for (int i = 0; i < length; i++) {
		stream << data[i];
	}

	stream.close();

	return 0;
}

string currentFormattedTime() {
	// Get the current time
	auto currentTime = std::chrono::system_clock::now();
	auto duration = currentTime.time_since_epoch();
	auto hours = std::chrono::duration_cast<std::chrono::hours>(duration);
	duration -= hours;
	auto minutes = std::chrono::duration_cast<std::chrono::minutes>(duration);
	duration -= minutes;
	auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
	duration -= seconds;
	auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
	duration -= milliseconds;
	time_t theTime = time(NULL);
	struct tm* aTime = localtime(&theTime);

	// Convert duration to YY-MM-DD-HH-MM-SS-MS format
	string formattedTime = to_string(aTime->tm_year + 1900) + "-" + 
		to_string(aTime->tm_mon + 1) + "-" + 
		to_string(aTime->tm_mday) + "-" + 
		to_string(aTime->tm_hour) + "-" + 
		to_string(aTime->tm_min) + "-" + 
		to_string(aTime->tm_sec) + "-" + 
		to_string(milliseconds.count());
	
	return formattedTime;
}


int main(int argc, char* argv[])
{
	// Initial Comment:
	//
	// Notice that many of the iCon classes are not entirely thread safe. It is perfectly safe to use
	// e.g. one thread per camera in a multi camera setup but it is not safe to let several threads use
	// the same camera and frame grabber objects. If e.g. Multi-threaded image analysis is needed make sure
	// to set up the application to use one thread in all communication with a specific camera and frame 
	// grabber.
	//
	// If you use callbacks to access the incoming data there will be a certain amount of multi-threading
	// The recommended and verified use is to let the callback functions (which are running in separate 
	// threads from the main program) only operate on the incoming buffers and not handle communication 
	// with camera and grabber. If a result from analysing the data is that a camera should be e.g. 
	// stopped or reconfigured the callback function should pass a message to the main thread and ask for
	// this action to be taken.

	// Construct & bind the error handler. The error handler is used to receive human readable 
	// error and warning messages from the camera. These messages are what you can read in the log
	// window in Ranger Studio
	MyErrHandler err;
	setErrorHandler(&err);

	// Give the program real time priority to indicate that it needs to run as uninterrupted as possible
	// This is not required for a simple example program but for more advanced analysis to reach required
	// performance it may be needed to boost the program priority.
	SetPriority();

	// In order to connect to an Ethernet camera we need to use its IP address. 
	// Here we ask the user for this information.
	// An alternative approach is to scan the local network for connected cameras using the 
	// EthernetCamera::discoverDevices() member function. This will return a collection of all 
	// connected devices from which you can retrieve information such as their IP address, camera type, etc
	string ip;

	// Grab the first argument as the IP address of the camera to use.
	if (argc > 1) {
		ip = argv[1];
		cout << "Using IP: " << ip << endl;
	}
	else {
		cout << "Please enter IP address of camera: ";
		cin >> ip;
	}

	//********** BEGIN CAMERA CONTROL INIT

	// In this section we will create and initialize an object to control the camera. 
	// We will also configure it with a parameter file from the PC and set it up to operate in
	// Measurement mode
	//
	// Before running the camera init function we also need to create a frame grabber object.
	// The reason is that we need to inform the camera about what IP and port number to use. The
	// IP address is known beforehand but the port number is assigned when creating the frame grabber
	// object

	// Call camera factory in the icon api. Ask for a EthernetCamera camera. This will generate a generic
	// camera object which needs to be converted to an Ethernet camera object
	Camera* cam_generic = createCamera("EthernetCamera", "MyCamera");

	// Check result from factory - shut down if creation failed
	if (cam_generic == 0 || cam_generic->getCameraType() != EthernetCamera::cameraType) {
		delete cam_generic;
		return -1;
	}

	// Cast the generic camera object to be an instance of the EthernetCamera class. This means it has
	// ethernet properties such as IP address, port number, etc. There is currently one other type of camera
	// which is the RangerC. A RangerC camera object has CameraLink properties such as frame grabber number 
	// and port, serial port number, etc. 
	EthernetCamera* cam = static_cast<EthernetCamera*>(cam_generic);

	// If the casting failed, shut down and exit
	if (cam == NULL)
	{
		/// The cast failed.
		delete cam_generic;
		return -1;
	}

	// Create a frame grabber object to be of Ethernet type. This means that it is a software grabber
	// used to collect data coming in from an Ethernet controller. We need the object already now to 
	// retrieve the frame grabbers port number. From the grabber we extract the frame grabber parameters.
	// These need to be cast to Ethernet grabber parameters since they are requested from a generic frame
	// grabber object
	FrameGrabber* grabber = createFrameGrabber("FGEthernetFast", "MyGrabber");
	FGEthernetFastParameters* prms = dynamic_cast<FGEthernetFastParameters*>(grabber->getParameters());
	if (prms == NULL)
	{
		cout << "Incorrect parameter type returned from getParameters()" << endl;
		cin.get();
		return closeDown(-1, cam, grabber);

	}

	// Tell the camera what UDP port on the PC to send data to.
	cam->setComParameters(ip.c_str(), prms->getFrameGrabberPort(), prms->getRedundancyPort(), EthernetCamera::HIGH_PERFORMANCE_DATA_CHANNEL);

	// Tell the camera to wait for maximum three seconds for response to sent messages
	cam->setProtocolTimeout(3000); // 3 s timeout

	// Init camera
	// Now we have set the camera parameters we need to set. Initialization of the camera gives this
	// program control of the camera. It does however not affect its current state.
	// 
	// If init fails, shut down neatly...
	cout << "initiating ... " << endl;
	int ret = cam->connect();
	if (ret != EthernetCamera::E_ALL_OK) {
		cout << "init failed. \n\npress enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}
	cout << "init ok." << endl;

	// To be on the safe side we stop the camera (in case some other application had started it 
	// and not stopped it when finishing.
	ret = cam->stop();
	if (ret == EthernetCamera::E_ALL_OK) {
		cout << "Stopping camera failed." << endl;
	}

	// Send a camera configuration file to the camera. This will only send the file to the camera
	// The camera will be configured to use the parameters of the file when we later set the active 
	// configuration

	// Download the current parameter file
	// ret = cam->fileSaveParameters("DOWNLOAD.PRM");
	// if (ret != EthernetCamera::E_ALL_OK) {
	//	cout << "Download of parameter file failed." << endl << endl;
	//	cout << "Press enter to exit application.";
	//	cin.get();
	//	return closeDown(-1, cam, NULL);
	// }
	// Grab the second argument as the parameter file to use.
	if (argc > 2) {
		cout << "Using parameter file: " << argv[2] << endl;
		ret = cam->fileLoadParameters(argv[2]);
	}
	else {
		cout << "Please enter parameter file to use: ";
		string prm;
		cin >> prm;
		ret = cam->fileLoadParameters(prm.c_str());
	}

	// If we were not able to send the file to the camera we shut down the application in this example.
	if (ret != EthernetCamera::E_ALL_OK) {
		cout << "upload of parameter file failed.\n\npress enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	// Set the camera's active configuration to be 'Measurement'. When this is done, the camera will
	// generate a sensor program based on the parameters we sent earlier and test it. This may take 
	// a second or two. In this process the minimum cycle time will be computed (by testing). This is
	// reported on the error channel but there is also an API function to explicitly ask for it. This
	// function is named Camera::getMeasuredMinCycleTime()

	// Get available configurations
	icon::StringVector configs;
	ret = cam->getAvailableConfigurations(configs);
	if (ret != EthernetCamera::E_ALL_OK) {
		cout << "Get available configurations failed." << endl << endl;
		cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}
	// Print the available configurations
	cout << endl << "Configurations:" << endl;
	for (size_t i = 0; i < configs.size(); i++) {
		cout << i << " -- " << configs[i].c_str() << endl;
	}
	// Grab the third argument as the configuration to use.
	if (argc > 3) {
		cout << "Using configuration: " << argv[3] << endl;
		ret = cam->setActiveConfiguration(argv[3]);
	}
	else {
		cout << "Using default configuration: Measurement" << endl;
		ret = cam->setActiveConfiguration("Measurement");
	}

	// ret = cam->setActiveConfiguration("Measurement");
	if (ret != EthernetCamera::E_ALL_OK) {
		cout << "Set active configuration failed with error code: " << ret << endl << endl;
		cout << "Press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	// Now the camera is configured and ready to use. By calling the camera object's start() member we can 
	// make it instantly start acquiring profiles.
	//********** END CAMERA CONTROL INIT





	//********** BEGIN FRAME GRABBER INIT

	// We have already earlier created a frame grabber object. Now it is time to configure it
	// to accept data from the camera.

	cout << "Initiating Frame Grabber." << endl;

	// Tell the frame grabber how many scans to store in each IconBuffer.
	// We use 512 scans in each buffer in this example.
	// Important notice! If the camera is operating in Image mode this value should always be set to 1
	// since an image mode image is to some extent considered as one long scan. The reason is that 
	// the whole image is acquired at one instance in time.
	int numberOfScans;
	// Check if the camera is operating in Image mode or not
	if (argv[4] == "Image") {
		numberOfScans = 1;
	}
	else {
		numberOfScans = 512;
	}
	prms->setNoScans(numberOfScans);

	// When using the FGEthernetFast framegrabber class you need to tell the camera the buffer height
	// before starting.
	ret = cam->setBufferHeight(numberOfScans);
	if (ret != EthernetCamera::E_ALL_OK) {
		cout << "set buffer height failed.\n\npress enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	// Tell the frame grabber what camera to listen to by handing it the IP address of the camera 
	prms->setCameraIP(ip.c_str());

	// Ask the camera for the data port from which it sends its measurement data and give this information
	// to the frame grabber. The grabber now knows what port on what IP address to listen to.
	int dataPort;
	if (cam->getCameraDataPort(dataPort))
	{
		cout << "get camera data port failed.\n\npress enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}
	prms->setCameraPort(dataPort);

	// Tell the grabber how much RAM (in Megabytes) to allocate for data buffers. This will
	// indirectly determine how many data buffers it will allocate. The size of one buffer is the
	// number of rows * the size of one scan. The number of buffers is thus the BufferSize/size of 
	// each buffer (always truncated since only full buffers are allowed). At this stage we do not yet
	// know the size of each buffer. We will soon ask the camera for information about its data format
	prms->setBufferSize(50);

	// Set up handling of lost packet recovery.
	// Occasionally data packets are lost in transfer over ethernet connections. The camera sends its
	// measurement data using the unverified UDP protocol and no data will ever be resent from the camera
	// to the PC. This design maximizes performance in terms of speed at the price of robustness.
	// To counter this there is some redundancy included in the data sent from the camera. This
	// redundant information can be used by iCon to reconstruct lost packets on the PC side.
	// You can select how much redundancy to send by adjusting the camera parameter 'redundancy frequency'
	// If this parameter is set to e.g. 10 one packet out of a sequence of ten can be recovered.
	// You need to tell the frame grabber what redundancy frequency to use for it to handle this properly
	// This is done by asking the camera for the current setting of that particular parameter:

	icon::String type, ver, serial;

	// This somewhat complicated example also shows how to get the current value of any 
	// camera parameter without even knowing what type of camera it is....

	// Ask the camera what camera type it is (Ruler E1200, Ranger E55, Ranger D40, etc)
	// This info goes in the XML parameter structure to find the redundancy frequency
	if (cam->getCameraInfo(type, ver, serial))
	{
		cout << "get camera info failed.\n\npress enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	// Compose the XML path where the redundancy frequency is found.
	std::string path = "<ROOT><CAMERA name='";
	path += type.c_str();
	path += "'><MODULE name='Ethernet'>";
	const std::string name = "redundancy frequency";
	icon::String paramValue2;

	// Ask the camera for the value of the "redundancy frequency" parameter of the "Ethernet" section
	// of the configuration structure
	if (cam->getParameterValue(path.c_str(), name.c_str(), paramValue2))
	{
		cout << "get recovery interval failed.\n\npress enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	// Transfer the redundancy frequency information to the frame grabber
	prms->setRecoveryInterval(atoi(paramValue2.c_str()));

	// Set up the frame grabber to use polling to access measurement data.
	// This means that the applicaiton program (this program) will repeatedly ask iCon if there is any
	// new data available. The alternative approach is to use callbacks (FGCALLBACK) and provide the frame
	// grabber with a pointer to a function which it will call as soon as there is a new buffer available.
	prms->setMode(icon::FrameGrabber::FGPOLLING);

	// Tell the frame grabber that manual release will be used. This means that whenever the user 
	// application has finished processing a buffer it will manually inform the grabber that it no longer
	// needs this buffer and that the grabber can reuse it. It is strongly recommended to always use
	// manual release in combination with polling. If you use the Pleora High Performance drivers you should
	// always use manual release when using polling.
	//
	// Remember that the frame grabber always owns the data buffer memory. That is, the grabber allocates
	// and deallocates the buffer memory. Releasing a buffer only means informing the grabber that you will
	// not access that buffer any longer.
	//
	// ALso remember that the buffers are organized in a cyclic FIFO and that you always release the oldest 
	// buffer first. If you need to access buffers in random order you need to store a copy of the buffers.
	// These buffers are then owned, allocated and deallocated by your application program
	prms->setRelease(icon::FrameGrabber::MANUAL_RELEASE);

	// Pointer to a user supplied argument to the callback. Since we are using polling this parameter is
	// not used. If you use callbacks it can be used to pass information about the calling object to the
	// callback function. This is typically used to let the callback function know how to use the data.
	// Callback functions are called from the frame grabber and are run in a separate thread from the 
	// main program.
	prms->setUserData(NULL);

	// Get the data format from the camera. This retrieves the scan format of the current configuration of
	// the camera. This will tell the frame grabber how to allocate its buffers. The data format contains
	// information about what components and subcomponents are part of the measurement data, how large 
	// these are (in bytes and pixels) etc. You can extract component info by calling the getComponent()
	// member of the dataFormat. Components contain detailed information about e.g the range of values a
	// certain subcomponent can have. This information is stored in the Component's Traits property.
	//
	// In this example we only ask the camera for its format to pass the information on to the frame grabber
	icon::String dataformat;
	ret = cam->getDataFormat("", dataformat);
	if (ret != EthernetCamera::E_ALL_OK) {
		cout << "get data format failed.\n\npress enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	// Notice that the data format is so far treated as a string. A DataFormat object
	// can be initialized with this string. We do this now to show how it is done. 
	// The created DataFormat object will also be used by the calibration filter further down.
	DataFormat df;
	ret = df.init(dataformat.c_str());
	if (ret != DataFormat::E_ALL_OK)
	{
		cout << "Initialization of DataFormat object failed.\n\npress enter to exit application.";
		cin.get();
		return closeDown(-1, cam, grabber);
	}

	// Get the packet payload size from the camera. This must be done after all camera
	// configuration is done to get packet size that is valid for the dataformat that
	// the camera will use.
	unsigned long packetSize;
	ret = cam->getPacketSize(packetSize);
	if (ret != EthernetCamera::E_ALL_OK) {
		cout << "get packet size failed.\n\npress enter to exit application.";
		cin.get();
		return closeDown(-1, cam, NULL);
	}

	// Set the data format and packet size used by the frame grabber to the same as the
	// camera currently uses.
	prms->setDataFormat(dataformat, packetSize);

	// Now the grabber knows all it needs to initialize its buffers and connect to a camera
	//
	// Summary:
	// Camera info, IP address and port number
	// Data format: Size of each scan
	// Number of rows (scans) per buffer
	// Data access mode (polling or callbacks)
	// Release mode (manual or automatic)
	ret = grabber->connect();
	if (ret != EthernetCamera::E_ALL_OK) {
		cout << "framegrabber connect failed.\n\npress enter to exit application.";
		cin.get();
		return closeDown(-1, cam, grabber);
	}

	// A connected grabber is configured and ready to start. It is however not yet started.
	// Starting the grabber just tells it to start listening for incoming measurement data.
	grabber->startGrab();
	if (ret != EthernetCamera::E_ALL_OK) {
		cout << "framegrabber startGrab failed.\n\npress enter to exit application.";
		cin.get();
		return closeDown(-1, cam, grabber);
	}

	// Now the grabber is up and running. As soon as the number of scans corresponding to one buffer
	// has been received from the camera, the first IconBuffer will be marked as available to the user 
	// application. Since we use polling the user application will not be informed automatically but needs
	// to ask the grabber at regular intervals if there is any new data available.
//********** 
// END FRAME GRABBER INIT





//********** START CALIBRATION AND RECTIFICATION INIT

	// This involves the following steps:
	//
	// Download the calibration LUT from the camera or load it from disk.
	// Create a calibration filter and an IconBuffer to store the calibrated data in.
	// Configure the calibration filter to use the current data format and calibration LUT.
	// Create a rectification filter and an IconBuffer to store the rectified data in.

	// Retrieve the calibration LUT from the camera and store it as a string
	// The calibration LUT tells the calibration filter how to translate sensor coordinates (pixels) 
	// into real world measurements (mm or inches). An alternative to retrieving the LUT from the camera 
	// is to load it from disk. The Coordinator tool can store its resulting LUT:s both on disk and camera
	// flash. If you use a MultiScan setup with several range (3D) components you will have to store LUT:s
	// on disk since you can only store one LUT on the camera flash. To load the LUT from file you
	// use the initFromFile() function instead of the initFromData() when configuring the filter
	// 
	// In this example we however download the LUT from the camera...

	// Create a string and load the LUT into it. The LUT is stored in a User Data area of the camera flash
	icon::String calibrationLUT;
	ret = cam->flashGetUserData(calibrationLUT);
	if (ret != EthernetCamera::E_ALL_OK) {
		cout << "reading calibration LUT from camera flash memory failed.\n\n" <<
			"press enter to exit application.";
		cin.get();
		return closeDown(-1, cam, grabber);
	}


	cout << "Length of calibration LUT: " << calibrationLUT.size() << endl;
	// Create a pointer which will point out the incoming raw measurement data 
	IconBuffer* inBuffer = NULL;

	// Create an empty iCon Buffer which will be used to store the calibrated data produced by
	// the calibration filter. Also create a buffer to store the output from rectification
	IconBuffer outBufferCalibrated;
	IconBuffer outBufferRectified;

	// Create a calibration filter object and configure it with the LUT from the camera, the data format
	// from the camera and the number of scans per buffer used by the frame grabber. Note that the
	// inBuffer will be a raw IconBuffer using the scan layout (where subcomponents are stored 
	// side by side in memory) whereas the output from the calibration (and rectification) filter will be
	// stored in the subcomponent layout (where subcomponents are stored one after the other in memory)
	// If you instead wish to load the LUT from file you use the initFromFile() function instead
	CalibrationFilter calibrationFilter;
	calibrationFilter.initFromData(calibrationLUT, df, numberOfScans);

	// Call the method prepareResults in order to perform all initiation work before analysing the first
	// scan. This is optional but if the initiation needs to be done when the applicatino is running, 
	// processing of the first buffer will be slower than the rest.
	calibrationFilter.prepareResult(outBufferCalibrated);

	// If you would like to know what the data format of the outBufferCalibrated buffer is you can 
	// always ask it. After running the prepareResult the data format of the buffer is setup properly
	// Here is an example on how to print the data format XML description in the console window.
	// You can also use the data format's member functions to ask it about individual subcomponents
	const DataFormat* d = outBufferCalibrated.getDataFormat();
	cout << d->toString().c_str() << endl;

	// Create a rectification filter and configure it to generate an output buffer with the width 
	// 1536 pixels. Rectification means resampling the calibrated set of (x, z) data points to a regular
	// coordinate system where the distance between two adjacent pixels is constant across the entire field
	// of view. This resampling can be done to an arbitrary resolution which is determined by the
	// rectification width. The value 1536 is the sensor width in pixels meaning that the widest part of the
	// field of view will be resampled to scale 1:1 whereas the more narrow parts of the FOV will be down-
	// sampled. In the downsampling some data will be lost. The alternative is to set a larger width to keep
	// the full resolution in the narrow sections of the FOV. This will have the drawback that the wider 
	// parts of the FOV are upsampled giving a false impression of the resolution being better that it 
	// actually is in those regions. It will also generate larger buffers containing more pixels which may
	// slow down the analysis process
	unsigned long rectificationWidth = 1536; // Rectify to the same width as the input data
	RectificationFilter rectificationFilter(rectificationWidth);

	// Plug in the rectification filter to use the output from the calibration filter as input
	// We have not done this for the calibration filter since the inBuffer pointer will point at different
	// buffers in the FIFO queue every time we run the calibration
	ret = rectificationFilter.setInput(*(outBufferCalibrated.getDataFormat()), numberOfScans);

	// Prepare the rectification filter to not let initiation and allocation slow down processing of
	// the first buffer
	ret = rectificationFilter.prepareResult(outBufferRectified);
	const DataFormat* dr = outBufferRectified.getDataFormat();
	cout << dr->toString().c_str() << endl;

	//********** END CALIBRATION AND RECTIFICATION INIT


	// Now everything is set up and configured. The frame grabber is running and the processing filters
	// are ready to calibrate and rectify the incoming data. No data will however come in as long as 
	// the camera is not running...

	// Start the camera...
	cout << "starting camera... ";
	ret = cam->start();
	if (ret != EthernetCamera::E_ALL_OK) {
		cout << " failed.\n\npress enter to exit application.";
		cin.get();
	}

	// Now the camera is running. The frame grabber is also running. The application software is ready to
	// start analyzing incoming data. When data will come in is now in the hands of the camera.
	// If the 'Use Enable' camera parameter is set, the camera will wait for its enable input to be active 
	// before it will do anything. If 'Use Enable' is cleared, the camera will start acquiring profiles...

	// ...depending on if the 'trig mode' camera parameter is set to 0 or 2. If it is set to 0 the camera
	// is running in free running mode and scans will be acquired at regular intervals determined by
	// the 'cycle time' camera parameter.
	// If trig mode is 2 scan acquisition is triggered by an external device (encoder)
	//
	// Conclusion: If both trig mode and use enable are set to 0 we know that data will now reach the PC
	// at a constant rate. If not, we do not know when neither the first scan nor the first full buffer
	// will reach the PC. It may take an arbitrary time since it is triggered by peripheral devices.


	// Setup a result variable to temporarily store the result status from various functions
	int res;

	// Setup a time counter and a scan counter to measure the profile rate
	int oldtick = 0, count = 0, count_hz = 0;

	// Create the filename with the formatted date and time
	std::string out_path = "", filename_tmp = "";

	// Main Loop. In this loop we will poll for incoming data until the user presses a key
	// Every time we get data we will perform calibration and rectification of this data
	while (!_kbhit()) // while no key was pressed...
	{
		// Ask the frame grabber if there is any new buffer available in the FIFO.
		// If no data has arrived within 1000 ms we will timeout and continue
		res = grabber->getNextIconBuffer(&inBuffer, 1000);

		// If a new buffer was received...
		if (res == 0)
		{
			// Add the number of scans per buffer to the scan counter
			count_hz += prms->getNoScans();
			count += 1;

			// Every 10 seconds we print out the average number of received profiles per second
			/*if ((GetTickCount() - oldtick) > 10000)
			{
				cout << count / 10 << " Hz." << endl;
				oldtick = GetTickCount();
				count_hz = 0;
			}*/

			// Print stats of current itreration
			// cout << "SCAN ID: " << outBufferRectified.getScanID(0) << endl;
			// cout << "Size Bytes: " << grabber->getDataFormat()->dataSize() << endl;
			// cout << "Number of scans: " << count << endl;

			// Apply the calibration filter to get calibrated data in world units.
			// The result is stored in outBufferCalibrated
			calibrationFilter.apply(*inBuffer, outBufferCalibrated);

			// Apply the rectification filter to generate resampled range data in a format which directly
			// corresponds to real world units.
			// This works in the same way as the calibration filters and you can retrieve read pointers to 
			// arrays of consecutive range, intensity and optionally also scatter data in the same way
			rectificationFilter.apply(outBufferCalibrated, outBufferRectified);

			// Print the structure of the buffer
			// accessData(&outBufferRectified);

			// Every 10 seconds we print out the average number of received profiles per second
			/*if ((GetTickCount64() - oldtick) > 10000)
			{
				cout << count_hz / 10 << " Hz." << endl;
				oldtick = GetTickCount64();
				count_hz = 0;
			}*/

			// Save the data to a xml and dat (at that time)
			// Generate the filename for the image
			filename_tmp = "SCAN_" + currentFormattedTime() + "_" + to_string(count);
			out_path = "../data/" + filename_tmp;
			cout << "Saving data to " << out_path << endl;
			outBufferRectified.saveBuffer(out_path.c_str());

			/// Save the image in BMP format.
			// cout << "Saving image to file... " << endl;
			// ostringstream str;
			// cout << "Saving image to " << str.str() << endl;
			// Generate the filename for the image
			// filename = filename + ".png";
			// Save the image
			// inBuffer->saveImage(filename.c_str(), "PNG");

			// We are now done with the original Icon Buffer. We therefore need to inform the grabber that
			// this buffer can be reused to store new incoming data
			grabber->releaseIconBuffer();

			cout << endl;

			// Exit the while loop
			// break;
		}
		/*else
		{
			// If no buffer arrived within the timeout period, fail and print "timeout" on screen
			cout << "Timeout. " << endl;
		}*/
	}


	// First stop the camera
	// This instantly stops the camera from acquiring and sending more scans to the PC. 
	// There may however still be buffers left in the frame grabber
	cout << "stopping camera." << endl;
	cam->stop();

	// Stop the frame grabber. If we were using callbacks, no more callbacks would be initiated 
	// after this point. There may however still be a callback function being executed in a separate thread
	cout << "stopping frame grabber." << endl;
	grabber->stopGrab();

	// If we intend to at this stage make changes to the configuration which will also affect the data format
	// we need to disconnect from the frame grabber. If we only wanted to stop buffer acquisition for a 
	// while, perhaps to change some parameter which does not affect the data format we would not even need
	// to stop the grabber. In this example we intend to shut down everything.

	// Disconnect the frame grabber. This will de-allocate all buffers in the FIFO (but of course not the 
	// ones created in the user application such as the calibration and rectification output buffers)
	// If we were in callback mode, disconnect would wait for the last callback to terminate before 
	// deallocating the FIFO
	grabber->disconnect();

	// Finally we call the closeDown function which deletes camera and grabber objects and shut down the API
	return closeDown(0, cam, grabber);
}

int
closeDown(int err, EthernetCamera* cam, FrameGrabber* grabber) {
	if (cam != 0) delete cam;
	if (grabber != 0) delete grabber;
	closeApi();
	return err;
}
