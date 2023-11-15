// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "sensor_msgs/msg/image.hpp"

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
#include "ranger/DataAccess.h"
#include "ranger/ThreadSafePrint.h"

using namespace std::chrono_literals;

using namespace std;
using namespace icon;

void closeDown(int err, EthernetCamera *cam, FrameGrabber *grabber);
string currentFormattedTime();

// Construct an error callback by inheriting the
// icon::ErrorHandler class and override onError
class MyErrHandler : public icon::ErrorHandler
{
  void onError(int errorLevel, const icon::String &err) { cout << err.c_str() << endl; };
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

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  // Member variables
  Camera *cam_generic;
  EthernetCamera *cam;
  FrameGrabber *grabber;
  FGEthernetFastParameters *prms;
  IconBuffer *inBuffer = NULL, outBufferCalibrated, outBufferRectified;
  CalibrationFilter calibrationFilter;
  RectificationFilter rectificationFilter;
  const DataFormat *dr_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::MultiArrayLayout>::SharedPtr publisher_intensity_;
  rclcpp::Publisher<std_msgs::msg::MultiArrayLayout>::SharedPtr publisher_range_;
  size_t count_;

  std::string param_file;

public:
  MinimalPublisher(const std::string &file_path_)
      : Node("minimal_publisher"), count_(0), rectificationFilter(1536), param_file(file_path_)
  {

    //********** BEGIN CAMERA CONTROL INIT
    string ip = "192.168.0.92", type_config = "Measurement";
    cout << "creating camera and framegrabber ... " << endl;
    // Create the Camera
    cam_generic = createCamera("EthernetCamera", "MyCamera");
    if (cam_generic == 0 || cam_generic->getCameraType() != EthernetCamera::cameraType)
    {
      delete cam_generic;
    }
    // Cast the camera to the correct type
    cam = static_cast<EthernetCamera *>(cam_generic);
    if (cam == NULL)
    {
      /// The cast failed.
      delete cam_generic;
    }
    // Create the Framegrabber
    grabber = createFrameGrabber("FGEthernetFast", "MyGrabber");
    // Cast the framegrabber to the correct type
    prms = dynamic_cast<FGEthernetFastParameters *>(grabber->getParameters());
    if (prms == NULL)
    {
      cout << "Incorrect parameter type returned from getParameters()" << endl;
      closeDown(-1, cam, grabber);
    }
    // Set the parameters for the framegrabber
    cam->setComParameters(ip.c_str(), prms->getFrameGrabberPort(), prms->getRedundancyPort(), EthernetCamera::HIGH_PERFORMANCE_DATA_CHANNEL);
    cam->setProtocolTimeout(3000); // 3 s timeout
    // Initialize the camera
    cout << "init camera ... " << endl;
    int ret = cam->connect();
    if (ret != EthernetCamera::E_ALL_OK)
    {
      cout << "init failed. \n\n";
      closeDown(-1, cam, NULL);
    }
    cout << "init ok." << endl;
    // Reset the camera connection
    ret = cam->stop();
    if (ret == EthernetCamera::E_ALL_OK)
    {
      cout << "Stopping camera failed." << endl;
    }
    //********** END CAMERA CONTROL INIT

    ret = cam->stop();
    if (ret == EthernetCamera::E_ALL_OK)
    {
      cout << "Stopping camera failed." << endl;
    }

    //********** BEGIN CONFIGURE CAMERA
    // cout << "Configuring camera ... " << endl;
    // ret = cam->fileSaveParameters("DOWNLOAD.PRM");
    // if (ret != EthernetCamera::E_ALL_OK) {
    //   cout << "Download of parameter file failed." << endl;
    //   closeDown(-1, cam, NULL);
    // }
    // Set the Camera file
    ret = cam->fileLoadParameters(param_file.c_str());
    if (ret != EthernetCamera::E_ALL_OK)
    {
      cout << "upload of parameter file failed.\n\n";
      closeDown(-1, cam, NULL);
    }
    // Print the configuration available
    icon::StringVector configs;
    ret = cam->getAvailableConfigurations(configs);
    if (ret != EthernetCamera::E_ALL_OK)
    {
      cout << "Get available configurations failed." << endl
           << endl;
      closeDown(-1, cam, NULL);
    }
    // Print the available configurations
    cout << endl
         << "Configurations:" << endl;
    for (size_t i = 0; i < configs.size(); i++)
    {
      cout << i << " -- " << configs[i].c_str() << endl;
    }
    // Set the configuration
    ret = cam->setActiveConfiguration(type_config.c_str());
    if (ret != EthernetCamera::E_ALL_OK)
    {
      cout << "Set active configuration failed with error code: " << ret << endl
           << endl;
      closeDown(-1, cam, NULL);
    }
    //********** END CONFIGURE CAMERA

    //********** BEGIN FRAME GRABBER INIT
    cout << "Configuring framegrabber ... " << endl;
    int numberOfScans = 512, ramDataBuffers = 50;
    icon::String type, ver, serial;
    // Set the number of scans
    prms->setNoScans(numberOfScans);
    // Set the buffer height
    ret = cam->setBufferHeight(numberOfScans);
    if (ret != EthernetCamera::E_ALL_OK)
    {
      cout << "set buffer height failed.\n\n";
      closeDown(-1, cam, NULL);
    }
    // Tell the frame grabber what camera to listen to by handing it the IP address of the camera
    prms->setCameraIP(ip.c_str());
    // Ask the camera for the data port from which it sends its measurement data and give this information
    // to the frame grabber. The grabber now knows what port on what IP address to listen to.
    int dataPort;
    if (cam->getCameraDataPort(dataPort))
    {
      cout << "get camera data port failed.\n\n";
      closeDown(-1, cam, NULL);
    }
    prms->setCameraPort(dataPort);
    // Tell the grabber how much RAM (in Megabytes) to allocate for data buffers. This will
    // indirectly determine how many data buffers it will allocate. The size of one buffer is the
    // number of rows * the size of one scan. The number of buffers is thus the BufferSize/size of
    // each buffer (always truncated since only full buffers are allowed). At this stage we do not yet
    // know the size of each buffer. We will soon ask the camera for information about its data format
    prms->setBufferSize(ramDataBuffers);
    // Set up handling of lost packet recovery.
    if (cam->getCameraInfo(type, ver, serial))
    {
      cout << "get camera info failed.\n\n"
           << endl;
      closeDown(-1, cam, NULL);
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
      cout << "get recovery interval failed.\n\n"
           << endl;
      closeDown(-1, cam, NULL);
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
    if (ret != EthernetCamera::E_ALL_OK)
    {
      cout << "get data format failed.\n\n"
           << endl;
      closeDown(-1, cam, NULL);
    }

    // Notice that the data format is so far treated as a string. A DataFormat object
    // can be initialized with this string. We do this now to show how it is done.
    // The created DataFormat object will also be used by the calibration filter further down.
    DataFormat df;
    ret = df.init(dataformat.c_str());
    if (ret != DataFormat::E_ALL_OK)
    {
      cout << "Initialization of DataFormat object failed.\n\n"
           << endl;
      closeDown(-1, cam, grabber);
    }

    // Get the packet payload size from the camera. This must be done after all camera
    // configuration is done to get packet size that is valid for the dataformat that
    // the camera will use.
    unsigned long packetSize;
    ret = cam->getPacketSize(packetSize);
    if (ret != EthernetCamera::E_ALL_OK)
    {
      cout << "get packet size failed.\n\n"
           << endl;
      closeDown(-1, cam, NULL);
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
    if (ret != EthernetCamera::E_ALL_OK)
    {
      cout << "framegrabber connect failed.\n\n"
           << endl;
      closeDown(-1, cam, grabber);
    }

    // A connected grabber is configured and ready to start. It is however not yet started.
    // Starting the grabber just tells it to start listening for incoming measurement data.
    grabber->startGrab();
    if (ret != EthernetCamera::E_ALL_OK)
    {
      cout << "framegrabber startGrab failed.\n\n"
           << endl;
      closeDown(-1, cam, grabber);
    }
    //********** END FRAME GRABBER INIT

    //********** START CALIBRATION AND RECTIFICATION INIT
    cout << "Configuring calibration and rectification ... " << endl;
    // Create a string and load the LUT into it. The LUT is stored in a User Data area of the camera flash
    icon::String calibrationLUT;
    ret = cam->flashGetUserData(calibrationLUT);
    if (ret != EthernetCamera::E_ALL_OK)
    {
      cout << "reading calibration LUT from camera flash memory failed.\n\n"
           << endl;
      closeDown(-1, cam, grabber);
    }

    cout << "Length of calibration LUT: " << calibrationLUT.size() << endl;
    // Create a calibration filter object and configure it with the LUT from the camera, the data format
    // from the camera and the number of scans per buffer used by the frame grabber. Note that the
    // inBuffer will be a raw IconBuffer using the scan layout (where subcomponents are stored
    // side by side in memory) whereas the output from the calibration (and rectification) filter will be
    // stored in the subcomponent layout (where subcomponents are stored one after the other in memory)
    // If you instead wish to load the LUT from file you use the initFromFile() function instead
    calibrationFilter.initFromData(calibrationLUT, df, numberOfScans);

    // Call the method prepareResults in order to perform all initiation work before analysing the first
    // scan. This is optional but if the initiation needs to be done when the applicatino is running,
    // processing of the first buffer will be slower than the rest.
    calibrationFilter.prepareResult(outBufferCalibrated);

    // If you would like to know what the data format of the outBufferCalibrated buffer is you can
    // always ask it. After running the prepareResult the data format of the buffer is setup properly
    // Here is an example on how to print the data format XML description in the console window.
    // You can also use the data format's member functions to ask it about individual subcomponents
    const DataFormat *d = outBufferCalibrated.getDataFormat();
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
    // unsigned long rectificationWidth = 1536; // Rectify to the same width as the input data
    // rectificationFilter = RectificationFilter(rectificationWidth);

    // Plug in the rectification filter to use the output from the calibration filter as input
    // We have not done this for the calibration filter since the inBuffer pointer will point at different
    // buffers in the FIFO queue every time we run the calibration
    ret = rectificationFilter.setInput(*(outBufferCalibrated.getDataFormat()), numberOfScans);

    // Prepare the rectification filter to not let initiation and allocation slow down processing of
    // the first buffer
    ret = rectificationFilter.prepareResult(outBufferRectified);
    dr_ = outBufferRectified.getDataFormat();
    cout << dr_->toString().c_str() << endl;

    //********** END CALIBRATION AND RECTIFICATION INIT

    cout << "starting camera... ";
    ret = cam->start();
    if (ret != EthernetCamera::E_ALL_OK)
    {
      cout << " failed.\n\npress enter to exit application.";
    }

    //********** BEGIN PUBLISHER
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    publisher_intensity_ = this->create_publisher<std_msgs::msg::MultiArrayLayout>("intensity", 10);
    publisher_range_ = this->create_publisher<std_msgs::msg::MultiArrayLayout>("range", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));

    //********** END PUBLISHER
  }

  ~MinimalPublisher()
  {
    cout << "stopping camera... " << endl;
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
    closeDown(0, cam, grabber);
  }

private:
  void timer_callback()
  {
    // Setup a result variable to temporarily store the result status from various functions
    int res;

    // Ask the frame grabber if there is any new buffer available in the FIFO.
    // If no data has arrived within 1000 ms we will timeout and continue
    res = grabber->getNextIconBuffer(&inBuffer, 1000);
    // If a new buffer was received...
    if (res == 0)
    {
      cout << "Scan received" << endl;
      cout << "SCAN ID: " << outBufferRectified.getScanID(0) << endl;
      // Apply the calibration filter to get calibrated data in world units.
      // The result is stored in outBufferCalibrated
      calibrationFilter.apply(*inBuffer, outBufferCalibrated);
      // Apply the rectification filter to generate resampled range data in a format which directly
      // corresponds to real world units.
      // This works in the same way as the calibration filters and you can retrieve read pointers to
      // arrays of consecutive range, intensity and optionally also scatter data in the same way
      rectificationFilter.apply(outBufferCalibrated, outBufferRectified);
      // Get the read pointers to the range and intensity data in the rectified buffer
      const float *range_data;
      int res_range = outBufferRectified.getReadPointer("Hi3D 1", "Range", range_data);
      const float *intensity_data;
      int res_intensity = outBufferRectified.getReadPointer("Hi3D 1", "Intensity", intensity_data);

      // Check that 'Hi3D 1 Range' and 'Hi3D 1 Intensity' exist is to try to access the data and see if it works.
      if (res_range == icon::E_ALL_OK && res_intensity == icon::E_ALL_OK)
      {
        // If we get here we know that the buffer contains the subcomponents we are looking for
        // and we can access the data

        // We first find out how many horizontal pixels there are in a buffer. The buffers are
        // organized row by row in memory and since this is a buffer with the subcomponent layout
        // we can easily access a certain pixel by just adding the offset from the origin.
        // This offset is (of course) x + buffer width * y
        unsigned int numberOfScans = inBuffer->getHeight();
        unsigned int bwidth = inBuffer->getDataFormat()->getNamedComponent("Hi3D 1")->getNamedSubComponent("Range")->getWidth();
        cout << "(Width, Height) = (" << bwidth << ", " << numberOfScans << ")" << endl;

        // Reserving the space for the range and intensity data
        std::vector<std::vector<float>> range_vec(numberOfScans, std::vector<float>(bwidth));
        std::vector<std::vector<float>> intensity_vec(numberOfScans, std::vector<float>(bwidth));

        // Loop through the range and intensity data and store them in the vector
        for (unsigned int scan = 0; scan < numberOfScans; scan++)
        {
          // Loop through all elements in each scan.
          for (unsigned int col = 0; col < bwidth; col++)
          {
            int offset = col + scan * bwidth;
            // Get the range and intensity values
            const float val_range = *(range_data + offset), val_intensity = *(intensity_data + offset);
            // print the obtained values
            // safeCout("Range: " << val_range << " Intensity: " << val_intensity << endl);
            // Store the values in the vector
            range_vec[scan][col] = val_range;
            intensity_vec[scan][col] = val_intensity;
          }
        }
        cout << "Range vector (Width, Height) = (" << range_vec[0].size() << ", " << range_vec.size() << ")" << endl;
        cout << "Intensity vector (Width, Height) = (" << intensity_vec[0].size() << ", " << intensity_vec.size() << ")" << endl;

        // We construct the message to be published for the range data
        auto message_range = std_msgs::msg::MultiArrayLayout();

        // We construct the message to be published for the intensity data
        auto message_intensity = std_msgs::msg::MultiArrayLayout();

        // We publish the message
        publisher_range_->publish(message_range);
        publisher_intensity_->publish(message_intensity);

        // We are now done with the original Icon Buffer. We therefore need to inform the grabber that
        // this buffer can be reused to store new incoming data
        grabber->releaseIconBuffer();
      }
      else
      {
        cout << "Could not find 'Hi3D 1 Range' and 'Hi3D 1 Intensity' in buffer" << endl;
      }
    } else {
      cout << "No scan received" << endl;
    }

    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
};

int main(int argc, char *argv[])
{
  MyErrHandler err;
  setErrorHandler(&err);
  SetPriority();

  std::string file_path = "C:\\Users\\magok\\source\\repos\\crcz25\\capstone_welding_analysis\\ros2_ws\\src\\ranger\\include\\ranger\\RangerEHi3D.prm";
  // std::string file_path = "RangerEHi3D.prm";

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>(file_path));
  rclcpp::shutdown();
  return 0;
}

void closeDown(int err, EthernetCamera *cam, FrameGrabber *grabber)
{
  if (cam != 0)
    delete cam;
  if (grabber != 0)
    delete grabber;
  closeApi();
}