/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
// #include <gz/msgs/opticalflow.pb.h>

#include <mutex>

#include <gz/common/Console.hh>
#include <gz/common/Image.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/SystemPaths.hh>

#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>

#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/OpticalFlowSensor.hh"// #include "gz/sensors/DepthCameraSensor.hh"
#include "gz/sensors/Manager.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/ImageGaussianNoiseModel.hh"
#include "gz/sensors/ImageNoise.hh"
#include "gz/sensors/RenderingEvents.hh"
#include <gz/sensors/Util.hh>

#include "PointCloudUtil.hh"

/*
syntax = "proto2";
package sensor_msgs.msgs;

message OpticalFlow
{
  required int64 time_usec              = 1;
  required int32 sensor_id              = 2;
  required int32 integration_time_us    = 3;
  required float integrated_x           = 4;
  required float integrated_y           = 5;
  required float integrated_xgyro       = 6;
  required float integrated_ygyro       = 7;
  required float integrated_zgyro       = 8;
  required float temperature            = 9;
  required int32 quality                = 10;
  required int32 time_delta_distance_us = 11;
  required float distance               = 12;
}
*/

// undefine near and far macros from windows.h
#ifdef _WIN32
  #undef near
  #undef far
#endif

/// \brief Private data for OpticalFlowSensor
class custom::OpticalFlowSensorPrivate
{
  /// \brief Save an image
  /// \param[in] _data the image data to be saved
  /// \param[in] _width width of image in pixels
  /// \param[in] _height height of image in pixels
  /// \param[in] _format The format the data is in
  /// \return True if the image was saved successfully. False can mean
  /// that the path provided to the constructor does exist and creation
  /// of the path was not possible.
  /// \sa ImageSaver
  public: bool SaveImage(const float *_data, unsigned int _width,
    unsigned int _height, gz::common::Image::PixelFormatType _format);

  /// \brief Helper function to convert depth data to depth image
  /// \param[in] _data depth data
  /// \param[out] _imageBuffer resulting depth image data
  /// \param[in] _width width of image
  /// \param[in] _height height of image
  public: bool ConvertDepthToImage(const float *_data,
    unsigned char *_imageBuffer, unsigned int _width, unsigned int _height);

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

    /// \brief Rendering camera
  public: gz::rendering::DepthCameraPtr depthCamera;

  /// \brief Depth data buffer.
  public: float *depthBuffer = nullptr;

  /// \brief point cloud data buffer.
  public: float *pointCloudBuffer = nullptr;

  /// \brief xyz data buffer.
  public: float *xyzBuffer = nullptr;

  /// \brief Near clip distance.
  public: float near = 0.0;

  /// \brief Pointer to an image to be published
  public: gz::rendering::Image image;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: gz::common::EventT<
          void(const gz::msgs::Image &)> imageEvent;

  /// \brief Connection from depth camera with new depth data
  public: gz::common::ConnectionPtr depthConnection;

  /// \brief Connection from depth camera with new point cloud data
  public: gz::common::ConnectionPtr pointCloudConnection;

  /// \brief Connection to the Manager's scene change event.
  public: gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief True to save images
  public: bool saveImage = false;

  /// \brief path directory to where images are saved
  public: std::string saveImagePath = "./";

  /// \prefix of an image name
  public: std::string saveImagePrefix = "./";

  /// \brief counter used to set the image filename
  public: std::uint64_t saveImageCounter = 0;

  /// \brief SDF Sensor DOM object.
  public: sdf::Sensor sdfSensor;

  /// \brief The point cloud message.
  public: msgs::PointCloudPacked pointMsg;

  /// \brief Helper class that can fill a msgs::PointCloudPacked
  /// image and depth data.
  public: PointCloudUtil pointsUtil;

  /// \brief publisher to publish point cloud
  public: transport::Node::Publisher pointPub;
  public: OpticalFlowOpenCV *optical_flow_;
  float hfov_;
  int dt_us_;
  int output_rate_;
  float focal_length_;
  std::chrono::steady_clock::duration first_frame_time_;
  uint32_t frame_time_us_;
  bool has_gyro_;
  bool first_frame = true;
};

using namespace gz;
using namespace sensors;
using namespace custom;

//////////////////////////////////////////////////
bool OpticalFlowSensorPrivate::ConvertDepthToImage(
    const float *_data,
    unsigned char *_imageBuffer,
    unsigned int _width, unsigned int _height)
{
  float maxDepth = 0;
  for (unsigned int i = 0; i < _height * _width; ++i)
  {
    if (_data[i] > maxDepth && !std::isinf(_data[i]))
    {
      maxDepth = _data[i];
    }
  }
  double factor = 255 / maxDepth;
  for (unsigned int j = 0; j < _height * _width; ++j)
  {
    unsigned char d = static_cast<unsigned char>(255 - (_data[j] * factor));
    _imageBuffer[j * 3] = d;
    _imageBuffer[j * 3 + 1] = d;
    _imageBuffer[j * 3 + 2] = d;
  }
  return true;
}

//////////////////////////////////////////////////
bool OpticalFlowSensorPrivate::SaveImage(const float *_data,
    unsigned int _width, unsigned int _height,
    common::Image::PixelFormatType /*_format*/)
{
  // Attempt to create the directory if it doesn't exist
  if (!common::isDirectory(this->saveImagePath))
  {
    if (!common::createDirectories(this->saveImagePath))
      return false;
  }

  if (_width == 0 || _height == 0)
    return false;

  common::Image localImage;

  unsigned int depthSamples = _width * _height;
  unsigned int depthBufferSize = depthSamples * 3;

  unsigned char * imgDepthBuffer = new unsigned char[depthBufferSize];

  this->ConvertDepthToImage(_data, imgDepthBuffer, _width, _height);

  std::string filename = this->saveImagePrefix +
                         std::to_string(this->saveImageCounter) + ".png";
  ++this->saveImageCounter;

  localImage.SetFromData(imgDepthBuffer, _width, _height,
      common::Image::RGB_INT8);
  localImage.SavePNG(
      common::joinPaths(this->saveImagePath, filename));

  delete[] imgDepthBuffer;
  return true;
}

//////////////////////////////////////////////////
OpticalFlowSensor::OpticalFlowSensor()
  : CameraSensor(), dataPtr(new OpticalFlowSensorPrivate())
{
}

//////////////////////////////////////////////////
OpticalFlowSensor::~OpticalFlowSensor()
{
  this->dataPtr->depthConnection.reset();
  this->dataPtr->pointCloudConnection.reset();
  if (this->dataPtr->depthBuffer)
    delete [] this->dataPtr->depthBuffer;
  if (this->dataPtr->pointCloudBuffer)
    delete [] this->dataPtr->pointCloudBuffer;
  if (this->dataPtr->xyzBuffer)
    delete [] this->dataPtr->xyzBuffer;
}

//////////////////////////////////////////////////
bool OpticalFlowSensor::Init()
{
  return this->CameraSensor::Init();
}

//////////////////////////////////////////////////
bool OpticalFlowSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool OpticalFlowSensor::Load(const sdf::Sensor &_sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!::Sensor::Load(_sdf))
  {
    return false;
  }

  // Check if this is the right type
  // if (_sdf.Type() != sdf::SensorType::DEPTH_CAMERA)
  // {
  //   gzerr << "Attempting to a load a Depth Camera sensor, but received "
  //     << "a " << _sdf.TypeStr() << std::endl;
  // }

  auto type = gz::sensors::customType(_sdf);
  if ("optical_flow" != type)
  {
    gzerr << "Trying to load [odometer] sensor, but got type ["
           << type << "] instead." << std::endl;
    return false;
  }

  // this->dataPtr->camera.emplace();
  // Errors err = this->dataPtr->camera->Load(_sdf->GetElement("camera"));
    
  if (_sdf.CameraSensor() == nullptr)
  {
    gzerr << "Attempting to a load a Depth Camera sensor, but received "
      << "a null sensor." << std::endl;
    // return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  if (this->Topic().empty())
    this->SetTopic("/camera/opticalflow");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<msgs::Image>(
          this->Topic());
  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  gzdbg << "Depth images for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  if (!this->AdvertiseInfo())
    return false;

  // Create the point cloud publisher
  this->dataPtr->pointPub =
      this->dataPtr->node.Advertise<msgs::PointCloudPacked>(
          this->Topic() + "/points");
  if (!this->dataPtr->pointPub)
  {
    gzerr << "Unable to create publisher on topic["
      << this->Topic() + "/points" << "].\n";
    return false;
  }

  gzdbg << "Points for [" << this->Name() << "] advertised on ["
         << this->Topic() << "/points]" << std::endl;

  gzerr << "create camera1\n";
  if (this->Scene())
  {
    gzerr << "create camera2\n";
    this->CreateCamera();
  }
  this->dataPtr->sceneChangeConnection =
      RenderingEvents::ConnectSceneChangeCallback(
      std::bind(&OpticalFlowSensor::SetScene, this, std::placeholders::_1));

  this->dataPtr->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool OpticalFlowSensor::CreateCamera()
{
  gzerr << "create camera3\n";
  const sdf::Camera *cameraSdf = this->dataPtr->sdfSensor.CameraSensor();

  if (!cameraSdf)
  {
    gzerr << "Unable to access camera SDF element\n";
    return false;
  }

  int width = cameraSdf->ImageWidth();
  int height = cameraSdf->ImageHeight();

  if (width != 64 || height != 64) {
    gzerr << "Incorrect image size, must by 64 x 64.\n";
  }

  double far = cameraSdf->FarClip();
  double near = cameraSdf->NearClip();

  this->PopulateInfo(cameraSdf);

  if (!this->dataPtr->depthCamera)
  {
    gzerr << "Camera doesn't exist.\n";
  }
  this->dataPtr->depthCamera = this->Scene()->CreateDepthCamera(
      this->Name());
  if (!this->dataPtr->depthCamera)
  {
    gzerr << "Camera doesn't exist.\n";
    return false;
  }
  this->dataPtr->depthCamera->SetImageWidth(width);
  this->dataPtr->depthCamera->SetImageHeight(height);
  this->dataPtr->depthCamera->SetNearClipPlane(near);
  this->dataPtr->depthCamera->SetFarClipPlane(far);
  this->dataPtr->depthCamera->SetVisibilityMask(
      cameraSdf->VisibilityMask());

  this->AddSensor(this->dataPtr->depthCamera);

  const std::map<SensorNoiseType, sdf::Noise> noises = {
    {CAMERA_NOISE, cameraSdf->ImageNoise()},
  };

  for (const auto & [noiseType, noiseSdf] : noises)
  {
    // Add gaussian noise to camera sensor
    if (noiseSdf.Type() == sdf::NoiseType::GAUSSIAN)
    {
      this->dataPtr->noises[noiseType] =
        ImageNoiseFactory::NewNoiseModel(noiseSdf, "depth");

      std::dynamic_pointer_cast<ImageGaussianNoiseModel>(
           this->dataPtr->noises[noiseType])->SetCamera(
             this->dataPtr->depthCamera);
    }
    else if (noiseSdf.Type() != sdf::NoiseType::NONE)
    {
      gzwarn << "The depth camera sensor only supports Gaussian noise. "
       << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
       << "] is not supported." << std::endl;
    }
  }

  // Near clip plane not set because we need to be able to detect occlusion
  // from objects before near clip plane
  this->dataPtr->near = near;

  // \todo(nkoeng) these parameters via sdf
  this->dataPtr->depthCamera->SetAntiAliasing(2);

  math::Angle angle = cameraSdf->HorizontalFov();
  if (angle < 0.01 || angle > GZ_PI*2)
  {
    gzerr << "Invalid horizontal field of view [" << angle << "]\n";

    return false;
  }
  this->dataPtr->depthCamera->SetAspectRatio(static_cast<double>(width)/height);
  this->dataPtr->depthCamera->SetHFOV(angle);

  // Create depth texture when the camera is reconfigured from default values
  this->dataPtr->depthCamera->CreateDepthTexture();

  // \todo(nkoenig) Port Distortion class
  // This->dataPtr->distortion.reset(new Distortion());
  // This->dataPtr->distortion->Load(this->sdf->GetElement("distortion"));

  this->Scene()->RootVisual()->AddChild(this->dataPtr->depthCamera);

  // Create the directory to store frames
  if (cameraSdf->SaveFrames())
  {
    this->dataPtr->saveImagePath = cameraSdf->SaveFramesPath();
    this->dataPtr->saveImagePrefix = this->Name() + "_";
    this->dataPtr->saveImage = true;
  }

  this->dataPtr->depthConnection =
      this->dataPtr->depthCamera->ConnectNewDepthFrame(
      std::bind(&OpticalFlowSensor::OnNewDepthFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  this->dataPtr->pointCloudConnection =
      this->dataPtr->depthCamera->ConnectNewRgbPointCloud(
      std::bind(&OpticalFlowSensor::OnNewRgbPointCloud, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  // Initialize the point message.
  // \todo(anyone) The true value in the following function call forces
  // the xyz and rgb fields to be aligned to memory boundaries. This is need
  // by ROS1: https://github.com/ros/common_msgs/pull/77. Ideally, memory
  // alignment should be configured.
  msgs::InitPointCloudPacked(this->dataPtr->pointMsg, this->Name(), true,
      {{"xyz", msgs::PointCloudPacked::Field::FLOAT32},
       {"rgb", msgs::PointCloudPacked::Field::FLOAT32}});

  // Set the values of the point message based on the camera information.
  this->dataPtr->pointMsg.set_width(this->ImageWidth());
  this->dataPtr->pointMsg.set_height(this->ImageHeight());
  this->dataPtr->pointMsg.set_row_step(
      this->dataPtr->pointMsg.point_step() * this->ImageWidth());

  this->dataPtr->focal_length_ = (width/2)/tan(angle.Radian()/2);
  this->dataPtr->output_rate_ = 20;

  this->dataPtr->optical_flow_ = new OpticalFlowOpenCV(this->dataPtr->focal_length_, this->dataPtr->focal_length_, this->dataPtr->output_rate_);

  return true;
}

/////////////////////////////////////////////////
void OpticalFlowSensor::OnNewDepthFrame(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &_format)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  unsigned int depthSamples = _width * _height;
  unsigned int depthBufferSize = depthSamples * sizeof(float);

  common::Image::PixelFormatType format =
    common::Image::ConvertPixelFormat(_format);

  if (!this->dataPtr->depthBuffer)
    this->dataPtr->depthBuffer = new float[depthSamples];

  memcpy(this->dataPtr->depthBuffer, _scan, depthBufferSize);

  // Save image
  if (this->dataPtr->saveImage)
  {
    this->dataPtr->SaveImage(_scan, _width, _height,
        format);
  }
}

/////////////////////////////////////////////////
void OpticalFlowSensor::OnNewRgbPointCloud(const float *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int _channels,
                    const std::string &/*_format*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  unsigned int pointCloudSamples = _width * _height;
  unsigned int pointCloudBufferSize = pointCloudSamples * _channels *
      sizeof(float);

  if (!this->dataPtr->pointCloudBuffer)
    this->dataPtr->pointCloudBuffer = new float[pointCloudSamples * _channels];

  memcpy(this->dataPtr->pointCloudBuffer, _scan, pointCloudBufferSize);
}

/////////////////////////////////////////////////
rendering::DepthCameraPtr OpticalFlowSensor::DepthCamera() const
{
  return this->dataPtr->depthCamera;
}

/////////////////////////////////////////////////
common::ConnectionPtr OpticalFlowSensor::ConnectImageCallback(
    std::function<void(const msgs::Image &)> _callback)
{
  return this->dataPtr->imageEvent.Connect(_callback);
}

/////////////////////////////////////////////////
void OpticalFlowSensor::SetScene(rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    // TODO(anyone) Remove camera from scene
    this->dataPtr->depthCamera = nullptr;
    RenderingSensor::SetScene(_scene);

    if (this->dataPtr->initialized)
      this->CreateCamera();
  }
}

//////////////////////////////////////////////////
bool OpticalFlowSensor::Update(
  const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("OpticalFlowSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->depthCamera)
  {
    gzerr << "Camera doesn't exist.\n";
    return false;
  }

  if (this->HasInfoConnections())
  {
    // publish the camera info message
    this->PublishInfo(_now);
  }

  if (!this->HasDepthConnections() && !this->HasPointConnections())
  {
    return false;
  }

  // generate sensor data
  this->Render();

  unsigned int width = this->dataPtr->depthCamera->ImageWidth();
  unsigned int height = this->dataPtr->depthCamera->ImageHeight();

  auto msgsFormat = msgs::PixelFormatType::R_FLOAT32;

  // create message
  msgs::Image msg;
  msg.set_width(width);
  msg.set_height(height);
  msg.set_step(width * rendering::PixelUtil::BytesPerPixel(
               rendering::PF_FLOAT32_R));
  msg.set_pixel_format_type(msgsFormat);
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);

  auto* frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->FrameId());

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  msg.set_data(this->dataPtr->depthBuffer,
      rendering::PixelUtil::MemorySize(rendering::PF_FLOAT32_R,
      width, height));

  this->AddSequence(msg.mutable_header(), "default");
  this->dataPtr->pub.Publish(msg);




  //optical_flow
  if(this->dataPtr->first_frame)
  {
    this->dataPtr->first_frame_time_ = _now;
    this->dataPtr->first_frame = false;
  }
  
  std::chrono::steady_clock::duration frame_time = _now;

  this->dataPtr->frame_time_us_ = (uint32_t)(std::chrono::duration_cast<std::chrono::duration<float>>(frame_time - this->dataPtr->first_frame_time_).count() * 1e6); //since start

  float flow_x_ang = 0.0f;
  float flow_y_ang = 0.0f;

  this->dataPtr->ConvertDepthToImage(this->dataPtr->depthBuffer,
    this->dataPtr->image.Data<unsigned char>(), width, height);

  unsigned char * _image = this->dataPtr->image.Data<unsigned char>();

  int quality = this->dataPtr->optical_flow_->calcFlow((uchar*)_image, this->dataPtr->frame_time_us_, this->dataPtr->dt_us_, flow_x_ang, flow_y_ang);

  msgs::OpticalFlow opticalFlow_message;
  // opticalFlow_message.set_time_usec(now.Double() * 1e6);
  *opticalFlow_message.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  opticalFlow_message.set_sensor_id(2.0);
  opticalFlow_message.set_integration_time_us(quality ? this->dataPtr->dt_us_ : 0);
  opticalFlow_message.set_integrated_x(quality ? flow_x_ang : 0.0f);
  opticalFlow_message.set_integrated_y(quality ? flow_y_ang : 0.0f);
  
  opticalFlow_message.set_integrated_xgyro(NAN);
  opticalFlow_message.set_integrated_ygyro(NAN);
  opticalFlow_message.set_integrated_zgyro(NAN);

  opticalFlow_message.set_temperature(20.0f);
  opticalFlow_message.set_quality(quality);
  opticalFlow_message.set_time_delta_distance_us(0);
  opticalFlow_message.set_distance(0.0f); //get real values in gazebo_mavlink_interface.cpp
  //send message
  this->dataPtr->pub.Publish(opticalFlow_message);// opticalFlow_pub_->Publish(opticalFlow_message);




  if (this->dataPtr->imageEvent.ConnectionCount() > 0u)
  {
    // Trigger callbacks.
    try
    {
      this->dataPtr->imageEvent(msg);
    }
    catch(...)
    {
      gzerr << "Exception thrown in an image callback.\n";
    }
  }

  // if (this->HasPointConnections() &&
  //     this->dataPtr->pointCloudBuffer)
  if (this->HasPointConnections() &&
      this->dataPtr->pointCloudBuffer)
  {
    // Set the time stamp
    *this->dataPtr->pointMsg.mutable_header()->mutable_stamp() =
      msgs::Convert(_now);
    this->dataPtr->pointMsg.set_is_dense(true);

    if (!this->dataPtr->xyzBuffer)
      this->dataPtr->xyzBuffer = new float[width*height*3];

    if (this->dataPtr->image.Width() != width
        || this->dataPtr->image.Height() != height)
    {
      this->dataPtr->image =
          rendering::Image(width, height, rendering::PF_R8G8B8);
    }

    // extract image data from point cloud data
    this->dataPtr->pointsUtil.XYZFromPointCloud(
        this->dataPtr->xyzBuffer,
        this->dataPtr->pointCloudBuffer,
        width, height);

    // convert depth to grayscale rgb image
    this->dataPtr->ConvertDepthToImage(this->dataPtr->depthBuffer,
        this->dataPtr->image.Data<unsigned char>(), width, height);

    // fill the point cloud msg with data from xyz and rgb buffer
    this->dataPtr->pointsUtil.FillMsg(this->dataPtr->pointMsg,
        this->dataPtr->xyzBuffer,
        this->dataPtr->image.Data<unsigned char>());

    this->AddSequence(this->dataPtr->pointMsg.mutable_header(), "pointMsg");
    this->dataPtr->pointPub.Publish(this->dataPtr->pointMsg);
  }
  return true;
}

//////////////////////////////////////////////////
unsigned int OpticalFlowSensor::ImageWidth() const
{
  return this->dataPtr->depthCamera->ImageWidth();
}

//////////////////////////////////////////////////
unsigned int OpticalFlowSensor::ImageHeight() const
{
  return this->dataPtr->depthCamera->ImageHeight();
}

//////////////////////////////////////////////////
double OpticalFlowSensor::FarClip() const
{
  return this->dataPtr->depthCamera->FarClipPlane();
}

//////////////////////////////////////////////////
double OpticalFlowSensor::NearClip() const
{
  return this->dataPtr->near;
}

//////////////////////////////////////////////////
bool OpticalFlowSensor::HasConnections() const
{
  return this->HasDepthConnections() || this->HasPointConnections() ||
      this->HasInfoConnections();
}

//////////////////////////////////////////////////
bool OpticalFlowSensor::HasDepthConnections() const
{
  return (this->dataPtr->pub && this->dataPtr->pub.HasConnections())
         || this->dataPtr->imageEvent.ConnectionCount() > 0u;
}

//////////////////////////////////////////////////
bool OpticalFlowSensor::HasPointConnections() const
{
  return this->dataPtr->pointPub && this->dataPtr->pointPub.HasConnections();
}
