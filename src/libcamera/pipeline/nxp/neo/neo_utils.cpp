/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * Neo ISP pipeline utilities
 */

#include "neo_utils.h"

#include <limits>
#include <regex>
#include <set>
#include <sstream>
#include <string>

#include <linux/v4l2-subdev.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/yaml_parser.h"

#include "isi_device.h"

using namespace std::chrono_literals;

namespace libcamera {

LOG_DECLARE_CATEGORY(NxpNeoPipe)

namespace nxpneo {

/**
 * \struct CameraProperties
 * \brief Camera properties defined by topology discovery or configuration file
 *
 * This structure reports to the pipeline handler a set of properties coming
 * from the platform configuration file and the graph discovery.
 *
 * \var CameraProperties::image1Stream
 * \brief Camera has an image1 stream for HDR or RGBIr context switch mode
 *
 * \var CameraProperties::eDataStream
 * \brief Camera has a dedicated stream for embedded data
 *
 * \var CameraProperties::multiCamera
 * \brief Camera is sharing MIPI-CSI port with other cameras
 *
 * This flag reports that the camera is sharing its MIPI-CSI port with other
 * cameras which induces some limitations in the capability of the front-end
 * graph to be reconfigured after startup.
 *
 * \var CameraProperties::formatBpp
 * \brief Format bit-per-pixel filter value (optional)
 *
 * This parameter restricts the camera formats exposed to the user to the subset
 * of formats whose mbus-code bit-per-pixel matches this value.
 *
 * \var CameraProperties::formatSize
 * \brief Camera format size filter value (optional)
 *
 * This parameter restricts the camera formats exposed to the user to the subset
 * of formats whose size matches this value.
 *
 * \var CameraProperties::orientation
 * \brief Camera orientation (optional)
 *
 * This parameter defines the preferred orientation to be used for the camera
 * streams. Range of values is the subset of orientations defined by the
 * Orientation enum class, relevant to the ones achievable with a combination of
 * horizontal and vertical flips:
 * Rotate0 (1), Rotate0Mirror (2), Rotate180 (3), Rotate180Mirror (4)
 *
 * \var CameraProperties::controlsDelay
 * \brief Delay to update the controls on front-end frame done event (optional)
 *
 * Camera controls update is synchronized on the front-end frame done events.
 * For cameras having issue with that timing, this allows delaying the controls
 * update by a user configured delay.
 *
 */

/**
 * \brief Get the sizes supported by a camera for a mbus code
 * \param[in] sensor The camera sensor
 * \param[in] code The mbus code
 * \param[out] sizes The sizes supported by \a sensor for \a code
 *
 * This is essentially a wrapper around the CameraSensor::sizes() function.
 * It introduces a workaround for external ISP that exposes a single
 * (min, max) size range corresponding to its full rescaling range capability.
 * The CameraSensor::sizes() function reports only the max value of the range
 * that may exceed the width limit of the ISI or ISP devices.
 * For that case, make sure that at least the sensor native resolution is
 * reported in the sizes list, to be considered as a valid size option.
 */
void cameraSizes(CameraSensor *sensor, int code, std::vector<Size> &sizes)
{
	sizes.clear();
	sizes = sensor->sizes(code);
	Size resolution = sensor->resolution();
	if (sizes.size() == 1 &&
	    std::find(sizes.begin(), sizes.end(), resolution) == sizes.end()) {
		sizes.push_back(std::move(resolution));
	}
	std::sort(sizes.begin(), sizes.end());
}

/* -----------------------------------------------------------------------------
 * CameraInfo class
 */

/**
 * \brief Return an optional CameraMediaStream for the camera
 * \param[in] streamType The CameraInfo stream identifier
 * \return The CameraMediaStream if it exists, nullptr otherwise
 */
const CameraMediaStream *CameraInfo::stream(StreamType streamType) const
{
	auto it = streams_.find(streamType);
	if (it != streams_.end())
		return &it->second;
	else
		return nullptr;
}

/* -----------------------------------------------------------------------------
 * PipelineConfig class
 */

/**
 * \brief Class destructor
 */
PipelineConfig::~PipelineConfig()
{
	/* Release allocated ISI channels */
	if (!isiDevice_)
		return;

	for (auto &[name, cameraInfo] : cameraMap_) {
		for (const auto &[_unused, cameraMediaStream] : cameraInfo.streams_)
			isiDevice_->releasePipe(cameraMediaStream.pipe());
	}
}

/**
 * \brief Load the pipeline configuration
 * \param[in] file The path to the pipeline configuration file
 * \param[in] isiDevice The ISI Device associated to the media controller device
 *
 * Build the pipeline configuration that consists in the parameters that may be
 * defined in the pipeline handler configuration file and the camera streams
 * graphs that are dynamically discovered in the media device.
 *
 * \return 0 on success or a negative error code otherwise
 */
int PipelineConfig::load(const std::string &filename, std::shared_ptr<ISIDevice> isiDevice)
{
	isiDevice_ = isiDevice;

	int ret = loadFileConfig(filename);
	if (ret)
		LOG(NxpNeoPipe, Info) << "Could not parse config file " << filename;

	ret = loadAutoDetect();
	return ret;
}

/**
 * \brief Report the CameraInfo associated to a camera
 * \param[in] name The name of the camera media device entity
 *
 * The CameraInfo structure carries information related to the integration
 * of the sensor into the media device, coming from the platform configuration
 * file and the graph discovery.
 *
 * \return The pointer to CameraInfo structure if it exists, nullptr otherwise
 */
const CameraInfo *PipelineConfig::cameraInfo(const std::string &name) const
{
	auto iter = cameraMap_.find(name);

	if (iter != cameraMap_.end())
		return &iter->second;
	else
		return nullptr;
}

/**
 * \brief Report the global routes for the frontend media controller device
 *
 * Routing is to be configured for entities that support streams.
 * RoutingMap is a <subdevice, routing> map that associates to each relevant
 * subdevice the corresponding list of routes to be applied.
 *
 * \return A reference to the RoutingMap
 */
const RoutingMap &PipelineConfig::routingMap() const
{
	return routingMap_;
}

/**
 * \brief Report the global pipeline handler configuration
 *
 * This function reports the global pipeline handler configuration that is not
 * specific to a given camera.
 *
 * \return A reference to the global configuration
 */
const GlobalInfo &PipelineConfig::globalInfo() const
{
	return globalInfo_;
}

/**
 * \brief Get the camera properties for a given camera name or model
 * \param[in] name The name of the camera media device entity
 * \param[in] model The model name of the camera sensor
 *
 * This function retrieves the CameraProperties structure associated with a
 * camera, giving precedence to name-based lookup over model-based lookup.
 * If neither the name nor the model is found in the properties map, a new
 * default CameraProperties entry is created and inserted for the model.
 *
 * \return A reference to the CameraProperties structure
 */
const CameraProperties &
PipelineConfig::cameraProperties(const std::string &name,
				 const std::string &model)

{
	auto itName = camPropertiesMap_.find(name);
	if (itName != camPropertiesMap_.end())
		return itName->second;
	auto itModel = camPropertiesMap_.find(model);
	if (itModel != camPropertiesMap_.end())
		return itModel->second;
	const auto [it, inserted] =
		camPropertiesMap_.emplace(model, CameraProperties{});
	return it->second;
}

/**
 * \brief Discover the valid camera graphs to the capture video device
 *
 * For every camera sensor in the media device, look for valid media links paths
 * to the capture video device. Also, build the aggregated global routing table
 * for all the cameras detected.
 *
 * \return 0 on success or a negative error code otherwise
 */
int PipelineConfig::loadAutoDetect()
{
	int ret;

	MediaDevice *media = isiDevice_->media().get();
	if (!media)
		return -EINVAL;

	/* Map aggregating stream identifiers for all pads of the media device */
	PadStreamsMap globalStreamMap;

	/*
	 * Build a set of the sensors entities from the media device to have an
	 * ordered list by name and guarantee a consistent topology detection.
	 * the usual sensor naming convention is 'model xx-yyyy'
	 * where xx is the i2c bus number and yyyy is the bus address.
	 * Sensors set ordering is defined with the criteria below:
	 * - simple alphabetical order when above convention is not used or
	 *   for different sensor model
	 * - increasing i2c bus for identical sensor model on different i2c bus
	 * - increasing i2c address for identical sensor model on the same bus
	 */
	auto compareName =
		[](MediaEntity *a, MediaEntity *b) {
			std::regex r("(\\S+) (\\d+)-(\\d+)");
			const std::string &nameA = a->name();
			const std::string &nameB = b->name();
			std::smatch matchesA, matchesB;
			if (std::regex_search(nameA, matchesA, r) &&
			    std::regex_search(nameB, matchesB, r)) {
				const std::string &modelA = matchesA[1];
				const std::string &modelB = matchesB[1];
				unsigned i2cBusA = std::stoul(matchesA[2]);
				unsigned i2cBusB = std::stoul(matchesB[2]);
				unsigned i2cAddrA = std::stoul(matchesA[3]);
				unsigned i2cAddrB = std::stoul(matchesB[3]);
				if (modelA != modelB)
					return nameA < nameB;
				else if (i2cBusA != i2cBusB)
					return i2cBusA < i2cBusB;
				else
					return i2cAddrA < i2cAddrB;
			} else {
				return nameA < nameB;
			}
		};

	/*
	 * Merge the routings for entities in the delta map, into the base
	 * map of routings.
	 */
	auto mergeRoutingMap =
		[](RoutingMap &base, RoutingMap &delta) {
			for (auto &[entity, routing] : delta) {
				if (base.count(entity)) {
					V4L2Subdevice::Routing &origin = base[entity];
					origin.reserve(origin.size() + routing.size());
					std::move(routing.begin(), routing.end(),
						  std::back_inserter(origin));
				} else {
					base[entity] = std::move(routing);
				}
			}
		};

	std::vector<MediaEntity *> sensorsEntities = locateSensors(media);
	std::sort(sensorsEntities.begin(), sensorsEntities.end(), compareName);

	/* Discover the topology of every sensor */
	ISIDevice *isiDevice = isiDevice_.get();
	for (MediaEntity *entity : sensorsEntities) {
		LOG(NxpNeoPipe, Debug) << "Auto detect camera " << entity->name();

		CameraInfo cameraInfo = {};

		std::unique_ptr<CameraSensor> sensor =
			CameraSensorFactoryBase::create(entity);
		if (!sensor) {
			LOG(NxpNeoPipe, Warning)
				<< "Could not construct camera sensor "
				<< entity->name();
			continue;
		}

		const std::string &name = sensor->entity()->name();
		const std::string &model = sensor->model();
		cameraInfo.properties_ = &cameraProperties(name, model);

		/* Select max sensor resolution compatible with an ISI pipe. */
		Size sizeMax;
		std::set<Size> sizesSet;
		const std::vector<unsigned int> mbusCodes = sensor->mbusCodes();
		for (const auto mbusCode : sensor->mbusCodes()) {
			std::vector<Size> sizes;
			cameraSizes(sensor.get(), mbusCode, sizes);
			for (const auto &size : sizes) {
				if (size.width <= ISIPipe::kChainedWidthMax)
					sizesSet.insert(size);
			}
		}
		if (sizesSet.size())
			sizeMax = *sizesSet.rbegin();
		else
			continue;

		/* Copy of the global streams map - revert changes in case of error */
		PadStreamsMap streamMap(globalStreamMap);

		for (StreamType stream : kStreamTypes) {
			V4L2Subdevice::Stream sensorStream;
			if (stream == StreamTypeImage0) {
				sensorStream = sensor->imageStream();
			} else if (stream == StreamTypeImage1) {
				bool enable = cameraInfo.properties_->image1Stream;
				if (!enable)
					continue;
				if (!sensor->auxiliaryStream().has_value()) {
					LOG(NxpNeoPipe, Warning)
						<< "Sensor has no auxiliary stream, image1 disabled";
					cameraInfo.properties_->image1Stream = false;
					continue;
				}
				sensorStream = sensor->auxiliaryStream().value();
			} else if (stream == StreamTypeEData) {
				bool enable = cameraInfo.properties_->eDataStream;
				if (!enable)
					continue;
				if (!sensor->embeddedDataStream().has_value()) {
					LOG(NxpNeoPipe, Warning)
						<< "Sensor has no embedded data stream, edata disabled";
					cameraInfo.properties_->eDataStream = false;
					continue;
				}
				sensorStream = sensor->embeddedDataStream().value();
			} else {
				LOG(NxpNeoPipe, Warning) << "Invalid sensor stream";
				return -EINVAL;
			}

			ISIPipe *isiPipe = isiDevice->reservePipe(sizeMax.width);
			if (!isiPipe) {
				LOG(NxpNeoPipe, Warning) << "Input pipe allocation failed";
				goto error;
			}
			auto [it, inserted] =
				cameraInfo.streams_.emplace(stream, isiPipe);
			CameraMediaStream &cameraMediaStream = it->second;

			const std::string &pipeName = isiPipe->videoDeviceName();
			MediaEntity *videoDevEntity = media->getEntityByName(pipeName);
			if (!videoDevEntity) {
				LOG(NxpNeoPipe, Warning)
					<< "Could not find video device entity for pipe";
				goto error;
			}

			ret = cameraMediaStream.streamGraph().init(
				entity, sensorStream, videoDevEntity, &streamMap);
			if (ret) {
				LOG(NxpNeoPipe, Info)
					<< "Failed to initialize stream graph for camera "
					<< sensor->model();
				goto error;
			}
		}

		/*
		 * CameraInfo successfully created
		 * - Merge camera streams routings to global routing
		 * - Store resulting entry into cameras database
		 * - Update the global streams map with the camera streams
		 */
		for (auto &[_unused, cameraMediaStream] : cameraInfo.streams_) {
			RoutingMap cameraRoutingMap(cameraMediaStream.streamGraph().routings());
			mergeRoutingMap(routingMap_, cameraRoutingMap);
		}

		cameraMap_[entity->name()] = std::move(cameraInfo);
		globalStreamMap = std::move(streamMap);

		continue;

	error:
		for (const auto &[_unused, cameraMediaStream] : cameraInfo.streams_)
			isiDevice->releasePipe(cameraMediaStream.pipe());
	}

	/* Finally, detect multi-camera conditions */
	loadAutoDetectMultiCamera();

	return cameraMap_.size() ? 0 : -EINVAL;
}

/**
 * \brief Detect cases where a MIPI CSI-2 port is shared by multiple cameras
 *
 * When the same MIPI CSI-2 port is shared by multiple cameras typically through
 * the usage of a SerDes, some restrictions apply regarding the allowed
 * configurations and transitions supported by the front-end media device.
 * The multi-camera use case is detected by checking if any two cameras share
 * components in their stream graphs. The CameraProperties structures of those
 * cameras are updated to reflect that condition so that the pipeline handler
 * knows about it.
 *
 * \return 0 on success or a negative error code otherwise
 */
int PipelineConfig::loadAutoDetectMultiCamera()
{
	for (auto &[name, cameraInfoLeft] : cameraMap_) {
		const CameraMediaStream *cameraMediaStreamLeft =
			cameraInfoLeft.stream(StreamTypeImage0);
		if (!cameraMediaStreamLeft) {
			LOG(NxpNeoPipe, Error) << "No image0 stream for camera";
			return -EINVAL;
		}
		const StreamGraph streamGraphLeft =
			cameraMediaStreamLeft->streamGraph();

		bool shared = false;
		for (auto &[__unused, cameraInfoRight] : cameraMap_) {
			if (&cameraInfoRight == &cameraInfoLeft)
				continue;
			const CameraMediaStream *cameraMediaStreamRight =
				cameraInfoRight.stream(StreamTypeImage0);
			if (!cameraMediaStreamRight) {
				LOG(NxpNeoPipe, Error) << "No image0 stream for camera";
				return -EINVAL;
			}
			const StreamGraph streamGraphRight =
				cameraMediaStreamRight->streamGraph();

			if (streamGraphLeft.isShared(&streamGraphRight)) {
				shared = true;
				break;
			}
		}

		cameraInfoLeft.properties_->multiCamera = shared;
		LOG(NxpNeoPipe, Debug)
			<< "Camera " << name << " shared " << shared;
	}

	return 0;
}

/*
 * ---------------------------- Config file parsing ----------------------------
 */

/**
 * \brief Parse the cameras section in the yaml configuration file
 * \param[in] cameras The cameras node in yaml file
 * \return 0 if no error was detected, a negative error code otherwise
 */
int PipelineConfig::parseCameras(const YamlObject &cameras)
{
	for (const auto &cameraObj : cameras.asList()) {
		CameraProperties properties = {};

		const YamlObject &modelObj = cameraObj["model"];
		std::string model = modelObj.get<std::string>().value_or("");

		const YamlObject &entityObj = cameraObj["entity"];
		std::string entity = entityObj.get<std::string>().value_or("");

		const YamlObject &streamsObj = cameraObj["streams"];
		for (const auto &streamObj : streamsObj.asList()) {
			std::string stream =
				streamObj.get<std::string>().value_or("");
			if (stream == "image1")
				properties.image1Stream = true;
			else if (stream == "edata")
				properties.eDataStream = true;
		}

		const YamlObject &fmtObj = cameraObj["format"];
		properties.formatBpp = fmtObj["bpp"].get<uint32_t>();
		properties.formatSize = fmtObj["size"].get<Size>();

		const YamlObject &rgbirCfaObj = cameraObj["rgbir-cfa"];
		if (rgbirCfaObj.isValue())
			properties.rgbirCfa = rgbirCfaObj.get<bool>().value_or(false);

		const YamlObject &orientationObj = cameraObj["orientation"];
		if (orientationObj.isValue()) {
			uint32_t orientation = orientationObj.get<uint32_t>().value_or(0);
			if (orientation >= static_cast<uint32_t>(Orientation::Rotate0) &&
			    orientation <= static_cast<uint32_t>(Orientation::Rotate180Mirror))
				properties.orientation = static_cast<Orientation>(orientation);
			else
				LOG(NxpNeoPipe, Warning)
					<< "Invalid orientation value " << orientation;
		}

		const YamlObject &controlsDelayObj = cameraObj["controls-delay"];
		uint32_t controlsDelay = controlsDelayObj.get<uint32_t>().value_or(0);
		if (controlsDelay)
			properties.controlsDelay = controlsDelay * 1ms;

		LOG(NxpNeoPipe, Debug)
			<< "Camera entry model [" << model
			<< "] entity [" << entity
			<< "] streams image1 " << properties.image1Stream
			<< " edata " << properties.eDataStream;

		if (!model.length() && !entity.length()) {
			LOG(NxpNeoPipe, Warning)
				<< "Camera needs model or entity definition";
			continue;
		}

		if (model.length()) {
			if (camPropertiesMap_.count(model)) {
				LOG(NxpNeoPipe, Warning) <<
					"Duplicate camera model " << model;
				continue;
			}
			camPropertiesMap_[model] = properties;
		}

		if (entity.length()) {
			if (camPropertiesMap_.count(entity)) {
				LOG(NxpNeoPipe, Warning) <<
					"Duplicate camera entity " << entity;
				continue;
			}
			camPropertiesMap_[entity] = properties;
		}
	}

	return 0;
}

/**
 * \brief Parse the global section in the yaml configuration file
 * \param[in] global The global node in yaml file
 * \return 0 if no error was detected, a negative error code otherwise
 */
int PipelineConfig::parseGlobal(const YamlObject &global)
{
	const YamlObject &bufferCountObj = global["buffer-count"];
	globalInfo_.bufferCount =
		bufferCountObj.get<unsigned int>().value_or(GlobalInfo::kBufferCount);

	return 0;
}

/**
 * \brief Load the pipeline configuration from a pipeline configuration file
 * \param[in] filename The path to configuration file
 * \return 0 if config file was parsed correctly, a negative error code otherwise
 */
int PipelineConfig::loadFileConfig(const std::string &filename)
{
	File file(filename);

	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(NxpNeoPipe, Info)
			<< "Failed to open pipeline config file" << filename;
		return -ENOENT;
	}

	std::unique_ptr<YamlObject> root = YamlParser::parse(file);
	if (!root) {
		LOG(NxpNeoPipe, Warning)
			<< "Failed to parse pipeline config file " << filename;
		return -EINVAL;
	}

	double version = (*root)["version"].get<double>().value_or(0.0);
	if (version != 1.0) {
		LOG(NxpNeoPipe, Warning)
			<< "Unexpected pipeline config file version "
			<< version;
		return -EINVAL;
	}

	LOG(NxpNeoPipe, Debug) << "Parsing pipeline config file " << filename;

	const YamlObject &global = (*root)["global"];
	int ret = parseGlobal(global);
	if (ret)
		LOG(NxpNeoPipe, Warning)
			<< "Invalid global section in config file";

	const YamlObject &cameras = (*root)["cameras"];
	ret = parseCameras(cameras);
	if (ret)
		LOG(NxpNeoPipe, Warning)
			<< "Invalid cameras section in config file";

	return ret;
}

/**
 * \brief Locate the sensors (or ISP) from a media device
 *
 * Locate the media entities from the media devices acting as a sensor, either
 * the sensor itself or the external ISP bundled to that sensor.
 * This function is copied from the simple pipeline.
 *
 * \return A vector of media entities acting as a sensor
 */
std::vector<MediaEntity *> locateSensors(MediaDevice *media)
{
	std::vector<MediaEntity *> entities;

	/*
	 * Gather all the camera sensor entities based on the function they
	 * expose.
	 */
	for (MediaEntity *entity : media->entities()) {
		if (entity->function() == MEDIA_ENT_F_CAM_SENSOR)
			entities.push_back(entity);
	}

	if (entities.empty())
		return {};

	/*
	 * Sensors can be made of multiple entities. For instance, a raw sensor
	 * can be connected to an ISP, and the combination of both should be
	 * treated as one sensor. To support this, as a crude heuristic, check
	 * the downstream entity from the camera sensor, and if it is an ISP,
	 * use it instead of the sensor.
	 */
	std::vector<MediaEntity *> sensors;

	for (MediaEntity *entity : entities) {
		/*
		 * Locate the downstream entity by following the first link
		 * from a source pad.
		 */
		const MediaLink *link = nullptr;

		for (const MediaPad *pad : entity->pads()) {
			if ((pad->flags() & MEDIA_PAD_FL_SOURCE) &&
			    !pad->links().empty()) {
				link = pad->links()[0];
				break;
			}
		}

		if (!link)
			continue;

		MediaEntity *remote = link->sink()->entity();
		if (remote->function() == MEDIA_ENT_F_PROC_VIDEO_ISP)
			sensors.push_back(remote);
		else
			sensors.push_back(entity);
	}

	/*
	 * Remove duplicates, in case multiple sensors are connected to the
	 * same ISP.
	 */
	std::sort(sensors.begin(), sensors.end());
	auto last = std::unique(sensors.begin(), sensors.end());
	sensors.erase(last, sensors.end());

	return sensors;
}

} // namespace nxpneo

} // namespace libcamera
