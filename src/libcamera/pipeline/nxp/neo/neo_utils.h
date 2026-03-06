/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * Neo ISP pipeline utilities
 */

#pragma once

#include <map>

#include <linux/v4l2-subdev.h>

#include <libcamera/base/utils.h>

#include <libcamera/orientation.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/yaml_parser.h"

#include "isi_device.h"
#include "media_graph.h"

namespace libcamera {

namespace nxpneo {

void cameraSizes(CameraSensor *sensor, int code, std::vector<Size> &sizes);

class PipelineConfig;

class CameraMediaStream
{
public:
	CameraMediaStream(ISIPipe *pipe)
		: isiPipe_(pipe) {}

	ISIPipe *pipe() const { return isiPipe_; }
	const StreamGraph &streamGraph() const { return streamGraph_; }
	StreamGraph &streamGraph() { return streamGraph_; }

private:
	ISIPipe *isiPipe_ = nullptr;
	StreamGraph streamGraph_;
};

struct CameraProperties {
	bool image1Stream;
	bool eDataStream;
	bool multiCamera;
	bool rgbirCfa;
	std::optional<unsigned int> formatBpp;
	std::optional<Size> formatSize;
	std::optional<Orientation> orientation;
	std::optional<utils::Duration> controlsDelay;
};

enum StreamType {
	StreamTypeImage0 = 0,
	StreamTypeImage1,
	StreamTypeEData,
};

constexpr std::array<StreamType, 3>
	kStreamTypes = { StreamTypeImage0, StreamTypeImage1, StreamTypeEData };

class CameraInfo
{
public:
	CameraInfo() {}
	virtual ~CameraInfo() {}

	const CameraMediaStream *stream(StreamType streamType) const;
	bool hasStream(StreamType streamType) const { return stream(streamType); }

	const CameraProperties &cameraProperties() const { return *properties_; }

private:
	std::map<StreamType, CameraMediaStream> streams_;
	CameraProperties *properties_ = nullptr;

	friend PipelineConfig;
};

using RoutingMap = std::map<MediaEntity *, V4L2Subdevice::Routing>;
using CameraMap = std::map<std::string, CameraInfo>;

struct GlobalInfo {
	static constexpr unsigned int kBufferCount = 4;
	GlobalInfo()
		: bufferCount(kBufferCount) {}

	unsigned int bufferCount;
};

class PipelineConfig
{
public:
	PipelineConfig() {}
	virtual ~PipelineConfig();
	int load(const std::string &file, std::shared_ptr<ISIDevice> isiDevice);
	const CameraInfo *cameraInfo(const std::string &name) const;
	const RoutingMap &routingMap() const;
	const GlobalInfo &globalInfo() const;

private:
	static constexpr unsigned int kPadAny =
		std::numeric_limits<unsigned int>::max();

	int loadAutoDetect();
	int loadAutoDetectMultiCamera();

	int parseCameras(const YamlObject &cameras);
	int parseGlobal(const YamlObject &global);

	int loadFileConfig(const std::string &file);

	RoutingMap routingMap_;
	CameraMap cameraMap_;
	std::shared_ptr<ISIDevice> isiDevice_;

	std::map<std::string, CameraProperties> namePropertiesMap_;
	std::map<std::string, CameraProperties> modelPropertiesMap_;

	/* Configuration file routes sequence elements */
	enum {
		ROUTE_SINK_PAD = 0,
		ROUTE_SINK_STREAM,
		ROUTE_SOURCE_PAD,
		ROUTE_SOURCE_STREAM,
		ROUTE_FLAGS,
		ROUTE_MAX,
	};

	GlobalInfo globalInfo_;
};

std::vector<MediaEntity *> locateSensors(MediaDevice *media);

} // namespace nxpneo

} // namespace libcamera
