/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * Neo ISP pipeline utilities
 */

#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>
#include <libcamera/orientation.h>

#include "libcamera/internal/media_device.h"
#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

namespace nxpneo {

struct CameraProperties {
	bool image1Stream;
	bool eDataStream;
	bool rgbirCfa;
	std::optional<unsigned int> formatBpp;
	std::optional<Size> formatSize;
	std::optional<Orientation> orientation;
	std::optional<utils::Duration> controlsDelay;
};

struct GlobalInfo {
	static constexpr unsigned int kBufferCount = 4;
	GlobalInfo()
		: bufferCount(kBufferCount) {}

	unsigned int bufferCount;
};

class PipelineConfig
{
public:
	PipelineConfig() = default;
	~PipelineConfig() = default;

	int load(const std::string &file);

	const GlobalInfo &globalInfo() const;
	const CameraProperties &cameraProperties(const std::string &name,
						 const std::string &model);

private:
	int parseCameras(const YamlObject &cameras);
	int parseGlobal(const YamlObject &global);
	int loadFileConfig(const std::string &file);

	std::map<std::string, CameraProperties> camPropertiesMap_;
	GlobalInfo globalInfo_;
};

std::vector<MediaEntity *> locateSensors(MediaDevice *media);

} // namespace nxpneo

} // namespace libcamera
