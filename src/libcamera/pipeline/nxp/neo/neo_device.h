/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * Neo ISP device
 *
 * Based on Intel IPU3 ImgU
 *     src/libcamera/pipeline/ipu3/imgu.h
 * Copyright (C) 2019, Google Inc.
 */

#pragma once

#include <memory>
#include <string>

#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class FrameBuffer;
class MediaDevice;
class Size;
struct StreamConfiguration;

class NeoDevice
{
public:
	static constexpr unsigned int kRawWidthMax = 4096;
	static constexpr unsigned int kWidthAlignment = 16;

	struct PipeConfig {
		unsigned int topLines;
	};

	enum class VideoDevice {
		Input0,
		Input1,
		Params,
		Frame,
		Ir,
		Stats,
	};

	NeoDevice() = default;
	~NeoDevice() = default;

	int init(MediaDevice *media);

	int allocateBuffers(unsigned int bufferCount);
	void freeBuffers();

	int configure(const PipeConfig &pipeConfig,
		      const std::map<VideoDevice, V4L2DeviceFormat *> &formats);

	int start();
	int stop();

	V4L2VideoDevice *videoDevice(VideoDevice device) const;
	static const std::string &driverName();
	static const std::string &subdeviceName();
	static const std::string &videoDeviceName(VideoDevice device);

	const std::vector<PixelFormat> &capturePixelFormats(VideoDevice device) const;
	static const std::vector<V4L2PixelFormat> &outputFormats(VideoDevice device);

	std::unique_ptr<V4L2Subdevice> isp_;
	std::unique_ptr<V4L2VideoDevice> input0_;
	std::unique_ptr<V4L2VideoDevice> input1_;
	std::unique_ptr<V4L2VideoDevice> params_;
	std::unique_ptr<V4L2VideoDevice> frame_;
	std::unique_ptr<V4L2VideoDevice> ir_;
	std::unique_ptr<V4L2VideoDevice> stats_;

	std::vector<std::unique_ptr<FrameBuffer>> paramsBuffers_;
	std::vector<std::unique_ptr<FrameBuffer>> statsBuffers_;

	std::string logPrefix() const
	{
		return "Neo[" + isp_->deviceNode() + "] ";
	}

	const MediaDevice *media() const
	{
		return media_;
	}

	uint32_t hwCapabilities() const
	{
		return hwCapabilities_;
	}

	uint32_t apiVersion() const
	{
		return apiVersion_;
	}

private:
	int configureVideoDeviceLink(VideoDevice device, bool enable);
	int configureVideoDevice(VideoDevice device, V4L2DeviceFormat *format);
	int configureVideoDeviceMeta(VideoDevice device, unsigned int apiVersion);

	MediaDevice *media_ = nullptr;
	std::map<VideoDevice, std::unique_ptr<V4L2VideoDevice> *> videos_;
	std::vector<VideoDevice> configured_;

	uint32_t hwCapabilities_ = 0;
	uint32_t apiVersion_ = NEOISP_LEGACY_META_BUFFER;
};

} /* namespace libcamera */
