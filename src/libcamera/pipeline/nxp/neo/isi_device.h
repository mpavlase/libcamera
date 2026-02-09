/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * ISI device
 *
 * Based on Intel IPU3 CIO2
 *     src/libcamera/pipeline/ipu3/cio2.h
 * Copyright (C) 2019, Google Inc.
 */

#pragma once

#include <memory>
#include <queue>
#include <vector>

#include <libcamera/base/signal.h>

#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class CameraSensor;
class FrameBuffer;
class MediaDevice;
class PixelFormat;
class Request;
class Size;
class SizeRange;
struct StreamConfiguration;

class ISIPipe
{
public:
	static constexpr unsigned int kUnchainedWidthMax = 2048;
	static constexpr unsigned int kChainedWidthMax = 4096;

	ISIPipe(unsigned int index)
		: index_(index) {}

	int init(const MediaDevice *media);

	int configure(V4L2SubdeviceFormat &sinkFormat,
		      V4L2DeviceFormat &videoFormat);

	int start();
	int stop();

	static void subdeviceName(std::string &name, unsigned int index);
	static void videoDeviceName(std::string &name, unsigned int index);

	std::string logPrefix() const
	{
		return "Pipe[" + std::to_string(index_) + "] ";
	}

	int allocateBuffers(unsigned int bufferCount);
	int importBuffers(unsigned int bufferCount);
	void freeBuffers();

	static const std::vector<uint32_t> &bayerMbusCodes();
	static const std::vector<uint32_t> &metaMbusCodes();
	static const std::vector<uint32_t> &sinkMbusCodesProcessed();
	static const std::vector<PixelFormat> &pixelFormatsProcessed();
	static const V4L2PixelFormat mbusCodeToPixelFormatBypass(unsigned int code);

	std::unique_ptr<V4L2VideoDevice> capture_;
	std::vector<std::unique_ptr<FrameBuffer>> captureBuffers_;

private:
	std::unique_ptr<V4L2Subdevice> pipe_;

	unsigned int index_;
};

class ISIDevice
{
public:
	ISIDevice() {}

	static constexpr unsigned int kPipesMax = 16;

	int init(MediaDevice *media);

	int reservePipeBySize(Size &sizeMax, unsigned int *index);
	int reservePipeByIndex(Size &sizeMax, unsigned int index);
	void releasePipe(unsigned int index);
	ISIPipe *getPipeByIndex(unsigned int index);

	static const std::string &driverName();
	static const std::string &crossbarSubdevName();

	V4L2Subdevice *crossbar() const { return crossbar_.get(); }
	unsigned int crossbarFirstSourcePad() const { return xbarSinkPads_; }
	unsigned int crossbarSourcePads() const { return pipeEntries_.size(); }
	MediaDevice *media() const { return media_; }

private:
	struct PipeWrapper {
		PipeWrapper(unsigned int index)
			: pipe_(index), free(true), chained(false) {}
		ISIPipe pipe_;
		bool free;
		bool chained;
	};

	std::vector<PipeWrapper> pipeEntries_;
	std::unique_ptr<V4L2Subdevice> crossbar_;
	unsigned int xbarSinkPads_ = 0;
	MediaDevice *media_ = nullptr;
};

} /* namespace libcamera */
