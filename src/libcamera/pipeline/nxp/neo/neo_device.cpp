/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2024-2026 NXP
 *
 * Neo ISP device
 *
 * Based on Intel IPU3 ImgU
 *     src/libcamera/pipeline/ipu3/imgu.cpp
 * Copyright (C) 2019, Google Inc.
 */

#include <algorithm>
#include <cmath>
#include <limits>

#include <linux/media-bus-format.h>
#include <linux/nxp_neoisp.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "libcamera/internal/media_device.h"

#include "neo_device.h"

namespace libcamera {

/* \todo Remove meta format local definitions. */
#ifndef V4L2_META_FMT_NEO_ISP_PARAMS
#define V4L2_META_FMT_NEO_ISP_PARAMS v4l2_fourcc('N', 'N', 'I', 'P')
#define V4L2_META_FMT_NEO_ISP_EXT_PARAMS v4l2_fourcc('N', 'N', 'E', 'P')
#define V4L2_META_FMT_NEO_ISP_STATS v4l2_fourcc('N', 'N', 'I', 'S')
#define V4L2_META_FMT_NEO_ISP_EXT_STATS v4l2_fourcc('N', 'N', 'E', 'S')
#endif

/* \todo Remove when control is available in v4l2-controls header */
#ifndef V4L2_CID_USER_NEOISP_BASE
#define V4L2_CID_USER_NEOISP_BASE (V4L2_CID_USER_BASE + 0x1230)
#endif

LOG_DEFINE_CATEGORY(NxpNeoDev)

namespace nxpneo {

/**
 * \struct PipeConfig
 * \brief Configuration parameters for the Neo ISP device
 *
 * This structure contains the configuration settings that control the behavior
 * of the Neo ISP processing pipeline.
 */

/**
 * \var PipeConfig::topLines
 * \brief The number of top embedded data line to crop from the input raw images
 */

/**
 * \enum NeoDevice::VideoDevice
 * \brief The video devices exposed by the ISP instance
 *
 * \var NeoDevice::VideoDevice::Input0
 * \brief The input0 video device output node (long/main exposure)
 *
 * \var NeoDevice::VideoDevice::Input1
 * \brief The input1 video device output node (short exposure for HDR)
 *
 * \var NeoDevice::VideoDevice::Params
 * \brief The params video device output node (ISP parameters)
 *
 * \var NeoDevice::VideoDevice::Frame
 * \brief The frame video device capture node (Rgb/Yuv output pixels)
 *
 * \var NeoDevice::VideoDevice::Ir
 * \brief The Infrared video device capture node (Infared output pixels)
 *
 * \var NeoDevice::VideoDevice::Stats
 * \brief The stats video device output node (ISP statistics)
 */

namespace {

/* Keep those definitions in sync with NeoDevice::VideoDevice enum class. */
constexpr unsigned int kVideoDeviceCount = 6;
constexpr std::array<NeoDevice::VideoDevice, kVideoDeviceCount>
kAllVideoDevices{
	NeoDevice::VideoDevice::Input0,
	NeoDevice::VideoDevice::Input1,
	NeoDevice::VideoDevice::Params,
	NeoDevice::VideoDevice::Frame,
	NeoDevice::VideoDevice::Ir,
	NeoDevice::VideoDevice::Stats,
};

constexpr unsigned int kVideoDeviceMetaCount = 2;
constexpr std::array<NeoDevice::VideoDevice, kVideoDeviceMetaCount>
kMetaVideoDevices{
	NeoDevice::VideoDevice::Params,
	NeoDevice::VideoDevice::Stats,
};

constexpr unsigned int kVideoDeviceMutableCount = 5;
constexpr std::array<NeoDevice::VideoDevice, kVideoDeviceMutableCount>
kMutableVideoDevices = {
	NeoDevice::VideoDevice::Input1,
	NeoDevice::VideoDevice::Params,
	NeoDevice::VideoDevice::Frame,
	NeoDevice::VideoDevice::Ir,
	NeoDevice::VideoDevice::Stats,
};

bool isDeviceValid(NeoDevice::VideoDevice device)
{
	return std::find(kAllVideoDevices.begin(),
			 kAllVideoDevices.end(),
			 device) != kAllVideoDevices.end();
}

bool isDeviceMeta(NeoDevice::VideoDevice device)
{
	return std::find(kMetaVideoDevices.begin(),
			 kMetaVideoDevices.end(),
			 device) != kMetaVideoDevices.end();
}

bool isDeviceMutable(NeoDevice::VideoDevice device)
{
	return std::find(kMutableVideoDevices.begin(),
			 kMutableVideoDevices.end(),
			 device) != kMutableVideoDevices.end();
}

} /* namespace */

/**
 * \brief Initialize the NEO ISP device and its video devices
 * \param[in] media The media device containing the NEO ISP entities
 *
 * This function performs the complete initialization of the NEO ISP device by:
 * - Opening the NEO ISP subdevice
 * - Opening all associated video devices
 * - Querying hardware and driver capabilities
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV if the NEO ISP device cannot be found or opened
 * \retval -EINVAL if the device uAPI version is not supported
 */
int NeoDevice::init(MediaDevice *media)
{
	media_ = media;

	isp_ = V4L2Subdevice::fromEntityName(media, subdeviceName());
	int ret = isp_.get() ? isp_->open() : -ENODEV;
	if (ret) {
		LOG(NxpNeoDev, Error) << logPrefix() << "Failed to open NEO";
		return ret;
	}

	/*
	 * Get uapi meta version from kernel, then select the most appropriate
	 * version compatible with user space.
	 */
	const struct v4l2_query_ext_ctrl *ctlInfo =
		isp_->controlInfo(V4L2_CID_NEOISP_META_API_VERSION);
	if (ctlInfo != nullptr) {
		int maxVersionUser = NEOISP_META_BUFFER_VERSION_COUNT - 1;
		int maxVersionKernel = static_cast<int>(ctlInfo->maximum);
		int minVersionKernel = static_cast<int>(ctlInfo->minimum);

		if (minVersionKernel > maxVersionUser) {
			LOG(NxpNeoDev, Error) << "Device uAPI version not supported";
			ret = -EINVAL;
			goto done;
		}

		int apiVersion = std::min(maxVersionUser, maxVersionKernel);
		LOG(NxpNeoDev, Debug) << "Highest compatible uAPI version is " << apiVersion;

		ControlList ctrls(isp_->controls());
		ctrls.set(V4L2_CID_NEOISP_META_API_VERSION, apiVersion);
		ret = isp_->setControls(&ctrls);
		if (ret)
			goto done;

		apiVersion_ = apiVersion;
	}

	/*
	 * Get Neo ISP hardware and driver capabilities.
	 * Fallback to MSB alignment for backward compatibility with older
	 * kernel drivers that don't expose V4L2_CID_NEOISP_QUERYCAP. This was
	 * the default behavior in earlier driver versions.
	 */
	ctlInfo = isp_->controlInfo(V4L2_CID_NEOISP_QUERYCAP);
	if (ctlInfo != nullptr) {
		const std::array<uint32_t, 1> cids = { V4L2_CID_NEOISP_QUERYCAP };
		ControlList ctrls = isp_->getControls(cids);
		if (!ctrls.empty())
			hwCapabilities_ =
				ctrls.get(V4L2_CID_NEOISP_QUERYCAP).get<int32_t>();
	} else {
		hwCapabilities_ = NEO_CAP_ALIGNMENT_MSB;
	}

	videos_ = {
		{ VideoDevice::Input0, &input0_ },
		{ VideoDevice::Input1, &input1_ },
		{ VideoDevice::Params, &params_ },
		{ VideoDevice::Frame, &frame_ },
		{ VideoDevice::Ir, &ir_ },
		{ VideoDevice::Stats, &stats_ },
	};
	ASSERT(videos_.size() == kVideoDeviceCount);

	for (const auto [device, _video] : videos_) {
		const std::string &name = videoDeviceName(device);
		*_video = std::move(V4L2VideoDevice::fromEntityName(media, name));
		V4L2VideoDevice *video = _video->get();
		ret = video ? video->open() : -ENODEV;
		if (ret) {
			LOG(NxpNeoDev, Error)
				<< logPrefix() << "Failed to open video device "
				<< static_cast<int>(device);
			break;
		}
	}

done:
	if (ret) {
		isp_.release();
		for (const auto [device, _video] : videos_)
			_video->release();
	}

	return ret;
}

/**
 * \brief Allocate buffers for all the NEO video devices
 * \param[in] bufferCount The number of buffers to allocate
 *
 * This function allocates and imports buffers for all configured NEO video
 * devices. It handles two types of buffer allocation:
 *
 * 1. **Pixel buffers**: These buffers are imported from external sources
 *
 * 2. **Metadata buffers**: These are allocated as internal buffers using a
 *      export-then-import technique:
 *    - exportBuffers() creates orphaned DMABUF buffers
 *    - importBuffers() configures the queue to use V4L2 DMABUF memory type
 *    - This approach allows metadata buffers to be shared across multiple ISP
 *      instances (e.g., during RGBIr dual context mode)
 *
 * The function only processes devices that were configured in the most recent
 * configure() call. If any allocation fails, all buffers are freed via
 * freeBuffers() to ensure a clean state.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENOMEM if buffer allocation or import fails
 */
int NeoDevice::allocateBuffers(unsigned int bufferCount)
{
	int ret = 0;
	LOG(NxpNeoDev, Debug) << logPrefix() << "Allocate buffers " << bufferCount;

	auto createBuffers =
		[this](VideoDevice device,
		       std::vector<std::unique_ptr<FrameBuffer>> &buffers,
		       unsigned int count) {
			V4L2VideoDevice *video = videoDevice(device);
			ASSERT(video);
			int res = video->exportBuffers(count, &buffers);
			if (res < 0 || static_cast<unsigned int>(res) != count) {
				LOG(NxpNeoDev, Error)
					<< logPrefix() << "Failed to export buffers device "
					<< static_cast<int>(device);
				return -ENOMEM;
			}

			res = video->importBuffers(count);
			if (res) {
				LOG(NxpNeoDev, Error)
					<< logPrefix() << "Failed to import buffers device "
					<< static_cast<int>(device);
				return -ENOMEM;
			}
			return res;
		};

	const std::map<VideoDevice, std::vector<std::unique_ptr<FrameBuffer>> *>
	metaBuffers = {
		{ VideoDevice::Params, &paramsBuffers_ },
		{ VideoDevice::Stats, &statsBuffers_ },
	};

	for (auto device : configured_) {
		if (isDeviceMeta(device))
			continue;
		V4L2VideoDevice *video = videoDevice(device);
		ASSERT(video);
		ret = video->importBuffers(bufferCount);
		if (ret) {
			LOG(NxpNeoDev, Error)
				<< logPrefix() << "Failed to import buffers device "
				<< static_cast<int>(device);
			break;
		}
	}
	if (ret)
		goto done;

	for (auto device : kMetaVideoDevices) {
		auto buffers = metaBuffers.at(device);
		ret = createBuffers(device, *buffers, bufferCount);
		if (ret)
			break;
	}

done:
	if (ret)
		freeBuffers();

	return ret;
}

/**
 * \brief Release buffers for all the NEO video devices
 *
 * This function releases all buffers that were previously allocated by
 * allocateBuffers(). It performs the following operations:
 *
 * 1. **Metadata buffers**: Clears the internal metadata buffer vectors
 *
 * 2. **Video device buffers**: Calls releaseBuffers() on all configured
 *    video devices to release their V4L2 buffer queues.
 *
 * The function processes only devices that were configured in the most recent
 * configure() call. Any errors during buffer release are logged but do not
 * prevent the function from attempting to release buffers from remaining
 * devices.
 */
void NeoDevice::freeBuffers()
{
	LOG(NxpNeoDev, Debug) << logPrefix() << "Free buffers";

	for (auto device : configured_) {
		V4L2VideoDevice *video = videoDevice(device);
		ASSERT(video);
		int ret = video->releaseBuffers();
		if (ret) {
			LOG(NxpNeoDev, Error)
				<< logPrefix() << "Failed to release buffers device "
				<< static_cast<int>(device);
		}
	}

	paramsBuffers_.clear();
	statsBuffers_.clear();
}

/**
 * \brief Configure NEO video devices according to their formats
 * \param[in] pipeConfig The ISP pipeline configuration
 * \param[in] formats Map of video devices to their corresponding V4L2 format
 *  configurations
 *
 * This function configures all NEO video devices based on the provided
 * format map. It performs the following operations:
 * - Validates that all required immutable devices have formats specified
 * - Configures pixel video devices (input0, input1, frame, ir) with their
 *   formats
 * - Enables links for mutable devices that are being configured
 * - Configures metadata video devices (params, stats) with appropriate API
 *   version
 * - Disables links for unconfigured mutable devices
 * - Applies crop settings to input devices to remove embedded data lines
 *
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::configure(const PipeConfig &pipeConfig,
			 const std::map<VideoDevice, V4L2DeviceFormat *> &formats)
{
	int ret;

	/* Validate that all required immutable devices have formats. */
	for (VideoDevice device : kAllVideoDevices) {
		if (isDeviceMutable(device))
			continue;
		if (formats.find(device) == formats.end()) {
			LOG(NxpNeoDev, Error)
				<< logPrefix() << "Missing format for immutable device "
				<< static_cast<int>(device);
			return -EINVAL;
		}
	}

	/* Configure pixel video devices. */
	std::vector<VideoDevice> configured;
	for (const auto [device, format] : formats) {
		if (isDeviceMeta(device)) {
			LOG(NxpNeoDev, Warning)
				<< logPrefix() << "Ignoring metadata format device "
				<< static_cast<int>(device);
			continue;
		}

		if (isDeviceMutable(device)) {
			ret = configureVideoDeviceLink(device, true);
			if (ret)
				return ret;
		}
		ret = configureVideoDevice(device, format);
		if (ret)
			return ret;

		configured.push_back(device);
	}

	/* Configure metadata video devices - mutable and always enabled. */
	for (VideoDevice device : kMetaVideoDevices) {
		ret = configureVideoDeviceLink(device, true);
		if (ret)
			return ret;
		ret = configureVideoDeviceMeta(device, apiVersion_);
		if (ret)
			return ret;

		configured.push_back(device);
	}

	/* Disable the links for unconfigured mutable devices. */
	for (VideoDevice device : kMutableVideoDevices) {
		if (std::find(configured.begin(),
			      configured.end(), device) == configured.end()) {
			ret = configureVideoDeviceLink(device, false);
			if (ret)
				return ret;
		}
	}

	/* Set crop on enabled input. */
	auto setCropSelection =
		[this](VideoDevice device, const V4L2DeviceFormat *format, int topLines) {
			V4L2VideoDevice *video = videoDevice(device);
			ASSERT(video);
			Size sizeInput = format->size;
			Rectangle rect{ 0, topLines, sizeInput.width,
					sizeInput.height - topLines };
			int res = video->setSelection(V4L2_SEL_TGT_CROP, &rect);
			if (res)
				LOG(NxpNeoDev, Error)
					<< logPrefix() << "Failed to set crop selection device "
					<< static_cast<int>(device);
			return res;
		};

	for (auto device : { VideoDevice::Input0, VideoDevice::Input1 }) {
		auto it = formats.find(device);
		if (it != formats.end()) {
			const auto &[dev, format] = *it;
			ret = setCropSelection(device, format, pipeConfig.topLines);
			if (ret)
				return ret;
		}
	}

	configured_ = std::move(configured);

	return 0;
}

/**
 * \brief Start streaming on all configured NEO video devices
 *
 * This function initiates video streaming on the NEO ISP device by:
 * - Enabling frame start events on the ISP subdevice
 * - Starting streaming (streamOn) on all configured video devices
 *
 * The function iterates through all video devices that were configured during
 * the most recent configure() call and starts streaming on each one.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EBUSY if a device is already streaming
 * \retval -EINVAL if the device is not properly configured
 */
int NeoDevice::start()
{
	LOG(NxpNeoDev, Debug) << logPrefix() << "Start";

	int ret = isp_->setFrameStartEnabled(true);
	if (ret) {
		LOG(NxpNeoDev, Error)
			<< logPrefix() << "FrameStart failure " << ret;
		return ret;
	}

	for (auto device : configured_) {
		V4L2VideoDevice *video = videoDevice(device);
		ASSERT(video);
		ret = video->streamOn();
		if (ret) {
			LOG(NxpNeoDev, Error)
				<< logPrefix() << "Failed to start NEO device "
				<< static_cast<int>(device);
			break;
		}
	}

	if (ret)
		stop();

	return ret;
}

/**
 * \brief Stop streaming on all configured NEO video devices
 *
 * This function stops video streaming on the NEO ISP device by:
 * - Disabling frame start events on the ISP subdevice
 * - Stopping streaming (streamOff) on all configured video devices
 *
 * The function iterates through all video devices that were configured during
 * the most recent configure() call and stops streaming on each one. Errors
 * from individual devices are logged but do not prevent the function from
 * attempting to stop remaining devices.
 *
 * \return 0 on success or a negative error code otherwise
 */
int NeoDevice::stop()
{
	LOG(NxpNeoDev, Debug) << logPrefix() << "Stop";

	int ret = isp_->setFrameStartEnabled(false);
	if (ret) {
		LOG(NxpNeoDev, Error)
			<< logPrefix() << "FrameStart failure " << ret;
	}

	for (auto device : configured_) {
		V4L2VideoDevice *video = videoDevice(device);
		ASSERT(video);
		int res = video->streamOff();
		if (res) {
			LOG(NxpNeoDev, Error)
				<< logPrefix() << "Failed to stop NEO device "
				<< static_cast<int>(device);
			ret |= res;
		}
	}

	return ret;
}

/**
 * \brief Retrieve a specific Neo video device
 * \param[in] device The video device identifier
 *
 * This function returns a pointer to the V4L2VideoDevice corresponding to
 * the specified device identifier. That is an alternative to accessing the
 * a video device firectly through its pointer exposed in the public interface
 * of the class.
 *
 * \return Pointer to the requested V4L2VideoDevice
 */
V4L2VideoDevice *NeoDevice::videoDevice(VideoDevice device) const
{
	auto it = videos_.find(device);
	ASSERT(it != videos_.end());
	return it->second->get();
}

/**
 * \brief Get the driver name for the Neo ISP
 *
 * This function returns the Linux kernel driver name for the Neo ISP device.
 * The driver name is used for device matching and enumeration in the media
 * device framework.
 *
 * \return The Neo ISP driver name string
 */
const std::string &NeoDevice::driverName()
{
	static const std::string driverName = "neoisp";
	return driverName;
}

/**
 * \brief Get the subdevice entity name for the Neo ISP
 *
 * This function returns the media entity name for the Neo ISP subdevice.
 *
 * \return The Neo ISP subdevice entity name string
 */
const std::string &NeoDevice::subdeviceName()
{
	static const std::string entityName = "neoisp";
	return entityName;
}

/**
 * \brief Get the entity name for a Neo video device
 * \param[in] device The video device identifier
 *
 * This function returns the media entity name associated with the specified
 * Neo video device.
 *
 * \return The entity name string for valid devices, or an empty string for
 * invalid device identifiers
 */
const std::string &NeoDevice::videoDeviceName(VideoDevice device)
{
	static const std::map<VideoDevice, std::string> deviceNames = {
		{ VideoDevice::Input0, "neoisp-input0" },
		{ VideoDevice::Input1, "neoisp-input1" },
		{ VideoDevice::Params, "neoisp-params" },
		{ VideoDevice::Frame, "neoisp-frame" },
		{ VideoDevice::Ir, "neoisp-ir" },
		{ VideoDevice::Stats, "neoisp-stats" },
	};
	static const std::string empty = "";

	auto it = deviceNames.find(device);
	if (it != deviceNames.end())
		return it->second;

	LOG(NxpNeoDev, Error) << "Invalid device " << static_cast<int>(device);
	return empty;
}

namespace {

std::vector<PixelFormat> queryPixelFormats(V4L2VideoDevice *device)
{
	V4L2VideoDevice::Formats deviceFormats = device->formats();
	std::vector<PixelFormat> formats;
	for (const auto &[format, ranges] : deviceFormats) {
		PixelFormat pixelFormat = format.toPixelFormat(false);
		if (pixelFormat.isValid())
			formats.push_back(pixelFormat);
	}
	return formats;
}

} /* namespace */

/**
 * \brief Get the supported pixel formats for a NEO capture video device
 * \param[in] device The video device identifier
 *
 * This function returns the list of supported pixel formats for the specified
 * NEO capture video device.
 *
 * The pixel formats are queried once during the first call and cached in
 * static variables for subsequent calls. The formats are obtained by querying
 * the underlying V4L2 video device and converting V4L2 formats to libcamera
 * PixelFormat objects.
 *
 * \return A reference to the vector of supported PixelFormat objects for the
 * specified device, or an empty vector if the device is invalid
 */
const std::vector<PixelFormat> &NeoDevice::capturePixelFormats(VideoDevice device) const
{
	static const std::vector<PixelFormat> frameFormats =
		queryPixelFormats(videoDevice(VideoDevice::Frame));
	static const std::vector<PixelFormat> irFormats =
		queryPixelFormats(videoDevice(VideoDevice::Ir));
	static const std::vector<PixelFormat> empty;

	if (device == VideoDevice::Frame)
		return frameFormats;
	else if (device == VideoDevice::Ir)
		return irFormats;

	LOG(NxpNeoDev, Error)
		<< "Invalid capture pixel device " << static_cast<int>(device);
	return empty;
}

/**
 * \brief Report the supported V4L2 pixel formats on the output nodes
 * \param[in] device The video device identifier
 *
 * This function returns the supported V4L2 pixel formats for the output nodes.
 * These formats represent raw Bayer patterns and grayscale formats at various
 * bit depths.
 *
 * This function is static with a predefined list of V4L2 pixel formats as it
 * may be called during the early stages of the pipeline handler creation before
 * the NeoDevice object is created.
 *
 * \return A reference to the vector of supported V4L2PixelFormat objects for
 * the relevant devices, or an empty vector if the device is invalid
 */
const std::vector<V4L2PixelFormat> &NeoDevice::outputFormats(VideoDevice device)
{
	static const std::vector<V4L2PixelFormat> outputFormats = {
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR8),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG8),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG8),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB8),
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR10),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG10),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG10),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB10),
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR12),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG12),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG12),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB12),
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR14),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG14),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG14),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB14),
		V4L2PixelFormat(V4L2_PIX_FMT_SBGGR16),
		V4L2PixelFormat(V4L2_PIX_FMT_SGBRG16),
		V4L2PixelFormat(V4L2_PIX_FMT_SGRBG16),
		V4L2PixelFormat(V4L2_PIX_FMT_SRGGB16),
		V4L2PixelFormat(V4L2_PIX_FMT_GREY),
		V4L2PixelFormat(V4L2_PIX_FMT_Y10),
		V4L2PixelFormat(V4L2_PIX_FMT_Y12),
		V4L2PixelFormat(V4L2_PIX_FMT_Y16),
	};
	static const std::vector<V4L2PixelFormat> empty;

	if (device == VideoDevice::Input0 || device == VideoDevice::Input1)
		return outputFormats;

	LOG(NxpNeoDev, Error)
		<< "Invalid output pixel device " << static_cast<int>(device);
	return empty;
}

/**
 * \brief Configure the media link for a NEO video device
 * \param[in] device The video device identifier
 * \param[in] enable True to enable the link, false to disable it
 *
 * This function enables or disables the media link associated with the
 * specified NEO video device. It retrieves the media entity corresponding
 * to the device, validates that it has exactly one pad with one link, and
 * then sets the link's enabled state.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -ENODEV if the video device entity or its link is not found
 */
int NeoDevice::configureVideoDeviceLink(VideoDevice device, bool enable)
{
	unsigned int id = static_cast<int>(device);
	LOG(NxpNeoDev, Debug)
		<< logPrefix() << "Configure video device " << id
		<< " link status " << (enable ? "enabled" : "disabled");

	const std::string &name = videoDeviceName(device);
	MediaEntity *entity = media_->getEntityByName(name);
	if (!entity || (entity->pads().size()) != 1 ||
	    (entity->pads()[0]->links().size() != 1)) {
		LOG(NxpNeoDev, Error)
			<< "Video device link not found device " << id;
		return -ENODEV;
	}

	MediaLink *link = entity->pads()[0]->links()[0];
	int ret = link->setEnabled(enable);
	if (ret)
		LOG(NxpNeoDev, Error) << "Error setting link device " << id;

	return ret;
}

/**
 * \brief Configure a NEO video device with the specified format
 * \param[in] device The video device identifier
 * \param[in,out] format The V4L2 device format to configure
 *
 * This function configures a pixel video device (input0, input1, frame, or ir)
 * with the provided format. Metadata devices (params, stats) are not supported
 * by this function and should be configured using configureVideoDeviceMeta()
 * instead.
 *
 * The function validates that the device is a valid pixel video device before
 * attempting to apply the format configuration.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EINVAL if the device is invalid or is a metadata device
 */
int NeoDevice::configureVideoDevice(VideoDevice device, V4L2DeviceFormat *format)
{
	unsigned int id = static_cast<int>(device);
	LOG(NxpNeoDev, Debug)
		<< logPrefix() << "Configure video device " << id
		<< " format " << format->toString();

	if (!isDeviceValid(device) || isDeviceMeta(device)) {
		LOG(NxpNeoDev, Error)
			<< "Invalid pixel video device " << id;
		return -EINVAL;
	}

	V4L2VideoDevice *video = videoDevice(device);
	ASSERT(video);
	int ret = video->setFormat(format);
	if (ret) {
		LOG(NxpNeoDev, Error)
			<< logPrefix() << "Failed to set video device format " << id;
	}

	return ret;
}

/**
 * \brief Configure a NEO metadata video device with API version-specific format
 * \param[in] device The video device identifier (must be a metadata device)
 * \param[in] apiVersion The NEO ISP metadata API version to use
 *
 * This function configures a metadata video device (params or stats) with the
 * appropriate format based on the specified API version. It supports two API
 * versions:
 * - NEOISP_LEGACY_META_BUFFER: Uses legacy metadata formats
 *   (V4L2_META_FMT_NEO_ISP_PARAMS and V4L2_META_FMT_NEO_ISP_STATS)
 * - Extensible format: Uses extensible metadata formats
 *   (V4L2_META_FMT_NEO_ISP_EXT_PARAMS and V4L2_META_FMT_NEO_ISP_EXT_STATS)
 *
 * The function validates that the device is a valid metadata device before
 * applying the format configuration. Pixel video devices (input0, input1,
 * frame, ir) are not supported by this function and should be configured
 * using configureVideoDevice() instead.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EINVAL if the device is invalid or is not a metadata device
 */
int NeoDevice::configureVideoDeviceMeta(VideoDevice device, unsigned int apiVersion)
{
	unsigned int id = static_cast<int>(device);
	LOG(NxpNeoDev, Debug)
		<< logPrefix() << "Configure video device meta " << id
		<< " apiVersion " << apiVersion;

	if (!isDeviceValid(device) || !isDeviceMeta(device)) {
		LOG(NxpNeoDev, Error)
			<< "Invalid metadata video device " << id;
		return -EINVAL;
	}

	V4L2DeviceFormat format = {};
	static const std::map<VideoDevice, std::pair<unsigned int, size_t>>
	metaFormatsLegacy = {
		{ VideoDevice::Params, { V4L2_META_FMT_NEO_ISP_PARAMS, sizeof(struct neoisp_meta_params_s) }},
		{ VideoDevice::Stats, { V4L2_META_FMT_NEO_ISP_STATS, sizeof(struct neoisp_meta_stats_s) }},
	};

	static const std::map<VideoDevice, std::pair<unsigned int, size_t>>
	metaFormatsExtensible = {
		{ VideoDevice::Params, { V4L2_META_FMT_NEO_ISP_EXT_PARAMS, sizeof(struct neoisp_ext_params_s) }},
		{ VideoDevice::Stats, { V4L2_META_FMT_NEO_ISP_EXT_STATS, sizeof(struct neoisp_ext_stats_s) }},
	};

	auto &formatMap = (apiVersion == NEOISP_LEGACY_META_BUFFER)
				  ? metaFormatsLegacy
				  : metaFormatsExtensible;
	auto it = formatMap.find(device);
	ASSERT(it != formatMap.end());
	const auto &[dev, formatPair] = *it;
	const auto &[fourcc, size] = formatPair;
	format.fourcc = V4L2PixelFormat(fourcc);
	format.planes[0].size = size;

	V4L2VideoDevice *video = videoDevice(device);
	ASSERT(video);
	int ret = video->setFormat(&format);
	if (ret) {
		LOG(NxpNeoDev, Error)
			<< logPrefix() << "Failed to set video device format device " << id;
	}

	return ret;
}

} /* namespace nxpneo */

} /* namespace libcamera */
