/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2025 NXP
 *
 * Autofocus control algorithm
 */

#include "af.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/core_ipa_interface.h>

#include "af_base.h"
#include "af_impl.h"

namespace libcamera {

namespace ipa::nxpneo::algorithms {

using namespace libcamera::controls;

LOG_DEFINE_CATEGORY(NxpNeoAlgoAf)

Af::Af()
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Af::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	algo_ = std::make_unique<AfImpl>();
	return algo_->doInit(tuningData);
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Af::configure([[maybe_unused]] IPAContext &context, const IPACameraSensorInfo &configInfo)
{
	/* Store lens position range in the context. */
	auto &afConfig = context.configuration.af;
	double min, max;
	algo_->getLensLimits(min, max);
	afConfig.minLensPosition = static_cast<float>(min);
	afConfig.maxLensPosition = static_cast<float>(max);
	double def = algo_->getDefaultLensPosition();
	afConfig.defLensPosition = static_cast<float>(def);

	/* Camera mode related values. */
	const Rectangle &analogCrop = configInfo.analogCrop;
	afConfig.cropX = analogCrop.x;
	afConfig.cropY = analogCrop.y;
	const Size &outputSize = configInfo.outputSize;
	uint16_t width = outputSize.width;
	uint16_t height = outputSize.height;
	afConfig.scaleX = analogCrop.width * 1.0 / width;
	afConfig.scaleY = analogCrop.height * 1.0 / height;

	/* Initialize the active state. */
	auto &afState = context.activeState.af;
	afState.mode = AfModeManual;
	algo_->setMode(AfModeManual);
	int32_t hwPosition;
	algo_->setLensPosition(def, &hwPosition);

	return algo_->doConfigure(configInfo);
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Af::queueRequest([[maybe_unused]] IPAContext &context,
		      [[maybe_unused]] const uint32_t frame,
		      [[maybe_unused]] IPAFrameContext &frameContext,
		      const ControlList &controls)
{
	auto &afFContext = frameContext.af;

	const std::array<AfModeEnum, 3> modes{ AfModeManual, AfModeAuto, AfModeContinuous };
	const auto &afMode = controls.get(controls::AfMode);
	if (afMode) {
		if (std::find(modes.begin(), modes.end(), *afMode) != modes.end()) {
			AfModeEnum mode = static_cast<AfModeEnum>(*afMode);
			afFContext.mode = mode;
		} else {
			LOG(NxpNeoAlgoAf, Warning) << "Invalid AfMode: " << *afMode;
		}
	}

	const std::array<AfRangeEnum, 3> ranges{ AfRangeNormal, AfRangeMacro, AfRangeFull };
	const auto &afRange = controls.get(controls::AfRange);
	if (afRange) {
		if (std::find(ranges.begin(), ranges.end(), *afRange) != ranges.end()) {
			AfRangeEnum range = static_cast<AfRangeEnum>(*afRange);
			afFContext.range = range;
		} else {
			LOG(NxpNeoAlgoAf, Warning) << "Invalid AfRange: " << *afRange;
		}
	}

	const std::array<AfSpeedEnum, 2> speeds{ AfSpeedNormal, AfSpeedFast };
	const auto &afSpeed = controls.get(controls::AfSpeed);
	if (afSpeed) {
		if (std::find(speeds.begin(), speeds.end(), *afSpeed) != speeds.end()) {
			AfSpeedEnum speed = static_cast<AfSpeedEnum>(*afSpeed);
			afFContext.speed = speed;
		} else {
			LOG(NxpNeoAlgoAf, Warning) << "Invalid AfSpeed: " << *afSpeed;
		}
	}

	const std::array<AfMeteringEnum, 2> meterings{ AfMeteringAuto, AfMeteringWindows };
	const auto &afMetering = controls.get(controls::AfMetering);
	if (afMetering) {
		if (std::find(meterings.begin(), meterings.end(), *afMetering) != meterings.end()) {
			AfMeteringEnum metering = static_cast<AfMeteringEnum>(*afMetering);
			afFContext.metering = metering;
		} else {
			LOG(NxpNeoAlgoAf, Warning) << "Invalid AfMetering: " << *afMetering;
		}
	}

	const auto &afWindows = controls.get(controls::AfWindows);
	auto &afConfig = context.configuration.af;
	if (afWindows) {
		afFContext.windows = std::vector<Rectangle>{};
		for (const auto &window : afWindows.value())
			afFContext.windows->push_back(window);
		Span<Rectangle> windows{ afFContext.windows->data(),
					 afFContext.windows->size() };
		windowsToNative(windows, afConfig.cropX, afConfig.cropY,
				afConfig.scaleX, afConfig.scaleY);
	}

	const std::array<AfPauseEnum, 3> pauses{ AfPauseImmediate, AfPauseDeferred,
						 AfPauseResume };
	const auto &afPause = controls.get(controls::AfPause);
	if (afPause) {
		if (std::find(pauses.begin(), pauses.end(), *afPause) != pauses.end()) {
			AfPauseEnum pauseValue = static_cast<AfPauseEnum>(*afPause);
			afFContext.pause = pauseValue;
		} else {
			LOG(NxpNeoAlgoAf, Warning) << "Invalid AfPause: " << *afPause;
		}
	}

	const std::array<AfTriggerEnum, 2> triggers{ AfTriggerStart, AfTriggerCancel };
	const auto &afTrigger = controls.get(controls::AfTrigger);
	if (afTrigger) {
		if (algo_->getMode() != AfModeAuto) {
			LOG(NxpNeoAlgoAf, Warning) << "AfTrigger is restricted to Auto mode";
		} else if (std::find(triggers.begin(), triggers.end(), *afTrigger) !=
			   triggers.end()) {
			AfTriggerEnum trigger = static_cast<AfTriggerEnum>(*afTrigger);
			afFContext.trigger = trigger;
		} else {
			LOG(NxpNeoAlgoAf, Warning) << "Invalid AfTrigger: " << *afTrigger;
		}
	}

	const auto &afPosition = controls.get(controls::LensPosition);
	if (afPosition)
		afFContext.lensPosition = afPosition.value();
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Af::prepare([[maybe_unused]] IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 [[maybe_unused]] NxpNeoParams *params)
{
	/* User controls handling */
	auto &afFContext = frameContext.af;
	auto &afState = context.activeState.af;

	if (afFContext.mode) {
		AfModeEnum mode = afFContext.mode.value();
		afState.mode = mode;
		algo_->setMode(mode);
	}
	if (afFContext.range) {
		AfRangeEnum range = afFContext.range.value();
		algo_->setRange(range);
	}
	if (afFContext.speed) {
		AfSpeedEnum speed = afFContext.speed.value();
		algo_->setSpeed(speed);
	}
	if (afFContext.metering) {
		AfMeteringEnum metering = afFContext.metering.value();
		algo_->setMetering(metering != AfMeteringAuto);
	}
	if (afFContext.windows) {
		const std::vector<Rectangle> &vec = afFContext.windows.value();
		Span<const Rectangle> afWindows(vec.data(), vec.size());
		algo_->setWindows(afWindows);
	}
	if (afFContext.pause) {
		AfPauseEnum pauseValue = afFContext.pause.value();
		algo_->pause(pauseValue);
	}
	if (afFContext.trigger) {
		AfTriggerEnum trigger = afFContext.trigger.value();
		if (trigger == AfTriggerStart)
			algo_->triggerScan();
		else
			algo_->cancelScan();
	}

	if (afFContext.lensPosition) {
		int32_t hwPosition;
		float position = afFContext.lensPosition.value();
		algo_->setLensPosition(position, &hwPosition);
	}

	/* \todo populate PDAF data. */
	PdafRegions pdafRegions;
	AfStatus afStatus;
	algo_->doPrepare(pdafRegions, afStatus);

	afFContext.state = afStatus.state;
	afFContext.pauseState = afStatus.pauseState;

	afState.hwPositionUpdate = false;
	if (afStatus.lensSetting) {
		int32_t hwPosition = afStatus.lensSetting.value();
		if (!afState.hwPosition) {
			/* Lens position unknown - update unconditionally. */
			afState.hwPosition = hwPosition;
			afState.hwPositionUpdate = true;
		} else {
			/* Lens position known - update only if changed. */
			if (afState.hwPosition.value() != hwPosition) {
				afState.hwPosition = hwPosition;
				afState.hwPositionUpdate = true;
			}
		}
	}
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Af::process([[maybe_unused]] IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 [[maybe_unused]] const NxpNeoStats *stats,
		 ControlList &metadata)
{
	/* \todo populate CDAF statistics. */
	FocusRegions focusRegions;
	/* \todo populate AWB statistics. */
	RgbyRegions awbRegions;
	algo_->doProcess(focusRegions, awbRegions);

	/* Populate metadata */
	auto &afFContext = frameContext.af;
	metadata.set(controls::AfState, afFContext.state);
	metadata.set(controls::AfPauseState, afFContext.pauseState);
	std::optional<float> position = algo_->getLensPosition();
	if (position)
		metadata.set(controls::LensPosition, position.value());
}

void Af::windowsToNative(Span<Rectangle> windows,
			 uint16_t cropX, uint16_t cropY,
			 double scaleX, double scaleY)
{
	/*
	 * AutoFocus algorithm expects windows in native sensor coordinates.
	 * Thus, adjust the coordinates based on the sensor cropping and binning
	 * applicable to the current mode.
	 */
	for (Rectangle &window : windows) {
		window.x = window.x * scaleX + cropX;
		window.y = window.y * scaleY + cropY;
		window.width *= scaleX;
		window.height *= scaleY;
	}
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::nxpneo::algorithms */

} /* namespace libcamera */
