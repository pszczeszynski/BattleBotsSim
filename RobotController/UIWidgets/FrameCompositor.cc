#include "FrameCompositor.h"

namespace {

void ApplyTintBgr(const cv::Mat& srcBgr, const cv::Scalar& bgrColor,
                  cv::Mat& outBgr) {
  std::vector<cv::Mat> channels(3);
  cv::split(srcBgr, channels);
  channels[0] *= (bgrColor[0] / 255.0);
  channels[1] *= (bgrColor[1] / 255.0);
  channels[2] *= (bgrColor[2] / 255.0);
  cv::merge(channels, outBgr);
}

}  // namespace

void FrameCompositor::ResetScratch(cv::Size outputSize) {
  if (outputSize.width <= 0 || outputSize.height <= 0) return;
  colorized_.create(outputSize, CV_8UC3);
  mask_.create(outputSize, CV_8UC1);
  temp3channel_.create(outputSize, CV_8UC3);
  grayTemp_.create(outputSize, CV_8UC1);
  blended_.create(outputSize, CV_8UC3);
  maskedColorized_.create(outputSize, CV_8UC3);
  inverseMask_.create(outputSize, CV_8UC1);
}

void FrameCompositor::Compose(const std::vector<VariantLayer>& layers,
                              cv::Size outputSize, cv::Mat& outBgr) {
  if (outputSize.width <= 0 || outputSize.height <= 0) {
    outBgr = cv::Mat();
    return;
  }

  ResetScratch(outputSize);
  outBgr = cv::Mat::zeros(outputSize, CV_8UC3);

  for (const auto& layer : layers) {
    if (!layer.visible || layer.image == nullptr || layer.image->empty()) {
      continue;
    }

    const cv::Mat& srcImage = *layer.image;
    if (srcImage.cols != outputSize.width ||
        srcImage.rows != outputSize.height) {
      continue;
    }

    cv::Scalar bgrColor(layer.color.z * 255, layer.color.y * 255,
                        layer.color.x * 255);
    const float alpha = layer.color.w;

    if (srcImage.type() == CV_8UC1) {
      cv::cvtColor(srcImage, temp3channel_, cv::COLOR_GRAY2BGR);
      ApplyTintBgr(temp3channel_, bgrColor, colorized_);
      cv::threshold(srcImage, mask_, 0, 255, cv::THRESH_BINARY);
    } else if (srcImage.type() == CV_8UC3) {
      srcImage.copyTo(colorized_);
      ApplyTintBgr(colorized_, bgrColor, colorized_);
      cv::cvtColor(srcImage, grayTemp_, cv::COLOR_BGR2GRAY);
      cv::threshold(grayTemp_, mask_, 0, 255, cv::THRESH_BINARY);
    } else {
      continue;
    }

    if (alpha >= 0.99f) {
      colorized_.copyTo(outBgr, mask_);
    } else {
      maskedColorized_.setTo(cv::Scalar(0, 0, 0));
      colorized_.copyTo(maskedColorized_, mask_);
      cv::bitwise_not(mask_, inverseMask_);
      cv::addWeighted(outBgr, 1.0 - alpha, maskedColorized_, alpha, 0,
                      blended_);
      blended_.copyTo(outBgr, mask_);
    }
  }
}
