#include <Kinect.h>
#include <stdexcept>
#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>

cv::Mat ModDepthForDisplay(const cv::Mat& mat)
{
    const float depth_near = 500;
    const float depth_far = 1500;

    const float alpha = 255.0f / (depth_far - depth_near);
    const float beta = -depth_near * alpha;

    cv::Mat fmat;
    mat.convertTo(fmat, CV_32F);

    for (int r = 0; r < mat.rows; ++r)
    {
        for (int c = 0; c < mat.cols; ++c)
        {
            float v = fmat.at<float>(r, c) * alpha + beta;

            if (v > 255) v = 255;
            if (v < 0)   v = 0;

            fmat.at<float>(r, c) = v;
        }
    }

    cv::Mat bmat;
    fmat.convertTo(bmat, CV_8U);

    cv::Mat cmat;
    cv::cvtColor(bmat, cmat, CV_GRAY2BGR);
    cv::applyColorMap(cmat, cmat, cv::COLORMAP_OCEAN);

    return cmat;
}

// Safe release for Windows interfaces
template<class Interface>
inline void SafeRelease(Interface*& ptr_int)
{
    if (ptr_int)
    {
        ptr_int->Release();
        ptr_int = nullptr;
    }
}

class App
{
public:
    App()
    {
        HRESULT hr = GetDefaultKinectSensor(&kin_sensor_);
        if (FAILED(hr))
        {
            throw std::runtime_error("Kinect sensor could not be found!");
        }

        if (kin_sensor_)
        {
            InitDepthSource();
            InitColorSource();
        }

        if (!kin_sensor_ || FAILED(hr))
        {
            throw std::runtime_error("Kinect init failed!");
        }
    }

    ~App()
    {
        SafeRelease(color_frame_reader_);
        SafeRelease(depth_frame_reader_);

        // Close the Kinect Sensor
        if (kin_sensor_)
        {
            kin_sensor_->Close();
        }

        SafeRelease(kin_sensor_);
    }

    void Run()
    {
        while (true)
        {
            ShowDepthFrame();
            ShowColorFrame();

            char c = cv::waitKey(10);
            if ('q' == c)
                break;
        }
    }

private:

    void InitDepthSource()
    {
        IDepthFrameSource* depth_frame_source = nullptr;

        HRESULT hr = kin_sensor_->Open();

        if (SUCCEEDED(hr))
        {
            hr = kin_sensor_->get_DepthFrameSource(&depth_frame_source);
        }

        if (SUCCEEDED(hr))
        {
            hr = depth_frame_source->OpenReader(&depth_frame_reader_);
        }

        SafeRelease(depth_frame_source);
    }

    void InitColorSource()
    {
        IColorFrameSource* color_frame_source = nullptr;

        HRESULT hr = kin_sensor_->Open();

        if (SUCCEEDED(hr))
        {
            hr = kin_sensor_->get_ColorFrameSource(&color_frame_source);
        }

        if (SUCCEEDED(hr))
        {
            hr = color_frame_source->OpenReader(&color_frame_reader_);
        }

        SafeRelease(color_frame_source);
    }

    IDepthFrame* WaitForDepthFrame()
    {
        while (true)
        {
            IDepthFrame* depth_frame = nullptr;
            HRESULT hr = depth_frame_reader_->AcquireLatestFrame(&depth_frame);

            if (SUCCEEDED(hr))
            {
                return depth_frame;
            }

            SafeRelease(depth_frame);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    IColorFrame* WaitForColorFrame()
    {
        while (true)
        {
            IColorFrame* color_frame = nullptr;
            HRESULT hr = color_frame_reader_->AcquireLatestFrame(&color_frame);

            if (SUCCEEDED(hr))
            {
                return color_frame;
            }

            SafeRelease(color_frame);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void ShowDepthFrame()
    {
        IDepthFrame* depth_frame = WaitForDepthFrame();

        IFrameDescription* frame_desc = nullptr;
        UINT nBufferSize = 0;
        UINT16 *pBuffer = NULL;

        HRESULT hr = depth_frame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);

        cv::Mat depth_mat(depth_h_, depth_w_, CV_16UC1);

        memcpy(depth_mat.data, pBuffer, nBufferSize * sizeof(uint16_t));

        cv::Mat disp_mat = ModDepthForDisplay(depth_mat);
        cv::imshow("Depth", disp_mat);

        SafeRelease(depth_frame);
    }

    void ShowColorFrame()
    {
        IColorFrame* color_frame = WaitForColorFrame();
        ColorImageFormat imageFormat = ColorImageFormat_None;
        HRESULT hr = color_frame->get_RawColorImageFormat(&imageFormat);

        cv::Mat color_mat(color_h_, color_w_, CV_8UC4);
        const int buf_size = color_h_ * color_w_ * sizeof(uint8_t) * 4;
        hr = color_frame->CopyConvertedFrameDataToArray(buf_size, color_mat.data, ColorImageFormat_Bgra);

        cv::imshow("Color", color_mat);

        SafeRelease(color_frame);
    }

    static const int depth_w_ = 512;
    static const int depth_h_ = 424;
    static const int color_w_ = 1920;
    static const int color_h_ = 1080;

    IKinectSensor* kin_sensor_;
    IDepthFrameReader* depth_frame_reader_;
    IColorFrameReader* color_frame_reader_;
};

int main()
{
    App app;
    app.Run();

    return 0;
}