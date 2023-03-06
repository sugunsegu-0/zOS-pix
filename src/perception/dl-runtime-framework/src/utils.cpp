#include "utils.hpp"

// #include "torch/torch.h"

// #include <opencv2/opencv.hpp>
// #include <opencv2/core/types.hpp>
// #include <opencv2/core/cuda.hpp>
// #include <opencv2/dnn.hpp>
// #include <opencv2/imgproc/types_c.h>

// using namespace cv;
// using namespace cv::dnn;

// cv::Point performHomography(cv::Point point)
// {
//     int32_t px=((-1.07891492e-01 * point.x) + (-1.04808878e+00 * point.y) + 3.30456227e+02) / ((9.15476605e-20 * point.x) + (-3.52188656e-03 * point.y) + 1.00000000e+00);
//     int32_t py=((2.95695650e-17 * point.x) + (-1.89395808e+00 * point.y) + 5.77657213e+02) / ((9.15476605e-20 * point.x) + (-3.52188656e-03 * point.y) + 1.00000000e+00);

//     cv::Point homoPoint(px, py);
//     return homoPoint;
// }

// bool BatchNormalization(cv::InputArrayOfArrays images_, cv::OutputArray blob_, double scalefactor,
//                         cv::Size size, const cv::Scalar& mean_, const cv::Scalar& stdDev_, bool swapRB, bool crop, int ddepth)
// {
//     // CV_TRACE_FUNCTION();
//     CV_CheckType(ddepth, ddepth == CV_32F || ddepth == CV_8U, "Blob depth should be CV_32F or CV_8U");
//     if (ddepth == CV_8U)
//     {
//         // CV_CheckEQ(scalefactor, 1.0, "Scaling is not supported for CV_8U blob depth");
//         CV_Assert(mean_ == cv::Scalar() && "Mean subtraction is not supported for CV_8U blob depth");
//         CV_Assert(stdDev_ == cv::Scalar() && "Std division is not supported for CV_8U blob depth");
//     }

//     // std::cout << "Here" << std::endl;

//     std::vector<Mat> images;
//     images_.getMatVector(images);
//     CV_Assert(!images.empty());
//     for (size_t i = 0; i < images.size(); i++)
//     {
//         cv::Size imgSize = images[i].size();
//         if (size == cv::Size())
//             size = imgSize;
//         if (size != imgSize)
//         {
//             if (crop)
//             {
//                 float resizeFactor = std::max(size.width / (float)imgSize.width,
//                         size.height / (float)imgSize.height);
//                 cv::resize(images[i], images[i], Size(), resizeFactor, resizeFactor, INTER_LINEAR);
//                 cv::Rect crop(Point(0.5 * (images[i].cols - size.width),
//                                 0.5 * (images[i].rows - size.height)),
//                         size);
//                 images[i] = images[i](crop);
//             }
//             else
//                 cv::resize(images[i], images[i], size, 0, 0, INTER_LINEAR);
//         }
//         if (images[i].depth() == CV_8U && ddepth == CV_32F)
//             images[i].convertTo(images[i], CV_32F);
//         cv::Scalar mean = mean_;
//         cv::Scalar stdDev = stdDev_;
//         // Scalar scaleFactor = scalefactor_;
//         if (swapRB)
//             std::swap(mean[0], mean[2]);
//             std::swap(stdDev[0], stdDev[2]);
//             // std::swap(scaleFactor[0], scaleFactor[2]);

//         images[i] -= mean;
//         images[i] /= stdDev;

//         // std::cout << "Here" << std::endl;

//         // images[i] *= scaleFactor;
//         images[i] *= scalefactor;
//         // std::cout << "Put" << std::endl;
//     }

//     size_t nimages = images.size();
//     cv::Mat image0 = images[0];
//     int nch = image0.channels();
//     CV_Assert(image0.dims == 2);
//     if (nch == 3 || nch == 4)
//     {
//         int sz[] = { (int)nimages, nch, image0.rows, image0.cols };
//         blob_.create(4, sz, ddepth);
//         cv::Mat blob = blob_.getMat();
//         cv::Mat ch[4];

//         for (size_t i = 0; i < nimages; i++)
//         {
//             const cv::Mat& image = images[i];
//             CV_Assert(image.depth() == blob_.depth());
//             nch = image.channels();
//             CV_Assert(image.dims == 2 && (nch == 3 || nch == 4));
//             CV_Assert(image.size() == image0.size());

//             for (int j = 0; j < nch; j++)
//                 ch[j] = cv::Mat(image.rows, image.cols, ddepth, blob.ptr((int)i, j));
//             if (swapRB)
//                 std::swap(ch[0], ch[2]);
//             cv::split(image, ch);
//         }
//     }
//     else
//     {
//         CV_Assert(nch == 1);
//         int sz[] = { (int)nimages, 1, image0.rows, image0.cols };
//         blob_.create(4, sz, ddepth);
//         cv::Mat blob = blob_.getMat();

//         for (size_t i = 0; i < nimages; i++)
//         {
//             const cv::Mat& image = images[i];
//             CV_Assert(image.depth() == blob_.depth());
//             nch = image.channels();
//             CV_Assert(image.dims == 2 && (nch == 1));
//             CV_Assert(image.size() == image0.size());

//             image.copyTo(cv::Mat(image.rows, image.cols, ddepth, blob.ptr((int)i, 0)));
//         }
//     }

//     return true;
// }

// bool CVtoNormalizedGPUTensor(cv::Mat img, torch::Tensor& imgTensor
//                          //    float& mean, float& std
// )
// {
//     auto tensorOptions = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA);
//     imgTensor = torch::from_blob(img.data, { img.rows, img.cols, 3 }, torch::TensorOptions().device(torch::kCUDA));
//     imgTensor = imgTensor.permute({ 2, 0, 1 });
//     // imgTensor[0] = imgTensor[0].sub(*(&mean+0)).div(*(&std+0));
// 	// imgTensor[1] = imgTensor[1].sub(*(&mean+1)).div(*(&std+1));
// 	// imgTensor[2] = imgTensor[2].sub(*(&mean+2)).div(*(&std+2));
//     imgTensor[0] = imgTensor[0].sub(0.5).div(0.5);
// 	imgTensor[1] = imgTensor[1].sub(0.5).div(0.5);
// 	imgTensor[2] = imgTensor[2].sub(0.5).div(0.5);
//     return true;
// }

