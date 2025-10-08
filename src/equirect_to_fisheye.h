#ifndef THETA_FISHEYE_CONVERTER_H
#define THETA_FISHEYE_CONVERTER_H

#include "common.h"
#include <cmath>

using namespace cv;
using namespace std;

class ThetaFisheyeConverter {
private:
    static inline int wrapX(int x, int width) {
        int m = x % width;
        if (m < 0) m += width;
        return m;
    }

    static Vec3b sampleEquirectBilinear(const cv::Mat &img, double u, double v) {
        int W = img.cols;
        int H = img.rows;
        
        u = u - floor(u);
        if (u < 0.0) u += 1.0;
        if (v < 0.0) v = 0.0;
        if (v > 1.0) v = 1.0;

        double fx = u * (W - 1.0);
        double fy = v * (H - 1.0);
        int x0 = (int)floor(fx);
        int y0 = (int)floor(fy);
        int x1 = x0 + 1;
        int y1 = y0 + 1;

        double wx = fx - x0;
        double wy = fy - y0;

        x0 = wrapX(x0, W);
        x1 = wrapX(x1, W);
        y0 = std::min(std::max(y0, 0), H - 1);
        y1 = std::min(std::max(y1, 0), H - 1);

        Vec3b c00 = img.at<Vec3b>(y0, x0);
        Vec3b c10 = img.at<Vec3b>(y0, x1);
        Vec3b c01 = img.at<Vec3b>(y1, x0);
        Vec3b c11 = img.at<Vec3b>(y1, x1);

        Vec3b out;
        for (int k = 0; k < 3; ++k) {
            double v00 = c00[k], v10 = c10[k], v01 = c01[k], v11 = c11[k];
            double v0 = v00 * (1.0 - wx) + v10 * wx;
            double v1 = v01 * (1.0 - wx) + v11 * wx;
            double vv = v0 * (1.0 - wy) + v1 * wy;
            out[k] = (uchar)std::clamp((int)round(vv), 0, 255);
        }
        return out;
    }

public:
    static cv::Mat makeFisheyeFromEquirect(const cv::Mat &equi, int diameter, double yawRadians) {
        cv::Mat fisheye(diameter, diameter, CV_8UC3, cv::Scalar(0, 0, 0));
        double cx = (diameter - 1.0) * 0.5;
        double cy = (diameter - 1.0) * 0.5;
        double radius = (diameter - 1.0) * 0.5;

        const double FULL_PI = M_PI;
        double cosYaw = cos(yawRadians);
        double sinYaw = sin(yawRadians);

        for (int y = 0; y < diameter; ++y) {
            for (int x = 0; x < diameter; ++x) {
                double nx = (x - cx) / radius;
                double ny = (cy - y) / radius;
                double r = sqrt(nx*nx + ny*ny);
                if (r > 1.0) continue;

                double theta = r * FULL_PI / 2.0;
                double phi = atan2(ny, nx);

                double sinT = sin(theta);
                double camX = sinT * cos(phi);
                double camY = sinT * sin(phi);
                double camZ = cos(theta);

                double worldX =  cosYaw * camX + sinYaw * camZ;
                double worldY =  camY;
                double worldZ = -sinYaw * camX + cosYaw * camZ;

                double lon = atan2(worldX, worldZ);
                double lat = asin(std::clamp(worldY, -1.0, 1.0));

                double u = (lon + M_PI) / (2.0 * M_PI);
                double v = (M_PI/2.0 - lat) / M_PI;

                Vec3b sample = sampleEquirectBilinear(equi, u, v);
                fisheye.at<Vec3b>(y, x) = sample;
            }
        }
        return fisheye;
    }

    static void convertThetaToFisheye(const cv::Mat &equirectangular, 
                                      cv::Mat &front_fisheye, 
                                      cv::Mat &back_fisheye,
                                      int diameter = -1) {
        if (equirectangular.empty()) {
            std::cerr << "Error: Empty equirectangular image" << std::endl;
            return;
        }

        int W = equirectangular.cols;
        int H = equirectangular.rows;
        
        if (diameter <= 0) {
            diameter = std::min(H, W / 2);
        }

        front_fisheye = makeFisheyeFromEquirect(equirectangular, diameter, 0.0);
        back_fisheye = makeFisheyeFromEquirect(equirectangular, diameter, M_PI);
    }
};

#endif // THETA_FISHEYE_CONVERTER_H