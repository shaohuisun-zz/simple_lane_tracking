/*
 * laneExtraction.cpp
 *
 *  Created on: Sep 13, 2014
 *      Author: Shaohui Sun
 */

#include "laneExtraction.h"

namespace aps
{

    laneExtraction::laneExtraction() :
            m_laneWidth(15)
    {
        // TODO Auto-generated constructor stub

    }

    laneExtraction::~laneExtraction()
    {
        // TODO Auto-generated destructor stub
    }

    void
    laneExtraction::compute(const cv::Mat img, const cv::Mat mask)
    {
        detectMarkers(img, m_laneWidth);

        m_IMMarker = m_IMMarker.mul(mask);
        cv::threshold(m_IMMarker, m_IMMarker, 25, 255, CV_THRESH_BINARY);

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(m_IMMarker, lines, 1, CV_PI / 180, 40, 20, 10);

        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            aps::line seg;
            seg.set(cv::Point(l[0], l[1]), cv::Point(l[2], l[3]));

            if ((seg.getOrientation() > 10 && seg.getOrientation() < 80)
                    || (seg.getOrientation() > 100 && seg.getOrientation() < 170))
            {
                m_laneSegCandid.push_back(seg);
            }
        }
        return;
    }

    void
    laneExtraction::detectMarkers(const cv::Mat img, int tau)
    {
        m_IMMarker.create(img.rows, img.cols, CV_8UC1);
        m_IMMarker.setTo(0);

        int aux = 0;
        for (int j = 0; j < img.rows; j++)
        {
            for (int i = tau; i < img.cols - tau; i++)
            {
                if (img.at<uchar>(j, i) != 0)
                {
                    aux = 2 * img.at<uchar>(j, i);
                    aux += -img.at<uchar>(j, i - tau);
                    aux += -img.at<uchar>(j, i + tau);

                    aux += -abs(
                            (int) (img.at<uchar>(j, i - tau)
                                    - img.at<uchar>(j, i + tau)));

                    aux = (aux < 0) ? (0) : (aux);
                    aux = (aux > 255) ? (255) : (aux);

                    m_IMMarker.at<uchar>(j, i) = (unsigned char) aux;
                }
            }
        }
        return;
    }

} /* namespace aps */
