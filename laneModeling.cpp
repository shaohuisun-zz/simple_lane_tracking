/*
 * laneModeling.cpp
 *
 *  Created on: Sep 11, 2014
 *      Author: shaohuisun
 */

#include "laneModeling.h"
#include <iostream>
#include <fstream>

namespace aps
{

    laneModeling::laneModeling() :
            m_success(false)
    {
        // TODO Auto-generated constructor stub
    }

    laneModeling::~laneModeling()
    {
        // TODO Auto-generated destructor stub
    }

    void
    laneModeling::compute(const std::vector<aps::line>& lines)
    {
        int clusterCount = 2;
        if (lines.empty() || lines.size() < clusterCount)
        {
            m_success = false;
            return;
        }

        cv::Mat points(lines.size(), 1, CV_32FC1);
        cv::Mat labels;
        cv::Mat centers(clusterCount, 1, CV_32FC1);

        for (size_t i = 0; i < lines.size(); i++)
        {
            points.at<float>(i, 0) = lines[i].getOrientation();
        }

        /*
         * put all line candidates into two clusters based on their orientations
         */
        cv::kmeans(points, clusterCount, labels,
                cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),
                3, cv::KMEANS_PP_CENTERS, centers);

        for (int k = 0; k < clusterCount; k++)
        {
            std::vector<cv::Point> candid_points;

            for (size_t i = 0; i < lines.size(); i++)
            {
                if (labels.at<int>(i, 0) == k)
                {
                    std::vector<cv::Point> pts = lines[i].interpolatePoints(10);
                    candid_points.insert(candid_points.end(), pts.begin(),
                            pts.end());
                }
            }

            if (candid_points.empty())
                continue;

            /*
             * fit the perfect line
             */
            aps::LineModel model;
            aps::RansacLine2D Ransac;
            Ransac.setObservationSet(candid_points);
            Ransac.setIterations(500);
            Ransac.setRequiredInliers(2);
            Ransac.setTreshold(1);
            Ransac.computeModel();
            Ransac.getBestModel(model);

            double m = model.mSlope, b = model.mIntercept;
            if (fabs(m) <= 1e-10 && fabs(b) <= 1e-10)
                continue;

            cv::Point p1((700 - b) / (m + 1e-5), 700);
            cv::Point p2((1200 - b) / (m + 1e-5), 1200);
            aps::line laneSide;
            laneSide.set(p1, p2);
            lanes.push_back(laneSide);
        }

        if (lanes.size() < 2)
        {
            m_success = false;
        }
        else
        {
            m_success = true;
        }

        return;
    }

    bool
    laneModeling::verifyLanes()
    {
        if (!m_success)
            return false;

        /*
         * two sides must not have similar orientations
         */
        if ((lanes[0].getOrientation() < 90 && lanes[1].getOrientation() < 90)
                || (lanes[0].getOrientation() > 90
                        && lanes[1].getOrientation() > 90))
            return false;

        /*
         * two sides must not cross each other
         */
        if ((lanes[0].getEndPoint1().x < lanes[1].getEndPoint1().x
                && lanes[0].getEndPoint2().x < lanes[1].getEndPoint2().x)
                || (lanes[0].getEndPoint1().x > lanes[1].getEndPoint1().x
                        && lanes[0].getEndPoint2().x > lanes[1].getEndPoint2().x))
            return true;
        else
            return false;
    }

} /* namespace aps */
