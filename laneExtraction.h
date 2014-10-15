/*
 * laneExtraction.h
 *
 *  Created on: Sep 13, 2014
 *      Author: Shaohui Sun
 */

#ifndef LANEEXTRACTION_H_
#define LANEEXTRACTION_H_

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "line.h"

namespace aps
{

    class laneExtraction
    {
    public:
        laneExtraction();
        virtual
        ~laneExtraction();

        void
        compute(const cv::Mat img, const cv::Mat mask);

        void
        setLaneWidth(int l)
        {
            m_laneWidth = l;
        }

        std::vector<aps::line>&
        getLaneCandidates()
        {
            return m_laneSegCandid;
        }

    private:
        /*
         * the method adopted from the reference in REPORT.md
         */
        void
        detectMarkers(const cv::Mat img, int tau);

    private:
        cv::Mat m_IMMarker; // map with pruned markers
        std::vector<aps::line> m_laneSegCandid;

        int m_laneWidth;
    };

} /* namespace aps */

#endif /* LANEEXTRACTION_H_ */
