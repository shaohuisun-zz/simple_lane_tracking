/*
 * ransacLine2D.h
 *
 *  Created on: Sep 12, 2014
 *      Author: Shaohui Sun
 */

#ifndef RANSACLINE2D_H_
#define RANSACLINE2D_H_

#include <stdlib.h>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <vector>
#include <time.h>
#include <iostream>

namespace aps
{
    class LineModel
    {
    public:
        LineModel() :
                mSlope(0), mIntercept(0)
        {
        }

        LineModel(double _slope, double _intercept) :
                mSlope(_slope), mIntercept(_intercept)
        {
        }

        double mSlope;
        double mIntercept;
    };

    class RansacLine2D
    {
    public:

        RansacLine2D();

        virtual
        ~RansacLine2D();

        void
        setObservationSet(const std::vector<cv::Point>& observationSet)
        {

            mObservationSet = observationSet;
        }

        void
        setTreshold(const float sigma)
        {
            mThreshold = sigma;
        }

        void
        setIterations(const int Iterations)
        {
            mIterations = Iterations;
        }

        void
        setRequiredInliers(const int requiredInliers)
        {
            mRequiredInliers = requiredInliers;
        }

        bool
        computeModel();

        void
        getBestModel(LineModel& bestLineModel)
        {
            bestLineModel.mSlope = mBestModel.mSlope;
            bestLineModel.mIntercept = mBestModel.mIntercept;
        }

        int
        getBestRank()
        {
            return mBestRank;
        }

    private:
        bool
        isMember(const cv::Point& point,
                const std::vector<cv::Point>& maybeInliers) const;
        bool
        fitsModel(const cv::Point& point, const LineModel& model) const;

        LineModel
        getModel(const cv::Point& p1, const cv::Point& p2) const;

        LineModel
        getModel(const std::vector<cv::Point>& observation) const;

        std::vector<cv::Point>
        getMaybeInliers() const;

        int
        getModelRank(const std::vector<cv::Point>& pointList,
                const LineModel& model) const;
        double
        getDistance(const cv::Point& p, const LineModel& model) const;

        bool
        isdegenerate(cv::Point p1, cv::Point p2);

    private:
        std::vector<cv::Point> mObservationSet;
        std::vector<cv::Point> mBestConsensusSet;
        LineModel mBestModel;
        int mRequiredInliers;
        int mIterations;
        int mBestRank;
        double mThreshold;

    };

} /* namespace cruise */

#endif /* RANSACLINE2D_H_ */
