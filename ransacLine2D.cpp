/*
 * ransacLine2D.cpp
 *
 *  Created on: Sep 12, 2014
 *      Author: Shaohui Sun
 */

#include "ransacLine2D.h"

namespace aps
{

    RansacLine2D::RansacLine2D() :
            mRequiredInliers(0), mIterations(0), mBestRank(0), mThreshold(0)
    {
        srand(time(NULL));
    }

    RansacLine2D::~RansacLine2D()
    {
        // TODO Auto-generated destructor stub
    }

    /*
     * compute
     */
    bool
    RansacLine2D::computeModel()
    {
        int foundModel = false;
        int iterations = 0;

        while (iterations < mIterations)
        {

            std::vector<cv::Point> maybeInliers = getMaybeInliers();
            if (isdegenerate(maybeInliers[0], maybeInliers[1]))
            {
                iterations++;
                continue;
            }

            std::vector<cv::Point> consensusSet = maybeInliers;
            LineModel model = getModel(maybeInliers[0], maybeInliers[1]);

            for (int i = 0; i < mObservationSet.size(); i++)
            {

                if (!isMember(mObservationSet[i], maybeInliers))
                {

                    if (fitsModel(mObservationSet[i], model))
                    {
                        consensusSet.push_back(mObservationSet[i]);
                    }
                }
            }

            if (consensusSet.size() >= mRequiredInliers)
            {

                model = getModel(consensusSet);
                int rank = getModelRank(consensusSet, model);

                if (rank > mBestRank)
                {
                    mBestConsensusSet = consensusSet;
                    mBestModel = model;
                    mBestRank = rank;
                    foundModel = true;
                }
            }
            iterations++;
        }
        return foundModel;
    }

    /*
     * get two random points from observation-set
     */
    std::vector<cv::Point>
    RansacLine2D::getMaybeInliers() const
    {
        std::vector<cv::Point> maybeInliers;

        double listSize = mObservationSet.size();
        int randPointIndex_0 = (int) (listSize * rand() / RAND_MAX - 1.0);
        int randPointIndex_1 = (int) (listSize * rand() / RAND_MAX - 1.0);

        maybeInliers.push_back(mObservationSet[randPointIndex_0]);
        maybeInliers.push_back(mObservationSet[randPointIndex_1]);

        return maybeInliers;
    }

    /*
     * get linear equation parameters (slope and intercept) from two points
     */
    LineModel
    RansacLine2D::getModel(const cv::Point& p0, const cv::Point& p1) const
    {
        LineModel model;

        model.mSlope = (p1.y - p0.y) / (p1.x - p0.x + 1e-10);
        model.mIntercept = p0.y - model.mSlope * p0.x;

        return model;
    }

    /*
     * get linear equation parameters (slope and intercept) from observation data
     */
    LineModel
    RansacLine2D::getModel(const std::vector<cv::Point>& observation) const
    {
        LineModel model;

        unsigned int noDataPoints = observation.size();

        cv::Mat valMat(noDataPoints, 2, CV_64FC1);
        cv::Mat vecMat(noDataPoints, 1, CV_64FC1);
        cv::Mat dest(2, 1, CV_64FC1);

        for (unsigned int i = 0; i < noDataPoints; i++)
        {
            valMat.at<double>(i, 0) = observation[i].x;
            valMat.at<double>(i, 1) = 1;
            vecMat.at<double>(i, 0) = observation[i].y;
        }

        cv::solve(valMat, vecMat, dest, cv::DECOMP_SVD);

        model.mIntercept = dest.at<double>(1, 0);
        model.mSlope = dest.at<double>(0, 0);

        return model;
    }

    /*
     * check whether point is a member of list
     */
    bool
    RansacLine2D::isMember(const cv::Point& p,
            const std::vector<cv::Point>& pList) const
    {
        bool isMember = false;

        for (int i = 0; i < pList.size(); i++)
        {
            if (p == pList[i])
            {
                isMember = true;
            }
        }
        return isMember;
    }

    /*
     * get distance between point and model
     */

    double
    RansacLine2D::getDistance(const cv::Point& p, const LineModel& model) const
    {
        double distance;

        distance = pow((model.mSlope * p.x - p.y + model.mIntercept), 2)
                / (pow(model.mSlope, 2) + 1);

        return distance;
    }

    /*
     * check whether distance of point from model is smaller than a given threshold
     */
    bool
    RansacLine2D::fitsModel(const cv::Point& p, const LineModel& model) const
    {
        bool fits = false;

        double distance = getDistance(p, model);

        if (distance < mThreshold)
        {
            fits = true;
        }

        return fits;
    }

    /*
     * Model rank ~ maximum points fitting the model
     */
    int
    RansacLine2D::getModelRank(const std::vector<cv::Point>& pointList,
            const LineModel& model) const
    {
        std::vector<cv::Point> consensusSet;

        for (unsigned int i = 0; i < pointList.size(); i++)
        {

            if (fitsModel(pointList[i], model))
            {
                consensusSet.push_back(pointList[i]);
            }
        }
        return consensusSet.size();
    }

    /*
     * two points that are super close should not be used to estimate a model
     */
    bool
    RansacLine2D::isdegenerate(cv::Point p1, cv::Point p2)
    {
        double norm = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));

        if (norm < 1.0)
        {
            return true;
        }
        return false;
    }

} /* namespace cruise */
