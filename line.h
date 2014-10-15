/*
 * line.h
 *
 *  Created on: Apr 9, 2014
 *      Author: Shaohui Sun
 */

#ifndef LINE_H_
#define LINE_H_

#include <opencv2/core/core.hpp>
#include <math.h>

namespace aps
{
    class line
    {
    public:
        line();

        line& operator = ( const line& rhs );

        virtual
        ~line();

        /*
         *
         */
        void
        set(cv::Point p1, cv::Point p2);

        /*
         *
         */
        void
        set(cv::Point2f p1, cv::Point2f p2);

        /*
         *
         */
        const double
        getOrientation() const;

        /*
         *
         */
        const bool
        is_Set() const;

        /*
         *
         */
        const cv::Point&
        getEndPoint1() const
        {
            return m_endp1;
        }

        /*
         *
         */
        const cv::Point&
        getEndPoint2() const
        {
            return m_endp2;
        }

        /*
         *
         */
        double
        getLength()
        {
            return sqrt(pow(m_endp1.x - m_endp2.x, 2) + pow(m_endp1.y - m_endp2.y, 2));
        }

        /*
         *
         */
        bool
        is_Headup();

        /*
         *
         */
        const double
        getDistFromLine(const line& l) const;

        /*
         *
         */
        static double
        point2Line(const line& l, cv::Point p);

        /*
         *
         */
        std::vector<cv::Point>
        interpolatePoints(int step_size) const;

    private:
        /*
         *
         */
        double
        computeOrientation();

    private:
        cv::Point m_endp1;
        cv::Point m_endp2;
        bool isset;
        double m_orient;
    };

} /* namespace ssh */

#endif /* LINE_H_ */
