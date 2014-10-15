/*
 * parabolicMask.h
 *
 *  Created on: Sep 11, 2014
 *      Author: shaohuisun
 */

#ifndef PARABOLICMASK_H_
#define PARABOLICMASK_H_

/*
 *  parabolic curve is defined as y = s * (x - a)^2 + b
 */

#include <opencv2/core/core.hpp>

namespace aps
{

    class parabolicMask
    {
    public:
        parabolicMask();

        parabolicMask(double w, double h, double s);

        virtual
        ~parabolicMask();

        const cv::Mat &
        mkMask();


    private:
        double m_a, m_b, m_s;
        int m_width, m_height;

        cv::Mat m_mask;
    };

} /* namespace cruise */

#endif /* PARABOLICMASK_H_ */
