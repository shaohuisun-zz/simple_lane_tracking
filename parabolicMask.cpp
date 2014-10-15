/*
 * parabolicMask.cpp
 *
 *  Created on: Sep 11, 2014
 *      Author: shaohuisun
 */

#include "parabolicMask.h"
#include <iostream>

namespace aps
{

    parabolicMask::parabolicMask():
            m_a(0.0), m_b(0.0), m_s(0.0), m_width(0), m_height(0)
    {
        // TODO Auto-generated constructor stub

    }

    parabolicMask::parabolicMask(double w, double h, double s)
    {
        m_a = w/2;
        m_b = h/2+50;
        m_s = s;
        m_width = w;
        m_height = h;
    }

    parabolicMask::~parabolicMask()
    {
        // TODO Auto-generated destructor stub
    }

    const cv::Mat &
    parabolicMask::mkMask()
    {
        // std::cout<<m_s<<" "<<m_a<<" "<<m_b<<std::endl;
        m_mask.create(m_height, m_width, CV_64FC1);

        for (int i = 0; i < m_height; i++)
        {
            for (int j = 0; j < m_width; j++)
            {
                double res = i - (m_s*(j - m_a)*(j - m_a) + m_b);

                if (res >= 0)
                    m_mask.at<double>(i, j) = 1.0;
                else
                    m_mask.at<double>(i, j) = 0.0;
            }
        }
        return m_mask;
    }

} /* namespace cruise */
