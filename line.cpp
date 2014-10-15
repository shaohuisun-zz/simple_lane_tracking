/*
 * line.cpp
 *
 *  Created on: Apr 9, 2014
 *      Author: Shaohui Sun
 */

#include "line.h"

namespace aps
{

    line::line()
    {
        // TODO Auto-generated constructor stub
        isset = false;
        m_orient = -1;
    }

    line &
    line::operator = (const line & rhs)
    {
        m_endp1 = rhs.m_endp1;
        m_endp2 = rhs.m_endp2;
        isset = rhs.isset;

        return *this;
    }

    line::~line()
    {
        // TODO Auto-generated destructor stub
    }

    void
    line::set(cv::Point p1, cv::Point p2)
    {
        m_endp1 = p1;
        m_endp2 = p2;
        isset = true;
        m_orient = this->computeOrientation();
    }

    void
    line::set(cv::Point2f p1, cv::Point2f p2)
    {
        m_endp1 = cv::Point(p1.x, p1.y);
        m_endp2 = cv::Point(p2.x, p2.y);
        isset = true;
        m_orient = this->computeOrientation();
    }

    const double
    line::getOrientation() const
    {
        return m_orient;
    }

    const bool
    line::is_Set() const
    {
        return isset;
    }

    bool
    line::is_Headup()
    {
        return (m_endp1.y < m_endp2.y) ? true: false;
    }

    const double
    line::getDistFromLine(const line& l) const
    {
        assert(this->is_Set() && l.is_Set());
        double this_ori = this->getOrientation();
        double other_ori = l.getOrientation();
        double diff_ori = fabs(this_ori - other_ori);

        double d = (point2Line(*this, l.m_endp1) <= point2Line(*this, l.m_endp2)) ?
                point2Line(*this, l.m_endp1): point2Line(*this, l.m_endp2);

        return d + 1.5*diff_ori; // 1.5 is a magic number

        return 0;
    }

    double
    line::point2Line(const line & l, cv::Point p)
    {
        int x1 = l.m_endp1.x;
        int y1 = l.m_endp1.y;
        int x2 = l.m_endp2.x;
        int y2 = l.m_endp2.y;

        return fabs((x2-x1)*(y1-p.y) - (x1-p.x)*(y2-y1))
                /sqrt(pow((x2-x1), 2) + pow((y2-y1), 2));
    }

    double
    line::computeOrientation()
    {
        int x = m_endp1.x - m_endp2.x;
        int y = m_endp2.y - m_endp1.y;

        double result = atan2(y, x + 1e-5) * 180 / CV_PI;

        if (result < 0)
            result = result + 180;

        return result;
    }

    std::vector<cv::Point>
    line::interpolatePoints(int step_size) const
    {
        std::vector<cv::Point> pl;
        pl.push_back(m_endp1);
        pl.push_back(m_endp2);

        double slope = tan((180-m_orient)*CV_PI/180);
        double intercept = m_endp1.y - slope*m_endp1.x;

        double start_x, end_x;
        int sign;
        if (m_endp1.x <= m_endp2.x)
        {
            start_x = m_endp1.x;
            end_x = m_endp2.x;
            sign = 1;
        }
        else
        {
            start_x = m_endp2.x;
            end_x = m_endp1.x;
            sign = -1;
        }

        int steps = (end_x - start_x) / step_size;

        if (steps < 1)
            return pl;

        for (int i = 1; i <= steps; i++)
        {
            int new_x = start_x + i*sign*step_size;
            int new_y = slope * new_x + intercept;
            pl.push_back(cv::Point(new_x, new_y));
        }

        return pl;
    }

} /* namespace ssh */
