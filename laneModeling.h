/*
 * laneModeling.h
 *
 *  Created on: Sep 11, 2014
 *      Author: Shaohui Sun
 */

#ifndef LANEMODELING_H_
#define LANEMODELING_H_

#include <vector>
#include <opencv2/core/core.hpp>
#include "line.h"
#include "ransacLine2D.h"

namespace aps
{

    class laneModeling
    {
    public:
        laneModeling();
        virtual
        ~laneModeling();

        void
        compute(const std::vector<aps::line>& lines);

        const std::vector<aps::line> &
        getLanes()
        {
            return lanes;
        }

        bool
        verifyLanes();

    private:
        std::vector<aps::line> lanes;
        bool m_success;
    };

} /* namespace aps */

#endif /* LANEMODELING_H_ */
