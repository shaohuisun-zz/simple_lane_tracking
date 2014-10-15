/*
 * track.cpp
 *
 *  Created on: Sep 10, 2014
 *      Author: Shaohui Sun
 */

#include <string>
#include <fstream>
#include "parabolicMask.h"
#include "line.h"
#include "laneExtraction.h"
#include "laneModeling.h"

int
main(int argc, char **argv)
{
    bool write2file = false; // flag to store results into images

    if (argc > 1)
    {
        std::string option(argv[1]);

        if (option == "--files")
        {
            write2file = true;
        }
        else
        {
            std::cout << "The available option is --files" << std::endl;
            return -1;
        }
    }

    cv::namedWindow("Display window", cv::WINDOW_NORMAL);

    /*
     * create the mask that can help remove unwanted noise
     */
    cv::Mat carmask = cv::imread("carmask.png", CV_LOAD_IMAGE_GRAYSCALE);
    aps::parabolicMask pmask(carmask.cols, carmask.rows,
            1.0 / carmask.rows);
    cv::Mat M = pmask.mkMask();
    M.convertTo(M, CV_8UC1);
    carmask.convertTo(carmask, CV_8UC1);
    carmask = carmask.mul(M);

    std::ifstream ifs("imnames.txt");
    std::ofstream ofs("intercepts.csv");
    std::string fname, ifname, ofname;
    while (std::getline(ifs, fname))
    {
        ofname = "results/" + fname;
        ifname = "images/" + fname;
        // std::cout << ifname << std::endl;

        cv::Mat img_org = cv::imread(ifname, CV_LOAD_IMAGE_GRAYSCALE);
        if (img_org.empty())
        {
            continue;
        }

        /*
         * extract line segments
         */
        aps::laneExtraction le;
        le.setLaneWidth(15);
        le.compute(img_org, carmask / 255);
        std::vector<aps::line> laneSegCandidates = le.getLaneCandidates();

        /*
         * model the lane
         */
        aps::laneModeling lm;
        lm.compute(laneSegCandidates);

        std::vector<aps::line> lanes = lm.getLanes();

        cv::cvtColor(img_org, img_org, CV_GRAY2BGR);
        ofs << fname << ",";
        if (lm.verifyLanes())
        {
            cv::line(img_org, lanes[0].getEndPoint1(), lanes[0].getEndPoint2(),
                    cv::Scalar(0, 255, 255), 5, CV_AA);
            cv::line(img_org, lanes[1].getEndPoint1(), lanes[1].getEndPoint2(),
                    cv::Scalar(0, 255, 255), 5, CV_AA);
            int x_left =
                    (lanes[0].getEndPoint2().x < lanes[1].getEndPoint2().x) ?
                            lanes[0].getEndPoint2().x :
                            lanes[1].getEndPoint2().x;
            int x_right =
                    (lanes[0].getEndPoint2().x > lanes[1].getEndPoint2().x) ?
                            lanes[0].getEndPoint2().x :
                            lanes[1].getEndPoint2().x;

            ofs << x_left << "," << x_right << std::endl;
        }
        else
        {
            ofs << "NONE,NONE\n";
        }

        if (write2file)
        {
            cv::imwrite(ofname, img_org);
        }

        cv::imshow("Display window", img_org);
        cv::waitKey(50);
    }
    ifs.close();
    ofs.close();

    cv::destroyAllWindows();

    return 0;
}

