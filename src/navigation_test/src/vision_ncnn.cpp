#include "vision_ncnn.h"

cv::Mat global_image;
struct object_rect
{
    int x;
    int y;
    int width;
    int height;
};

Picodet_vision::Picodet_vision()
{
    this->cap = cv::VideoCapture(0);
    if (this->cap.isOpened())
        cout << "Camera loaded successfully" << endl;
    else
        cout << "Camera loading failed" << endl;
    for (const auto &str : CNAME)
        class_names.push_back(str.c_str());
    cout << "Param Path: " << param << endl;
    cout << "Bin Path: " << bin << endl;
    cout << "Model Size W: " << model_size_w << endl;
    cout << "Score Threshold: " << score_threshold << endl;
    cout << "NMS Threshold: " << nms_threshold << endl;
    cout << "class_names: ";
    for (const auto &val : class_names)
        cout << val << " ";
    cout << endl;
    cout << "class_order: ";
    for (int val : class_order)
        cout << val << " ";
    cout << endl;
    cout << "BoxLim: ";
    for (int val : BoxLim)
        cout << val << " ";
    cout << endl;
};

Picodet_vision::Picodet_vision(cv::VideoCapture &CAP)
{
    cap = CAP;
    if (cap.isOpened())
        std::cout << "Camera loaded successfully" << std::endl;
    else
        std::cout << "Camera loading failed" << std::endl;
    for (const auto &str : CNAME)
        class_names.push_back(str.c_str());
    cout << "Param Path: " << param << endl;
    cout << "Bin Path: " << bin << endl;
    cout << "Model Size W: " << model_size_w << endl;
    cout << "Score Threshold: " << score_threshold << endl;
    cout << "NMS Threshold: " << nms_threshold << endl;
    cout << "class_names: ";
    for (const auto &val : class_names)
        cout << val << " ";
    cout << endl;
    cout << "class_order: ";
    for (int val : class_order)
        cout << val << " ";
    cout << endl;
    cout << "BoxLim: ";
    for (int val : BoxLim)
        cout << val << " ";
    cout << endl;
};

void Picodet_vision::get_image() { cap >> image; } // 显而易见的封装了等于没封装大概

void Picodet_vision::to_detect()
{
    cv::Mat A;
    resize_uniform(image, A, cv::Size(this->model_size_w, this->model_size_w));
    this->Results.clear();
    this->detector.detect(A, this->Results, score_threshold, nms_threshold);
}
// ////////////////////////////////////////////////////////////////////////////////////////////////////////
// std::vector<Clocassion> Picodet_vision::get_label()
// {
//     auto CL = std::vector<Clocassion>(Results.size());

//     for (int i = 0; i < Results.size(); i++)
//     {

//         if ((Results[i].x2 - Results[i].x1) > BoxLim[0] || (Results[i].y2 - Results[i].y1) > BoxLim[1])
//         {
//             CL[i].label = null_class;
//             continue;
//         }
//         int C = this->Results[i].label;
//         if (4 == C) //(3 == C) //(0 == C)
//         {
//             C = baton;
//         }
//         else if (3 == C || 2 == C) //(7 == C || 2 == C) //(1 == C || 2 == C)
//         {
//             C = body_armor;
//         }
//         else if (0 == C || 1 == C) //(3 == C || 4 == C)
//         {
//             C = teargas;
//         }
//         else if (6 == C) //(4 == C) //(5 == C)
//         {
//             C = terrorists_1;
//         } // 13
//         else if (5 == C) //(5 == C) //(6 == C)
//         {
//             C = terrorists_2;
//         } // 7
//         else if (7 == C) //(6 == C)
//         {
//             C = terrorists_3;
//         }
//         CL[i].label = C;
//         CL[i].location = (Results[i].x2 + Results[i].x1) / 2; //*2
//         CL[i].cx1 = Results[i].x1;
//         CL[i].cy1 = (Results[i].y2 + Results[i].y1) / 2;
//         CL[i].cx2 = CL[i].location; // Results[i].x2; //
//         CL[i].cy2 = Results[i].y2;
//     }
//     return CL;
// };

// int Terr[4] = {0, 0, 0, 0};
// int Terr_Times = 1;
// Clocassion Picodet_vision::get_label_with_filter()
// {
//     Clocassion CL = Clocassion();
//     CL.label = null_class;
//     if (this->Results.size() != 0)
//         for (int i = 0; i < this->Results.size(); i++)
//         {
//             if ((Results[i].x2 - Results[i].x1) > BoxLim[0] || (Results[i].y2 - Results[i].y1) > BoxLim[1])
//             {
//                 CL.label = null_class;
//                 continue;
//             }
//             int C = this->Results[i].label;
//             cout << class_names[C] << endl;
//             if (6 == C && !this->F_Terrorists) //(4 == C && !this->F_Terrorists) //(5 == C && !this->F_Terrorists)
//             {
//                 C = terrorists_1;
//                 Terr[C]++;
//                 if (Terr[C] == Terr_Times)
//                     this->F_Terrorists = C;
//             }
//             else if (5 == C && !this->F_Terrorists) //(5 == C && !this->F_Terrorists) //(6 == C && !this->F_Terrorists)
//             {
//                 C = terrorists_2;
//                 Terr[C]++;
//                 if (Terr[C] == Terr_Times)
//                     this->F_Terrorists = C;
//             }
//             else if (7 == C && !this->F_Terrorists) //(6 == C && !this->F_Terrorists)
//             {
//                 C = terrorists_3;
//                 Terr[C]++;
//                 if (Terr[C] == Terr_Times)
//                     this->F_Terrorists = C;
//             }
//             else if ((4 == C) && this->F_Terrorists == 1 && !this->F_Get_stage) //((3 == C) && this->F_Terrorists == 1 && !this->F_Get_stage) //((0 == C) && this->F_Terrorists == 1 && !this->F_Get_stage)
//             {
//                 C = baton;
//             }
//             else if ((3 == C || 2 == C) && this->F_Terrorists == 2 && !this->F_Get_stage) //((7 == C || 2 == C) && this->F_Terrorists == 2 && !this->F_Get_stage) //((1 == C || 2 == C) && this->F_Terrorists == 2 && !this->F_Get_stage)
//             {
//                 C = body_armor;
//             }
//             else if ((0 == C || 1 == C) && this->F_Terrorists == 3 && !this->F_Get_stage) //((3 == C || 4 == C) && this->F_Terrorists == 3 && !this->F_Get_stage)
//             {
//                 C = teargas;
//             }
//             else
//             {
//                 C = null_class;
//             }
//             if (C)
//             {
//                 CL.label = C;
//                 CL.location = (Results[i].x2 + Results[i].x1) / 2;
//                 CL.cx1 = Results[i].x1;
//                 CL.cy1 = (Results[i].y2 + Results[i].y1) / 2;
//                 CL.cx2 = CL.location; // Results[i].x2; //
//                 CL.cy2 = Results[i].y2;
//                 break;
//             }
//         }
//     return CL;
// };
// ////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<Clocassion> Picodet_vision::get_label()
{
    auto CL = std::vector<Clocassion>(Results.size());

    for (int i = 0; i < Results.size(); i++)
    {

        // if ((Results[i].x2 - Results[i].x1) > BoxLim[0] || (Results[i].y2 - Results[i].y1) > BoxLim[1])
        // {
        //     CL[i].label = null_class;
        //     continue;
        // }
        int C = this->Results[i].label;
        if (class_order[3] == C) //(3 == C) //(0 == C)
        {
            C = pie;
        }
        else if (class_order[4] == C)
        {
            C = red;
        }
        else if( class_order[5] == C) //(7 == C || 2 == C) //(1 == C || 2 == C)
        {
            C = nana;
        }
        else if (class_order[6] == C)
        {
            C = coke;
        }
        else if(class_order[7] == C) //(3 == C || 4 == C)
        {
            C = pep;
        }
        else if (class_order[0] == C) //(4 == C) //(5 == C)
        {
            C = green;
        } // 13
        else if (class_order[1] == C) //(5 == C) //(6 == C)
        {
            C = tom;
        } // 7
        else if (class_order[2] == C) //(6 == C)
        {
            C = milk;
        }
        else if (class_order[8] == C)
        {
            C = pot;
        }
        else if(class_order[9] == C)
        {
            C = apple;
        }
        else if(class_order[10] == C)
        {
            C = melon;
        }
        CL[i].label = C;
        CL[i].location = (Results[i].x2 + Results[i].x1) / 2; //*2
        CL[i].cx1 = Results[i].x1;
        CL[i].cy1 = (Results[i].y2 + Results[i].y1) / 2;
        CL[i].cx2 = CL[i].location; // Results[i].x2; //
        CL[i].cy2 = Results[i].y2;
    }
    return CL;
};

int Terr[4] = { 0, 0, 0, 0 };
int Terr_Times = 1;
int LCL_F = 0;
Clocassion LCL = Clocassion();
// Clocassion Picodet_vision::get_label_with_filter()
// {
//     LCL.cx1 = 0;
//     LCL_F=0;
//     Clocassion CL = Clocassion();
//     CL.label = null_class;
//     if (this->Results.size() != 0)
//         for (int i = 0; i < this->Results.size(); i++)
//         {
//             if ((Results[i].x2 - Results[i].x1) > BoxLim[0] || (Results[i].y2 - Results[i].y1) > BoxLim[1])
//             {
//                 CL.label = null_class;
//                 continue;
//             }
//             int C = this->Results[i].label;
//             cout << class_names[C] << endl;
//             if (class_order[0] == C && !this->F_Terrorists) //(4 == C && !this->F_Terrorists) //(5 == C && !this->F_Terrorists)
//             {
//                 C = terrorists_1;
//                 Terr[C]++;
//                 if (Terr[C] == Terr_Times)
//                     this->F_Terrorists = C;
//             }
//             else if (class_order[1] == C && !this->F_Terrorists) //(5 == C && !this->F_Terrorists) //(6 == C && !this->F_Terrorists)
//             {
//                 C = terrorists_2;
//                 Terr[C]++;
//                 if (Terr[C] == Terr_Times)
//                     this->F_Terrorists = C;
//             }
//             else if (class_order[2] == C && !this->F_Terrorists) //(6 == C && !this->F_Terrorists)
//             {
//                 C = terrorists_3;
//                 Terr[C]++;
//                 if (Terr[C] == Terr_Times)
//                     this->F_Terrorists = C;
//             }
//             else if ((class_order[3] == C) && this->F_Terrorists == 1 && !this->F_Get_stage) //((3 == C) && this->F_Terrorists == 1 && !this->F_Get_stage) //((0 == C) && this->F_Terrorists == 1 && !this->F_Get_stage)
//             {
//                 C = baton;
//             }
//             else if ((class_order[4] == C || class_order[5] == C) && this->F_Terrorists == 2 && !this->F_Get_stage) //((7 == C || 2 == C) && this->F_Terrorists == 2 && !this->F_Get_stage) //((1 == C || 2 == C) && this->F_Terrorists == 2 && !this->F_Get_stage)
//             {
//                 C = body_armor;
//             }
//             else if ((class_order[6] == C || class_order[7] == C) && this->F_Terrorists == 3 && !this->F_Get_stage) //((3 == C || 4 == C) && this->F_Terrorists == 3 && !this->F_Get_stage)
//             {
//                 C = teargas;
//             }
//             else
//             {
//                 C = null_class;
//             }
            
//             if (C)
//             {
//                 CL.label = C;
//                 CL.location = (Results[i].x2 + Results[i].x1) / 2;
//                 CL.cx1 = Results[i].score;
//                 CL.cy1 = (Results[i].y2 + Results[i].y1) / 2;
//                 CL.cx2 = CL.location; // Results[i].x2; //
//                 CL.cy2 = Results[i].y2;
//                 if (CL.cx1 > LCL.cx1)
//                 {
//                     LCL.label = CL.label;
//                     LCL.location = CL.location;
//                     LCL.cx1 = CL.cx1;
//                     LCL.cy1 = CL.cy1;
//                     LCL.cx2 = CL.cx2;
//                     LCL.cy2 = CL.cy2;
//                     LCL_F = 1;
//                 }
//             }
//         }
//     if (LCL_F) {
//         CL.label = LCL.label;
//         CL.location = LCL.location;
//         CL.cx1 = LCL.cx1;
//         CL.cy1 = LCL.cy1;
//         CL.cx2 = LCL.cx2;
//         CL.cy2 = LCL.cy2;
//     }
//     return CL;
// };
////////////////////////////////////////////////////////////////////////////////////////////////////////

void Picodet_vision::resize_uniform(cv::Mat &src, cv::Mat &dst, cv::Size dst_size)
{
    int w = src.cols;
    int h = src.rows;
    int dst_w = dst_size.width;
    int dst_h = dst_size.height;

    dst = cv::Mat(cv::Size(dst_w, dst_h), CV_8UC3, cv::Scalar(0));

    float ratio_src = w * 1.0 / h;
    float ratio_dst = dst_w * 1.0 / dst_h;

    int tmp_w = 0;
    int tmp_h = 0;
    if (ratio_src > ratio_dst)
    {
        tmp_w = dst_w;
        tmp_h = floor((dst_w * 1.0 / w) * h);
    }
    else if (ratio_src < ratio_dst)
    {
        tmp_h = dst_h;
        tmp_w = floor((dst_h * 1.0 / h) * w);
    }
    else
    {
        cv::resize(src, dst, dst_size);
        return;
    }

    cv::Mat tmp;
    cv::resize(src, tmp, cv::Size(tmp_w, tmp_h));

    if (tmp_w != dst_w)
    {
        int index_w = floor((dst_w - tmp_w) / 2.0);
        for (int i = 0; i < dst_h; i++)
        {
            memcpy(dst.data + i * dst_w * 3 + index_w * 3, tmp.data + i * tmp_w * 3, tmp_w * 3);
        }
    }
    else if (tmp_h != dst_h)
    {
        int index_h = floor((dst_h - tmp_h) / 2.0);
        memcpy(dst.data + index_h * dst_w * 3, tmp.data, tmp_w * tmp_h * 3);
    }
    else
    {
        std::cout << "error\n";
    }
};

cv::Mat Picodet_vision::draw_bboxes(const cv::Mat &Im, const std::vector<BoxInfo> &bboxes, int show)
{
    cv::Mat im = Im.clone();
    int src_w = im.cols;
    int src_h = im.rows;
    for (size_t i = 0; i < bboxes.size(); i++)
    {
        const BoxInfo &bbox = bboxes[i];
        cv::Scalar color = cv::Scalar(color_list[bbox.label][0], color_list[bbox.label][1], color_list[bbox.label][2]);
        cv::rectangle(im, cv::Rect(cv::Point(bbox.x1, bbox.y1), cv::Point(bbox.x2, bbox.y2)), color, 1);
        char text[256];
        std::sprintf(text, "%s %.1f%%", class_names[bbox.label], bbox.score * 100);
        int baseLine = 0;
        cv::Size label_size =
            cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
        int x = bbox.x1;
        int y = bbox.y1 - label_size.height - baseLine;
        if (y < 0)
            y = 0;
        if (x + label_size.width > im.cols)
            x = im.cols - label_size.width;

        cv::rectangle(im, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                      color, -1);

        cv::putText(im, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
    }
    if (show)
        cv::imshow("draw", im);
    return im;
}

std::vector<int> Picodet_vision::GenerateColorMap(int num_class)
{
    auto colormap = std::vector<int>(3 * num_class, 0);
    for (int i = 0; i < num_class; ++i)
    {
        int j = 0;
        int lab = i;
        while (lab)
        {
            colormap[i * 3] |= (((lab >> 0) & 1) << (7 - j));
            colormap[i * 3 + 1] |= (((lab >> 1) & 1) << (7 - j));
            colormap[i * 3 + 2] |= (((lab >> 2) & 1) << (7 - j));
            ++j;
            lab >>= 3;
        }
    }
    return colormap;
}

std::vector<BoxInfo> Picodet_vision::trans_Box(std::vector<BoxInfo> &Box, int w, int h)
{

    int tmp_w = this->model_size_w;
    int tmp_h = floor((this->model_size_w * 1.0 / w) * h);
    float width_ratio = (float)w / (float)tmp_w;
    float height_ratio = (float)h / (float)tmp_h;
    int index_w = 0;
    int index_h = floor((this->model_size_w - tmp_h) / 2.0);
    for (int i = 0; i < Box.size(); i++)
    {
        Box[i].x1 = (Box[i].x1 - index_w) * width_ratio;
        Box[i].y1 = (Box[i].y1 - index_h) * height_ratio;
        Box[i].x2 = (Box[i].x2 - index_w) * width_ratio;
        Box[i].y2 = (Box[i].y2 - index_h) * height_ratio;
    }
    return std::vector<BoxInfo>(Box);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////基本上用不到的玩意
int Picodet_vision::image_demo(PicoDet &detector, const char *imagepath)
{
    std::vector<cv::String> filenames;
    cv::glob(imagepath, filenames, false);
    int height = detector.in_h;
    int width = detector.in_w;
    for (auto img_name : filenames)
    {
        cv::Mat image = cv::imread(img_name);
        if (image.empty())
        {
            fprintf(stderr, "cv::imread failed\n");
            return -1;
        }
        cv::Mat resized_img;
        resize_uniform(image, resized_img, cv::Size(width, height));
        std::vector<BoxInfo> results;
        detector.detect(resized_img, results, score_threshold, nms_threshold);
        draw_bboxes(image, results, 1);
        cv::waitKey(0);
    }
    return 0;
}

int Picodet_vision::webcam_demo()
{
    int height = detector.in_h;
    int width = detector.in_w;
    int key;
    double fps[5] = {0, 0, 0, 0, 0};
    int i = 0;

    while (true)
    {
        double t = (double)cv::getTickCount();

        this->cap >> this->image;

        cv::Mat resized_img;
        resize_uniform(image, resized_img, cv::Size(width, height));
        std::vector<BoxInfo> results;
        detector.detect(resized_img, results, score_threshold, nms_threshold);

        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        fps[i] = 1.0 / t;
        char FPStext[20];
        std::sprintf(FPStext, "FPS:%.2f", (fps[0] + fps[1] + fps[2] + fps[3] + fps[4]) / 5.);
        cv::putText(image, FPStext, cv::Point(5, 20),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 0, 0));
        trans_Box(results, image.cols, image.rows);
        draw_bboxes(image, results, 1);
        key = cv::waitKey(1);
        if (key == 'q')
            break;
        i = (++i) % 5;
    }
    return 0;
}

int Picodet_vision::video_demo(PicoDet &detector, const char *path, int save_frame, int draw)
{
    cv::Mat Image;
    cv::VideoCapture CAp(path);
    int height = detector.in_h;
    int width = detector.in_w;

    while (true)
    {
        CAp >> Image;
        cv::Mat resized_img;
        std::vector<BoxInfo> results;
        resize_uniform(Image, resized_img, cv::Size(width, height));
        detector.detect(resized_img, results, this->score_threshold, this->nms_threshold);
        trans_Box(results, image.cols, image.rows);
        draw_bboxes(image, results, 1);
        cv::waitKey(1);
    }
    return 0;
}