#include "../include/eight_neighborhood.h"

using namespace cv;
using namespace std;

uint8_t G_initial_L[2] = {0}; // 左边起点的x，y值
uint8_t G_initial_R[2] = {0}; // 右边起点的x，y值

uint16_t G_points_L[num_VPR][2] = {{0}}; // 左边线点位置数据_全局变量
uint16_t G_points_R[num_VPR][2] = {{0}}; // 右边线点位置数据_全局变量
uint16_t G_dir_L[num_VPR] = {0};         // 左生长方向数据_全局变量
uint16_t G_dir_R[num_VPR] = {0};         // 右生长方向数据_全局变量

uint8_t G_border_L[120] = {0}; // 左线数组
uint8_t G_border_R[120] = {0}; // 右线数组
uint8_t G_line_M[120] = {0};   // 中线数组

uint16_t count_L = 0; // 左边线点数统计
uint16_t count_R = 0; // 右边线点数统计

uint8_t Get_Initial_Point(std::vector<std::vector<uint8_t>> &image, uint8_t width, uint8_t initial_row)
{
    uint8_t i = 0, found_L = 0, found_R = 0;
    G_initial_L[0] = 0;
    G_initial_L[1] = 0; // 起点坐标置零(数组数据顺序为：0-x，1-y)
    G_initial_R[0] = 0;
    G_initial_R[1] = 0;
    /*由中间向左侧寻找八邻域左边线起点*/
    for (i = width / 2; i > 1; i--) // ，先找起点
    {
        G_initial_L[0] = i;           // x
        G_initial_L[1] = initial_row; // y
        if (image[initial_row][i] == 255 && image[initial_row][i - 1] == 0)
        {
            found_L = 1;
            break;
        }
    }
    /*由中间向左侧寻找八邻域左边线起点*/
    for (i = width / 2; i < width - 2; i++)
    {
        G_initial_R[0] = i;           // x
        G_initial_R[1] = initial_row; // y
        if (image[initial_row][i] == 255 && image[initial_row][i + 1] == 0)
        {
            found_R = 1;
            break;
        }
    }
    /*确认左右边线起点是否存在*/
    if (found_L && found_R)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void Eight_Neighborhood_Crawl_L_R(uint16_t times, const std::vector<std::vector<uint8_t>> &image, uint8_t highest)
{
    uint8_t i = 0, j = 0;

    // 左变量
    uint8_t search_fields_L[8][2] = {{0}};                                                                // 搜索空间
    uint8_t current_point_L[2] = {0};                                                                     // 当前点位
    uint8_t index_L = 0;                                                                                  // 索引下标
    uint8_t temp_L[8][2] = {{0}};                                                                         // 中继数组
    static int8_t seeds_L[8][2] = {{0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}}; // 定义八个邻域，顺序为顺时针
    // 右变量
    uint8_t search_fields_R[8][2] = {{0}};                                                                // 搜索空间
    uint8_t current_point_R[2] = {0};                                                                     // 当前点位
    uint8_t index_R = 0;                                                                                  // 索引下标
    uint8_t temp_R[8][2] = {{0}};                                                                         // 中继数组
    static int8_t seeds_R[8][2] = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}}; // 定义八个邻域，顺序为逆时针

    /*初始化*/
    count_L = 0;
    count_R = 0;
    current_point_L[0] = G_initial_L[0]; // 第一次更新坐标点，将寻找起点时找到的起点值传进来(数组数据顺序为：0-x，1-y)
    current_point_L[1] = G_initial_L[1];
    current_point_R[0] = G_initial_R[0];
    current_point_R[1] = G_initial_R[1];

    /*开始循环爬取*/
    while (times--)
    {
        /*更新左侧目标点位八邻域位置*/
        for (i = 0; i < 8; i++)
        {
            search_fields_L[i][0] = current_point_L[0] + seeds_L[i][0]; // 传递x坐标
            search_fields_L[i][1] = current_point_L[1] + seeds_L[i][1]; // 传递y坐标
        }
        /*将中心坐标点填充到已经找到的点内*/
        if (count_L < sizeof(G_points_L) / sizeof(G_points_L[0]))
        {
            G_points_L[count_L][0] = current_point_L[0]; // 传递x坐标
            G_points_L[count_L][1] = current_point_L[1]; // 传递y坐标
        }
        count_L++; // 左点数量计数加一

        /*更新右侧目标点位八邻域位置*/
        for (i = 0; i < 8; i++)
        {
            search_fields_R[i][0] = current_point_R[0] + seeds_R[i][0]; // 传递x坐标
            search_fields_R[i][1] = current_point_R[1] + seeds_R[i][1]; // 传递y坐标
        }
        /*将中心坐标点填充到已经找到的点内*/
        if (count_R < sizeof(G_points_R) / sizeof(G_points_R[0]))
        {
            G_points_R[count_R][0] = current_point_R[0]; // 传递x坐标
            G_points_R[count_R][1] = current_point_R[1]; // 传递y坐标
        }

        /*左侧边线爬取前重置*/
        index_L = 0; // 重置
        for (i = 0; i < 8; i++)
        {
            temp_L[i][0] = 0; // 重置中继数组为零，避免出现问题
            temp_L[i][1] = 0;
        }
        /*左侧边线爬取判断*/
        for (i = 0; i < 8; i++) // 围绕中心点0开始，从一号位开始到第八位，顺时针循环进行对比，满足色彩跳变，则更新坐标进行下一轮爬取
        {
            if (search_fields_L[i][0] < image[0].size() &&
                search_fields_L[i][1] < image.size() &&
                search_fields_L[(i + 1) & 7][0] < image[0].size() &&
                search_fields_L[(i + 1) & 7][1] < image.size())
            {
                if (image[search_fields_L[i][1]][search_fields_L[i][0]] == 0 &&
                    image[search_fields_L[(i + 1) & 7][1]][search_fields_L[(i + 1) & 7][0]] == 255)
                {
                    temp_L[index_L][0] = search_fields_L[i][0];
                    temp_L[index_L][1] = search_fields_L[i][1];
                    index_L++;
                    if (count_L - 1 < sizeof(G_dir_L) / sizeof(G_dir_L[0]))
                    {
                        G_dir_L[count_L - 1] = i; // 记录生长方向
                    }
                }
            }
        }
        /*更新坐标点*/
        if (index_L)
        {

            current_point_L[0] = temp_L[0][0]; // x
            current_point_L[1] = temp_L[0][1]; // y
            for (j = 0; j < index_L; j++)
            {
                if (current_point_L[1] > temp_L[j][1])
                {
                    current_point_L[0] = temp_L[j][0]; // x
                    current_point_L[1] = temp_L[j][1]; // y
                }
            }
        }

        /* 检查是否需要退出循环*/
        if (count_R >= 2 && count_L >= 3)
        { // 如果任意一侧三次进入同一个点，则退出
            if ((G_points_R[count_R][0] == G_points_R[count_R - 1][0] &&
                 G_points_R[count_R][0] == G_points_R[count_R - 2][0] &&
                 G_points_R[count_R][1] == G_points_R[count_R - 1][1] &&
                 G_points_R[count_R][1] == G_points_R[count_R - 2][1]) ||
                (G_points_L[count_L - 1][0] == G_points_L[count_L - 2][0] &&
                 G_points_L[count_L - 1][0] == G_points_L[count_L - 3][0] &&
                 G_points_L[count_L - 1][1] == G_points_L[count_L - 2][1] &&
                 G_points_L[count_L - 1][1] == G_points_L[count_L - 3][1]))
            {
                break;
            }
        }
        if (abs(G_points_R[count_R][0] - G_points_L[count_L - 1][0]) < 2 && // 如果左右相遇，退出
            abs(G_points_R[count_R][1] - G_points_L[count_L - 1][1]) < 2)
        {
            highest = (G_points_R[count_R][1] + G_points_L[count_L - 1][1]) >> 1; // 取出最高点
            break;
        }
        if (G_points_R[count_R][1] < G_points_L[count_L - 1][1]) // 如果左边比右边高了，左边等待右边
        {
            continue;
        }
        if (G_dir_L[count_L - 1] == 7 && G_points_R[count_R][1] > G_points_L[count_L - 1][1]) // 如果左边开始向下了，等待右边
        {
            current_point_L[0] = G_points_L[count_L - 1][0]; // x
            current_point_L[1] = G_points_L[count_L - 1][1]; // y
            count_L--;
        }
        count_R++; // 右点数量计数加一

        /*右侧边线爬取前判断*/
        index_R = 0; // 重置
        for (i = 0; i < 8; i++)
        {
            temp_R[i][0] = 0; // 重置中继数组为零，避免出现问题
            temp_R[i][1] = 0;
        }
        /*右侧边线爬取判断*/
        for (i = 0; i < 8; i++) // 围绕中心点0开始，从一号位开始到第八位，逆时针循环进行对比，满足色彩跳变，则更新坐标进行下一轮爬取
        {
            if (search_fields_R[i][0] < image[0].size() &&
                search_fields_R[i][1] < image.size() &&
                search_fields_R[(i + 1) & 7][0] < image[0].size() &&
                search_fields_R[(i + 1) & 7][1] < image.size())
            {
                if (image[search_fields_R[i][1]][search_fields_R[i][0]] == 0 &&
                    image[search_fields_R[(i + 1) & 7][1]][search_fields_R[(i + 1) & 7][0]] == 255)
                {
                    temp_R[index_R][0] = search_fields_R[i][0];
                    temp_R[index_R][1] = search_fields_R[i][1];
                    index_R++;
                    if (count_R - 1 < sizeof(G_dir_R) / sizeof(G_dir_R[0]))
                    {
                        G_dir_R[count_R - 1] = i; // 记录生长方向
                    }
                }
            }
        }
        /*更新坐标点*/
        if (index_R)
        {
            current_point_R[0] = temp_R[0][0]; // x
            current_point_R[1] = temp_R[0][1]; // y
            for (j = 0; j < index_R; j++)
            {
                if (current_point_R[1] > temp_R[j][1])
                {
                    current_point_R[0] = temp_R[j][0]; // x
                    current_point_R[1] = temp_R[j][1]; // y
                }
            }
        }
    }
}

void Extract_L(uint16_t total_L)
{
    uint8_t i = 0;
    uint16_t j = 0;
    uint8_t h = 0;

    for (i = 0; i < 120; i++) // 左边线初始化放到最左边，右边线放到最右边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    {
        G_border_L[i] = 1;
    }
    h = 120 - 2;

    /*提取有用的边线*/
    for (j = 0; j < total_L; j++)
    {
        if (G_points_L[j][1] == h) // 一条边线每一列只需要一个最靠中间的点
        {
            G_border_L[h] = G_points_L[j][0] + 1;
        }
        else
            continue; // 每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0) // 到最后一行退出
        {
            break;
        }
    }
}

void Extract_R(uint16_t total_R)
{
    uint8_t i = 0;
    uint16_t j = 0;
    uint8_t h = 0;

    for (i = 0; i < 120; i++) // 右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    {
        G_border_R[i] = 158;
    }
    h = 120 - 2;
    // 右边
    for (j = 0; j < total_R; j++)
    {
        if (G_points_R[j][1] == h) // 一条边线每一列只需要一个最靠中间的点
        {
            G_border_R[h] = G_points_R[j][0] - 1;
        }
        else
            continue; // 每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0) // 到最后一行退出
        {
            break;
        }
    }
}

void Eight_Neighborhood_Method(Mat &original_frame)
{

    resize(original_frame, original_frame, Size(160, 120), 0, 0, INTER_LINEAR); // 压缩分辨率
    flip(original_frame, original_frame, 1);                                    // 水平反转
    cvtColor(original_frame, original_frame, COLOR_BGR2GRAY);                   // 原始图像转换灰度图像

    if (!original_frame.empty())
    {
        /*将图片转换为二维数组*/
        std::vector<std::vector<uint8_t>> image(original_frame.rows, std::vector<uint8_t>(original_frame.cols));
        for (int i = 0; i < original_frame.rows; ++i)
        {
            for (int j = 0; j < original_frame.cols; ++j)
            {
                image[i][j] = original_frame.at<uint8_t>(i, j);
            }
        }

        /*使用大津法自适应二值化对图像再次处理*/
        uint8_t threshold = Otsu_Threshold(image, original_frame.cols, original_frame.rows); // 计算大津阈值
        Binari_Image(image, threshold);                                                      // 二值化
        Image_Filter(image, original_frame.cols, original_frame.rows);                       // 滤波
        Add_Border(image, original_frame.cols, original_frame.rows, 0);                      // 增加边框，方便框出有效范围

        /*八邻域爬取左右有效边线*/
        if (Get_Initial_Point(image, original_frame.cols, original_frame.rows - 1)) // 是否能找到八邻域起点
        {
            Eight_Neighborhood_Crawl_L_R(num_VPR, image, original_frame.rows * 0.6 + 1); // 八邻域爬取左右边线
            Extract_L(count_L);                                                          // 提取有效左线
            Extract_R(count_R);                                                          // 提取有效右线
        }
        /*计算中线位置*/
        for (int i = original_frame.rows * 0.6; i < original_frame.rows - 1; i++)
        {
            G_line_M[i] = (G_border_L[i] + G_border_R[i]) / 2; // 简单取均值
        }

        Mat debug_frame = Array2Mat(image);
        cvtColor(debug_frame, debug_frame, COLOR_GRAY2BGR);                           // 灰度图转换为彩图(为了显示左中右三条线时更加清晰)
        for (int i = original_frame.rows * 0.6 + 1; i < original_frame.rows - 2; i++) // 显示三条线
        {
            line(debug_frame, Point(G_border_L[i], i), Point(G_border_L[i + 1], i + 1), Scalar(0, 255, 0), 1);
            line(debug_frame, Point(G_border_R[i], i), Point(G_border_R[i + 1], i + 1), Scalar(0, 255, 0), 1);
            line(debug_frame, Point(G_line_M[i], i), Point(G_line_M[i + 1], i + 1), Scalar(255, 0, 0), 1);
        }
        resize(debug_frame, debug_frame, Size(640, 480), 0, 0, INTER_LINEAR); // 压缩分辨率
        imshow("debug frame", debug_frame);
    }
}
